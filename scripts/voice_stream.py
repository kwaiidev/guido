#!/usr/bin/env python3
"""Stream microphone speech recognition to stdout using Vosk.

Recommended offline usage:
  python3 scripts/voice_stream.py --model /path/to/unpacked-vosk-model

The script keeps stdout transcript-only so it can be piped into another
process. Status, warnings, and errors are written to stderr.
"""

from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path
import queue
import sys
from typing import Any


class VoiceStreamError(RuntimeError):
    """Raised when the voice stream cannot start or continue safely."""


def eprint(message: str) -> None:
    print(message, file=sys.stderr, flush=True)


def int_or_str(value: str) -> int | str:
    try:
        return int(value)
    except ValueError:
        return value


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--model",
        type=str,
        help="Path to an unpacked Vosk model directory. Recommended for fully offline use.",
    )
    parser.add_argument(
        "--language",
        type=str,
        default="en-us",
        help="Vosk language/model identifier when --model is not provided (default: en-us).",
    )
    parser.add_argument(
        "--device",
        type=int_or_str,
        help="Input device numeric ID or substring match. Use --list-devices to inspect devices.",
    )
    parser.add_argument(
        "--samplerate",
        type=int,
        help="Input sample rate in Hz. Defaults to the selected device's default input rate.",
    )
    parser.add_argument(
        "--jsonl",
        action="store_true",
        help="Emit JSON Lines with text, timestamp, and is_final fields.",
    )
    parser.add_argument(
        "--timestamps",
        action="store_true",
        help="Include wall-clock timestamps in text mode and word timing metadata in JSONL mode.",
    )
    parser.add_argument(
        "--partials",
        action="store_true",
        help="Emit partial hypotheses as they change.",
    )
    parser.add_argument(
        "--list-devices",
        action="store_true",
        help="List available audio devices and exit.",
    )
    return parser.parse_args(argv)


def import_sounddevice():
    try:
        import sounddevice as sd
    except ModuleNotFoundError as exc:
        raise VoiceStreamError(
            "Missing Python dependency 'sounddevice'. Install with "
            "'python3 -m pip install -r scripts/requirements-voice.txt'. "
            "On Ubuntu/Jetson you may also need 'sudo apt install portaudio19-dev'."
        ) from exc
    return sd


def import_vosk():
    try:
        from vosk import KaldiRecognizer, Model, SetLogLevel
    except ModuleNotFoundError as exc:
        raise VoiceStreamError(
            "Missing Python dependency 'vosk'. Install with "
            "'python3 -m pip install -r scripts/requirements-voice.txt'."
        ) from exc
    return Model, KaldiRecognizer, SetLogLevel


def list_devices() -> int:
    sd = import_sounddevice()
    try:
        print(sd.query_devices())
    except Exception as exc:  # pragma: no cover - depends on host audio stack
        raise VoiceStreamError(f"Could not query audio devices: {exc}") from exc
    return 0


def resolve_sample_rate(sd, device: int | str | None, samplerate: int | None) -> tuple[int, dict[str, Any]]:
    try:
        device_info = sd.query_devices(device, "input")
    except Exception as exc:  # pragma: no cover - depends on host audio stack
        raise VoiceStreamError(
            "Could not open the requested input device. Use --list-devices to inspect "
            "available microphones and pass --device with a valid ID or name."
        ) from exc

    max_input_channels = int(device_info.get("max_input_channels", 0))
    if max_input_channels < 1:
        raise VoiceStreamError("The selected audio device does not expose an input channel.")

    if samplerate is None:
        default_rate = device_info.get("default_samplerate")
        if not default_rate:
            raise VoiceStreamError(
                "Could not determine a default input sample rate for the selected device. "
                "Pass --samplerate explicitly."
            )
        samplerate = int(default_rate)

    if samplerate <= 0:
        raise VoiceStreamError("--samplerate must be a positive integer.")

    return samplerate, device_info


def load_model(model_arg: str | None, language: str, model_cls):
    if model_arg:
        model_path = Path(model_arg).expanduser()
        if not model_path.exists():
            raise VoiceStreamError(f"Model path does not exist: {model_path}")
        if not model_path.is_dir():
            raise VoiceStreamError(f"Model path is not a directory: {model_path}")

        try:
            return model_cls(str(model_path)), str(model_path)
        except Exception as exc:
            raise VoiceStreamError(
                f"Failed to load Vosk model from {model_path}: {exc}"
            ) from exc

    try:
        return model_cls(lang=language), language
    except TypeError as exc:
        raise VoiceStreamError(
            "This Vosk build does not support loading by --language alone. "
            "Download an unpacked model from https://alphacephei.com/vosk/models "
            "and pass --model /path/to/model."
        ) from exc
    except Exception as exc:
        raise VoiceStreamError(
            f"Failed to load a Vosk model for language '{language}'. "
            "For predictable offline use, download an unpacked model from "
            "https://alphacephei.com/vosk/models and pass --model /path/to/model. "
            f"Original error: {exc}"
        ) from exc


def now_timestamp() -> str:
    return datetime.now().astimezone().isoformat(timespec="milliseconds")


def clean_text(text: str) -> str:
    return " ".join(text.split())


def parse_result(raw: str, label: str) -> dict[str, Any]:
    try:
        payload = json.loads(raw)
    except json.JSONDecodeError as exc:
        eprint(f"warning: ignoring invalid {label} JSON from Vosk: {exc}")
        return {}

    if not isinstance(payload, dict):
        eprint(f"warning: ignoring unexpected {label} payload from Vosk")
        return {}

    return payload


def extract_word_timing(payload: dict[str, Any]) -> tuple[float | None, float | None]:
    words = payload.get("result")
    if not isinstance(words, list) or not words:
        return None, None

    first = words[0]
    last = words[-1]
    if not isinstance(first, dict) or not isinstance(last, dict):
        return None, None

    start = first.get("start")
    end = last.get("end")
    if not isinstance(start, (int, float)) or not isinstance(end, (int, float)):
        return None, None

    return round(float(start), 3), round(float(end), 3)


def emit_event(
    *,
    text: str,
    is_final: bool,
    args: argparse.Namespace,
    payload: dict[str, Any] | None = None,
) -> None:
    text = clean_text(text)
    if not text:
        return

    timestamp = now_timestamp()

    if args.jsonl:
        line: dict[str, Any] = {
            "text": text,
            "timestamp": timestamp,
            "is_final": is_final,
        }
        if args.timestamps and payload:
            start_sec, end_sec = extract_word_timing(payload)
            if start_sec is not None:
                line["start_sec"] = start_sec
            if end_sec is not None:
                line["end_sec"] = end_sec

        sys.stdout.write(json.dumps(line, ensure_ascii=False) + "\n")
        sys.stdout.flush()
        return

    prefix = ""
    if args.timestamps:
        prefix = f"{timestamp}\t"

    if is_final:
        sys.stdout.write(f"{prefix}{text}\n")
    else:
        sys.stdout.write(f"{prefix}PARTIAL\t{text}\n")
    sys.stdout.flush()


def flush_final_result(recognizer, args: argparse.Namespace) -> None:
    payload = parse_result(recognizer.FinalResult(), "final")
    emit_event(text=str(payload.get("text", "")), is_final=True, args=args, payload=payload)


def run_stream(args: argparse.Namespace) -> int:
    sd = import_sounddevice()
    model_cls, recognizer_cls, set_log_level = import_vosk()
    set_log_level(-1)

    samplerate, device_info = resolve_sample_rate(sd, args.device, args.samplerate)
    model, model_label = load_model(args.model, args.language, model_cls)

    device_name = str(device_info.get("name", args.device if args.device is not None else "default"))
    audio_queue: queue.Queue[bytes] = queue.Queue()

    def callback(indata, frames, time_info, status) -> None:
        del frames, time_info
        if status:
            eprint(f"audio: {status}")
        audio_queue.put(bytes(indata))

    recognizer = recognizer_cls(model, samplerate)
    if args.timestamps:
        recognizer.SetWords(True)
        if args.partials and hasattr(recognizer, "SetPartialWords"):
            recognizer.SetPartialWords(True)

    eprint(
        f"voice_stream listening on '{device_name}' at {samplerate} Hz using model '{model_label}'. "
        "Press Ctrl+C to stop."
    )

    last_partial = ""

    try:
        with sd.RawInputStream(
            samplerate=samplerate,
            blocksize=8000,
            device=args.device,
            dtype="int16",
            channels=1,
            callback=callback,
        ):
            while True:
                data = audio_queue.get()
                if recognizer.AcceptWaveform(data):
                    payload = parse_result(recognizer.Result(), "result")
                    emit_event(
                        text=str(payload.get("text", "")),
                        is_final=True,
                        args=args,
                        payload=payload,
                    )
                    last_partial = ""
                    continue

                if not args.partials:
                    continue

                payload = parse_result(recognizer.PartialResult(), "partial")
                partial_text = clean_text(str(payload.get("partial", "")))
                if not partial_text or partial_text == last_partial:
                    continue

                emit_event(text=partial_text, is_final=False, args=args)
                last_partial = partial_text
    except KeyboardInterrupt:
        eprint("voice_stream interrupted; stopping.")
        flush_final_result(recognizer, args)
        return 0
    except Exception as exc:  # pragma: no cover - depends on host audio stack
        raise VoiceStreamError(
            f"Microphone stream failed. Check microphone permissions, device selection, "
            f"and sample rate support. Original error: {exc}"
        ) from exc


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)

    try:
        if args.list_devices:
            return list_devices()
        return run_stream(args)
    except VoiceStreamError as exc:
        eprint(f"error: {exc}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
