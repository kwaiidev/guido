#!/usr/bin/env python3
"""Consume transcript lines from stdin and feed them into Guido's ADK agent.

Designed to sit after scripts/voice_stream.py:

  python3 scripts/voice_stream.py --model /path/to/vosk-model \
    | python3 scripts/adk_transcript_bridge.py

The bridge reads transcript lines from stdin, ignores partial hypotheses,
normalizes spoken phrasing, rate-limits ADK requests, and prints ADK replies
to stdout. Logs and warnings are written to stderr.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import os
from pathlib import Path
import re
import sys
import time
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from google.adk.runners import Runner
from google.adk.sessions import InMemorySessionService

from agents.guido_mission_agent.agent import WAYPOINTS, root_agent

DEFAULT_MIN_INTERVAL_SECONDS = 12.5
DEFAULT_DEDUPE_WINDOW_SECONDS = 2.0
DEFAULT_USER_ID = "voice_stream_user"
DEFAULT_SESSION_ID = "voice_stream_session"
EXIT_WORDS = {"exit", "quit", "stop listening"}
LEADING_FILLER_WORDS = {"hey", "hi", "okay", "ok"}
POLITE_PREFIXES = (
    "please",
    "can you",
    "could you",
    "would you",
    "will you",
)
COMMAND_REWRITES = (
    (r"\b(?:take me to|bring me to|drive me to|head to|navigate to)\b", "go to"),
    (r"\bwhat is my status\b", "what's my status"),
)
TIMESTAMP_PATTERN = re.compile(r"^\d{4}-\d{2}-\d{2}T")


def eprint(message: str) -> None:
    print(message, file=sys.stderr, flush=True)


def build_alias_replacements() -> list[tuple[re.Pattern[str], str]]:
    replacements: list[tuple[re.Pattern[str], str]] = []
    seen_aliases: set[str] = set()

    for destination_id, payload in WAYPOINTS.items():
        aliases = payload.get("aliases", [])
        for alias in aliases:
            alias_key = alias.strip().lower()
            if " " not in alias_key or alias_key in seen_aliases:
                continue
            seen_aliases.add(alias_key)
            replacements.append(
                (re.compile(rf"\b{re.escape(alias_key)}\b", flags=re.IGNORECASE), destination_id)
            )

    replacements.sort(key=lambda item: len(item[0].pattern), reverse=True)
    return replacements


ALIAS_REPLACEMENTS = build_alias_replacements()


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--once",
        help="Process one transcript string and exit.",
    )
    parser.add_argument(
        "--show-normalized",
        action="store_true",
        help="Print normalized transcript lines to stderr before calling ADK.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Normalize and filter transcript input without calling ADK.",
    )
    parser.add_argument(
        "--min-interval",
        type=float,
        default=DEFAULT_MIN_INTERVAL_SECONDS,
        help="Minimum seconds between ADK requests. Default protects Gemini free-tier RPM.",
    )
    parser.add_argument(
        "--dedupe-window",
        type=float,
        default=DEFAULT_DEDUPE_WINDOW_SECONDS,
        help="Ignore repeated normalized transcripts inside this time window.",
    )
    parser.add_argument(
        "--user-id",
        default=DEFAULT_USER_ID,
        help="ADK user id for this transcript session.",
    )
    parser.add_argument(
        "--session-id",
        default=DEFAULT_SESSION_ID,
        help="ADK session id for this transcript session.",
    )
    return parser.parse_args(argv)


def parse_stream_line(raw_line: str) -> tuple[str, bool] | None:
    line = raw_line.strip()
    if not line:
        return None

    if line.startswith("{"):
        try:
            payload = json.loads(line)
        except json.JSONDecodeError:
            payload = None

        if isinstance(payload, dict) and "text" in payload:
            text = str(payload.get("text", "")).strip()
            is_final = bool(payload.get("is_final", True))
            return (text, is_final) if text else None

    if line.startswith("PARTIAL\t"):
        text = line.split("\t", 1)[1].strip()
        return (text, False) if text else None

    if "\t" in line:
        fields = line.split("\t")
        if fields and TIMESTAMP_PATTERN.match(fields[0]):
            if len(fields) >= 3 and fields[1] == "PARTIAL":
                text = "\t".join(fields[2:]).strip()
                return (text, False) if text else None
            if len(fields) >= 2:
                text = "\t".join(fields[1:]).strip()
                return (text, True) if text else None

    return (line, True)


def is_guido_like_wake_word(token: str) -> bool:
    if token == "guido":
        return True

    return len(token) == 5 and token.endswith("uido") and token[0].isalpha()


def strip_wake_prefix(normalized: str) -> str:
    words = normalized.split()
    if not words:
        return normalized

    while words and words[0] in LEADING_FILLER_WORDS:
        words.pop(0)

    while words and is_guido_like_wake_word(words[0]):
        words.pop(0)

    return " ".join(words)


def normalize_transcript(text: str) -> str:
    normalized = " ".join(text.strip().split())
    if not normalized:
        return ""

    normalized = normalized.casefold()
    normalized = re.sub(r"[!?.,]+", " ", normalized)
    normalized = re.sub(r"\s+", " ", normalized).strip()
    normalized = strip_wake_prefix(normalized)

    for prefix in POLITE_PREFIXES:
        prefix_with_space = prefix + " "
        if normalized.startswith(prefix_with_space):
            normalized = normalized[len(prefix_with_space) :].strip()
            break

    for pattern, replacement in COMMAND_REWRITES:
        normalized = re.sub(pattern, replacement, normalized)

    for pattern, destination_id in ALIAS_REPLACEMENTS:
        normalized = pattern.sub(destination_id, normalized)

    normalized = re.sub(r"\s+", " ", normalized).strip()
    return normalized


def extract_text(parts: list[Any] | None) -> str:
    if not parts:
        return ""

    chunks: list[str] = []
    for part in parts:
        text = getattr(part, "text", None)
        if text:
            chunks.append(text)
    return " ".join(" ".join(chunks).split())


async def invoke_agent(
    runner: Runner,
    *,
    transcript: str,
    user_id: str,
    session_id: str,
) -> str:
    events = await runner.run_debug(
        transcript,
        user_id=user_id,
        session_id=session_id,
        quiet=True,
    )

    for event in reversed(events):
        content = getattr(event, "content", None)
        text = extract_text(getattr(content, "parts", None))
        if not text:
            continue
        if getattr(event, "author", "") == root_agent.name and event.is_final_response():
            return text

    for event in reversed(events):
        content = getattr(event, "content", None)
        text = extract_text(getattr(content, "parts", None))
        if text:
            return text

    return "No assistant response was returned."


async def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)

    if not args.dry_run and "GOOGLE_API_KEY" not in os.environ:
        eprint("error: GOOGLE_API_KEY must be set unless --dry-run is used.")
        return 2

    runner = Runner(
        app_name=root_agent.name,
        agent=root_agent,
        session_service=InMemorySessionService(),
    )

    last_transcript = ""
    last_transcript_at = 0.0
    last_request_at = 0.0

    source: Any
    if args.once is not None:
        source = [args.once]
    elif sys.stdin.isatty():
        eprint("adk_transcript_bridge listening for transcript lines on stdin. Type 'exit' to quit.")
        source = None
    else:
        source = sys.stdin

    async def handle_line(raw_line: str) -> bool:
        nonlocal last_transcript, last_transcript_at, last_request_at

        parsed = parse_stream_line(raw_line)
        if not parsed:
            return True

        text, is_final = parsed
        if not is_final:
            return True

        transcript = normalize_transcript(text)
        if not transcript:
            return True

        if transcript in EXIT_WORDS:
            return False

        now = time.monotonic()
        if transcript == last_transcript and (now - last_transcript_at) <= args.dedupe_window:
            eprint(f"adk_transcript_bridge: ignored duplicate transcript: {transcript}")
            return True

        last_transcript = transcript
        last_transcript_at = now

        if args.show_normalized:
            eprint(f"adk_transcript_bridge: transcript -> {transcript}")

        if args.dry_run:
            print(transcript, flush=True)
            return True

        remaining = args.min_interval - (now - last_request_at)
        if remaining > 0:
            eprint(f"adk_transcript_bridge: waiting {remaining:.1f}s to avoid rate limits...")
            await asyncio.sleep(remaining)

        try:
            reply = await invoke_agent(
                runner,
                transcript=transcript,
                user_id=args.user_id,
                session_id=args.session_id,
            )
        except Exception as exc:
            eprint(f"adk_transcript_bridge: ADK request failed: {exc}")
            return True

        last_request_at = time.monotonic()
        print(" ".join(reply.split()), flush=True)
        return True

    if source is None:
        while True:
            try:
                raw_line = input("> ")
            except EOFError:
                break
            if not await handle_line(raw_line):
                break
    else:
        for raw_line in source:
            if not await handle_line(raw_line):
                break

    return 0


if __name__ == "__main__":
    raise SystemExit(asyncio.run(main()))
