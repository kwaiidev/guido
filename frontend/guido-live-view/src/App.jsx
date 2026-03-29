import {
  startTransition,
  useDeferredValue,
  useEffect,
  useRef,
  useState,
} from 'react';


const API_BASE = import.meta.env.VITE_TELEMETRY_BASE ?? 'http://localhost:8765';
const STATE_POLL_MS = 350;


export default function App() {
  const [telemetry, setTelemetry] = useState(null);
  const [mapPayload, setMapPayload] = useState({ version: 0, map: null });
  const [error, setError] = useState('');

  const deferredTelemetry = useDeferredValue(telemetry);

  useEffect(() => {
    let active = true;

    async function pullState() {
      try {
        const response = await fetch(`${API_BASE}/api/state`, { cache: 'no-store' });
        if (!response.ok) {
          throw new Error(`Telemetry request failed with ${response.status}`);
        }
        const data = await response.json();
        if (!active) {
          return;
        }
        startTransition(() => {
          setTelemetry(data);
          setError('');
        });
      } catch (fetchError) {
        if (!active) {
          return;
        }
        setError(fetchError instanceof Error ? fetchError.message : 'Unable to reach telemetry bridge.');
      }
    }

    pullState();
    const intervalId = window.setInterval(pullState, STATE_POLL_MS);
    return () => {
      active = false;
      window.clearInterval(intervalId);
    };
  }, []);

  useEffect(() => {
    if (!telemetry?.map?.version || telemetry.map.version === mapPayload.version) {
      return;
    }

    let active = true;

    async function pullMap() {
      try {
        const response = await fetch(`${API_BASE}/api/map`, { cache: 'no-store' });
        if (!response.ok) {
          throw new Error(`Map request failed with ${response.status}`);
        }
        const data = await response.json();
        if (!active) {
          return;
        }
        startTransition(() => {
          setMapPayload(data);
        });
      } catch (fetchError) {
        if (!active) {
          return;
        }
        setError(fetchError instanceof Error ? fetchError.message : 'Unable to fetch map.');
      }
    }

    pullMap();
    return () => {
      active = false;
    };
  }, [mapPayload.version, telemetry?.map?.version]);

  const connectionState = deriveConnectionState(deferredTelemetry, error);
  const topics = Object.entries(deferredTelemetry?.freshness ?? {});
  const robot = deferredTelemetry?.robot;
  const path = deferredTelemetry?.path?.points ?? [];
  const waypoints = deferredTelemetry?.waypoints ?? [];
  const statusText = deferredTelemetry?.status?.text ?? 'Waiting for /auto_nav/status';

  return (
    <main className="min-h-svh px-4 pb-6 pt-5 text-ink md:px-6 lg:px-8">
      <header className="animate-driftIn border-b border-ink/15 pb-4">
        <div className="flex flex-col gap-4 xl:flex-row xl:items-end xl:justify-between">
          <div className="max-w-2xl">
            <p className="font-mono text-[11px] uppercase tracking-[0.35em] text-ink/55">
              Guido Live View
            </p>
            <h1 className="mt-2 max-w-3xl text-4xl font-semibold leading-[0.95] md:text-6xl">
              RViz telemetry, translated into a cleaner operator surface.
            </h1>
          </div>

          <div className="flex flex-wrap items-center gap-3 text-sm">
            <StatusChip label="Bridge" value={connectionState.label} tone={connectionState.tone} />
            <StatusChip
              label="Map"
              value={deferredTelemetry?.map ? `${deferredTelemetry.map.width}×${deferredTelemetry.map.height}` : 'waiting'}
              tone={deferredTelemetry?.map ? 'ready' : 'idle'}
            />
            <StatusChip
              label="Path"
              value={path.length ? `${path.length} pts` : 'none'}
              tone={path.length ? 'ready' : 'idle'}
            />
            <StatusChip
              label="Scan"
              value={deferredTelemetry?.scan?.ranges?.length ? `${deferredTelemetry.scan.ranges.length} rays` : 'none'}
              tone={deferredTelemetry?.scan?.ranges?.length ? 'ready' : 'idle'}
            />
          </div>
        </div>
      </header>

      <section className="mt-6 grid gap-6 xl:grid-cols-[minmax(0,1.7fr)_22rem]">
        <div className="border border-ink/15 bg-white/55 shadow-glow backdrop-blur-sm">
          <div className="flex items-center justify-between border-b border-ink/10 px-4 py-3 md:px-5">
            <div>
              <h2 className="text-lg font-medium">Live map workspace</h2>
              <p className="text-sm text-ink/60">
                Map, robot pose, global path, and saved places in one plane.
              </p>
            </div>
            <p className="font-mono text-xs uppercase tracking-[0.28em] text-ink/45">
              {robot ? `${formatMeters(robot.x)}, ${formatMeters(robot.y)}` : 'No pose'}
            </p>
          </div>
          <MapViewport mapPayload={mapPayload} telemetry={deferredTelemetry} />
        </div>

        <aside className="border border-ink/15 bg-paper/70 shadow-glow backdrop-blur-sm">
          <section className="border-b border-ink/10 px-4 py-4 md:px-5">
            <h2 className="text-lg font-medium">Selected feeds</h2>
            <p className="mt-1 text-sm text-ink/60">
              The rail stays utility-first: freshness, pose, and the last mission signal.
            </p>
          </section>

          <section className="border-b border-ink/10 px-4 py-4 md:px-5">
            <div className="grid grid-cols-2 gap-x-3 gap-y-4 text-sm">
              <Metric label="X" value={robot ? formatMeters(robot.x) : 'n/a'} />
              <Metric label="Y" value={robot ? formatMeters(robot.y) : 'n/a'} />
              <Metric label="Yaw" value={robot ? formatAngle(robot.yaw) : 'n/a'} />
              <Metric label="Mode" value={robot?.source ?? 'idle'} />
              <Metric label="Linear" value={robot ? `${robot.linear.toFixed(2)} m/s` : 'n/a'} />
              <Metric label="Angular" value={robot ? `${robot.angular.toFixed(2)} rad/s` : 'n/a'} />
            </div>
          </section>

          <section className="border-b border-ink/10 px-4 py-4 md:px-5">
            <div className="flex items-center justify-between">
              <h3 className="text-sm font-medium uppercase tracking-[0.24em] text-ink/55">
                Local scan
              </h3>
              <span className="font-mono text-xs text-ink/45">
                {deferredTelemetry?.scan?.frameId ?? 'waiting'}
              </span>
            </div>
            <ScanRadar scan={deferredTelemetry?.scan} />
          </section>

          <section className="border-b border-ink/10 px-4 py-4 md:px-5">
            <h3 className="text-sm font-medium uppercase tracking-[0.24em] text-ink/55">
              Feed freshness
            </h3>
            <div className="mt-4 space-y-3">
              {topics.length ? (
                topics.map(([topicName, freshness]) => (
                  <div key={topicName} className="flex items-center justify-between gap-4 text-sm">
                    <span className="font-mono text-xs text-ink/60">{topicName}</span>
                    <span className={freshness.fresh ? 'text-ink' : 'text-red-700'}>
                      {freshness.ageSec.toFixed(2)}s
                    </span>
                  </div>
                ))
              ) : (
                <p className="text-sm text-ink/55">No topics have been observed yet.</p>
              )}
            </div>
          </section>

          <section className="border-b border-ink/10 px-4 py-4 md:px-5">
            <h3 className="text-sm font-medium uppercase tracking-[0.24em] text-ink/55">
              Mission line
            </h3>
            <p className="mt-3 text-base leading-relaxed text-ink/78">{statusText}</p>
          </section>

          <section className="px-4 py-4 md:px-5">
            <div className="flex items-center justify-between">
              <h3 className="text-sm font-medium uppercase tracking-[0.24em] text-ink/55">
                Saved places
              </h3>
              <span className="font-mono text-xs text-ink/45">{waypoints.length}</span>
            </div>
            <div className="mt-4 space-y-3">
              {waypoints.length ? (
                waypoints.slice(0, 8).map((waypoint) => (
                  <div key={waypoint.name} className="flex items-center justify-between gap-4 text-sm">
                    <span>{waypoint.name}</span>
                    <span className="font-mono text-xs text-ink/55">
                      {formatMeters(waypoint.x)}, {formatMeters(waypoint.y)}
                    </span>
                  </div>
                ))
              ) : (
                <p className="text-sm text-ink/55">No saved waypoints were found.</p>
              )}
            </div>
          </section>
        </aside>
      </section>

      <footer className="mt-6 border-t border-ink/15 pt-4">
        <div className="grid gap-4 text-sm text-ink/62 md:grid-cols-[1.2fr_1fr]">
          <p>
            Endpoint: <span className="font-mono text-ink">{API_BASE}</span>
          </p>
          <p>
            Bridge this app to ROS with <span className="font-mono text-ink">telemetry_bridge</span> and
            keep the frontend pointed at the same host.
          </p>
        </div>
        {error ? <p className="mt-3 text-sm text-red-700">{error}</p> : null}
      </footer>
    </main>
  );
}


function MapViewport({ mapPayload, telemetry }) {
  const canvasRef = useRef(null);
  const containerRef = useRef(null);

  useEffect(() => {
    function drawMap() {
      const canvas = canvasRef.current;
      const container = containerRef.current;
      const map = mapPayload.map;
      if (!canvas || !container || !map) {
        return;
      }

      const rect = container.getBoundingClientRect();
      if (!rect.width || !rect.height) {
        return;
      }

      const dpr = window.devicePixelRatio || 1;
      canvas.width = Math.round(rect.width * dpr);
      canvas.height = Math.round(rect.height * dpr);

      const ctx = canvas.getContext('2d');
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      ctx.clearRect(0, 0, rect.width, rect.height);

      const tempCanvas = document.createElement('canvas');
      tempCanvas.width = map.width;
      tempCanvas.height = map.height;
      const tempContext = tempCanvas.getContext('2d');
      const image = tempContext.createImageData(map.width, map.height);

      for (let index = 0; index < map.data.length; index += 1) {
        const value = map.data[index];
        const column = index % map.width;
        const row = Math.floor(index / map.width);
        const flippedRow = map.height - row - 1;
        const offset = (flippedRow * map.width + column) * 4;

        let tone = 242;
        if (value < 0) {
          tone = 226;
        } else if (value > 55) {
          tone = 34;
        } else {
          tone = Math.round(248 - _mapRange(value, 0, 55, 0, 126));
        }

        image.data[offset] = tone;
        image.data[offset + 1] = tone;
        image.data[offset + 2] = tone;
        image.data[offset + 3] = 255;
      }

      tempContext.putImageData(image, 0, 0);
      ctx.imageSmoothingEnabled = false;
      ctx.drawImage(tempCanvas, 0, 0, rect.width, rect.height);

      ctx.strokeStyle = 'rgba(17, 20, 15, 0.08)';
      ctx.lineWidth = 1;
      const majorStep = Math.max(40, Math.round(rect.width / 10));
      for (let x = 0; x <= rect.width; x += majorStep) {
        ctx.beginPath();
        ctx.moveTo(x, 0);
        ctx.lineTo(x, rect.height);
        ctx.stroke();
      }
      for (let y = 0; y <= rect.height; y += majorStep) {
        ctx.beginPath();
        ctx.moveTo(0, y);
        ctx.lineTo(rect.width, y);
        ctx.stroke();
      }
    }

    drawMap();
    window.addEventListener('resize', drawMap);
    return () => {
      window.removeEventListener('resize', drawMap);
    };
  }, [mapPayload]);

  const map = mapPayload.map;
  const path = telemetry?.path?.points ?? [];
  const waypoints = telemetry?.waypoints ?? [];
  const robot = telemetry?.robot;

  return (
    <div
      ref={containerRef}
      className="relative h-[68svh] min-h-[28rem] overflow-hidden bg-white/50"
    >
      {map ? <canvas ref={canvasRef} className="absolute inset-0 h-full w-full" /> : null}

      {!map ? (
        <div className="flex h-full items-center justify-center">
          <div className="max-w-sm px-6 text-center">
            <p className="font-mono text-xs uppercase tracking-[0.3em] text-ink/40">
              Waiting for /map
            </p>
            <h3 className="mt-3 text-2xl font-medium">The canvas comes alive once the occupancy grid arrives.</h3>
          </div>
        </div>
      ) : null}

      {map ? (
        <svg className="absolute inset-0 h-full w-full" viewBox="0 0 100 100" preserveAspectRatio="none">
          {path.length > 1 ? (
            <polyline
              fill="none"
              stroke="rgba(17,20,15,0.8)"
              strokeWidth="1.2"
              points={path
                .map((point) => worldPointToViewport(point, map))
                .map((point) => `${point.x},${point.y}`)
                .join(' ')}
            />
          ) : null}

          {waypoints.map((waypoint) => {
            const point = worldPointToViewport(waypoint, map);
            return (
              <g key={waypoint.name}>
                <circle cx={point.x} cy={point.y} r="1.2" fill="#96ff62" />
                <text
                  x={point.x + 1.8}
                  y={point.y - 1.8}
                  fill="rgba(17,20,15,0.82)"
                  fontSize="3"
                  fontFamily="IBM Plex Mono, monospace"
                >
                  {waypoint.name}
                </text>
              </g>
            );
          })}

          {robot ? <RobotGlyph robot={robot} map={map} /> : null}
        </svg>
      ) : null}

      <div className="absolute bottom-0 left-0 right-0 flex flex-wrap items-center gap-x-6 gap-y-2 border-t border-ink/10 bg-paper/80 px-4 py-3 text-sm backdrop-blur-sm">
        <span className="font-mono text-xs uppercase tracking-[0.22em] text-ink/45">
          {map ? `${map.width} × ${map.height} @ ${map.resolution.toFixed(2)}m` : 'Map unavailable'}
        </span>
        <span className="text-ink/62">{path.length ? `${path.length} path samples` : 'No global path yet'}</span>
        <span className="text-ink/62">{waypoints.length ? `${waypoints.length} saved places` : 'No saved places yet'}</span>
      </div>
    </div>
  );
}


function RobotGlyph({ robot, map }) {
  const point = worldPointToViewport(robot, map);
  const length = 4.2;
  const heading = robot.yaw - Math.PI / 2;
  const noseX = point.x + Math.cos(heading) * length;
  const noseY = point.y + Math.sin(heading) * length;

  return (
    <g>
      <circle
        cx={point.x}
        cy={point.y}
        r="2.4"
        fill="rgba(150,255,98,0.18)"
        className="origin-center animate-pulseDot"
      />
      <circle cx={point.x} cy={point.y} r="1.4" fill="#11140f" />
      <line
        x1={point.x}
        y1={point.y}
        x2={noseX}
        y2={noseY}
        stroke="#96ff62"
        strokeWidth="1.4"
        strokeLinecap="round"
      />
    </g>
  );
}


function ScanRadar({ scan }) {
  const ranges = scan?.ranges ?? [];
  const rangeMax = scan?.rangeMax ?? 1;

  return (
    <div className="mt-4 overflow-hidden border border-ink/10 bg-white/55">
      <svg viewBox="0 0 100 100" className="aspect-square w-full">
        <circle cx="50" cy="50" r="42" fill="transparent" stroke="rgba(17,20,15,0.08)" />
        <circle cx="50" cy="50" r="28" fill="transparent" stroke="rgba(17,20,15,0.08)" />
        <circle cx="50" cy="50" r="14" fill="transparent" stroke="rgba(17,20,15,0.08)" />
        <line x1="50" y1="8" x2="50" y2="92" stroke="rgba(17,20,15,0.08)" />
        <line x1="8" y1="50" x2="92" y2="50" stroke="rgba(17,20,15,0.08)" />

        {ranges.map((range, index) => {
          const angle = (scan.angleMin ?? 0) + index * (scan.angleIncrement ?? 0);
          const normalized = _clamp(range / Math.max(rangeMax, 0.001), 0, 1);
          const radius = normalized * 38;
          const x = 50 + Math.cos(angle - Math.PI / 2) * radius;
          const y = 50 + Math.sin(angle - Math.PI / 2) * radius;
          return <circle key={`${index}-${range}`} cx={x} cy={y} r="0.75" fill="#96ff62" opacity="0.84" />;
        })}

        <circle cx="50" cy="50" r="2.2" fill="#11140f" />
      </svg>
      <div className="flex items-center justify-between border-t border-ink/10 px-3 py-2 font-mono text-xs uppercase tracking-[0.22em] text-ink/45">
        <span>{ranges.length ? `${ranges.length} samples` : 'No scan'}</span>
        <span>{scan ? `${scan.rangeMax.toFixed(1)}m max` : ''}</span>
      </div>
    </div>
  );
}


function StatusChip({ label, value, tone }) {
  const toneClass =
    tone === 'ready'
      ? 'border-accent/35 text-ink'
      : tone === 'error'
        ? 'border-red-300 text-red-700'
        : 'border-ink/12 text-ink/60';

  return (
    <div className={`border px-3 py-2 ${toneClass}`}>
      <p className="font-mono text-[10px] uppercase tracking-[0.28em]">{label}</p>
      <p className="mt-1 text-sm font-medium">{value}</p>
    </div>
  );
}


function Metric({ label, value }) {
  return (
    <div>
      <p className="font-mono text-[10px] uppercase tracking-[0.24em] text-ink/45">{label}</p>
      <p className="mt-1 text-base">{value}</p>
    </div>
  );
}


function deriveConnectionState(telemetry, error) {
  if (error) {
    return { label: 'offline', tone: 'error' };
  }
  if (!telemetry) {
    return { label: 'booting', tone: 'idle' };
  }

  const freshest = Object.values(telemetry.freshness ?? {}).some((entry) => entry.fresh);
  return freshest
    ? { label: 'live', tone: 'ready' }
    : { label: 'waiting', tone: 'idle' };
}


function worldPointToViewport(point, map) {
  const mapWidth = Math.max(map.width * map.resolution, 0.001);
  const mapHeight = Math.max(map.height * map.resolution, 0.001);
  const x = ((point.x - map.origin.x) / mapWidth) * 100;
  const y = 100 - ((point.y - map.origin.y) / mapHeight) * 100;
  return {
    x: _clamp(x, 0, 100),
    y: _clamp(y, 0, 100),
  };
}


function formatMeters(value) {
  return `${Number(value).toFixed(2)}m`;
}


function formatAngle(value) {
  return `${Number(value).toFixed(2)}rad`;
}


function _clamp(value, lower, upper) {
  return Math.max(lower, Math.min(upper, value));
}


function _mapRange(value, inMin, inMax, outMin, outMax) {
  if (inMax === inMin) {
    return outMin;
  }
  const ratio = (value - inMin) / (inMax - inMin);
  return outMin + (outMax - outMin) * ratio;
}
