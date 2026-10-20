"use client";

import { useEffect, useMemo, useRef, useState } from "react";

type Pose = {
  x: number;
  y: number;
  heading: number;
};

type SegmentKind = "move" | "turn" | "action" | "system";
type Station = "raw" | "process" | "storage" | "start" | null;

type Segment = {
  id: number;
  label: string;
  detail: string;
  kind: SegmentKind;
  station: Station;
  round: 0 | 1 | 2;
  start: Pose;
  end: Pose;
  duration: number;
  startAt: number;
  endAt: number;
};

const FIELD = 2400;
const START: Pose = { x: 2215, y: 2250, heading: 0 };
const ARM_OFFSET = 225;

const stationMeta = {
  raw: { name: "原料区", color: "#f29f67" },
  process: { name: "粗加工区", color: "#70d6c6" },
  storage: { name: "暂存区", color: "#b898f4" },
  start: { name: "启停区 1", color: "#8bb9ff" },
} as const;

function buildRoute(startLeg: number) {
  const parts: Omit<Segment, "id" | "startAt" | "endAt">[] = [];
  let pose = { ...START };

  const addMove = (
    x: number,
    y: number,
    label: string,
    detail: string,
    round: 0 | 1 | 2 = 0,
    station: Station = null,
    precise = false,
  ) => {
    const next = { ...pose, x, y };
    const distance = Math.hypot(next.x - pose.x, next.y - pose.y);
    parts.push({
      label,
      detail,
      kind: "move",
      station,
      round,
      start: { ...pose },
      end: next,
      duration: Math.max(520, distance * (precise ? 2.55 : 1.15)),
    });
    pose = next;
  };

  const addTurn = (
    heading: number,
    label: string,
    detail: string,
    round: 1 | 2,
    station: Station,
  ) => {
    const next = { ...pose, heading };
    parts.push({
      label,
      detail,
      kind: "turn",
      station,
      round,
      start: { ...pose },
      end: next,
      duration: Math.abs(heading - pose.heading) * 9 + 420,
    });
    pose = next;
  };

  const addPause = (
    label: string,
    detail: string,
    round: 0 | 1 | 2,
    station: Station,
    kind: SegmentKind = "action",
    duration = 1150,
  ) => {
    parts.push({
      label,
      detail,
      kind,
      station,
      round,
      start: { ...pose },
      end: { ...pose },
      duration,
    });
  };

  const qrY = START.y - startLeg;
  const rawApproachDistance = 1950 - qrY;

  addMove(
    START.x,
    qrY,
    "起步直达中心横线",
    `3、4 侧朝 −Y，一次走完 ${startLeg} mm；二维码区不扫码、不停车`,
  );
  addMove(
    1200,
    qrY,
    "横移到场地中线",
    "向 1、3 侧平移 1015 mm，车体中心 X=1200",
  );
  addMove(
    1200,
    1950,
    "第 1 轮 · 原料区前",
    `自动补偿首段标定：本段 ${rawApproachDistance} mm`,
    1,
    "raw",
  );
  addTurn(90, "原料区转向", "逆时针 90°，2、4 侧朝北", 1, "raw");
  addMove(
    1200,
    2100,
    "低速进入原料区",
    "最后 150 mm 精准进入；机械臂中心对准 (1200,2325)",
    1,
    "raw",
    true,
  );
  addPause("原料区动作", "机械臂中心落在三圆中的中间圆", 1, "raw");

  addMove(
    1200,
    450,
    "移向粗加工区前",
    "沿中心线移动 1650 mm，保留 150 mm 低速进入距离",
    1,
    "process",
  );
  addTurn(270, "粗加工区转向", "逆时针 180°，2、4 侧朝南", 1, "process");
  addMove(
    1200,
    300,
    "低速进入粗加工区",
    "车体中心 (1200,300)，机械臂中心 (1200,75)",
    1,
    "process",
    true,
  );
  addPause("粗加工区动作", "机械臂中心落在三圆中的中间圆", 1, "process");

  addMove(1200, 1200, "返回场地中心", "沿中心线回到 Y=1200", 1, null);
  addMove(
    450,
    1200,
    "移向暂存区前",
    "沿水平中心线移动，预留 150 mm",
    1,
    "storage",
  );
  addTurn(180, "暂存区转向", "顺时针 90°，2、4 侧朝西", 1, "storage");
  addMove(
    300,
    1200,
    "低速进入暂存区",
    "车体中心 (300,1200)，机械臂中心 (75,1200)",
    1,
    "storage",
    true,
  );
  addPause("暂存区动作", "第 1 轮完成", 1, "storage");

  addMove(1200, 1200, "第 2 轮 · 返回中心", "保持 2、4 侧朝西", 2, null);
  addMove(1200, 1950, "第 2 轮 · 原料区前", "沿中心线北移 750 mm", 2, "raw");
  addTurn(90, "原料区转向", "顺时针 90°，2、4 侧朝北", 2, "raw");
  addMove(
    1200,
    2100,
    "低速进入原料区",
    "机械臂中心再次对准 (1200,2325)",
    2,
    "raw",
    true,
  );
  addPause("原料区动作", "第 2 轮原料动作", 2, "raw");

  addMove(1200, 450, "移向粗加工区前", "中心线长距离移动 1650 mm", 2, "process");
  addTurn(270, "粗加工区转向", "逆时针 180°，2、4 侧朝南", 2, "process");
  addMove(
    1200,
    300,
    "低速进入粗加工区",
    "机械臂中心再次对准 (1200,75)",
    2,
    "process",
    true,
  );
  addPause("粗加工区动作", "第 2 轮粗加工动作", 2, "process");

  addMove(1200, 1200, "返回场地中心", "沿中心线回到 Y=1200", 2, null);
  addMove(450, 1200, "移向暂存区前", "水平移动 750 mm", 2, "storage");
  addTurn(180, "暂存区转向", "顺时针 90°，2、4 侧朝西", 2, "storage");
  addMove(
    300,
    1200,
    "低速进入暂存区",
    "机械臂中心再次对准 (75,1200)",
    2,
    "storage",
    true,
  );
  addPause("暂存区动作", "两轮任务完成", 2, "storage");

  addMove(1200, 1200, "回程 · 返回中心", "不再增加原地转圈", 0, null);
  addMove(2150, 1200, "进入右侧回程通道", "车体中心 X=2150", 0, "start");
  addMove(2150, 2150, "上移至入库转接点", "保持 2、4 侧朝西，不转向", 0, "start");
  addPause(
    "切换精准低速",
    "最后 165 mm 使用更低速度与更小加速度",
    0,
    "start",
    "system",
    520,
  );
  addMove(2150, 2250, "平移到启停区行", "向 3、4 侧平移 100 mm", 0, "start", true);
  addMove(2215, 2250, "直接平移入库", "向 1、3 侧平移 65 mm；不原地掉头", 0, "start", true);
  addPause("最终对齐", "车体完整落入 300×300 启停区 1", 0, "start", "action", 1350);
  addPause("机械臂归零", "底盘保持静止，M5 从 −90° 回到 0°", 0, "start", "system", 1200);
  addPause("任务完成", "最终朝向：2、4 侧朝西（180°）", 0, "start", "system", 900);

  let elapsed = 0;
  return parts.map((part, index) => {
    const segment: Segment = {
      ...part,
      id: index + 1,
      startAt: elapsed,
      endAt: elapsed + part.duration,
    };
    elapsed = segment.endAt;
    return segment;
  });
}

function mix(a: number, b: number, t: number) {
  return a + (b - a) * t;
}

function ease(t: number) {
  return t * t * (3 - 2 * t);
}

function formatTime(milliseconds: number) {
  const seconds = Math.max(0, milliseconds) / 1000;
  return `${seconds.toFixed(1)} s`;
}

function stationName(station: Station) {
  return station ? stationMeta[station].name : "场内移动";
}

export default function Home() {
  const [startLeg, setStartLeg] = useState(1050);
  const route = useMemo(() => buildRoute(startLeg), [startLeg]);
  const totalTime = route.at(-1)?.endAt ?? 1;
  const [time, setTime] = useState(0);
  const [playing, setPlaying] = useState(false);
  const [speed, setSpeed] = useState(1.5);
  const frameRef = useRef<number | null>(null);
  const previousRef = useRef<number | null>(null);

  useEffect(() => {
    if (!playing) {
      previousRef.current = null;
      if (frameRef.current !== null) cancelAnimationFrame(frameRef.current);
      return;
    }

    const tick = (now: number) => {
      if (previousRef.current === null) previousRef.current = now;
      const delta = now - previousRef.current;
      previousRef.current = now;
      setTime((current) => {
        const next = Math.min(totalTime, current + delta * speed);
        if (next >= totalTime) setPlaying(false);
        return next;
      });
      frameRef.current = requestAnimationFrame(tick);
    };

    frameRef.current = requestAnimationFrame(tick);
    return () => {
      if (frameRef.current !== null) cancelAnimationFrame(frameRef.current);
    };
  }, [playing, speed, totalTime]);

  const activeIndex =
    time >= totalTime
      ? route.length - 1
      : Math.max(0, route.findIndex((segment) => time < segment.endAt));
  const active = route[activeIndex];
  const segmentProgress =
    active.duration > 0
      ? Math.min(1, Math.max(0, (time - active.startAt) / active.duration))
      : 1;
  const easedProgress = ease(segmentProgress);
  const pose: Pose = {
    x: mix(active.start.x, active.end.x, easedProgress),
    y: mix(active.start.y, active.end.y, easedProgress),
    heading: mix(active.start.heading, active.end.heading, easedProgress),
  };
  const normalizedHeading = ((pose.heading % 360) + 360) % 360;
  const armX = pose.x + Math.cos((pose.heading * Math.PI) / 180) * ARM_OFFSET;
  const armY = pose.y + Math.sin((pose.heading * Math.PI) / 180) * ARM_OFFSET;
  const firstSegment = route[0];
  const armHomeSegment = route.at(-2)!;
  let armBaseAngle = -90;
  if (time <= firstSegment.endAt) {
    armBaseAngle = mix(0, -90, Math.min(1, time / (firstSegment.duration * 0.72)));
  } else if (time >= armHomeSegment.startAt) {
    armBaseAngle = mix(
      -90,
      0,
      Math.min(1, Math.max(0, (time - armHomeSegment.startAt) / armHomeSegment.duration)),
    );
  }

  const jumpSegment = (direction: -1 | 1) => {
    const target = Math.min(route.length - 1, Math.max(0, activeIndex + direction));
    setPlaying(false);
    setTime(route[target].startAt + 1);
  };

  const restart = () => {
    setPlaying(false);
    setTime(0);
    requestAnimationFrame(() => setPlaying(true));
  };

  const completedMoves = route.filter(
    (segment) => segment.kind === "move" && time >= segment.endAt,
  );
  const activeMove = active.kind === "move" ? active : null;

  return (
    <main className="appShell">
      <header className="topbar">
        <div className="brandBlock">
          <div className="brandMark" aria-hidden="true">
            <span />
            <span />
          </div>
          <div>
            <p className="eyebrow">TMCode 1 · Route visualizer</p>
            <h1>麦轮小车两轮路线动态仿真</h1>
          </div>
        </div>
        <div className="statusCluster">
          <span className={`liveDot ${playing ? "isPlaying" : ""}`} />
          <div>
            <small>仿真状态</small>
            <strong>{playing ? "运行中" : time >= totalTime ? "已完成" : "已暂停"}</strong>
          </div>
        </div>
      </header>

      <section className="heroStrip">
        <div>
          <p className="eyebrow">严格按当前 tmcode1.cpp 坐标</p>
          <h2>看清每一次平移、转向与机械臂圆心落点。</h2>
        </div>
        <div className="heroFacts">
          <div>
            <strong>2400 × 2400</strong>
            <span>场地 / mm</span>
          </div>
          <div>
            <strong>230 × 300</strong>
            <span>车体外廓 / mm</span>
          </div>
          <div>
            <strong>225</strong>
            <span>机械臂偏移 / mm</span>
          </div>
        </div>
      </section>

      <section className="workspace">
        <div className="simPanel">
          <div className="panelHead">
            <div>
              <p className="panelKicker">TOP VIEW · 左下角为 (0,0)</p>
              <h3>场地实时俯视图</h3>
            </div>
            <div className="legend">
              <span><i className="legendCar" />车体长方体</span>
              <span><i className="legendArm" />机械臂中心</span>
              <span><i className="legendTarget" />目标中间圆</span>
            </div>
          </div>

          <div className="fieldFrame">
            <div className="axisLabel axisY">Y / mm</div>
            <div className="axisLabel axisX">X / mm</div>
            <div className="field">
              <div className="centerLane laneVertical" />
              <div className="centerLane laneHorizontal" />

              {[
                { x: 550, y: 1400 },
                { x: 1400, y: 1400 },
                { x: 550, y: 550 },
                { x: 1400, y: 550 },
              ].map((block, index) => (
                <div
                  className="obstacle"
                  key={index}
                  style={{
                    left: `${(block.x / FIELD) * 100}%`,
                    bottom: `${(block.y / FIELD) * 100}%`,
                  }}
                >
                  <span>障碍区</span>
                </div>
              ))}

              <div className="startZone">
                <span>启停区 1</span>
              </div>

              <TargetGroup station="raw" />
              <TargetGroup station="process" />
              <TargetGroup station="storage" />

              <div className="centerCross">
                <span>1200</span>
              </div>

              <svg className="routeLayer" viewBox="0 0 2400 2400" aria-hidden="true">
                {route
                  .filter((segment) => segment.kind === "move")
                  .map((segment) => (
                    <line
                      key={`ghost-${segment.id}`}
                      className="routeGhost"
                      x1={segment.start.x}
                      y1={FIELD - segment.start.y}
                      x2={segment.end.x}
                      y2={FIELD - segment.end.y}
                    />
                  ))}
                {completedMoves.map((segment) => (
                  <line
                    key={`done-${segment.id}`}
                    className="routeDone"
                    x1={segment.start.x}
                    y1={FIELD - segment.start.y}
                    x2={segment.end.x}
                    y2={FIELD - segment.end.y}
                  />
                ))}
                {activeMove && (
                  <line
                    className="routeActive"
                    x1={activeMove.start.x}
                    y1={FIELD - activeMove.start.y}
                    x2={pose.x}
                    y2={FIELD - pose.y}
                  />
                )}
              </svg>

              <div
                className="vehicle"
                data-testid="vehicle"
                style={{
                  left: `${(pose.x / FIELD) * 100}%`,
                  top: `${((FIELD - pose.y) / FIELD) * 100}%`,
                  transform: `translate(-50%, -50%) rotate(${-pose.heading}deg)`,
                }}
              >
                <div className="armReach">
                  <span className="armDot" />
                  <em>臂心</em>
                </div>
                <div className="vehicleBody">
                  <span className="side side24">2·4</span>
                  <span className="side side13">1·3</span>
                  <span className="side side34">3·4 车头</span>
                  <i className="wheel wheel1" />
                  <i className="wheel wheel2" />
                  <i className="wheel wheel3" />
                  <i className="wheel wheel4" />
                  <div className="headingArrow" />
                </div>
              </div>

              <div
                className="armCoordinate"
                style={{
                  left: `${(armX / FIELD) * 100}%`,
                  top: `${((FIELD - armY) / FIELD) * 100}%`,
                }}
              >
                ({Math.round(armX)}, {Math.round(armY)})
              </div>

              <div className="originMarker">
                <i />
                <span>(0,0)</span>
              </div>
              <div className="tick tickX0">0</div>
              <div className="tick tickX12">1200</div>
              <div className="tick tickX24">2400</div>
              <div className="tick tickY12">1200</div>
              <div className="tick tickY24">2400</div>
            </div>
          </div>

          <div className="transport">
            <div className="mainControls">
              <button
                className="primaryControl"
                onClick={() => {
                  if (time >= totalTime) setTime(0);
                  setPlaying((value) => !value);
                }}
                aria-label={playing ? "暂停仿真" : "播放仿真"}
              >
                <span aria-hidden="true">{playing ? "Ⅱ" : "▶"}</span>
                {playing ? "暂停" : "播放"}
              </button>
              <button onClick={restart}>↻ 重新播放</button>
              <button onClick={() => jumpSegment(-1)} aria-label="上一步">←</button>
              <button onClick={() => jumpSegment(1)} aria-label="下一步">→</button>
            </div>

            <div className="timeline">
              <input
                aria-label="仿真时间轴"
                type="range"
                min={0}
                max={totalTime}
                step={10}
                value={Math.min(time, totalTime)}
                onChange={(event) => {
                  setPlaying(false);
                  setTime(Number(event.target.value));
                }}
                style={{ "--progress": `${(time / totalTime) * 100}%` } as React.CSSProperties}
              />
              <div className="timelineMeta">
                <span>{formatTime(time)}</span>
                <span>步骤 {activeIndex + 1} / {route.length}</span>
                <span>{formatTime(totalTime)}</span>
              </div>
            </div>

            <label className="speedControl">
              <span>速度</span>
              <select value={speed} onChange={(event) => setSpeed(Number(event.target.value))}>
                <option value={0.5}>0.5×</option>
                <option value={1}>1×</option>
                <option value={1.5}>1.5×</option>
                <option value={2}>2×</option>
                <option value={3}>3×</option>
              </select>
            </label>
          </div>
        </div>

        <aside className="sidePanel">
          <div className="nowCard">
            <div className="nowTopline">
              <span className={`kindTag kind-${active.kind}`}>
                {active.kind === "move"
                  ? "平移"
                  : active.kind === "turn"
                    ? "转向"
                    : active.kind === "action"
                      ? "工位"
                      : "系统"}
              </span>
              <span>{active.round ? `第 ${active.round} 轮` : stationName(active.station)}</span>
            </div>
            <p>当前动作</p>
            <h3>{active.label}</h3>
            <div className="progressTrack">
              <span style={{ width: `${segmentProgress * 100}%` }} />
            </div>
            <p className="activeDetail">{active.detail}</p>
          </div>

          <div className="telemetryGrid">
            <div>
              <span>车体中心 X</span>
              <strong>{Math.round(pose.x)} <small>mm</small></strong>
            </div>
            <div>
              <span>车体中心 Y</span>
              <strong>{Math.round(pose.y)} <small>mm</small></strong>
            </div>
            <div>
              <span>2、4 侧朝向</span>
              <strong>{Math.round(normalizedHeading)}°</strong>
            </div>
            <div>
              <span>M5 底座角</span>
              <strong>{Math.round(armBaseAngle)}°</strong>
            </div>
          </div>

          <div className="calibrationCard">
            <div className="cardHeading">
              <div>
                <p className="panelKicker">MANUAL CALIBRATION</p>
                <h3>起步距离手调</h3>
              </div>
              <output>{startLeg} mm</output>
            </div>
            <input
              aria-label="第一段起步距离"
              type="range"
              min={1000}
              max={1100}
              step={5}
              value={startLeg}
              onChange={(event) => {
                setPlaying(false);
                setTime(0);
                setStartLeg(Number(event.target.value));
              }}
              style={{ "--progress": `${startLeg - 1000}%` } as React.CSSProperties}
            />
            <div className="calibrationScale">
              <span>1000</span>
              <span>当前代码 1050</span>
              <span>1100</span>
            </div>
            <p>
              修改后只改变第一落点 Y；到原料区前的下一段会自动补偿，三个工位坐标保持不变。
            </p>
          </div>

          <div className="routeList">
            <div className="cardHeading">
              <div>
                <p className="panelKicker">ROUTE CHECKPOINTS</p>
                <h3>关键坐标</h3>
              </div>
              <span className="verified">已核对</span>
            </div>
            <ol>
              {[
                ["起点 / 终点", "(2215, 2250)"],
                ["首段落点", `(2215, ${2250 - startLeg})`],
                ["场地中心", "(1200, 1200)"],
                ["原料区车心", "(1200, 2100)"],
                ["粗加工区车心", "(1200, 300)"],
                ["暂存区车心", "(300, 1200)"],
                ["回程转接点", "(2150, 2150)"],
              ].map(([label, value], index) => (
                <li key={label} className={index === 0 ? "importantPoint" : ""}>
                  <span>{label}</span>
                  <strong>{value}</strong>
                </li>
              ))}
            </ol>
          </div>

          <div className="safetyNote">
            <span className="safetyIcon">✓</span>
            <div>
              <strong>最终入库不转圈</strong>
              <p>最后先向北平移 100 mm，再向东平移 65 mm，保持 2、4 侧朝西。</p>
            </div>
          </div>
        </aside>
      </section>

      <footer>
        <p>仿真模型：场地原点位于左下角；逆时针为正；坐标单位均为 mm。</p>
        <p>数据源：true_example/tmcode1.cpp</p>
      </footer>
    </main>
  );
}

function TargetGroup({ station }: { station: "raw" | "process" | "storage" }) {
  const isVertical = station === "storage";
  const positions =
    station === "raw"
      ? [
          { x: 1120, y: 2325 },
          { x: 1200, y: 2325 },
          { x: 1280, y: 2325 },
        ]
      : station === "process"
        ? [
            { x: 1120, y: 75 },
            { x: 1200, y: 75 },
            { x: 1280, y: 75 },
          ]
        : [
            { x: 75, y: 1120 },
            { x: 75, y: 1200 },
            { x: 75, y: 1280 },
          ];

  return (
    <div
      className={`targetGroup target-${station} ${isVertical ? "isVertical" : ""}`}
      aria-label={`${stationMeta[station].name}三个圆，中间圆为目标`}
    >
      <span className="targetLabel">{stationMeta[station].name}</span>
      {positions.map((position, index) => (
        <i
          key={`${position.x}-${position.y}`}
          className={index === 1 ? "targetRing isMiddle" : "targetRing"}
          style={{
            left: `${(position.x / FIELD) * 100}%`,
            top: `${((FIELD - position.y) / FIELD) * 100}%`,
          }}
        />
      ))}
    </div>
  );
}
