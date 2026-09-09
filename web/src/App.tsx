import { useCallback, useEffect, useMemo, useRef, useState } from "react";

import JointSliders from "./components/JointSliders";
import PoseReadout from "./components/PoseReadout";
import RobotViewer, {
  type DisplayMode,
  type EllipsoidMode,
  type GizmoMode,
  type MeshStatus,
  type ViewerObstacle,
} from "./components/RobotViewer";
import { useKinematics } from "./hooks/useKinematics";
import type {
  IkMethod,
  IkOptions,
  IkResult,
  ObstacleWorld,
  PlanResult,
  Robot,
  SimController,
  SimulateOptions,
} from "./kinematics/types";

/** The dynamics panel's mode: off, or one of the C++ controllers. */
type SimMode = "off" | SimController;

/** Joint-space replay speed for a planned path, radians per second. */
const PLAYBACK_SPEED = 1.2;

/** Where freshly added obstacles appear: in front of the arm, fanned out in y. */
const OBSTACLE_SPAWNS: [number, number, number][] = [
  [0.55, 0.0, 0.35],
  [0.45, 0.3, 0.5],
  [0.45, -0.3, 0.5],
];

const BASE_IK_OPTIONS: IkOptions = {
  maxIterations: 60,
  positionTolerance: 1e-4,
  orientationTolerance: 1e-4,
  // Enough damping to stay well-behaved when a drag pulls the target through a
  // singularity or just outside the workspace.
  damping: 0.02,
  maxStep: 0.25,
};

export default function App() {
  const [robotId, setRobotId] = useState("ur5");
  const { robots, arm, jointLimits, presets, error, loading } =
    useKinematics(robotId);

  const [gizmoMode, setGizmoMode] = useState<GizmoMode>("translate");
  const [showJointAxes, setShowJointAxes] = useState(true);
  const [showCollision, setShowCollision] = useState(false);
  const [ellipsoidMode, setEllipsoidMode] = useState<EllipsoidMode>("linear");
  const [displayMode, setDisplayMode] = useState<DisplayMode>("mesh");
  const [meshStatus, setMeshStatus] = useState<MeshStatus>({
    state: "loading",
  });
  const [ikResult, setIkResult] = useState<IkResult | null>(null);
  const [ikMethod, setIkMethod] = useState<IkMethod>("qp");
  const ikOptions = useMemo<IkOptions>(
    () => ({ ...BASE_IK_OPTIONS, method: ikMethod }),
    [ikMethod],
  );

  // Obstacles live in the world frame, so they survive a robot switch.
  const [obstacles, setObstacles] = useState<ViewerObstacle[]>([]);
  const [selectedObstacleId, setSelectedObstacleId] = useState<number | null>(
    null,
  );
  const nextObstacleIdRef = useRef(1);
  const world = useMemo<ObstacleWorld>(
    () => ({
      spheres: obstacles.map(({ center, radius }) => ({ center, radius })),
    }),
    [obstacles],
  );

  // Planner state is owned by the arm that produced it, like the joint angles.
  const [planGoal, setPlanGoal] = useState<{
    owner: Robot;
    angles: number[];
  } | null>(null);
  const [plan, setPlan] = useState<{ owner: Robot; result: PlanResult } | null>(
    null,
  );
  const [playing, setPlaying] = useState(false);

  // Joint angles are owned by whichever arm produced them. Tagging them with that
  // arm means switching robots falls back to the new default instead of rendering
  // a configuration of the wrong length for a frame.
  const [edited, setEdited] = useState<{
    owner: Robot;
    angles: number[];
  } | null>(null);
  const defaults = useMemo(
    () => (arm ? arm.defaultConfiguration() : []),
    [arm],
  );
  const jointAngles = edited && edited.owner === arm ? edited.angles : defaults;

  // Dynamics simulation. While it runs, the displayed configuration is the sim
  // state and every pose input (sliders, presets, the IK gizmo) becomes the
  // reference the controller chases instead of teleporting the arm.
  const [simMode, setSimMode] = useState<SimMode>("off");
  const [simTarget, setSimTarget] = useState<{
    owner: Robot;
    angles: number[];
  } | null>(null);
  const simStateRef = useRef<{
    owner: Robot;
    q: number[];
    qd: number[];
  } | null>(null);
  const simModeRef = useRef(simMode);
  simModeRef.current = simMode;
  const simTargetRef = useRef(simTarget);
  simTargetRef.current = simTarget;
  const trackProgressRef = useRef(0);

  const setJointAngles = useCallback(
    (angles: number[]) => {
      if (!arm) {
        return;
      }
      const mode = simModeRef.current;
      if (mode === "off") {
        setEdited({ owner: arm, angles });
      } else if (mode === "pd") {
        // PD chases the reference — that is the whole demonstration. Write the
        // ref directly too, so a fast drag reaches the very next sim frame
        // instead of waiting one React render.
        const target = { owner: arm, angles };
        simTargetRef.current = target;
        setSimTarget(target);
      } else {
        // Passive, gravity comp, track: pose inputs carry the simulated body
        // itself, the way you would hand-guide a gravity-compensated arm.
        // Velocity clears so it stays (or falls, or gets pulled back) from there.
        simStateRef.current = {
          owner: arm,
          q: [...angles],
          qd: angles.map(() => 0),
        };
        setSimTarget({ owner: arm, angles });
        setEdited({ owner: arm, angles });
      }
    },
    [arm],
  );

  const handleIkResult = useCallback(
    (result: IkResult | null) => setIkResult(result),
    [],
  );
  const handleMeshStatus = useCallback(
    (status: MeshStatus) => setMeshStatus(status),
    [],
  );

  const addObstacle = useCallback(() => {
    const id = nextObstacleIdRef.current;
    nextObstacleIdRef.current += 1;
    const center = OBSTACLE_SPAWNS[(id - 1) % OBSTACLE_SPAWNS.length];
    setObstacles((current) => [...current, { id, center, radius: 0.12 }]);
    setSelectedObstacleId(id);
  }, []);

  const removeSelectedObstacle = useCallback(() => {
    setObstacles((current) => {
      const target = selectedObstacleId ?? current[current.length - 1]?.id;
      return current.filter((obstacle) => obstacle.id !== target);
    });
    setSelectedObstacleId(null);
  }, [selectedObstacleId]);

  const handleObstacleMoved = useCallback(
    (id: number, center: [number, number, number]) => {
      setObstacles((current) =>
        current.map((obstacle) =>
          obstacle.id === id ? { ...obstacle, center } : obstacle,
        ),
      );
    },
    [],
  );

  const setSelectedRadius = useCallback(
    (radius: number) => {
      setObstacles((current) =>
        current.map((obstacle) =>
          obstacle.id === selectedObstacleId
            ? { ...obstacle, radius }
            : obstacle,
        ),
      );
    },
    [selectedObstacleId],
  );

  // The plan runs synchronously in WASM; a few thousand extends take milliseconds.
  const runPlan = useCallback(() => {
    if (!arm || planGoal?.owner !== arm) {
      return;
    }
    const result = arm.plan(jointAngles, planGoal.angles, world, {});
    setPlan({ owner: arm, result });
    // Kinematic replay only when the dynamics is off; under "Track plan" the
    // simulated controller follows the trajectory instead.
    setPlaying(result.status === "success" && simModeRef.current === "off");
    trackProgressRef.current = 0;
  }, [arm, planGoal, jointAngles, world]);

  const nudge = useCallback(() => {
    const state = simStateRef.current;
    if (state) {
      simStateRef.current = {
        ...state,
        qd: state.qd.map((value) => value + (Math.random() - 0.5) * 3),
      };
    }
  }, []);

  // Replays the planned path by interpolating the waypoints at constant
  // joint-space speed; each frame flows through the normal FK pipeline.
  useEffect(() => {
    if (
      !playing ||
      !arm ||
      plan?.owner !== arm ||
      plan.result.status !== "success"
    ) {
      return undefined;
    }
    // Replay the optimized path — it falls back to the densified shortcut when
    // the optimizer's validation failed, so it is always safe to follow.
    const path =
      plan.result.optimizedPath.length > 1
        ? plan.result.optimizedPath
        : plan.result.path;
    const cumulative = [0];
    for (let i = 1; i < path.length; i += 1) {
      cumulative.push(
        cumulative[i - 1] +
          Math.hypot(...path[i].map((value, j) => value - path[i - 1][j])),
      );
    }
    const total = cumulative[cumulative.length - 1];

    let handle = 0;
    const startedAt = performance.now();
    const tick = (now: number) => {
      const travelled = ((now - startedAt) / 1000) * PLAYBACK_SPEED;
      if (travelled >= total || total <= 0) {
        setEdited({ owner: arm, angles: path[path.length - 1] });
        setPlaying(false);
        return;
      }
      let segment = 1;
      while (cumulative[segment] < travelled) {
        segment += 1;
      }
      const span = cumulative[segment] - cumulative[segment - 1];
      const t = span > 0 ? (travelled - cumulative[segment - 1]) / span : 1;
      const angles = path[segment - 1].map(
        (value, j) => value + t * (path[segment][j] - value),
      );
      setEdited({ owner: arm, angles });
      handle = requestAnimationFrame(tick);
    };
    handle = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(handle);
  }, [playing, plan, arm]);

  // Ref mirror so the dynamics loop reads the latest plan without re-arming.
  const planRef = useRef(plan);
  planRef.current = plan;

  // The dynamics loop: one WASM `simulate` call per animation frame, substepped
  // at 1 ms inside C++. The loop owns the state; React only displays it.
  useEffect(() => {
    if (!arm || simMode === "off") {
      return undefined;
    }
    if (!simStateRef.current || simStateRef.current.owner !== arm) {
      simStateRef.current = {
        owner: arm,
        q: [...jointAngles],
        qd: jointAngles.map(() => 0),
      };
    }
    trackProgressRef.current = 0;

    let handle = 0;
    let last = performance.now();
    const tick = (now: number) => {
      handle = requestAnimationFrame(tick);
      const dt = Math.min((now - last) / 1000, 0.05);
      last = now;
      const state = simStateRef.current;
      if (!state || dt <= 0) {
        return;
      }

      const options: SimulateOptions = { controller: simMode, duration: dt };
      if (simMode === "pd") {
        const target = simTargetRef.current;
        options.qRef = target && target.owner === arm ? target.angles : state.q;
      } else if (simMode === "track") {
        const current = planRef.current;
        const path =
          current?.owner === arm && current.result.status === "success"
            ? current.result.optimizedPath
            : null;
        if (path && path.length > 1) {
          // Advance along the waypoints at constant joint-space speed and hand
          // the controller position + velocity feedforward for the segment.
          trackProgressRef.current += PLAYBACK_SPEED * dt;
          let travelled = trackProgressRef.current;
          let segment = 1;
          let length = 0;
          for (; segment < path.length - 1; segment += 1) {
            length = Math.hypot(
              ...path[segment].map((v, j) => v - path[segment - 1][j]),
            );
            if (travelled <= length) {
              break;
            }
            travelled -= length;
          }
          length = Math.hypot(
            ...path[segment].map((v, j) => v - path[segment - 1][j]),
          );
          const t = length > 0 ? Math.min(travelled / length, 1) : 1;
          const from = path[segment - 1];
          const to = path[segment];
          options.qRef = from.map((v, j) => v + t * (to[j] - v));
          const done = segment === path.length - 1 && t >= 1;
          if (!done && length > 0) {
            options.qdRef = to.map(
              (v, j) => ((v - from[j]) / length) * PLAYBACK_SPEED,
            );
          }
        } else {
          options.qRef = state.q;
        }
      }

      const result = arm.simulate(state.q, state.qd, options);
      simStateRef.current = {
        owner: arm,
        q: result.position,
        qd: result.velocity,
      };
      setEdited({ owner: arm, angles: result.position });
    };
    handle = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(handle);
    // jointAngles is only the seed at enable time; the loop owns it afterwards.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [arm, simMode]);

  // FK, manipulability and the ellipsoid are cheap enough to recompute per change.
  const pose = useMemo(
    () =>
      arm && jointAngles.length === arm.dof() ? arm.forward(jointAngles) : null,
    [arm, jointAngles],
  );
  const manipulability = useMemo(
    () =>
      arm && jointAngles.length === arm.dof()
        ? arm.manipulability(jointAngles)
        : 0,
    [arm, jointAngles],
  );
  const ellipsoid = useMemo(() => {
    if (!arm || ellipsoidMode === "off" || jointAngles.length !== arm.dof()) {
      return null;
    }
    const data = arm.manipulabilityEllipsoids(jointAngles)[ellipsoidMode];
    return ellipsoidMode === "linear"
      ? { label: "linear", unit: "m/rad", data }
      : { label: "angular", unit: "rad/rad", data };
  }, [arm, jointAngles, ellipsoidMode]);

  // Only queried while the overlay is on; the viewer polls the same call per frame.
  const contact = useMemo(
    () =>
      arm && showCollision && jointAngles.length === arm.dof()
        ? arm.nearestContact(jointAngles, world)
        : null,
    [arm, jointAngles, showCollision, world],
  );

  const selectedObstacle =
    obstacles.find((obstacle) => obstacle.id === selectedObstacleId) ?? null;

  const plannedPaths = useMemo(
    () =>
      plan?.owner === arm && plan.result.status === "success"
        ? {
            raw: plan.result.rawPath,
            smoothed: plan.result.path,
            optimized: plan.result.optimizedPath,
          }
        : null,
    [arm, plan],
  );

  const planHint = useMemo(() => {
    if (planGoal?.owner !== arm) {
      return "Pose the arm at the desired goal and press Set goal; planning starts from wherever the arm is when you press Plan.";
    }
    if (plan?.owner !== arm) {
      return "Goal stored. Plan path runs RRT-Connect from the current pose, avoiding the spheres.";
    }
    const result = plan.result;
    switch (result.status) {
      case "success":
        return (
          `Path found: raw ${result.rawLength.toFixed(2)} rad (${result.rawPath.length} waypoints) → ` +
          `shortcut ${result.pathLength.toFixed(2)} rad → optimized ${result.optimizedLength.toFixed(2)} rad` +
          `${result.optimizedFeasible ? "" : " (optimizer fell back to the shortcut path)"}; ` +
          `${result.nodes} tree nodes in ${result.iterations} iterations. The bright trace replays.`
        );
      case "start_invalid":
        return "The current pose is in collision or out of limits — move the arm clear and plan again.";
      case "goal_invalid":
        return "The stored goal collides with the obstacles — pose the arm clear and set a new goal.";
      default:
        return `No path found within ${result.iterations} iterations — move or shrink the obstacles.`;
    }
  }, [arm, planGoal, plan]);

  const simHint = useMemo(() => {
    switch (simMode) {
      case "off":
        return "Newton–Euler rigid-body dynamics at 1 ms substeps; pick a controller to hand the arm to physics.";
      case "passive":
        return "No actuation — the arm falls and swings under gravity against light joint friction. Presets and the gizmo pick it up and drop it.";
      case "gravity":
        return "τ = g(q): gravity is cancelled exactly, so the arm floats wherever you carry it — drag the gizmo or jump to a preset, then Nudge it and watch friction bleed the motion off.";
      case "pd":
        return "τ = g(q) + M(q)(Kp·e − Kd·q̇): sliders, presets and the gizmo now set the reference, and the arm chases it dynamically.";
      case "track":
        return "Computed torque: the model cancels the real dynamics and feeds the planned trajectory forward — the plan replays under physics.";
      default:
        return "";
    }
  }, [simMode]);

  const meshStatusHint = useMemo(() => {
    switch (meshStatus.state) {
      case "loading":
        return "Loading the vendor meshes…";
      case "ready":
        return meshStatus.deviation === undefined
          ? "Vendor meshes loaded from the official URDF."
          : `Vendor meshes loaded. The URDF flange agrees with the C++ model to ${(
              meshStatus.deviation * 1000
            ).toFixed(3)} mm.`;
      case "absent":
        return meshStatus.message ?? "No meshes registered for this robot.";
      default:
        return (
          meshStatus.message ?? "Meshes unavailable — showing the schematic."
        );
    }
  }, [meshStatus]);

  if (loading) {
    return (
      <div className="status">Loading the WebAssembly kinematics module…</div>
    );
  }

  if (error || !arm || !pose) {
    return (
      <div className="status status--error">
        <p>Could not load the kinematics module.</p>
        <pre>{error}</pre>
        <p>
          Build it first: <code>scripts/build_wasm.sh</code>
        </p>
      </div>
    );
  }

  return (
    <div className="app">
      <header className="app__header">
        <div className="app__headerRow">
          <a className="app__brand" href="https://cmaybe.github.io/">
            Modern Robotics
          </a>
          <nav className="app__nav" aria-label="Project navigation">
            <a
              href="https://github.com/cmaybe/modern-robotics-wasm"
              target="_blank"
              rel="noreferrer"
            >
              GitHub
            </a>
            <a
              href="https://cmaybe.github.io/notes/modern-robotics"
              target="_blank"
              rel="noreferrer"
            >
              Docs
            </a>
          </nav>
        </div>
      </header>

      <main className="app__body">
        <RobotViewer
          arm={arm}
          jointAngles={jointAngles}
          onJointAnglesChange={setJointAngles}
          onIkResult={handleIkResult}
          gizmoMode={gizmoMode}
          showJointAxes={showJointAxes}
          showCollision={showCollision}
          obstacles={obstacles}
          selectedObstacleId={selectedObstacleId}
          onObstacleMoved={handleObstacleMoved}
          plannedPaths={plannedPaths}
          simulationActive={simMode !== "off"}
          ellipsoidMode={ellipsoidMode}
          displayMode={displayMode}
          onMeshStatus={handleMeshStatus}
          ikOptions={ikOptions}
        />

        <aside className="panel">
          <h1>{arm.label()}</h1>
          <section className="panel__section">
            <h2>Robot</h2>
            <div className="buttons">
              {robots.map((robot) => (
                <button
                  key={robot.id}
                  type="button"
                  className={robot.id === robotId ? "is-active" : ""}
                  onClick={() => setRobotId(robot.id)}
                >
                  {robot.label} · {robot.dof} DOF
                </button>
              ))}
            </div>
          </section>

          <section className="panel__section">
            <h2>Display</h2>
            <div className="buttons">
              <button
                type="button"
                className={displayMode === "mesh" ? "is-active" : ""}
                onClick={() => setDisplayMode("mesh")}
              >
                Meshes
              </button>
              <button
                type="button"
                className={displayMode === "schematic" ? "is-active" : ""}
                onClick={() => setDisplayMode("schematic")}
              >
                Schematic
              </button>
            </div>
            <p className="hint">{meshStatusHint}</p>
            <label className="toggle">
              <input
                type="checkbox"
                checked={showCollision}
                onChange={(event) => setShowCollision(event.target.checked)}
              />
              Collision capsules
            </label>
            {contact && (
              <p className="hint" data-testid="clearance">
                {!Number.isFinite(contact.distance)
                  ? "No contacts to check."
                  : contact.distance <= 0
                    ? `Collision: ${(-contact.distance * 1000).toFixed(1)} mm penetration.`
                    : `Tightest clearance: ${(contact.distance * 1000).toFixed(1)} mm.`}
              </p>
            )}
          </section>

          <section className="panel__section">
            <h2>Joints (FK)</h2>
            <JointSliders
              limits={jointLimits}
              angles={
                simMode === "pd" && simTarget?.owner === arm
                  ? simTarget.angles
                  : jointAngles
              }
              onChange={setJointAngles}
            />
          </section>

          <section className="panel__section">
            <h2>End-effector (IK)</h2>
            <div className="buttons">
              <button
                type="button"
                className={gizmoMode === "translate" ? "is-active" : ""}
                onClick={() => setGizmoMode("translate")}
              >
                Translate
              </button>
              <button
                type="button"
                className={gizmoMode === "rotate" ? "is-active" : ""}
                onClick={() => setGizmoMode("rotate")}
              >
                Rotate
              </button>
            </div>
            <PoseReadout
              pose={pose}
              manipulability={manipulability}
              ellipsoid={ellipsoid}
              ikResult={ikResult}
            />
          </section>

          <section className="panel__section">
            <h2>IK solver</h2>
            <div className="buttons">
              {(
                [
                  ["qp", "Box QP"],
                  ["dls", "DLS + clamp"],
                ] as [IkMethod, string][]
              ).map(([method, label]) => (
                <button
                  key={method}
                  type="button"
                  className={ikMethod === method ? "is-active" : ""}
                  onClick={() => setIkMethod(method)}
                >
                  {label}
                </button>
              ))}
            </div>
            <p className="hint">
              Both minimise the same local model of the task error.{" "}
              <strong>Box QP</strong> treats the joint limits as constraints on
              the step, so a joint that saturates hands its share of the motion
              to the others. <strong>DLS + clamp</strong> solves as if the
              joints were unbounded and truncates afterwards, which loses that
              redistribution.
            </p>
          </section>

          <section className="panel__section">
            <h2>Manipulability ellipsoid</h2>
            <div className="buttons">
              {(
                [
                  ["linear", "Linear"],
                  ["angular", "Angular"],
                  ["off", "Off"],
                ] as [EllipsoidMode, string][]
              ).map(([mode, label]) => (
                <button
                  key={mode}
                  type="button"
                  className={ellipsoidMode === mode ? "is-active" : ""}
                  onClick={() => setEllipsoidMode(mode)}
                >
                  {label}
                </button>
              ))}
            </div>
            <p className="hint">
              The image of the unit ball of joint velocities, drawn at the
              end-effector. A long axis is a direction the tool moves easily; a
              flat one is a direction it barely moves at all.
            </p>
          </section>

          <section className="panel__section">
            <h2>Motion planning</h2>
            <div className="buttons">
              <button type="button" onClick={addObstacle}>
                Add obstacle
              </button>
              <button
                type="button"
                onClick={removeSelectedObstacle}
                disabled={obstacles.length === 0}
              >
                Remove
              </button>
            </div>
            {obstacles.length > 0 && (
              <div className="buttons">
                {obstacles.map((obstacle, index) => (
                  <button
                    key={obstacle.id}
                    type="button"
                    className={
                      obstacle.id === selectedObstacleId ? "is-active" : ""
                    }
                    onClick={() =>
                      setSelectedObstacleId(
                        obstacle.id === selectedObstacleId ? null : obstacle.id,
                      )
                    }
                  >
                    #{index + 1} · r {(obstacle.radius * 100).toFixed(0)} cm
                  </button>
                ))}
              </div>
            )}
            {selectedObstacle && (
              <label className="slider">
                <span className="slider__label">
                  <span className="slider__name">obstacle radius</span>
                  <span className="slider__value">
                    {(selectedObstacle.radius * 100).toFixed(0)} cm
                  </span>
                </span>
                <input
                  type="range"
                  min={0.05}
                  max={0.3}
                  step={0.01}
                  value={selectedObstacle.radius}
                  onChange={(event) =>
                    setSelectedRadius(Number(event.target.value))
                  }
                />
              </label>
            )}
            <div className="buttons">
              <button
                type="button"
                onClick={() => setPlanGoal({ owner: arm, angles: jointAngles })}
              >
                Set goal = current
              </button>
              <button
                type="button"
                onClick={runPlan}
                disabled={planGoal?.owner !== arm}
              >
                Plan path
              </button>
              {plan?.owner === arm && plan.result.status === "success" && (
                <button
                  type="button"
                  onClick={() => setPlaying((value) => !value)}
                >
                  {playing ? "Stop" : "Replay"}
                </button>
              )}
            </div>
            <p className="hint" data-testid="plan-status">
              {planHint}
            </p>
          </section>

          <section className="panel__section">
            <h2>Dynamics</h2>
            <div className="buttons">
              {(
                [
                  ["off", "Off"],
                  ["passive", "Passive"],
                  ["gravity", "Gravity comp"],
                  ["pd", "PD hold"],
                  ["track", "Track plan"],
                ] as [SimMode, string][]
              ).map(([mode, label]) => (
                <button
                  key={mode}
                  type="button"
                  className={simMode === mode ? "is-active" : ""}
                  disabled={
                    mode === "track" &&
                    !(plan?.owner === arm && plan.result.status === "success")
                  }
                  onClick={() => {
                    trackProgressRef.current = 0;
                    setSimMode(mode);
                  }}
                >
                  {label}
                </button>
              ))}
              <button
                type="button"
                onClick={nudge}
                disabled={simMode === "off"}
              >
                Nudge
              </button>
            </div>
            <p className="hint" data-testid="sim-status">
              {simHint}
            </p>
          </section>

          <section className="panel__section">
            <h2>Presets</h2>
            <div className="buttons">
              {presets.map((preset) => (
                <button
                  key={preset.label}
                  type="button"
                  onClick={() => setJointAngles(preset.angles)}
                >
                  {preset.label}
                </button>
              ))}
            </div>
            <label className="toggle">
              <input
                type="checkbox"
                checked={showJointAxes}
                onChange={(event) => setShowJointAxes(event.target.checked)}
              />
              Show joint rotation axes
            </label>
          </section>
        </aside>
      </main>
    </div>
  );
}
