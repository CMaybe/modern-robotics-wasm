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
} from "./kinematics/types";

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
  const { robots, arm, jointLimits, presets, error, loading } = useKinematics(robotId);

  const [gizmoMode, setGizmoMode] = useState<GizmoMode>("translate");
  const [showJointAxes, setShowJointAxes] = useState(true);
  const [showCollision, setShowCollision] = useState(false);
  const [ellipsoidMode, setEllipsoidMode] = useState<EllipsoidMode>("linear");
  const [displayMode, setDisplayMode] = useState<DisplayMode>("mesh");
  const [meshStatus, setMeshStatus] = useState<MeshStatus>({ state: "loading" });
  const [ikResult, setIkResult] = useState<IkResult | null>(null);
  const [ikMethod, setIkMethod] = useState<IkMethod>("qp");
  const ikOptions = useMemo<IkOptions>(() => ({ ...BASE_IK_OPTIONS, method: ikMethod }), [ikMethod]);

  // Obstacles live in the world frame, so they survive a robot switch.
  const [obstacles, setObstacles] = useState<ViewerObstacle[]>([]);
  const [selectedObstacleId, setSelectedObstacleId] = useState<number | null>(null);
  const nextObstacleIdRef = useRef(1);
  const world = useMemo<ObstacleWorld>(
    () => ({ spheres: obstacles.map(({ center, radius }) => ({ center, radius })) }),
    [obstacles],
  );

  // Planner state is owned by the arm that produced it, like the joint angles.
  const [planGoal, setPlanGoal] = useState<{ owner: Robot; angles: number[] } | null>(null);
  const [plan, setPlan] = useState<{ owner: Robot; result: PlanResult } | null>(null);
  const [playing, setPlaying] = useState(false);

  // Joint angles are owned by whichever arm produced them. Tagging them with that
  // arm means switching robots falls back to the new default instead of rendering
  // a configuration of the wrong length for a frame.
  const [edited, setEdited] = useState<{ owner: Robot; angles: number[] } | null>(null);
  const defaults = useMemo(() => (arm ? arm.defaultConfiguration() : []), [arm]);
  const jointAngles = edited && edited.owner === arm ? edited.angles : defaults;

  const setJointAngles = useCallback(
    (angles: number[]) => {
      if (arm) {
        setEdited({ owner: arm, angles });
      }
    },
    [arm],
  );

  const handleIkResult = useCallback((result: IkResult | null) => setIkResult(result), []);
  const handleMeshStatus = useCallback((status: MeshStatus) => setMeshStatus(status), []);

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

  const handleObstacleMoved = useCallback((id: number, center: [number, number, number]) => {
    setObstacles((current) =>
      current.map((obstacle) => (obstacle.id === id ? { ...obstacle, center } : obstacle)),
    );
  }, []);

  const setSelectedRadius = useCallback(
    (radius: number) => {
      setObstacles((current) =>
        current.map((obstacle) =>
          obstacle.id === selectedObstacleId ? { ...obstacle, radius } : obstacle,
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
    setPlaying(result.status === "success");
  }, [arm, planGoal, jointAngles, world]);

  // Replays the planned path by interpolating the waypoints at constant
  // joint-space speed; each frame flows through the normal FK pipeline.
  useEffect(() => {
    if (!playing || !arm || plan?.owner !== arm || plan.result.status !== "success") {
      return undefined;
    }
    const path = plan.result.path;
    const cumulative = [0];
    for (let i = 1; i < path.length; i += 1) {
      cumulative.push(
        cumulative[i - 1] + Math.hypot(...path[i].map((value, j) => value - path[i - 1][j])),
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
      const angles = path[segment - 1].map((value, j) => value + t * (path[segment][j] - value));
      setEdited({ owner: arm, angles });
      handle = requestAnimationFrame(tick);
    };
    handle = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(handle);
  }, [playing, plan, arm]);

  // FK, manipulability and the ellipsoid are cheap enough to recompute per change.
  const pose = useMemo(
    () => (arm && jointAngles.length === arm.dof() ? arm.forward(jointAngles) : null),
    [arm, jointAngles],
  );
  const manipulability = useMemo(
    () => (arm && jointAngles.length === arm.dof() ? arm.manipulability(jointAngles) : 0),
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

  const selectedObstacle = obstacles.find((obstacle) => obstacle.id === selectedObstacleId) ?? null;

  const plannedPaths = useMemo(
    () =>
      plan?.owner === arm && plan.result.status === "success"
        ? { smoothed: plan.result.path, raw: plan.result.rawPath }
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
          `Path found: ${result.path.length} waypoints, ${result.pathLength.toFixed(2)} rad after ` +
          `shortcutting (raw ${result.rawPath.length} waypoints, ${result.rawLength.toFixed(2)} rad); ` +
          `${result.nodes} tree nodes in ${result.iterations} iterations.`
        );
      case "start_invalid":
        return "The current pose is in collision or out of limits — move the arm clear and plan again.";
      case "goal_invalid":
        return "The stored goal collides with the obstacles — pose the arm clear and set a new goal.";
      default:
        return `No path found within ${result.iterations} iterations — move or shrink the obstacles.`;
    }
  }, [arm, planGoal, plan]);

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
        return meshStatus.message ?? "Meshes unavailable — showing the schematic.";
    }
  }, [meshStatus]);

  if (loading) {
    return <div className="status">Loading the WebAssembly kinematics module…</div>;
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
        <h1>Forward &amp; inverse kinematics — {arm.label()}</h1>
        <p>
          Product-of-exponentials kinematics in C++ (Eigen + Sophus), compiled to WebAssembly and
          rendered with WebGL. Drag a slider for FK; drag the end-effector gizmo for IK.
        </p>
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
          ellipsoidMode={ellipsoidMode}
          displayMode={displayMode}
          onMeshStatus={handleMeshStatus}
          ikOptions={ikOptions}
        />

        <aside className="panel">
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
            <JointSliders limits={jointLimits} angles={jointAngles} onChange={setJointAngles} />
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
              Both minimise the same local model of the task error. <strong>Box QP</strong> treats
              the joint limits as constraints on the step, so a joint that saturates hands its
              share of the motion to the others. <strong>DLS + clamp</strong> solves as if the
              joints were unbounded and truncates afterwards, which loses that redistribution.
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
              The image of the unit ball of joint velocities, drawn at the end-effector. A long
              axis is a direction the tool moves easily; a flat one is a direction it barely moves
              at all.
            </p>
          </section>

          <section className="panel__section">
            <h2>Motion planning</h2>
            <div className="buttons">
              <button type="button" onClick={addObstacle}>
                Add obstacle
              </button>
              <button type="button" onClick={removeSelectedObstacle} disabled={obstacles.length === 0}>
                Remove
              </button>
            </div>
            {obstacles.length > 0 && (
              <div className="buttons">
                {obstacles.map((obstacle, index) => (
                  <button
                    key={obstacle.id}
                    type="button"
                    className={obstacle.id === selectedObstacleId ? "is-active" : ""}
                    onClick={() =>
                      setSelectedObstacleId(obstacle.id === selectedObstacleId ? null : obstacle.id)
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
                  <span className="slider__value">{(selectedObstacle.radius * 100).toFixed(0)} cm</span>
                </span>
                <input
                  type="range"
                  min={0.05}
                  max={0.3}
                  step={0.01}
                  value={selectedObstacle.radius}
                  onChange={(event) => setSelectedRadius(Number(event.target.value))}
                />
              </label>
            )}
            <div className="buttons">
              <button type="button" onClick={() => setPlanGoal({ owner: arm, angles: jointAngles })}>
                Set goal = current
              </button>
              <button type="button" onClick={runPlan} disabled={planGoal?.owner !== arm}>
                Plan path
              </button>
              {plan?.owner === arm && plan.result.status === "success" && (
                <button type="button" onClick={() => setPlaying((value) => !value)}>
                  {playing ? "Stop" : "Replay"}
                </button>
              )}
            </div>
            <p className="hint" data-testid="plan-status">
              {planHint}
            </p>
          </section>

          <section className="panel__section">
            <h2>Presets</h2>
            <div className="buttons">
              {presets.map((preset) => (
                <button key={preset.label} type="button" onClick={() => setJointAngles(preset.angles)}>
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
