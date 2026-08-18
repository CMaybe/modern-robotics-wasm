import { useCallback, useMemo, useState } from "react";

import JointSliders from "./components/JointSliders";
import PoseReadout from "./components/PoseReadout";
import RobotViewer, {
  type DisplayMode,
  type EllipsoidMode,
  type GizmoMode,
  type MeshStatus,
} from "./components/RobotViewer";
import { useKinematics } from "./hooks/useKinematics";
import type { IkMethod, IkOptions, IkResult, Robot } from "./kinematics/types";

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
  const [ellipsoidMode, setEllipsoidMode] = useState<EllipsoidMode>("linear");
  const [displayMode, setDisplayMode] = useState<DisplayMode>("mesh");
  const [meshStatus, setMeshStatus] = useState<MeshStatus>({ state: "loading" });
  const [ikResult, setIkResult] = useState<IkResult | null>(null);
  const [ikMethod, setIkMethod] = useState<IkMethod>("qp");
  const ikOptions = useMemo<IkOptions>(() => ({ ...BASE_IK_OPTIONS, method: ikMethod }), [ikMethod]);

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
