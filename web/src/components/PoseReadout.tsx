import type { Ellipsoid, IkResult, Pose } from "../kinematics/types";

export interface PoseReadoutProps {
  /** End-effector pose for the current configuration, straight from C++ FK. */
  pose: Pose;
  /** Yoshikawa manipulability; approaches zero near a singularity. */
  manipulability: number;
  /** The ellipsoid currently drawn, with the unit its radii are measured in. */
  ellipsoid: { label: string; unit: string; data: Ellipsoid } | null;
  /** Diagnostics from the most recent IK step, or null when not dragging. */
  ikResult: IkResult | null;
}

/** Converts a three.js-ordered quaternion to roll/pitch/yaw in degrees, for readability. */
function toRollPitchYaw(quaternion: [number, number, number, number]): [number, number, number] {
  const [x, y, z, w] = quaternion;

  const sinRoll = 2 * (w * x + y * z);
  const cosRoll = 1 - 2 * (x * x + y * y);
  const roll = Math.atan2(sinRoll, cosRoll);

  const sinPitch = 2 * (w * y - z * x);
  const pitch = Math.abs(sinPitch) >= 1 ? Math.sign(sinPitch) * (Math.PI / 2) : Math.asin(sinPitch);

  const sinYaw = 2 * (w * z + x * y);
  const cosYaw = 1 - 2 * (y * y + z * z);
  const yaw = Math.atan2(sinYaw, cosYaw);

  const toDegrees = 180 / Math.PI;
  return [roll * toDegrees, pitch * toDegrees, yaw * toDegrees];
}

/** Live end-effector pose plus the solver's report on the current drag. */
export default function PoseReadout({ pose, manipulability, ellipsoid, ikResult }: PoseReadoutProps) {
  const [x, y, z] = pose.position;
  const [roll, pitch, yaw] = toRollPitchYaw(pose.quaternion);
  const nearSingular = manipulability < 1e-3;

  return (
    <div className="readout">
      <div className="readout__row">
        <span className="readout__key">position</span>
        <span className="readout__value">
          {x.toFixed(4)}, {y.toFixed(4)}, {z.toFixed(4)} m
        </span>
      </div>
      <div className="readout__row">
        <span className="readout__key">rpy</span>
        <span className="readout__value">
          {roll.toFixed(1)}, {pitch.toFixed(1)}, {yaw.toFixed(1)} &deg;
        </span>
      </div>
      <div className="readout__row">
        <span className="readout__key">manipulability</span>
        <span className={`readout__value${nearSingular ? " readout__value--warn" : ""}`}>
          {manipulability.toExponential(2)}
          {nearSingular ? " (near singular)" : ""}
        </span>
      </div>

      {ellipsoid ? (
        <>
          <div className="readout__divider" />
          <div className="readout__row">
            <span className="readout__key">{ellipsoid.label} axes</span>
            <span className="readout__value">
              {ellipsoid.data.radii.map((radius) => radius.toFixed(3)).join(", ")} {ellipsoid.unit}
            </span>
          </div>
          <div className="readout__row">
            <span className="readout__key">isotropy</span>
            <span
              className={`readout__value${ellipsoid.data.isotropy < 0.05 ? " readout__value--warn" : ""}`}
            >
              {ellipsoid.data.isotropy.toFixed(4)}
              {ellipsoid.data.isotropy < 0.05 ? " (degenerate)" : ""}
            </span>
          </div>
        </>
      ) : null}

      <div className="readout__divider" />

      {ikResult ? (
        <>
          <div className="readout__row">
            <span className="readout__key">ik</span>
            <span
              className={`readout__value${ikResult.converged ? "" : " readout__value--warn"}`}
            >
              {ikResult.converged ? "converged" : "not converged"} in {ikResult.iterations} iters
            </span>
          </div>
          <div className="readout__row">
            <span className="readout__key">residual</span>
            <span className="readout__value">
              {(ikResult.positionError * 1000).toFixed(3)} mm,{" "}
              {(ikResult.orientationError * (180 / Math.PI)).toFixed(3)} &deg;
            </span>
          </div>
        </>
      ) : (
        <div className="readout__row">
          <span className="readout__key">ik</span>
          <span className="readout__value readout__value--muted">
            drag the end-effector gizmo to solve
          </span>
        </div>
      )}
    </div>
  );
}
