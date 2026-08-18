import type { JointLimit } from "../kinematics/types";

const RAD_TO_DEG = 180 / Math.PI;
const DEG_TO_RAD = Math.PI / 180;

export interface JointSlidersProps {
  limits: JointLimit[];
  /** Joint angles in radians. */
  angles: number[];
  /** Receives the full angle vector, in radians. */
  onChange: (angles: number[]) => void;
}

/**
 * One slider per joint, in degrees.
 *
 * Moving a slider is pure forward kinematics: the value goes straight into the
 * configuration and the viewer recomputes the pose from it.
 */
export default function JointSliders({ limits, angles, onChange }: JointSlidersProps) {
  const setJoint = (index: number, degrees: number) => {
    const next = angles.slice();
    next[index] = degrees * DEG_TO_RAD;
    onChange(next);
  };

  return (
    <div className="sliders">
      {limits.map((limit, index) => {
        const degrees = (angles[index] ?? 0) * RAD_TO_DEG;
        return (
          <label className="slider" key={limit.name}>
            <span className="slider__label">
              <span className="slider__name">
                <span className="slider__index">{index + 1}</span>
                {limit.name}
              </span>
              <span className="slider__value">{degrees.toFixed(1)}&deg;</span>
            </span>
            <input
              type="range"
              // Rounded because the limits arrive as float32 radians, so 2*pi
              // converts to 360.00001 degrees.
              min={Math.round(limit.lower * RAD_TO_DEG * 10) / 10}
              max={Math.round(limit.upper * RAD_TO_DEG * 10) / 10}
              step={0.5}
              value={degrees}
              onChange={(event) => setJoint(index, Number(event.target.value))}
            />
          </label>
        );
      })}
    </div>
  );
}
