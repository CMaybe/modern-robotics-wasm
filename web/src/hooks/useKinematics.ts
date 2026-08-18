import { useEffect, useState } from "react";

import { loadKinematics } from "../kinematics/loadKinematics";
import type { JointLimit, Robot, RobotInfo, RobotPreset } from "../kinematics/types";

export interface KinematicsState {
  /** Every robot the WASM module can build. Empty until the module loads. */
  robots: RobotInfo[];
  /** The live model for `robotId`, or null while loading or on error. */
  arm: Robot | null;
  jointLimits: JointLimit[];
  presets: RobotPreset[];
  error: string | null;
  loading: boolean;
}

const EMPTY: Pick<KinematicsState, "arm" | "jointLimits" | "presets"> = {
  arm: null,
  jointLimits: [],
  presets: [],
};

/**
 * Instantiates the requested robot from the WASM module and tears it down when
 * the selection changes or the component unmounts.
 *
 * @param robotId Identifier from `availableRobots()`, e.g. "ur5" or "fr3".
 */
export function useKinematics(robotId: string): KinematicsState {
  const [state, setState] = useState<KinematicsState>({
    robots: [],
    ...EMPTY,
    error: null,
    loading: true,
  });

  useEffect(() => {
    let cancelled = false;
    let instance: Robot | null = null;

    setState((previous) => ({ ...previous, ...EMPTY, loading: true, error: null }));

    loadKinematics()
      .then((module) => {
        const robots = module.availableRobots();
        instance = module.createRobot(robotId);

        if (cancelled) {
          instance?.delete();
          instance = null;
          return;
        }
        if (!instance) {
          setState({
            robots,
            ...EMPTY,
            error: `Unknown robot "${robotId}". Available: ${robots.map((r) => r.id).join(", ")}`,
            loading: false,
          });
          return;
        }

        setState({
          robots,
          arm: instance,
          jointLimits: instance.jointLimits(),
          presets: instance.presets(),
          error: null,
          loading: false,
        });
      })
      .catch((error: unknown) => {
        if (cancelled) {
          return;
        }
        setState({
          robots: [],
          ...EMPTY,
          error: error instanceof Error ? error.message : String(error),
          loading: false,
        });
      });

    return () => {
      cancelled = true;
      // embind instances live on the WASM heap and are not garbage collected.
      instance?.delete();
    };
  }, [robotId]);

  return state;
}
