/**
 * Maps each C++ robot id onto the vendor URDF that supplies its meshes.
 *
 * The assets are downloaded by `scripts/fetch_meshes.sh`; when they are absent
 * the viewer falls back to its schematic rendering, so this map is the only place
 * that knows about them.
 *
 * The joint list must be in the same order as the C++ model's joints, since the
 * viewer drives the URDF by index from the configuration the solver produces.
 */
export interface RobotAsset {
  /** URDF served out of web/public. */
  urdf: string;
  /** `package://<name>/...` prefixes to rewrite, for urdf-loader. */
  packages: Record<string, string>;
  /** URDF joint names, ordered to match the C++ model. */
  joints: string[];
  /** URDF link whose frame should coincide with the C++ end-effector pose. */
  tipLink: string;
}

const assetUrl = (path: string) => new URL(path, window.location.href).toString();

export const ROBOT_ASSETS: Record<string, RobotAsset> = {
  ur5: {
    urdf: assetUrl("robots/ur5/ur5.urdf"),
    packages: { ur_description: assetUrl("robots/ur5/") },
    joints: [
      "shoulder_pan_joint",
      "shoulder_lift_joint",
      "elbow_joint",
      "wrist_1_joint",
      "wrist_2_joint",
      "wrist_3_joint",
    ],
    tipLink: "tool0",
  },
  fr3: {
    urdf: assetUrl("robots/fr3/fr3.urdf"),
    packages: { franka_description: assetUrl("robots/fr3/") },
    joints: [
      "fr3_joint1",
      "fr3_joint2",
      "fr3_joint3",
      "fr3_joint4",
      "fr3_joint5",
      "fr3_joint6",
      "fr3_joint7",
    ],
    tipLink: "fr3_link8",
  },
};
