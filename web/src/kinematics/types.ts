/**
 * TypeScript mirror of the embind surface declared in cpp/wasm/bindings.cpp.
 * Keep the two in sync: embind performs no compile-time checking on this side.
 */

/** A rigid transform, delivered in the three shapes the viewer needs. */
export interface Pose {
  /** `[x, y, z]` in metres, space (base) frame. */
  position: [number, number, number];
  /** `[x, y, z, w]`, matching THREE.Quaternion ordering. */
  quaternion: [number, number, number, number];
  /** 16 elements, column-major, matching THREE.Matrix4.fromArray(). */
  matrix: number[];
}

export interface JointLimit {
  name: string;
  /** Radians. */
  lower: number;
  /** Radians. */
  upper: number;
}

export interface LinkFrames {
  /** One pose per joint frame, base to tip, with the end-effector last. */
  frames: Pose[];
  /** Each joint's rotation axis in the space frame, as a unit vector. */
  axes: [number, number, number][];
}

/**
 * A manipulability ellipsoid: the image of the unit ball of joint velocities.
 * Apply `quaternion` and scale by `radii` to turn a unit sphere into it.
 */
export interface Ellipsoid {
  /** Principal-axis orientation, `[x, y, z, w]`. */
  quaternion: [number, number, number, number];
  /** Semi-axis lengths, descending. */
  radii: [number, number, number];
  /** Product of the semi-axes (the Yoshikawa measure for this block). */
  volume: number;
  /** `sigma_min / sigma_max` in [0, 1]; 1 is a sphere, 0 is singular. */
  isotropy: number;
}

export interface ManipulabilityEllipsoids {
  /** End-effector linear velocity, in m/rad. */
  linear: Ellipsoid;
  /** End-effector angular velocity, dimensionless. */
  angular: Ellipsoid;
}

/** One capsule of the robot's collision body, posed in the space frame. */
export interface CollisionCapsule {
  /** Axis start, `[x, y, z]` in metres. */
  start: [number, number, number];
  /** Axis end, `[x, y, z]` in metres. */
  end: [number, number, number];
  /** Metres. */
  radius: number;
  /** Link the capsule rides on. */
  link: number;
}

/** The closest approach a collision query found. */
export interface CollisionContact {
  /** Signed clearance in metres: negative = penetration, Infinity = nothing checked. */
  distance: number;
  /** Robot link involved, or -1 when nothing was checked. */
  link: number;
  /** Witness point on the involved capsule's axis, space frame. */
  point: [number, number, number];
  /** Direction of increasing clearance, unit length. */
  normal: [number, number, number];
}

/** The collision body at one configuration, plus its tightest self pair. */
export interface CollisionBody {
  capsules: CollisionCapsule[];
  selfContact: CollisionContact;
}

/** A spherical obstacle in the space frame. */
export interface SphereObstacle {
  /** `[x, y, z]` in metres. */
  center: [number, number, number];
  /** Metres. */
  radius: number;
}

/** The obstacle world passed to collision queries and the planner. */
export interface ObstacleWorld {
  spheres: SphereObstacle[];
}

/** How a motion plan ended. */
export type PlanStatus = "success" | "start_invalid" | "goal_invalid" | "not_found";

export interface PlanOptions {
  maxIterations?: number;
  /** Joint-space extension step, radians. */
  step?: number;
  /** Collision-check spacing along an edge, radians. */
  resolution?: number;
  /** Clearance every configuration must keep, metres. */
  margin?: number;
  shortcutRounds?: number;
  /** RNG seed; the same seed reproduces the same path. */
  seed?: number;
  /** Waypoint spacing the optimizer densifies to, radians. */
  spacing?: number;
  /** Coordinate-descent sweeps of the trajectory optimizer. */
  optimizerSweeps?: number;
  smoothnessWeight?: number;
  obstacleWeight?: number;
  /** Clearance below which the obstacle cost activates, metres. */
  safeDistance?: number;
}

export interface PlanResult {
  status: PlanStatus;
  /** Shortcut waypoints, start to goal; empty on failure. */
  path: number[][];
  /** The path as the trees found it, before shortcutting. */
  rawPath: number[][];
  /** The shortcut path after optimisation-based smoothing (densified shortcut when infeasible). */
  optimizedPath: number[][];
  /** Whether the optimizer's own output passed full edge validation. */
  optimizedFeasible: boolean;
  iterations: number;
  nodes: number;
  /** Joint-space length of `path`, radians. */
  pathLength: number;
  /** Joint-space length of `rawPath`, radians. */
  rawLength: number;
  /** Joint-space length of `optimizedPath`, radians. */
  optimizedLength: number;
}

/**
 * How each IK iteration turns the task error into a joint-space step.
 *
 * - `"qp"` — box-constrained QP: the joint limits are constraints of the step
 *   itself, so saturating one joint redistributes the motion onto the others.
 * - `"dls"` — damped least squares solved as if unbounded, then clamped.
 */
export type IkMethod = "qp" | "dls";

export interface IkOptions {
  method?: IkMethod;
  maxIterations?: number;
  positionTolerance?: number;
  orientationTolerance?: number;
  /** Levenberg-Marquardt lambda; larger is more stable but slower to converge. */
  damping?: number;
  /** Per-iteration clamp on the largest joint step, in radians. */
  maxStep?: number;
}

export interface IkResult {
  angles: number[];
  converged: boolean;
  iterations: number;
  /** Metres. */
  positionError: number;
  /** Radians. */
  orientationError: number;
}

/** A named configuration, guaranteed by C++ to sit inside the joint limits. */
export interface RobotPreset {
  label: string;
  angles: number[];
}

/** Summary of a robot the module can build, from `availableRobots()`. */
export interface RobotInfo {
  id: string;
  label: string;
  dof: number;
}

/** The C++ `Robot` class, as seen from JavaScript. */
export interface Robot {
  id(): string;
  label(): string;
  dof(): number;
  jointNames(): string[];
  jointLimits(): JointLimit[];
  /** The configuration the viewer should open in; always within the limits. */
  defaultConfiguration(): number[];
  presets(): RobotPreset[];
  forward(angles: number[]): Pose;
  linkFrames(angles: number[]): LinkFrames;
  /** 6xDOF space Jacobian, row-major, in `[v; w]` ordering. */
  jacobian(angles: number[]): number[];
  manipulability(angles: number[]): number;
  manipulabilityEllipsoids(angles: number[]): ManipulabilityEllipsoids;
  /** The collision capsules posed at `angles`, plus the tightest self-collision pair. */
  collisionBody(angles: number[]): CollisionBody;
  /** The tightest approach at `angles` — obstacles and self both. */
  nearestContact(angles: number[], world: ObstacleWorld): CollisionContact;
  /** Plans a collision-free joint path from `start` to `goal` with RRT-Connect. */
  plan(start: number[], goal: number[], world: ObstacleWorld, options: PlanOptions): PlanResult;
  inverse(
    initialGuess: number[],
    position: number[],
    quaternion: number[],
    options: IkOptions,
  ): IkResult;
  /** embind objects are not garbage collected; call this to free the C++ instance. */
  delete(): void;
}

export interface KinematicsModule {
  /** Every robot the module can build, in menu order. */
  availableRobots(): RobotInfo[];
  /** Builds a robot by id, or returns null when the id is unknown. */
  createRobot(id: string): Robot | null;
}
