import { useEffect, useRef } from "react";
import * as THREE from "three";
import { OrbitControls, TransformControls } from "three-stdlib";
import URDFLoader, { type URDFRobot } from "urdf-loader";

import { ROBOT_ASSETS } from "../kinematics/robotAssets";
import type { IkOptions, IkResult, Robot } from "../kinematics/types";

export type GizmoMode = "translate" | "rotate";

/** Vendor meshes from the official URDF, or the abstract link/joint skeleton. */
export type DisplayMode = "mesh" | "schematic";

/** Progress of the vendor mesh load, surfaced so the UI can explain a fallback. */
export interface MeshStatus {
  state: "absent" | "loading" | "ready" | "error";
  /** Largest end-effector disagreement between the URDF chain and the C++ model, in metres. */
  deviation?: number;
  message?: string;
}

/** Which manipulability ellipsoid to draw at the end-effector, if any. */
export type EllipsoidMode = "off" | "linear" | "angular";

export interface RobotViewerProps {
  /** The WASM kinematics model; every pose in the scene comes from it. */
  arm: Robot;
  /** Current joint angles in radians, length == arm.dof(). */
  jointAngles: number[];
  /** Called when dragging the end-effector gizmo produces a new configuration. */
  onJointAnglesChange: (angles: number[]) => void;
  /** Called with the solver diagnostics after each IK step, and with null on drag end. */
  onIkResult: (result: IkResult | null) => void;
  /** Whether the gizmo moves or reorients the end-effector target. */
  gizmoMode: GizmoMode;
  /** Draws a coloured arrow along every joint's rotation axis. */
  showJointAxes: boolean;
  /** Overlays the collision capsules; they turn red on a self-collision. */
  showCollision: boolean;
  /** Which manipulability ellipsoid to overlay on the end-effector. */
  ellipsoidMode: EllipsoidMode;
  /** Whether to draw the vendor meshes or the schematic skeleton. */
  displayMode: DisplayMode;
  /** Reports vendor-mesh load progress and how well it agrees with the C++ model. */
  onMeshStatus: (status: MeshStatus) => void;
  /** Solver tuning forwarded to Robot.inverse(). */
  ikOptions: IkOptions;
}

const LINK_RADIUS = 0.028;
const JOINT_RADIUS = 0.045;
const EE_AXIS_LENGTH = 0.22;
const JOINT_AXIS_LENGTH = 0.16;

const COLOR_BACKGROUND = 0x11151c;
const COLOR_LINK = 0x8899aa;
const COLOR_JOINT = 0xf5a623;
const COLOR_BASE = 0x39424e;
const COLOR_JOINT_AXIS = 0x4dd4ac;
const COLOR_ELLIPSOID_LINEAR = 0x4d9dff;
const COLOR_ELLIPSOID_ANGULAR = 0xc46dff;
const COLOR_COLLISION_CLEAR = 0x59c96b;
const COLOR_COLLISION_HIT = 0xff4d5e;

/**
 * Metres drawn per unit of ellipsoid radius. The linear block is in m/rad and
 * peaks near 1, the angular block is dimensionless and peaks near 2, so each
 * gets its own factor to end up a comparable size next to a ~0.9 m arm.
 */
const ELLIPSOID_SCALE: Record<"linear" | "angular", number> = {
  linear: 0.26,
  angular: 0.10,
};

/** A collapsed axis would scale the mesh to zero; keep a sliver so the disc stays visible. */
const MIN_ELLIPSOID_RADIUS = 2e-3;

/** Cylinders are generated along +Y; this is the axis we rotate away from. */
const CYLINDER_AXIS = new THREE.Vector3(0, 1, 0);

/**
 * TransformControls emits events outside three's typed Object3DEventMap, so its
 * listeners are registered through this narrower view of the same object.
 */
interface GizmoEventSource {
  addEventListener(type: "dragging-changed", listener: (event: { value: boolean }) => void): void;
  addEventListener(type: "objectChange", listener: () => void): void;
  removeEventListener(type: "dragging-changed", listener: (event: { value: boolean }) => void): void;
  removeEventListener(type: "objectChange", listener: () => void): void;
}

/**
 * WebGL view of the arm.
 *
 * Poses are never computed in JavaScript: every frame drawn here comes straight
 * out of the C++ `linkFrames()` call, and dragging the end-effector gizmo feeds
 * the target pose back into the C++ IK solver.
 */
export default function RobotViewer({
  arm,
  jointAngles,
  onJointAnglesChange,
  onIkResult,
  gizmoMode,
  showJointAxes,
  showCollision,
  ellipsoidMode,
  displayMode,
  onMeshStatus,
  ikOptions,
}: RobotViewerProps) {
  const mountRef = useRef<HTMLDivElement>(null);

  // Props that the imperative three.js callbacks need without re-creating the scene.
  const onJointAnglesChangeRef = useRef(onJointAnglesChange);
  const onIkResultRef = useRef(onIkResult);
  const ikOptionsRef = useRef(ikOptions);
  const jointAnglesRef = useRef(jointAngles);

  onJointAnglesChangeRef.current = onJointAnglesChange;
  onIkResultRef.current = onIkResult;
  ikOptionsRef.current = ikOptions;
  jointAnglesRef.current = jointAngles;

  // Imperative handles owned by the setup effect and used by the update effects.
  const applyAnglesRef = useRef<((angles: number[]) => void) | null>(null);
  const transformControlsRef = useRef<TransformControls | null>(null);
  const jointAxisArrowsRef = useRef<THREE.ArrowHelper[]>([]);
  const gizmoTargetRef = useRef<THREE.Object3D | null>(null);
  const ellipsoidModeRef = useRef(ellipsoidMode);
  const setEllipsoidModeRef = useRef<((mode: EllipsoidMode) => void) | null>(null);
  ellipsoidModeRef.current = ellipsoidMode;
  const showCollisionRef = useRef(showCollision);
  const setShowCollisionRef = useRef<((show: boolean) => void) | null>(null);
  showCollisionRef.current = showCollision;
  const onMeshStatusRef = useRef(onMeshStatus);
  onMeshStatusRef.current = onMeshStatus;
  const displayModeRef = useRef(displayMode);
  displayModeRef.current = displayMode;
  const setDisplayModeRef = useRef<((mode: DisplayMode) => void) | null>(null);
  const applyUrdfJointsRef = useRef<((angles: number[]) => void) | null>(null);
  // TransformControls keeps its drag flag private, so the drag state is mirrored here.
  const draggingRef = useRef(false);

  useEffect(() => {
    const mount = mountRef.current;
    if (!mount) {
      return undefined;
    }

    const dof = arm.dof();

    const scene = new THREE.Scene();
    scene.background = new THREE.Color(COLOR_BACKGROUND);

    // The robot models are Z-up, so the whole view is Z-up too. Keeping the scene
    // in the same frame as the C++ code means no coordinate conversion anywhere.
    const camera = new THREE.PerspectiveCamera(45, 1, 0.01, 100);
    camera.up.set(0, 0, 1);
    // Far enough back that a fully extended arm still fits; both the UR5 and the
    // FR3 reach roughly 0.9 m.
    camera.position.set(1.35, -1.35, 1.05);

    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.shadowMap.enabled = false;
    mount.appendChild(renderer.domElement);

    const orbitControls = new OrbitControls(camera, renderer.domElement);
    orbitControls.target.set(0, 0, 0.3);
    orbitControls.enableDamping = true;
    orbitControls.dampingFactor = 0.12;
    orbitControls.minDistance = 0.3;
    orbitControls.maxDistance = 12;

    scene.add(new THREE.AmbientLight(0xffffff, 0.55));
    const keyLight = new THREE.DirectionalLight(0xffffff, 1.4);
    keyLight.position.set(2, -3, 4);
    scene.add(keyLight);
    const fillLight = new THREE.DirectionalLight(0x88aaff, 0.5);
    fillLight.position.set(-3, 2, 1);
    scene.add(fillLight);

    // Ground grid, rotated into the XY plane because the world is Z-up.
    // 2.4 m across in 24 divisions, so one cell reads as 10 cm of reach.
    const grid = new THREE.GridHelper(2.4, 24, 0x3a4553, 0x222831);
    grid.rotation.x = Math.PI / 2;
    scene.add(grid);
    scene.add(new THREE.AxesHelper(0.35));

    const robotGroup = new THREE.Group();
    scene.add(robotGroup);

    // The schematic skeleton and the vendor meshes are alternatives; overlays such
    // as the joint axes, the end-effector triad and the ellipsoid sit outside both.
    const schematicGroup = new THREE.Group();
    robotGroup.add(schematicGroup);

    const basePlinth = new THREE.Mesh(
      new THREE.CylinderGeometry(0.09, 0.11, 0.02, 32),
      new THREE.MeshStandardMaterial({ color: COLOR_BASE, roughness: 0.7, metalness: 0.2 }),
    );
    basePlinth.rotation.x = Math.PI / 2;
    basePlinth.position.set(0, 0, 0.01);
    schematicGroup.add(basePlinth);

    const linkMaterial = new THREE.MeshStandardMaterial({
      color: COLOR_LINK,
      roughness: 0.42,
      metalness: 0.35,
    });
    const jointMaterial = new THREE.MeshStandardMaterial({
      color: COLOR_JOINT,
      roughness: 0.35,
      metalness: 0.25,
    });

    // Unit-height cylinder scaled along Y per frame, so the geometry is built once.
    const linkGeometry = new THREE.CylinderGeometry(LINK_RADIUS, LINK_RADIUS, 1, 20);
    const jointGeometry = new THREE.SphereGeometry(JOINT_RADIUS, 24, 16);

    const frameCount = dof + 1;
    const jointMeshes: THREE.Mesh[] = [];
    const linkMeshes: THREE.Mesh[] = [];
    const jointAxisArrows: THREE.ArrowHelper[] = [];

    for (let i = 0; i < frameCount; i += 1) {
      const joint = new THREE.Mesh(jointGeometry, jointMaterial);
      schematicGroup.add(joint);
      jointMeshes.push(joint);

      // One segment per gap in [base, frame_0, ..., frame_dof]. The leading gap
      // matters: an FR3's first joint sits 0.333 m up, so without it the arm floats.
      const link = new THREE.Mesh(linkGeometry, linkMaterial);
      schematicGroup.add(link);
      linkMeshes.push(link);

      if (i < dof) {
        const arrow = new THREE.ArrowHelper(
          new THREE.Vector3(0, 0, 1),
          new THREE.Vector3(),
          JOINT_AXIS_LENGTH,
          COLOR_JOINT_AXIS,
          0.05,
          0.03,
        );
        robotGroup.add(arrow);
        jointAxisArrows.push(arrow);
      }
    }
    jointAxisArrowsRef.current = jointAxisArrows;

    // The end-effector triad the user asked to see, plus the drag target it follows.
    const endEffectorAxes = new THREE.AxesHelper(EE_AXIS_LENGTH);
    (endEffectorAxes.material as THREE.Material & { depthTest: boolean }).depthTest = false;
    endEffectorAxes.renderOrder = 2;
    robotGroup.add(endEffectorAxes);

    // Manipulability ellipsoid: a unit sphere transformed by the principal axes
    // and semi-axis lengths that the C++ side returns.
    const ellipsoidGeometry = new THREE.SphereGeometry(1, 32, 24);
    const ellipsoidSurface = new THREE.MeshStandardMaterial({
      color: COLOR_ELLIPSOID_LINEAR,
      transparent: true,
      opacity: 0.22,
      roughness: 0.5,
      metalness: 0.0,
      depthWrite: false,
      side: THREE.DoubleSide,
    });
    const ellipsoidWireframe = new THREE.MeshBasicMaterial({
      color: COLOR_ELLIPSOID_LINEAR,
      wireframe: true,
      transparent: true,
      opacity: 0.3,
      depthWrite: false,
    });

    const ellipsoidGroup = new THREE.Group();
    ellipsoidGroup.visible = false;
    ellipsoidGroup.renderOrder = 1;
    ellipsoidGroup.add(new THREE.Mesh(ellipsoidGeometry, ellipsoidSurface));
    ellipsoidGroup.add(new THREE.Mesh(ellipsoidGeometry, ellipsoidWireframe));
    robotGroup.add(ellipsoidGroup);

    setEllipsoidModeRef.current = (mode: EllipsoidMode) => {
      ellipsoidGroup.visible = mode !== "off";
      const color = mode === "angular" ? COLOR_ELLIPSOID_ANGULAR : COLOR_ELLIPSOID_LINEAR;
      ellipsoidSurface.color.setHex(color);
      ellipsoidWireframe.color.setHex(color);
    };

    // ---- Collision capsules ------------------------------------------------
    // Each capsule is rigid, so its geometry is built once from the C++ body and
    // only re-posed per frame. The shared material flips to red on self-collision.
    const collisionGroup = new THREE.Group();
    collisionGroup.visible = showCollisionRef.current;
    robotGroup.add(collisionGroup);

    const collisionMaterial = new THREE.MeshStandardMaterial({
      color: COLOR_COLLISION_CLEAR,
      transparent: true,
      opacity: 0.3,
      roughness: 0.5,
      metalness: 0.0,
      depthWrite: false,
    });
    const collisionGeometries: THREE.CapsuleGeometry[] = [];
    const collisionMeshes: THREE.Mesh[] = [];
    for (const capsule of arm.collisionBody(arm.defaultConfiguration()).capsules) {
      const length = new THREE.Vector3()
        .fromArray(capsule.end)
        .distanceTo(new THREE.Vector3().fromArray(capsule.start));
      const geometry = new THREE.CapsuleGeometry(capsule.radius, length, 6, 16);
      collisionGeometries.push(geometry);
      const mesh = new THREE.Mesh(geometry, collisionMaterial);
      collisionGroup.add(mesh);
      collisionMeshes.push(mesh);
    }

    // Marks the witness point of the tightest self pair while it penetrates.
    const contactMarker = new THREE.Mesh(
      new THREE.SphereGeometry(0.02, 16, 12),
      new THREE.MeshBasicMaterial({ color: COLOR_COLLISION_HIT, depthTest: false }),
    );
    contactMarker.renderOrder = 3;
    contactMarker.visible = false;
    collisionGroup.add(contactMarker);

    setShowCollisionRef.current = (show: boolean) => {
      collisionGroup.visible = show;
    };

    // ---- Vendor meshes ---------------------------------------------------
    // The URDF chain and the C++ PoE model describe the same robot, so the meshes
    // need no alignment offsets: feeding the solver's joint angles into the URDF
    // puts the rendered flange exactly where forward() says it is. That claim is
    // measured once after loading and reported through onMeshStatus.
    const asset = ROBOT_ASSETS[arm.id()];
    let urdfRobot: URDFRobot | null = null;
    let meshesReady = false;
    let disposed = false;

    const applyUrdfJoints = (angles: number[]) => {
      if (!urdfRobot || !asset) {
        return;
      }
      for (let i = 0; i < asset.joints.length && i < angles.length; i += 1) {
        urdfRobot.setJointValue(asset.joints[i], angles[i]);
      }
    };
    applyUrdfJointsRef.current = applyUrdfJoints;

    const setDisplayMode = (mode: DisplayMode) => {
      // Falling back to the schematic keeps the app usable before the meshes have
      // downloaded, and when they were never fetched at all.
      const useMesh = mode === "mesh" && meshesReady;
      schematicGroup.visible = !useMesh;
      if (urdfRobot) {
        urdfRobot.visible = useMesh;
      }
    };
    setDisplayModeRef.current = setDisplayMode;
    setDisplayMode(displayModeRef.current);

    if (!asset) {
      onMeshStatusRef.current({ state: "absent", message: `No URDF registered for "${arm.id()}"` });
    } else {
      onMeshStatusRef.current({ state: "loading" });

      // The URDF resolves as soon as it is parsed, but its meshes are still in
      // flight; the LoadingManager is what tells us the robot is fully built.
      const manager = new THREE.LoadingManager();
      const loader = new URDFLoader(manager);
      loader.packages = asset.packages;
      loader.parseCollision = false;

      manager.onLoad = () => {
        if (disposed || !urdfRobot) {
          return;
        }
        meshesReady = true;
        urdfRobot.visible = false;
        robotGroup.add(urdfRobot);
        applyUrdfJoints(jointAnglesRef.current);
        urdfRobot.updateMatrixWorld(true);

        // Cross-check the vendor chain against the C++ model at the current pose.
        let deviation: number | undefined;
        const tip = urdfRobot.links[asset.tipLink];
        if (tip) {
          const meshTip = tip.getWorldPosition(new THREE.Vector3());
          const solverTip = new THREE.Vector3().fromArray(
            arm.forward(jointAnglesRef.current).position,
          );
          deviation = meshTip.distanceTo(solverTip);
        }

        setDisplayMode(displayModeRef.current);
        onMeshStatusRef.current({ state: "ready", deviation });
      };

      const reportFailure = (detail: string) => {
        if (!disposed) {
          onMeshStatusRef.current({
            state: "error",
            message: `${detail} — run scripts/fetch_meshes.sh to download the robot meshes.`,
          });
        }
      };
      manager.onError = (url) => reportFailure(`Could not load ${url}`);

      loader.load(
        asset.urdf,
        (result) => {
          urdfRobot = result;
        },
        undefined,
        () => reportFailure(`Could not load ${asset.urdf}`),
      );
    }

    const target = new THREE.Object3D();
    scene.add(target);
    gizmoTargetRef.current = target;

    const transformControls = new TransformControls(camera, renderer.domElement);
    transformControls.setMode(gizmoMode);
    transformControls.setSize(0.9);
    transformControls.attach(target);
    transformControlsRef.current = transformControls;

    // three r169+ moved the gizmo geometry behind getHelper(); older builds are Object3D themselves.
    const gizmoHelper =
      (transformControls as unknown as { getHelper?: () => THREE.Object3D }).getHelper?.() ??
      (transformControls as unknown as THREE.Object3D);
    scene.add(gizmoHelper);

    /** Redraws the arm for the given configuration. */
    const applyAngles = (angles: number[]) => {
      // The vendor meshes are posed by the URDF's own forward kinematics, driven
      // by the very angles the C++ solver produced.
      applyUrdfJoints(angles);

      const { frames, axes } = arm.linkFrames(angles);

      for (let i = 0; i < frames.length && i < jointMeshes.length; i += 1) {
        const [x, y, z] = frames[i].position;
        jointMeshes[i].position.set(x, y, z);
      }

      // Chain the segments through the base so the first link is drawn too.
      const points: [number, number, number][] = [[0, 0, 0], ...frames.map((frame) => frame.position)];

      for (let i = 0; i < linkMeshes.length && i + 1 < points.length; i += 1) {
        const [ax, ay, az] = points[i];
        const [bx, by, bz] = points[i + 1];

        const start = new THREE.Vector3(ax, ay, az);
        const direction = new THREE.Vector3(bx - ax, by - ay, bz - az);
        const length = direction.length();
        const link = linkMeshes[i];

        // Zero-length segments (coincident joint origins) would produce a NaN quaternion.
        if (length < 1e-6) {
          link.visible = false;
          continue;
        }
        link.visible = true;
        link.position.copy(start).addScaledVector(direction, 0.5);
        link.quaternion.setFromUnitVectors(CYLINDER_AXIS, direction.clone().normalize());
        link.scale.set(1, length, 1);
      }

      for (let i = 0; i < jointAxisArrows.length && i < axes.length; i += 1) {
        const [x, y, z] = frames[i].position;
        const [dx, dy, dz] = axes[i];
        const direction = new THREE.Vector3(dx, dy, dz);
        // A prismatic axis has no rotation direction; leave the last one drawn.
        if (direction.lengthSq() < 1e-12) {
          continue;
        }
        jointAxisArrows[i].position.set(x, y, z);
        jointAxisArrows[i].setDirection(direction.normalize());
      }

      const endEffector = frames[frames.length - 1];
      endEffectorAxes.position.fromArray(endEffector.position);
      endEffectorAxes.quaternion.fromArray(endEffector.quaternion);

      if (collisionGroup.visible) {
        const body = arm.collisionBody(angles);
        for (let i = 0; i < collisionMeshes.length && i < body.capsules.length; i += 1) {
          const start = new THREE.Vector3().fromArray(body.capsules[i].start);
          const direction = new THREE.Vector3().fromArray(body.capsules[i].end).sub(start);
          collisionMeshes[i].position.copy(start).addScaledVector(direction, 0.5);
          collisionMeshes[i].quaternion.setFromUnitVectors(CYLINDER_AXIS, direction.normalize());
        }

        const colliding = body.selfContact.distance <= 0;
        collisionMaterial.color.setHex(colliding ? COLOR_COLLISION_HIT : COLOR_COLLISION_CLEAR);
        contactMarker.visible = colliding;
        if (colliding) {
          contactMarker.position.fromArray(body.selfContact.point);
        }
      }

      const mode = ellipsoidModeRef.current;
      if (mode !== "off") {
        const { radii, quaternion } = arm.manipulabilityEllipsoids(angles)[mode];
        const scale = ELLIPSOID_SCALE[mode];
        ellipsoidGroup.position.fromArray(endEffector.position);
        ellipsoidGroup.quaternion.fromArray(quaternion);
        ellipsoidGroup.scale.set(
          Math.max(radii[0] * scale, MIN_ELLIPSOID_RADIUS),
          Math.max(radii[1] * scale, MIN_ELLIPSOID_RADIUS),
          Math.max(radii[2] * scale, MIN_ELLIPSOID_RADIUS),
        );
      }
    };
    applyAnglesRef.current = applyAngles;

    /** Moves the drag target onto the arm's actual end-effector pose. */
    const syncTargetToEndEffector = (angles: number[]) => {
      const pose = arm.forward(angles);
      target.position.fromArray(pose.position);
      target.quaternion.fromArray(pose.quaternion);
      target.updateMatrixWorld();
    };

    applyAngles(jointAnglesRef.current);
    syncTargetToEndEffector(jointAnglesRef.current);

    const handleDraggingChanged = (event: { value: boolean }) => {
      draggingRef.current = event.value;
      orbitControls.enabled = !event.value;
      if (!event.value) {
        // On release, snap the target back onto the pose the arm actually reached,
        // so an unreachable drag does not leave the gizmo stranded.
        syncTargetToEndEffector(jointAnglesRef.current);
        onIkResultRef.current(null);
      }
    };

    const handleObjectChange = () => {
      if (!draggingRef.current) {
        return;
      }
      const result = arm.inverse(
        jointAnglesRef.current,
        target.position.toArray(),
        target.quaternion.toArray(),
        ikOptionsRef.current,
      );

      // Draw immediately so the arm tracks the pointer without waiting for React.
      jointAnglesRef.current = result.angles;
      applyAngles(result.angles);

      onJointAnglesChangeRef.current(result.angles);
      onIkResultRef.current(result);
    };

    const gizmoEvents = transformControls as unknown as GizmoEventSource;
    gizmoEvents.addEventListener("dragging-changed", handleDraggingChanged);
    gizmoEvents.addEventListener("objectChange", handleObjectChange);

    const resize = () => {
      const width = mount.clientWidth || 1;
      const height = mount.clientHeight || 1;
      camera.aspect = width / height;
      camera.updateProjectionMatrix();
      // updateStyle must stay on: the drawing buffer is devicePixelRatio times
      // larger than the container, and without a matching CSS size the canvas
      // element renders at buffer size and overflows the page on HiDPI screens.
      renderer.setSize(width, height);
    };
    resize();

    const resizeObserver = new ResizeObserver(resize);
    resizeObserver.observe(mount);

    let frameHandle = 0;
    const renderLoop = () => {
      frameHandle = requestAnimationFrame(renderLoop);
      orbitControls.update();
      renderer.render(scene, camera);
    };
    renderLoop();

    return () => {
      cancelAnimationFrame(frameHandle);
      resizeObserver.disconnect();

      gizmoEvents.removeEventListener("dragging-changed", handleDraggingChanged);
      gizmoEvents.removeEventListener("objectChange", handleObjectChange);
      transformControls.detach();
      transformControls.dispose();
      scene.remove(gizmoHelper);
      transformControlsRef.current = null;
      gizmoTargetRef.current = null;
      draggingRef.current = false;
      applyAnglesRef.current = null;
      jointAxisArrowsRef.current = [];

      orbitControls.dispose();

      linkGeometry.dispose();
      jointGeometry.dispose();
      linkMaterial.dispose();
      jointMaterial.dispose();
      basePlinth.geometry.dispose();
      (basePlinth.material as THREE.Material).dispose();
      jointAxisArrows.forEach((arrow) => arrow.dispose());
      endEffectorAxes.dispose();
      grid.dispose();
      ellipsoidGeometry.dispose();
      ellipsoidSurface.dispose();
      ellipsoidWireframe.dispose();
      setEllipsoidModeRef.current = null;

      collisionGeometries.forEach((geometry) => geometry.dispose());
      collisionMaterial.dispose();
      contactMarker.geometry.dispose();
      (contactMarker.material as THREE.Material).dispose();
      setShowCollisionRef.current = null;

      disposed = true;
      setDisplayModeRef.current = null;
      applyUrdfJointsRef.current = null;
      if (urdfRobot) {
        robotGroup.remove(urdfRobot);
        // Vendor meshes are the biggest allocation in the scene; release them
        // explicitly so switching robots does not leak GPU memory.
        urdfRobot.traverse((child) => {
          const mesh = child as THREE.Mesh;
          if (!mesh.isMesh) {
            return;
          }
          mesh.geometry?.dispose();
          const material = mesh.material;
          if (Array.isArray(material)) {
            material.forEach((entry) => entry.dispose());
          } else {
            material?.dispose();
          }
        });
        urdfRobot = null;
      }

      renderer.dispose();
      if (renderer.domElement.parentNode === mount) {
        mount.removeChild(renderer.domElement);
      }
    };
  }, [arm]);

  // Slider-driven updates: redraw and pull the gizmo along with the end-effector.
  useEffect(() => {
    applyAnglesRef.current?.(jointAngles);

    // While a drag is in flight the gizmo is the input, not the output; moving it
    // here would fight the pointer whenever IK cannot reach the target exactly.
    const target = gizmoTargetRef.current;
    if (target && !draggingRef.current) {
      const pose = arm.forward(jointAngles);
      target.position.fromArray(pose.position);
      target.quaternion.fromArray(pose.quaternion);
      target.updateMatrixWorld();
    }
  }, [arm, jointAngles]);

  useEffect(() => {
    transformControlsRef.current?.setMode(gizmoMode);
  }, [gizmoMode]);

  useEffect(() => {
    jointAxisArrowsRef.current.forEach((arrow) => {
      arrow.visible = showJointAxes;
    });
  }, [arm, showJointAxes]);

  // Changing the mode swaps visibility and colour, then redraws so the ellipsoid
  // is correct immediately rather than at the next configuration change.
  useEffect(() => {
    setEllipsoidModeRef.current?.(ellipsoidMode);
    applyAnglesRef.current?.(jointAnglesRef.current);
  }, [arm, ellipsoidMode]);

  // The capsules are only re-posed while visible, so enabling them needs a redraw.
  useEffect(() => {
    setShowCollisionRef.current?.(showCollision);
    applyAnglesRef.current?.(jointAnglesRef.current);
  }, [arm, showCollision]);

  useEffect(() => {
    setDisplayModeRef.current?.(displayMode);
  }, [arm, displayMode]);

  return <div className="viewer" ref={mountRef} />;
}
