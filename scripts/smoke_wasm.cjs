#!/usr/bin/env node
/**
 * Headless check that the compiled WebAssembly module exposes a working embind API.
 * Mirrors what the browser does, so a broken binding fails here instead of in the UI.
 *
 *   node scripts/smoke_wasm.cjs
 */
"use strict";

const path = require("path");

const MODULE_PATH = path.resolve(__dirname, "../web/public/wasm/kinematics.js");
const HALF_PI = Math.PI / 2;
const QUARTER_PI = Math.PI / 4;

let failures = 0;

function check(name, condition, detail) {
  if (condition) {
    console.log(`  ok   ${name}`);
  } else {
    failures += 1;
    console.log(`  FAIL ${name}${detail ? ` — ${detail}` : ""}`);
  }
}

function close(a, b, tolerance) {
  return Math.abs(a - b) <= tolerance;
}

/** Checks that apply to every robot, whatever its DOF. */
function checkCommonContract(arm, expectedDof) {
  const dof = arm.dof();
  check(`${arm.id()}: dof === ${expectedDof}`, dof === expectedDof, `got ${dof}`);
  check(`${arm.id()}: label is non-empty`, typeof arm.label() === "string" && arm.label().length > 0);
  check(`${arm.id()}: jointNames has ${expectedDof} entries`, arm.jointNames().length === dof);
  check(`${arm.id()}: jointLimits has ${expectedDof} entries`, arm.jointLimits().length === dof);
  check(`${arm.id()}: defaultConfiguration has ${expectedDof} entries`, arm.defaultConfiguration().length === dof);

  const limits = arm.jointLimits();
  const preset = arm.defaultConfiguration();
  check(`${arm.id()}: default configuration is within the joint limits`,
    preset.every((angle, i) => angle >= limits[i].lower - 1e-5 && angle <= limits[i].upper + 1e-5),
    JSON.stringify(preset));

  const presets = arm.presets();
  check(`${arm.id()}: presets are non-empty and legal`,
    presets.length > 0 && presets.every((p) =>
      p.angles.length === dof &&
      p.angles.every((angle, i) => angle >= limits[i].lower - 1e-5 && angle <= limits[i].upper + 1e-5)),
    JSON.stringify(presets.map((p) => p.label)));

  const { frames, axes } = arm.linkFrames(preset);
  check(`${arm.id()}: linkFrames returns dof + 1 frames`, frames.length === dof + 1, `got ${frames.length}`);
  check(`${arm.id()}: linkFrames returns dof axes`, axes.length === dof, `got ${axes.length}`);
  check(`${arm.id()}: every joint axis is a unit vector`,
    axes.every((axis) => close(Math.hypot(axis[0], axis[1], axis[2]), 1, 1e-4)));

  const pose = arm.forward(preset);
  check(`${arm.id()}: last frame equals forward()`,
    frames[dof].position.every((value, i) => close(value, pose.position[i], 1e-5)));
  check(`${arm.id()}: pose.matrix has 16 entries`, pose.matrix.length === 16);
  check(`${arm.id()}: jacobian has 6 x dof entries`, arm.jacobian(preset).length === 6 * dof);

  const ellipsoids = arm.manipulabilityEllipsoids(preset);
  for (const name of ["linear", "angular"]) {
    const e = ellipsoids[name];
    check(`${arm.id()}: ${name} quaternion is normalised`,
      close(Math.hypot(e.quaternion[0], e.quaternion[1], e.quaternion[2], e.quaternion[3]), 1, 1e-4));
    check(`${arm.id()}: ${name} radii descend and are non-negative`,
      e.radii.length === 3 && e.radii[0] >= e.radii[1] && e.radii[1] >= e.radii[2] && e.radii[2] >= 0,
      JSON.stringify(e.radii));
    check(`${arm.id()}: ${name} volume equals the product of the radii`,
      close(e.volume, e.radii[0] * e.radii[1] * e.radii[2], 1e-5));
  }

  // IK round trip from a perturbed seed.
  const seed = preset.map((angle, i) => angle + (i % 2 === 0 ? 0.15 : -0.12));
  const result = arm.inverse(seed, pose.position, pose.quaternion, {});
  check(`${arm.id()}: IK converges on a reachable pose`, result.converged === true,
    `posErr=${result.positionError} oriErr=${result.orientationError}`);
  check(`${arm.id()}: IK returns dof angles`, result.angles.length === dof);
  check(`${arm.id()}: IK solution respects the joint limits`,
    result.angles.every((angle, i) => angle >= limits[i].lower - 1e-4 && angle <= limits[i].upper + 1e-4),
    JSON.stringify(result.angles));

  // Both step rules must stay legal and terminate.
  for (const method of ["qp", "dls"]) {
    const solved = arm.inverse(seed, pose.position, pose.quaternion, { method });
    check(`${arm.id()}: IK converges with method="${method}"`, solved.converged === true,
      `posErr=${solved.positionError}`);
    check(`${arm.id()}: method="${method}" respects the joint limits`,
      solved.angles.every((angle, i) => angle >= limits[i].lower - 1e-4 && angle <= limits[i].upper + 1e-4));
  }

  const unreachable = arm.inverse(preset, [12, 0, 0], [0, 0, 0, 1], { maxIterations: 40 });
  check(`${arm.id()}: unreachable target terminates with finite angles`,
    unreachable.converged === false && unreachable.iterations <= 40 && unreachable.angles.every(Number.isFinite));
}

async function main() {
  const createKinematicsModule = require(MODULE_PATH);
  const module = await createKinematicsModule();

  console.log("Robot registry:");
  const robots = module.availableRobots();
  check("availableRobots lists ur5 and fr3",
    robots.length === 2 && robots[0].id === "ur5" && robots[1].id === "fr3",
    JSON.stringify(robots));
  check("registry reports the right DOF",
    robots[0].dof === 6 && robots[1].dof === 7, JSON.stringify(robots));
  check("createRobot returns null for an unknown id", module.createRobot("nope") === null);

  const ur5 = module.createRobot("ur5");
  const fr3 = module.createRobot("fr3");
  check("createRobot builds ur5", ur5 !== null);
  check("createRobot builds fr3", fr3 !== null);

  console.log("\nUR5 contract:");
  checkCommonContract(ur5, 6);

  console.log("\nUR5 reference values:");
  const folded = ur5.forward([0, -HALF_PI, 0, 0, HALF_PI, 0]);
  // Modern Robotics, Example 4.5.
  check("theta = (0, -pi/2, 0, 0, pi/2, 0) -> (0.095, 0.109, 0.988)",
    close(folded.position[0], 0.095, 1e-3) &&
    close(folded.position[1], 0.109, 1e-3) &&
    close(folded.position[2], 0.988, 1e-3),
    JSON.stringify(folded.position));
  // At home every UR5 joint axis lies in the y-z plane, so the angular ellipsoid is a disc.
  const ur5Home = ur5.manipulabilityEllipsoids([0, 0, 0, 0, 0, 0]);
  check("UR5 angular ellipsoid collapses to a disc at home",
    close(ur5Home.angular.radii[2], 0, 1e-5) && close(ur5Home.angular.isotropy, 0, 1e-5),
    JSON.stringify(ur5Home.angular.radii));

  console.log("\nFR3 contract:");
  checkCommonContract(fr3, 7);

  console.log("\nFR3 reference values:");
  const zero = fr3.forward([0, 0, 0, 0, 0, 0, 0]);
  check("zero configuration flange -> (0.088, 0, 0.926)",
    close(zero.position[0], 0.088, 1e-4) &&
    close(zero.position[1], 0.0, 1e-4) &&
    close(zero.position[2], 0.926, 1e-4),
    JSON.stringify(zero.position));

  const ready = fr3.forward([0, -QUARTER_PI, 0, -3 * QUARTER_PI, 0, HALF_PI, QUARTER_PI]);
  // Franka documents the ready pose at the gripper; the flange sits 0.1034 m short of it.
  check("ready pose flange x matches Franka's 0.307",
    close(ready.position[0], 0.307, 1e-3), `${ready.position[0]}`);
  check("ready pose flange z is the documented 0.487 plus the hand offset",
    close(ready.position[2], 0.487 + 0.1034, 1e-3), `${ready.position[2]}`);

  // Joint 4 is capped below zero and joint 6 above it, so zeros are unreachable.
  const fr3Limits = fr3.jointLimits();
  check("FR3 joint 4 upper limit is negative", fr3Limits[3].upper < 0, `${fr3Limits[3].upper}`);
  check("FR3 joint 6 lower limit is positive", fr3Limits[5].lower > 0, `${fr3Limits[5].lower}`);
  check("FR3 default configuration is the ready pose, not zeros",
    fr3.defaultConfiguration().some((angle) => Math.abs(angle) > 1e-6));

  ur5.delete();
  fr3.delete();

  console.log("");
  if (failures > 0) {
    console.error(`${failures} check(s) failed.`);
    process.exit(1);
  }
  console.log("All WebAssembly checks passed.");
}

main().catch((error) => {
  console.error(error);
  process.exit(1);
});
