import type { KinematicsModule } from "./types";

/**
 * Loads the Emscripten module produced by cpp/wasm.
 *
 * The glue is injected as a classic <script> rather than bundled: emcc emits it
 * with `-sMODULARIZE=1 -sEXPORT_NAME=createKinematicsModule`, and keeping it out
 * of webpack means the artifact the C++ build copies into public/wasm is exactly
 * what the browser runs — no bundler rewriting of its wasm fetch path.
 */

const SCRIPT_URL = new URL("wasm/kinematics.js", window.location.href).toString();
const WASM_DIR = new URL("wasm/", window.location.href).toString();

type ModuleFactory = (overrides?: {
  locateFile?: (path: string, prefix: string) => string;
}) => Promise<KinematicsModule>;

declare global {
  interface Window {
    createKinematicsModule?: ModuleFactory;
  }
}

let scriptPromise: Promise<ModuleFactory> | null = null;
let modulePromise: Promise<KinematicsModule> | null = null;

function loadGlueScript(): Promise<ModuleFactory> {
  if (scriptPromise) {
    return scriptPromise;
  }

  scriptPromise = new Promise<ModuleFactory>((resolve, reject) => {
    if (window.createKinematicsModule) {
      resolve(window.createKinematicsModule);
      return;
    }

    const script = document.createElement("script");
    script.src = SCRIPT_URL;
    script.async = true;
    script.onload = () => {
      if (window.createKinematicsModule) {
        resolve(window.createKinematicsModule);
      } else {
        reject(new Error(`${SCRIPT_URL} loaded but did not define createKinematicsModule`));
      }
    };
    script.onerror = () =>
      reject(
        new Error(
          `Failed to load ${SCRIPT_URL}. Build the WebAssembly module first: scripts/build_wasm.sh`,
        ),
      );
    document.head.appendChild(script);
  });

  return scriptPromise;
}

/**
 * Instantiates the kinematics module once and returns the same instance afterwards.
 * @returns The embind namespace exposing availableRobots() and createRobot().
 */
export async function loadKinematics(): Promise<KinematicsModule> {
  if (modulePromise) {
    return modulePromise;
  }

  modulePromise = loadGlueScript()
    .then((factory) =>
      factory({
        // The glue defaults to resolving kinematics.wasm against the page URL,
        // which breaks on any route other than "/".
        locateFile: (path) => `${WASM_DIR}${path}`,
      }),
    )
    .catch((error: unknown) => {
      // Let a later attempt retry instead of caching the failure forever.
      modulePromise = null;
      scriptPromise = null;
      throw error;
    });

  return modulePromise;
}
