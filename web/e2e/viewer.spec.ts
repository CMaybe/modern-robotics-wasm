import { expect, test, type Page } from "@playwright/test";

/**
 * End-to-end coverage of the parts unit tests cannot reach: that the WebAssembly
 * module instantiates in a browser, that WebGL renders, and that the layout holds
 * up across the whole WASM-to-WebGL path.
 */

const readoutRows = (page: Page) => page.locator(".readout__row");
const sliders = (page: Page) => page.locator('.slider input[type="range"]');

/** Range inputs ignore fill(); set the value and fire the event React listens for. */
async function setSlider(page: Page, index: number, value: number) {
  await sliders(page)
    .nth(index)
    .evaluate((element, next) => {
      const setter = Object.getOwnPropertyDescriptor(
        window.HTMLInputElement.prototype,
        "value",
      )!.set!;
      setter.call(element, String(next));
      element.dispatchEvent(new Event("input", { bubbles: true }));
    }, value);
}

async function waitForViewer(page: Page) {
  await page.goto("/");
  await page.waitForSelector(".viewer canvas", { timeout: 60_000 });
  // The first frame needs the WASM module and one render pass.
  await expect(readoutRows(page).first()).toContainText("position");
}

test.describe("viewer", () => {
  test("loads the WebAssembly module and renders with WebGL", async ({ page }) => {
    const pageErrors: string[] = [];
    page.on("pageerror", (error) => pageErrors.push(error.message));

    // The vendor meshes are optional, so a 404 for them is the fallback working
    // as designed rather than a failure. Anything else is not expected.
    const missingAssets: string[] = [];
    const consoleErrors: string[] = [];
    page.on("response", (response) => {
      if (response.status() === 404) missingAssets.push(new URL(response.url()).pathname);
    });
    page.on("console", (message) => {
      if (message.type() === "error") consoleErrors.push(message.text());
    });

    await waitForViewer(page);

    await expect(
      page.getByRole("button", { name: /Universal Robots UR5/ }),
    ).toHaveClass(/is-active/);
    await expect(sliders(page)).toHaveCount(6);

    const context = await page.locator(".viewer canvas").evaluate((canvas) => {
      const element = canvas as HTMLCanvasElement;
      return Boolean(element.getContext("webgl2") ?? element.getContext("webgl"));
    });
    expect(context).toBe(true);
    expect(pageErrors).toEqual([]);
    expect(missingAssets.filter((path) => !path.startsWith("/robots/"))).toEqual([]);
    // Only the missing-mesh 404s may have logged, one per failed request.
    expect(consoleErrors.length).toBeLessThanOrEqual(missingAssets.length);
  });

  test("reproduces the Modern Robotics example through the whole stack", async ({ page }) => {
    await waitForViewer(page);
    await page.getByRole("button", { name: "MR Example 4.5", exact: true }).click();

    // Book value for theta = (0, -pi/2, 0, 0, pi/2, 0).
    await expect(readoutRows(page).first()).toContainText("0.0947");
    await expect(readoutRows(page).first()).toContainText("0.1092");
    await expect(readoutRows(page).first()).toContainText("0.9887");
  });

  test("a slider drives forward kinematics", async ({ page }) => {
    await waitForViewer(page);
    const before = await readoutRows(page).first().textContent();

    await setSlider(page, 0, 45);

    await expect(readoutRows(page).first()).not.toHaveText(before ?? "");
  });

  test("switching robots swaps the joint count and the limits", async ({ page }) => {
    await waitForViewer(page);
    await page.getByRole("button", { name: /Franka Research 3/ }).click();

    await expect(
      page.getByRole("button", { name: /Franka Research 3/ }),
    ).toHaveClass(/is-active/);
    await expect(sliders(page)).toHaveCount(7);

    // FR3 joint 4 is capped below zero and joint 6 above it.
    const limits = await sliders(page).evaluateAll((elements) =>
      elements.map((element) => {
        const input = element as HTMLInputElement;
        return { min: Number(input.min), max: Number(input.max), value: Number(input.value) };
      }),
    );
    expect(limits[3].max).toBeLessThan(0);
    expect(limits[5].min).toBeGreaterThan(0);

    // The opening pose must be legal on every joint.
    for (const [index, limit] of limits.entries()) {
      expect(limit.value, `joint ${index + 1}`).toBeGreaterThanOrEqual(limit.min);
      expect(limit.value, `joint ${index + 1}`).toBeLessThanOrEqual(limit.max);
    }
  });

  test("the manipulability ellipsoid collapses at a singular pose", async ({ page }) => {
    await waitForViewer(page);
    await page.getByRole("button", { name: "Angular", exact: true }).click();
    await page.getByRole("button", { name: "Home", exact: true }).click();

    // Every UR5 joint axis lies in the y-z plane at home, so the angular
    // ellipsoid is exactly a disc. Scope to the isotropy row: the manipulability
    // row also warns at this pose, for the same underlying reason.
    const isotropy = page.locator(".readout__row").filter({ hasText: "isotropy" });
    await expect(isotropy.locator(".readout__value--warn")).toContainText("degenerate");
    await expect(isotropy).toContainText("0.0000");
  });

  test("the collision overlay reports clearance and flags a folded elbow", async ({ page }) => {
    await waitForViewer(page);
    await page.getByLabel("Collision capsules").check();

    // The opening pose is legal, so the readout shows a positive clearance.
    await expect(page.getByTestId("clearance")).toContainText("clearance");

    // Elbow at 180 degrees folds the wrist into the base column; the capsule
    // model computed in C++ must call that a self-collision.
    await page.getByRole("button", { name: "Home", exact: true }).click();
    await setSlider(page, 2, 180);
    await expect(page.getByTestId("clearance")).toContainText("Collision:");
  });

  test("the planner finds a path and replays it to the goal", async ({ page }) => {
    await waitForViewer(page);
    await page.getByRole("button", { name: "Add obstacle", exact: true }).click();

    // Store the Modern Robotics pose as the goal, return to Ready, and plan.
    await page.getByRole("button", { name: "MR Example 4.5", exact: true }).click();
    await page.getByRole("button", { name: "Set goal = current", exact: true }).click();
    await page.getByRole("button", { name: "Ready", exact: true }).click();
    await page.getByRole("button", { name: "Plan path", exact: true }).click();

    await expect(page.getByTestId("plan-status")).toContainText("Path found");

    // Replay drives the joints along the path; the readout must settle on the
    // goal pose — the book values for theta = (0, -pi/2, 0, 0, pi/2, 0).
    await expect(readoutRows(page).first()).toContainText("0.0947", { timeout: 15_000 });
    await expect(readoutRows(page).first()).toContainText("0.9887");
  });

  test("the dynamics simulation drops the arm and PD holds it", async ({ page }) => {
    await waitForViewer(page);
    const before = await readoutRows(page).first().textContent();

    // The ellipsoid section also has an "Off" button, so scope to Dynamics.
    const dynamics = page
      .locator(".panel__section")
      .filter({ has: page.getByRole("heading", { name: "Dynamics" }) });

    // Passive: gravity must visibly move the end-effector within a second.
    await dynamics.getByRole("button", { name: "Passive", exact: true }).click();
    await expect(page.getByTestId("sim-status")).toContainText("No actuation");
    await expect(readoutRows(page).first()).not.toHaveText(before ?? "", { timeout: 10_000 });

    // Gravity comp: a preset carries the body itself, and it floats there —
    // the readout must settle on the Modern Robotics pose and hold it.
    await dynamics.getByRole("button", { name: "Gravity comp", exact: true }).click();
    await page.getByRole("button", { name: "MR Example 4.5", exact: true }).click();
    await expect(readoutRows(page).first()).toContainText("0.0947", { timeout: 5_000 });
    await expect(readoutRows(page).first()).toContainText("0.9887");

    // PD hold: the arm chases the preset reference instead of teleporting.
    await dynamics.getByRole("button", { name: "PD hold", exact: true }).click();
    await page.getByRole("button", { name: "Ready", exact: true }).click();
    await expect(page.getByTestId("sim-status")).toContainText("reference");

    await dynamics.getByRole("button", { name: "Off", exact: true }).click();
  });

  test("both IK step rules are selectable", async ({ page }) => {
    await waitForViewer(page);

    for (const label of ["DLS + clamp", "Box QP"]) {
      await page.getByRole("button", { name: label, exact: true }).click();
      await expect(page.locator(`.buttons button.is-active`, { hasText: label })).toBeVisible();
    }
  });

  test("the layout holds at a HiDPI viewport", async ({ page }) => {
    await waitForViewer(page);

    // On a HiDPI display the drawing buffer is larger than the container, so the
    // canvas element must still be sized to the container or the page overflows
    // and the control panel is pushed out of view.
    const layout = await page.evaluate(() => {
      const root = document.documentElement;
      const canvas = document.querySelector(".viewer canvas") as HTMLCanvasElement;
      const viewer = document.querySelector(".viewer") as HTMLElement;
      return {
        overflowX: root.scrollWidth - root.clientWidth,
        overflowY: root.scrollHeight - root.clientHeight,
        canvasWidth: Math.round(canvas.getBoundingClientRect().width),
        containerWidth: Math.round(viewer.getBoundingClientRect().width),
      };
    });

    expect(layout.overflowX).toBe(0);
    expect(layout.overflowY).toBe(0);
    expect(Math.abs(layout.canvasWidth - layout.containerWidth)).toBeLessThanOrEqual(1);

    // Every slider must be inside the viewport.
    const count = await sliders(page).count();
    for (let index = 0; index < count; index += 1) {
      await expect(sliders(page).nth(index)).toBeInViewport();
    }
  });
});
