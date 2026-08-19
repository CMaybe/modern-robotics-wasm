# modern-robotics-wasm — WebGL + WASM FK/IK

C++로 작성한 **Product of Exponentials(PoE)** 기반 정/역기구학을 **WebAssembly**로 컴파일하고,
**WebGL(three.js)** 로 렌더링하는 인터랙티브 예제입니다.
**UR5 (6 DOF)** 와 **Franka Research 3 (7 DOF)** 두 로봇을 지원하며, 화면에서 바로 전환할 수 있습니다.
제조사 공식 URDF의 **실제 메시**로 렌더링합니다 (메시 없이도 도식(schematic) 모드로 동작).

- **슬라이더** 를 움직이면 → 관절값이 바뀌고 C++ `forward()` 가 EE 자세를 계산합니다 (**FK**).
- **EE에 붙은 axis 기즈모** 를 드래그하면 → 목표 자세가 C++ `inverse()` 로 들어가 관절값이 풀립니다 (**IK**).

기구학은 space-frame PoE 공식을 Eigen + Sophus로 구현했습니다. FK · Jacobian에 더해
팔을 그리는 데 필요한 링크 프레임, 조작성 타원체, 관절 리밋을 지키는 IK를 함께 제공합니다.
프론트엔드는 React + webpack + three-stdlib 구성입니다.

> 이 문서는 [README.md](README.md)의 한국어 번역본입니다.

---

## 데모

[![FK/IK 데모 — 이미지를 클릭하면 영상이 재생됩니다](docs/demo-preview.jpg)](docs/demo.mp4)

**이미지를 클릭하면 영상을 볼 수 있습니다** ([docs/demo.mp4](docs/demo.mp4)).

---

## 빠른 시작

```sh
# 1. 개발용 도커 이미지 빌드 (Eigen · Sophus · GoogleTest · Emscripten · Node 20)
docker/build.sh

# 2. 컨테이너 안에서 WASM 빌드 + 프론트엔드 실행
docker/run.sh scripts/dev.sh
```

브라우저에서 <http://localhost:3000> 을 엽니다.

`scripts/dev.sh` 는 WASM을 빌드하고, `web/node_modules` 가 없으면 `npm install` 을 실행한 뒤
webpack dev server를 띄웁니다.

VS Code를 쓴다면 `.devcontainer/devcontainer.json` 으로 **Reopen in Container** 해도 동일합니다.

---

## 조작 방법

| 조작 | 동작 |
| --- | --- |
| 관절 슬라이더 드래그 | **FK** — 관절값 → EE 자세 |
| EE 기즈모의 화살표 드래그 | **IK** — 목표 위치 → 관절값 (Translate 모드) |
| EE 기즈모의 링 드래그 | **IK** — 목표 자세 → 관절값 (Rotate 모드) |
| 빈 공간 드래그 / 휠 | 카메라 회전 · 줌 |
| Presets 버튼 | 대표 자세로 점프 (특이점 자세 포함) |
| Show joint rotation axes | 각 관절의 회전축(청록 화살표) 표시 토글 |
| Manipulability ellipsoid | EE에 그려지는 조작성 타원체 — Linear / Angular / Off |
| Collision capsules | 충돌 모델(캡슐) 오버레이 토글 — 자기충돌 시 빨간색으로 표시, 최소 여유가 mm로 표시됨 |
| IK solver 버튼 | Box QP / DLS + clamp 전환 — 드래그하며 차이를 비교 |
| Robot 버튼 | UR5 (6 DOF) / FR3 (7 DOF) 전환 |
| Display 버튼 | Meshes (제조사 실제 메시) / Schematic (링크·조인트 도식) 전환 |

우측 하단 readout에는 EE 위치·RPY, **manipulability**(0에 가까우면 특이점), 그리고
드래그 중인 IK의 수렴 여부 · 반복 횟수 · 잔차가 실시간으로 표시됩니다.

---

## 디렉터리 구조

```
cpp/
  include/robotics/
    core/types.hpp              # Scalar · Pose · Twist · JointVector · JointLimit
    core/screw.hpp              # ScrewAxis (revolute / prismatic 팩토리)
    core/dh.hpp                 # 수정(Craig) DH 행 → Pose
    kinematics/serial_chain.hpp # SerialChain<Dof> — FK · 링크 프레임 · 관절축 · Jacobian
    kinematics/manipulability.hpp
    collision/shapes.hpp        # Sphere · Capsule + 해석적 signed distance
    collision/robot_geometry.hpp # 링크에 붙는 캡슐 — 체인 프레임에서 자동 유도
    collision/world.hpp         # 장애물 월드 + 자기/장애물 충돌 쿼리
    solvers/box_qp.hpp          # box 제약 QP (projected Gauss–Seidel)
    solvers/inverse_kinematics.hpp
    models/ur5.hpp              # UR5 (6 DOF)
    models/fr3.hpp              # FR3 (7 DOF), DH 표에서 스크류 축 유도
    robot.hpp                   # Robot 인터페이스 + 레지스트리
  src/robot.cpp                 # ChainRobot<Dof> 어댑터 · 카탈로그
  apps/demo/main.cpp            # 네이티브 실행 예제 (gdb 디버깅용)
  bindings/wasm/bindings.cpp    # embind — JS에 노출되는 Robot + createRobot()
  tests/                        # GoogleTest 스위트 (chain · models · collision · solvers · registry)
web/
  src/kinematics/               # WASM 로더 + TypeScript 타입 + URDF 자산 매핑
  src/components/               # RobotViewer · JointSliders · PoseReadout
  e2e/viewer.spec.ts            # Playwright 브라우저 테스트
  public/wasm/                  # 빌드 산출물 (git 제외)
  public/robots/                # 제조사 URDF + 메시 (fetch_meshes.sh, git 제외)
docker/                         # Dockerfile · build.sh · run.sh · docker-compose.yml
scripts/                        # build_native · build_wasm · test · fetch_meshes · dev · ci · e2e
.github/workflows/ci.yaml       # CI
```

---

## 코드 컨벤션

C++20 기준이며, `.clang-format` 과 `.clang-tidy` 가 기계적으로 강제합니다
(`scripts/ci.sh` 가 포맷 검사를 포함).

| 대상 | 규칙 | 예 |
| --- | --- | --- |
| 타입 · 별칭 | `CamelCase` | `SerialChain`, `JointLimit`, `Pose` |
| 함수 · 메서드 | `snake_case` | `space_jacobian()`, `clamp_to_limits()` |
| 변수 · 파라미터 | `snake_case` | `joint_angles`, `error_twist` |
| private 멤버 | `snake_case_` | `joints_`, `end_effector_home_` |
| 상수 · constexpr | `kCamelCase` | `kPi`, `kFr3DhTable` |
| 네임스페이스 | `snake_case` | `robotics::models`, `robotics::ik` |
| 파일 | `snake_case.hpp` | `serial_chain.hpp` |

---

## 스크립트

모두 컨테이너 안에서 실행합니다 (`docker/run.sh <script>`).

| 스크립트 | 설명 |
| --- | --- |
| `scripts/build_native.sh` | 네이티브 C++ 빌드 (example + test) |
| `scripts/test.sh` | 빌드 후 GoogleTest 실행 (`ctest`) |
| `scripts/build_wasm.sh` | Emscripten 빌드 → `web/public/wasm/` 로 복사 |
| `scripts/smoke_wasm.cjs` | Node에서 WASM 바인딩 검증 (브라우저 없이) |
| `scripts/fetch_meshes.sh` | 제조사 URDF·메시 다운로드 → `web/public/robots/` (~32 MB) |
| `scripts/dev.sh` | WASM 빌드 + (필요시) 메시 다운로드 + dev server 실행 |
| `scripts/ci.sh` | CI가 돌리는 전체 검사 (포맷 · C++ 테스트 · WASM · 타입체크 · 빌드) |
| `scripts/e2e.sh` | Playwright 브라우저 테스트 |

```sh
docker/run.sh scripts/ci.sh            # 전체 검사 (CI와 동일)
docker/run.sh scripts/test.sh          # C++ 단위 테스트 실행
docker/run.sh node scripts/smoke_wasm.cjs
```

---

## 기구학 정리

공간 프레임(space frame) PoE 공식을 사용합니다.

$$T(\theta) = e^{[\mathcal{S}_0]\theta_0} \cdots e^{[\mathcal{S}_{n-1}]\theta_{n-1}} M$$

- 스크류 축 $\mathcal{S}_i$ 는 교재(Modern Robotics)의 `[w, v]` 순서로 입력받고,
  내부에서 Sophus가 쓰는 `[v, w]` 순서로 변환합니다 (`toSophusTwist`).
- $M$ 은 모든 관절각이 0일 때의 EE 자세입니다.
- **Jacobian** 은 space Jacobian이며, $J_i = \mathrm{Ad}_{(e^{[\mathcal{S}_0]\theta_0}\cdots e^{[\mathcal{S}_{i-1}]\theta_{i-1}})} \mathcal{S}_i$ 입니다.

### IK — 두 가지 스텝 방식

IK는 반복 해법이고, 이 예제는 사람이 기즈모를 끄는 동안 매 프레임 호출됩니다.
드래그 중에는 목표가 특이점을 지나가거나 작업공간 밖으로 나가는 일이 **정상적으로 자주** 일어나므로,
세 가지가 항상 켜져 있습니다 — `damping`(특이점에서 발산 방지),
`max_step`(프레임 간 점프 방지), `max_iterations`(도달 불가능한 목표에서 종료 보장).

관절 리밋을 다루는 방식은 두 가지를 제공하고 UI에서 전환할 수 있습니다
(`ik::Options::method`, 기본값 `BoxQp`). **둘 다 같은 2차 모델을 최소화**합니다.

$$\min_{\Delta\theta}\ \tfrac12\lVert J\,\Delta\theta - e \rVert^2 + \tfrac12\lambda^2\lVert \Delta\theta \rVert^2,
\qquad e = \log(T_d T^{-1})$$

**`DampedLeastSquares`** — 리밋이 **없는 것처럼** 풀고 나중에 자릅니다.

$$\Delta\theta = (J^\top J + \lambda^2 I)^{-1} J^\top e,
\qquad \theta \leftarrow \mathrm{clip}(\theta + \Delta\theta,\ \theta_{\min},\ \theta_{\max})$$

**`BoxQp`** — 리밋을 스텝의 **제약조건**으로 넣습니다 (box-constrained QP).

$$\text{s.t.}\quad \max(\theta_{\min}-\theta,\ -\delta) \le \Delta\theta \le \min(\theta_{\max}-\theta,\ +\delta)$$

핵심 차이는 **재분배**입니다. 어떤 관절이 리밋에 닿았을 때, 클램프 방식은 그 성분을 그냥 잘라냅니다 —
나머지 관절은 "그 관절이 움직인다"는 가정 하에 계산된 값 그대로라, 남은 자유도로 과제를 넘겨받지 못합니다.
QP는 KKT 조건이 자유 관절들에게 그 몫을 넘기게 만듭니다.
기하학적으로 클램핑은 스텝을 box에 **직교 투영**하는 것이고, QP는 $\lVert\cdot\rVert_H$ (**H-metric**) 으로
투영하는 것입니다. $H = J^\top J + \lambda^2 I$ 가 대각이 아닌 한 (즉 관절이 서로 결합된 한) 둘은 다릅니다.

`max_step` 도 QP에서는 그냥 box가 좁아지는 것뿐입니다. 클램프 방식은 한 성분이 넘치면 **전체 스텝을 축소**해서
관계없는 관절까지 같이 느려지는데, QP는 관절마다 독립적으로 제한됩니다.

QP는 primal **projected Gauss–Seidel** (좌표별 하강 + 구간 투영) 로 풉니다
(`solvers/box_qp.hpp`). $H$ 가 양정치라 최소점이 유일하고 수렴이 보장됩니다.
primal 형태를 쓰면 역행렬을 만들지 않고 할당도 없어서, 마우스를 움직일 때마다 도는 코드에 적합합니다.

#### 실제로 얼마나 다른가

무작위 도달 가능 목표 300개, 리밋이 실제로 걸리도록 좁힌 box 기준:

| | DLS + clamp | Box QP |
| --- | --- | --- |
| UR5 (6 DOF, 대칭 box) | **258/300** (33 it) | 234/300 (39 it) |
| FR3 (7 DOF, 비대칭 리밋) | 128/300 (68 it) | **149/300** (62 it) |
| 1회 solve 비용 | ~10 µs | ~30 µs |

- **스텝 하나만 놓고 보면 QP가 절대 지지 않습니다** — 실현 가능 영역에서의 최소점이므로.
  테스트(`QpStepIsNeverWorseThanClampedDampedLeastSquares`)가 모델 비용으로 이를 확인합니다.
- **리밋이 세게 걸리는 FR3에서 QP가 확실히 유리합니다** (+16 %, 반복도 적음).
- **반대로 box가 넉넉한 UR5에서는 클램프 쪽이 조금 낫습니다.** QP의 스텝은 탐욕적이라
  box 모서리에 확정적으로 들어가 **제약 있는 국소 최소점**에 주저앉는 경우가 있는데,
  클램프의 "틀린" 스텝은 그 지점을 우연히 빠져나오기도 합니다.
  (관측 예: 좁은 리밋 UR5에서 QP가 `posErr = 8.2 mm` 에 고정 — 반복을 3000회로 늘려도 동일,
  damping을 0.2로 올리면 탈출.) 국소법의 성질이지 구현 버그가 아닙니다.
- 비용은 3배지만 절대값이 30 µs라 프레임당 500회 이상 가능 — 인터랙티브에서는 무의미합니다.

기본값을 `BoxQp` 로 둔 이유는 **리밋이 실제로 의미를 갖는 경우에 더 낫기 때문**입니다.
UR5의 실제 리밋은 $\pm 2\pi$ 라 거의 걸리지 않아 두 방식이 사실상 같습니다.

### Manipulability ellipsoid (조작성 타원체)

관절속도 단위구 $\|\dot\theta\| \le 1$ 를 Jacobian으로 보낸 상 (image) 이 조작성 타원체입니다.
주축 방향은 $J$ 의 좌특이벡터, 반지름은 특이값입니다.

한 가지 주의점이 있습니다. **space Jacobian의 선속도 블록은 툴이 아니라 원점에 붙은 점의 속도**입니다.
그래서 EE에 타원체를 그리려면 툴 지점으로 옮겨야 합니다.

$$\dot p = v_s + \omega_s \times p = (J_v - [p]_\times J_\omega)\,\dot\theta$$

각속도는 강체의 모든 점에서 같으므로 $J_\omega$ 는 그대로 씁니다.
`manipulability_ellipsoids()` 가 두 블록을 각각 SVD해서 주축(우수좌표계 보장) · 특이값 ·
`volume`($\sigma_1\sigma_2\sigma_3$) · `isotropy`($\sigma_{\min}/\sigma_{\max}$)를 돌려줍니다.

Home 자세를 눌러보면 UR5의 6개 관절축이 모두 y–z 평면에 놓이기 때문에
**각속도 타원체가 정확히 원판으로 붕괴**하는 것을 볼 수 있습니다 (`isotropy = 0`).
반대로 손목 특이점($\theta_5 = 0$)에서는 나머지 4축이 여전히 3차원을 span하므로
각속도 블록의 rank는 유지되고 타원체가 **납작해지기만** 합니다 — 6D 전체 Jacobian이 rank를 잃는 것과 구별됩니다.

### FR3 — 수정 DH 표에서 스크류 축 유도하기

UR5는 교재에 스크류 축이 표로 나와 있지만, Franka는 **수정(Craig) DH 파라미터**로 기구학을 공개합니다.
손으로 옮겨 적으면 틀리기 쉬워서, DH 표를 영(zero) 자세로 한 번 훑으면서 스크류 축을 유도합니다.

관절 $i$ 는 자기 DH 프레임의 $z$ 축을 중심으로 회전하므로, 베이스 프레임에서

$$\mathcal{S}_i = \begin{bmatrix}\omega_i \\ -\omega_i \times q_i\end{bmatrix},\quad
\omega_i = z_i,\; q_i = o_i$$

이렇게 얻은 축을 그대로 `add_joint_axis()` 에 넣으면 나머지 코드(FK · IK · Jacobian · 타원체)는
DOF만 바뀐 채 그대로 동작합니다.

**주의할 점 — FR3는 영 자세가 관절 리밋 밖입니다.**
관절 4는 상한이 $-0.1518$, 관절 6은 하한이 $+0.5445$ 라서 `q = 0` 은 실제 하드웨어에서 도달할 수 없습니다.
그래서 FR3의 기본 자세는 Franka의 **ready** 자세 $(0, -\pi/4, 0, -3\pi/4, 0, \pi/2, \pi/4)$ 이며,
모든 프리셋이 리밋 안에 있는지 테스트로 강제합니다.

또 하나, Franka 문서의 ready 자세 위치 $(0.307, 0, 0.487)$ 은 **그리퍼(TCP)** 기준입니다.
이 프로젝트는 UR5와 맞추어 **플랜지**를 EE로 쓰므로 $z = 0.5903$ 이 나오고,
Franka Hand의 `F_T_EE`(z로 $0.1034$, z축 $-45°$)를 곱하면 문서 값과 정확히 일치합니다 — 테스트로 확인합니다.

### 제조사 메시 렌더링

`scripts/fetch_meshes.sh` 가 제조사 공식 description 저장소에서 URDF와 visual 메시를 받아
`web/public/robots/` 에 넣습니다. ROS2 xacro는 `ament_index_python` 스텁으로 처리해
**ROS 설치 없이** 평범한 URDF로 변환합니다.

메시 정렬에 별도 오프셋이 **필요 없습니다.** URDF 체인과 이 프로젝트의 PoE 모델이 같은 로봇을
기술하므로, C++이 푼 관절값을 그대로 URDF에 넣으면 렌더된 플랜지가 `forward()` 위치에 정확히 놓입니다.
뷰어는 로드 직후 이 주장을 실제로 측정해서 Display 패널에 표시합니다 — 두 로봇 모두 **0.001 mm 이하**입니다.

> **UR5 링크 길이 주의.** *Modern Robotics* 는 UR5 치수를 소수점 3자리로 반올림해 싣습니다
> (0.109 / 0.082 / 0.392 / 0.089 / 0.095). 그 값을 쓰면 메시가 **0.75 mm** 어긋나므로,
> 이 프로젝트는 제조사 `default_kinematics.yaml` 의 전체 정밀도 값을 씁니다.
> 교재 예제 4.5 는 교재가 실제로 인쇄한 3자리 정밀도로 그대로 일치합니다.

메시는 서드파티 자산이라 저장소에 커밋하지 않고 받아 씁니다. 라이선스는 모두 허용적입니다.

| 로봇 | 출처 | 라이선스 |
| --- | --- | --- |
| UR5 | [Universal_Robots_ROS2_Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description) | BSD-3-Clause |
| FR3 | [franka_description](https://github.com/frankaemika/franka_description) | Apache-2.0 |

다운로드 시 각 저장소의 `LICENSE` 와 커밋 해시가 `web/public/robots/ATTRIBUTION.md` 에 함께 기록됩니다.
FR3는 `hand:=false` 로 생성해 **플랜지**가 EE가 되도록 맞췄습니다.

### 렌더링용 링크 프레임

FK/IK에는 EE 자세만 있으면 되지만, 팔을 그리려면 중간 관절 위치가 필요합니다.
그래서 각 관절이 실어 나르는 프레임의 홈 자세(`JointSpec::link_home`)를 모델에 담고,
`SerialChain::link_poses()` 가 현재 자세를 돌려줍니다.

$$T_i(\theta) = e^{[\mathcal{S}_0]\theta_0} \cdots e^{[\mathcal{S}_i]\theta_i} M_i$$

`linkFrames()` 는 관절 프레임 `DOF`개 + EE 1개, 총 `DOF + 1`개의 자세와
각 관절의 현재 회전축을 함께 돌려줍니다. 뷰어의 모든 좌표는 여기서 나오므로
**JavaScript 쪽에는 기구학 계산이 전혀 없습니다.**

---

## JS ↔ WASM 인터페이스

`cpp/wasm/bindings.cpp` 의 embind 바인딩이며, TypeScript 선언은
`web/src/kinematics/types.ts` 에 있습니다 (embind는 컴파일 타임 검사를 하지 않으므로 둘을 함께 수정해야 합니다).

```ts
const module = await loadKinematics();

module.availableRobots();     // [{ id: "ur5", label, dof: 6 }, { id: "fr3", ..., dof: 7 }]
const arm = module.createRobot("fr3");   // 모르는 id면 null

arm.dof();                    // 7
arm.label();                  // "Franka Research 3"
arm.jointLimits();            // [{ name, lower, upper }, ...]
arm.defaultConfiguration();   // 리밋 안이 보장된 시작 자세
arm.presets();                // [{ label, angles }, ...] — 프리셋도 C++이 소유

// FK
const pose = arm.forward(angles);
// pose.position [x,y,z] · pose.quaternion [x,y,z,w] · pose.matrix (16, column-major)

// 렌더링용 프레임
const { frames, axes } = arm.linkFrames(angles);

// IK
const result = arm.inverse(currentAngles, targetPosition, targetQuaternion, {
  maxIterations: 60,
  damping: 0.02,
  maxStep: 0.25,
});
// result.angles · converged · iterations · positionError · orientationError

arm.manipulability(angles);   // 0에 가까우면 특이점
arm.jacobian(angles);         // 6 x DOF, row-major, [v; w] 순서

// 조작성 타원체 — 단위구에 quaternion을 적용하고 radii로 스케일하면 그대로 타원체가 됩니다
const { linear, angular } = arm.manipulabilityEllipsoids(angles);
// linear.quaternion [x,y,z,w] · linear.radii [a,b,c] · linear.volume · linear.isotropy

arm.delete();                 // embind 객체는 GC되지 않음
```

자세는 `position`/`quaternion` 과 column-major `matrix` 를 모두 반환하므로
`THREE.Matrix4.fromArray()` 에 그대로 넣을 수 있습니다.

---

## 검증

```
$ docker/run.sh scripts/test.sh
100% tests passed, 0 tests failed out of 48
```

테스트는 네 파일로 나뉩니다.

| 파일 | 다루는 것 |
| --- | --- |
| `tests/test_serial_chain.cpp` | 스크류 축, FK, 링크 프레임, 관절축, Jacobian(수치미분 대조), 리밋, 조작성 타원체 |
| `tests/test_models.cpp` | UR5(교재 예제 4.5 · 홈 자세 원판 붕괴 · 손목 특이점), FR3(DH 체인 · Franka ready 자세 · 리밋 · Jacobian) |
| `tests/test_solvers.cpp` | box QP(무제약 일치 · KKT), IK(왕복 · 종료 · 리밋 · 여유자유도 · **QP 스텝이 클램프보다 나쁘지 않음**) |
| `tests/test_robot_registry.cpp` | 레지스트리 생성, 알 수 없는 id, 동적 인터페이스 == 템플릿, 두 방식 모두 리밋 준수 |

특히 값을 검증하는 것들:

- **Modern Robotics 예제 4.5** — $\theta = (0, -\pi/2, 0, 0, \pi/2, 0)$ → $p = (0.095, 0.109, 0.988)$
- **Franka ready 자세** — 플랜지 × Franka Hand `F_T_EE` == 문서의 $(0.307, 0, 0.487)$
- **해석적 Jacobian == 수치 미분** (두 로봇 모두)
- **선속도 타원체가 수치미분으로 샘플링한 툴 속도를 실제로 포함**
- **QP 스텝의 모델 비용 ≤ 클램프된 DLS 스텝** (제약 상황에서는 엄격히 작음)

브라우저 동작은 `web/e2e/viewer.spec.ts` 의 Playwright 테스트가 확인합니다 —
WASM 로드 · WebGL 컨텍스트 · 교재 값 재현 · 슬라이더 FK · 로봇 전환 시 DOF/리밋 ·
특이점 타원체 붕괴 · IK 방식 전환 · **HiDPI 레이아웃 오버플로**.
메시가 없어도 도식 모드로 통과하므로 서드파티 다운로드에 의존하지 않습니다.

WASM 바인딩은 `scripts/smoke_wasm.cjs` 로 Node에서 동일한 항목을 다시 검증합니다.

---

## CI

`.github/workflows/ci.yaml` 이 두 잡을 돌립니다. **CI는 얇게** 유지했습니다 —
개발용 도커 이미지를 빌드하고 스크립트를 부를 뿐이라,
`docker/run.sh scripts/ci.sh` 로 로컬에서 CI를 그대로 재현할 수 있습니다.

| 잡 | 내용 |
| --- | --- |
| `checks` | `scripts/ci.sh` — clang-format · C++ 테스트 · WASM 빌드 + 바인딩 스모크 · TS 타입체크 · 프로덕션 빌드 |
| `e2e` | `scripts/e2e.sh` — Playwright 브라우저 테스트 |

- 이미지는 GitHub Actions 레이어 캐시(`type=gha`)로 캐싱합니다.
  Dockerfile이 바뀔 때만 전체 비용을 냅니다.
- `e2e` 를 분리한 이유는 브라우저가 필요하고 느리기 때문입니다.
  **메시는 일부러 받지 않습니다** — 서드파티 저장소 장애가 빌드를 깨뜨리지 않게 하기 위해서이고,
  뷰어가 도식 모드로 폴백하는 경로 자체가 검증 대상이기도 합니다.
- 이 CI가 특히 값어치를 하는 지점은 **embind 경계**입니다. `bindings.cpp` 와
  `web/src/kinematics/types.ts` 사이에는 컴파일 타임 검사가 전혀 없어서,
  `scripts/smoke_wasm.cjs` 가 돌지 않으면 조용히 어긋납니다.

## 새 로봇 추가하기

기구학은 `Kinematics<DOF>` 템플릿으로 고정 크기 연산을 유지하고,
`RobotBase` 가 DOF를 지운 런타임 인터페이스를 제공합니다.
덕분에 바인딩과 프론트엔드는 로봇마다 고칠 필요가 없습니다.

1. `models/ur5.hpp` (축과 축 위의 점을 직접 지정) 또는 `models/fr3.hpp` (DH 표에서 유도) 를
   본떠 모델 헤더를 만들고, `SerialChain<Dof>` 를 반환하는 팩토리를 하나 씁니다.
2. `src/robot.cpp` 의 `robot_catalog()` 와 `make_robot()` 에 id · 라벨 · 기본 자세 · 프리셋을 등록합니다.
   `ChainRobot<Dof>` 가 템플릿이라 DOF는 자동으로 맞습니다.

이게 전부입니다. `bindings.cpp` · TypeScript · UI는 레지스트리를 그대로 읽으므로 수정할 곳이 없고,
`RobotRegistry` 테스트가 새 로봇의 프리셋과 기본 자세가 리밋 안인지 자동으로 검사합니다.
메시까지 붙이려면 `web/src/kinematics/robotAssets.ts` 에 URDF 경로와 조인트 이름을 추가하세요.

---

## 참고

- Lynch & Park, *Modern Robotics*, Ch. 4 (Forward Kinematics) · Ch. 5 (Velocity Kinematics, manipulability) · Ch. 6 (Inverse Kinematics)
- [Sophus](https://github.com/strasdat/Sophus) — SE(3) / SO(3) Lie group 연산
- [Emscripten](https://emscripten.org/docs/porting/connecting_cpp_and_javascript/embind.html) — embind 바인딩
