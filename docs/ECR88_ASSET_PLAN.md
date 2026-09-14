# ECR88 Isaac Sim Asset-ization Plan

| | |
|---|---|
| **문서 상태** | Draft v0.2 — 검토 및 결정 요청 (§0.1 에 09-14 진행 상황) |
| **작성일** | 2026-09-11 (갱신 2026-09-14) |
| **작성** | jude |
| **독자** | Olivia (X1 PanelLift product), David / 신흥주 (XpannerLab/X1Exc owner), 이찬호 |
| **대상 장비** | Volvo ECR88 short-swing excavator + Xpanner X1 PanelLift front end |
| **Ground truth** | `resources/ECR88_kinematic_parameters.md` → sheet `KinematicPara_new`, **`ECR88 기장` 열** |

---

## 0. 한 장 요약 — 지금 결정이 필요한 것

| # | 결정 사항 | 결정권자 | 기한 |
|---|---|---|---|
| D1 | ECR88 3D 모델(STEP/FBX) 전달 — 메쉬 없이는 collision이 primitive box 수준에 머무름 | Olivia | 2주 내 |
| D2 | 센서 구성 확정 (카메라/LiDAR **몇 개, 어느 프레임에**) — §5의 후보 프레임 중 선택 | Olivia, 이찬호 | Teams call |
| D3 | Joint limit 확보 경로 — 캘리브레이션 영상 vs 실린더 스트로크 실측 vs X1Exc 소스 상수 | David | 2주 내 |
| D4 | 리포지토리 경계 — `xpanner-sim`(asset/sim) ↔ `X1Exc`(제어) 분리안 승인 | David | 2주 내 |
| D5 | AGX Dynamics 30일 체험 착수 시점 — 지금 vs 굴착(trenching) 단계 진입 시 | 팀 합의 | 열린 상태 |

### 0.1 진행 상황 — 2026-09-14

| # | 상태 | 내용 |
|---|---|---|
| D1 | 대기 | Teo 확인: 상부체 모델만 존재, 소재 정보 없음. **프리미티브가 당분간 정식 경로.** |
| D2 | 후보 확정, 결정 대기 | 1,190 후보 탐색으로 카메라 2대 권고안 도출 (노션 5장). |
| D3 | **경로 확정** | X1Exc 를 끝까지 읽은 결과 **펌웨어에는 관절 limit·실린더 스트로크가 없고, 구조상 있을 수 없다** (각도를 IMU 차분으로 잰다). 캘리브 NVM 까지 확인. → **Teo 의 스트로크 값이 유일한 경로.** 받으면 펌웨어 실린더 기하로 `L(q)` 를 역산해 바로 관절 범위가 된다. |
| D4 | **실행 중** | `xpanner-sim/sil/` 이 X1Exc 생성 C 를 **읽기만 해서** 로컬 빌드한다. X1Exc 에는 아무것도 쓰지 않는다. 경계는 "X1Exc = 소스 오브 트루스, xpanner-sim = 빌드·하네스·플랜트". |
| D5 | 열린 상태 | 변경 없음. |
| **D6 (신규)** | **결정 필요 — David** | 레포의 생성 C 는 **ShortArm(1.7 m, 한국)** 파라미터로 빌드돼 있고, `ECR88D_LongArm.m` 은 구 스키마라 Picking/Positioning 에 필요한 필드가 없다. **미국 2.1 m 장비를 검증하려면 현행 스키마 LongArm 파라미터가 필요.** 현장 장비가 어느 빌드인지도 확인 필요. |
| **D7 (신규)** | **요청 — Olivia/Teo** | 흡착 시뮬레이션용: 진공 매니폴드(어느 컵이 어느 압력센서 회로인지 — 펌웨어·DBC 어디에도 없음), 스윙 정렬 근접스위치 형상(없으면 자동 모드 진입 불가), 버킷 IMU(CAN 0x74)가 4절 입력링크에 붙어 있는지. |

**펌웨어 SIL 현황:** 생성 C 가 이 서버에서 컴파일돼 프로세스 안에서 돈다. 부팅 인히빗, 스윙 초기화 게이트,
NoTarget → Standby 가 명세대로 동작함을 테스트로 확인 (`sil/tests/`). 상태머신·입출력 전체 분석은 서버의
`resources/X1Exc_SIL_spec.md`. 다음은 Isaac 쪽 IMU 퍼블리셔 + 근접스위치를 붙여 **캘리브 step 28** 을 첫 실연결로 돌리는 것.

> **정직성 원칙**: 이 문서와 앞으로 작성될 URDF에서 출처가 없는 수치는 전부 `ESTIMATE` 로 명시하고 근거 가정을 인라인으로 적는다.
> 질량·관성·joint limit은 **현재 확보된 어떤 문서에도 존재하지 않는다.** 그럴듯한 값을 조용히 채워 넣지 않는다.

---

## 1. 목표

ECR88을 **Isaac Sim에서 바로 쓸 수 있는 asset(URDF → USD)** 으로 만든다.
1단계 목표는 물리 정확도가 아니라 **"카메라/LiDAR를 어디에 몇 개 달지 눈으로 보고 결정할 수 있는 상태"** 다.

### 왜 Richard의 30톤급 장비가 아니라 ECR88인가

| 조건 | ECR88 | 30톤급 (Richard) |
|---|---|---|
| 3D 모델 | 보유 (Olivia) | 미확보 |
| 실장비 접근 | 확보 (부산 / 현장) | 불확실 |
| 실측 kinematic 파라미터 | **있음** — Leica MC1 + Total Station 실측, 릴리스 파라미터 셋으로 정리됨 | 없음 |
| 캘리브레이션 절차 문서 | 있음 (Production V1 2종) | 없음 |

세 가지가 동시에 갖춰진 장비는 현재 ECR88뿐이다. 실측 파라미터가 없으면 URDF 원점 값을 전부 추정해야 하고,
그 순간 시뮬레이션 결과는 실장비와 대조 불가능해진다. **ECR88은 "실장비와 숫자를 맞출 수 있는" 유일한 후보다.**
30톤급은 ECR88 파이프라인이 검증된 뒤 동일 절차를 재적용하는 것이 비용이 훨씬 낮다.

### 스코프 밖 (이번 단계에서 하지 않는 것)
- 굴착 반력 / 토양 상호작용 (→ §7, AGX 판단 사항)
- 유압 시스템 모델링 (실린더는 기하학적 부착점만 반영)
- X1Exc 제어 로직 이식 (→ §6)

---

## 2. 확보된 것 / 없는 것

### 2.1 확보 ✅

| 자산 | 내용 | 위치 |
|---|---|---|
| **KinematicPara_new 실측 파라미터** | dist\*/len\*/ang\* 전체 파라미터. ECR88 기장 / 美#1 / 美#1 신규흡착기 3개 사양 열 | `resources/ECR88_kinematic_parameters.md` |
| **old 시트 (hidden)** | Leica 원측정 원장 (DX650 비교 포함) + **바운더리 박스 치수** (LenBottom\*/LenUpp\*) | 동 파일 |
| **측정 절차 문서** | `Length Kinematic Measurement_ProductionV1.pptx` (13p) — MC1 → X1 PanelLift 변환 규칙, 부호 반전 규약 | `resources/Length_Kinematic_Measurement_ProductionV1.md` |
| **캘리브레이션 절차 문서** | `Sensor and Valvle Calibration_ProductionV1.pptx` (19p) — CalibChs~CalibFork 10단계 절차 | `resources/Sensor_and_Valve_Calibration_ProductionV1.md` |
| **캘리브레이션 영상 7종** | Chassis/Boom/Arm/Bucket/Travel/Tilt/Rotator, 총 3.03 GB | SharePoint `.../Kinematic/260801/CalibrationVideo/` (미다운로드) |

**파라미터 신뢰도를 뒷받침하는 내부 정합성 검증** (jude가 수치로 확인함):

```
lenGndLink 검증:
  arm tip(1.7) - distArmToInpLink(1.441, 0, 0.0185)  →  dx=0.259, dz=0.0185
  |gnd| = hypot(0.259, 0.0185) = 0.2597  ==  lenGndLink 0.2597        ✅ 완전 일치
  angle = atan2(0.0185, 0.259) = 4.086°  ≈  angArmToGndLink 4.1°      ✅ 일치

  → distArmToInpLink 의 Z=0.0185 는 "측정 오차"나 "비평면 핀"이 아니라
    ground link 가 arm 축 대비 4.1° 기울어져 있다는 사실의 직접 표현이다.
    (Length Measurement 덱 요약은 이를 "planar 가정 시 18.5mm 오차"로 경고했으나,
     실제로는 angArmToGndLink 와 동일 정보의 중복 표현이다.)

4절 링크 조립 가능성:
  links = [gnd 0.2597, inp 0.42, conn 0.40, outp 0.33]
  s+l = 0.6797  ≤  p+q = 0.73        → Grashof 조건 만족
  |r1-r2|=0.09 ≤ d=0.2597 ≤ r1+r2=0.75 → 폐합 가능 (coupler 0.40 이 도달 범위 내)  ✅

Attachment 체인 가산 검증:
  distAttToProbe + distProbeToContactSurface = (0.45, 0, -0.836) == distAttToContactSurface  ✅
```

### 2.2 없는 것 ❌ — 항목별 요청처

| 없는 것 | 왜 필요한가 | **누구에게 무엇을 요청** | 대체 수단 (임시) |
|---|---|---|---|
| **질량 / CoG / 관성 텐서** | 동역학 시뮬레이션, 자중 처짐, 굴착 반력 전부 불가 | **Olivia** → ECR88 3D 모델(STEP/Parasolid)과 **재질/밀도 정보**. CAD에서 mass property 추출이 가장 빠름. 불가 시 **David** → Volvo 스펙시트의 링크별 중량 | 링크별 `ESTIMATE` 밀도 기반(강재 7850 kg/m³) box 근사, TODO 블록에 전량 명기 |
| **Joint limit (min/max 각도)** | limit 없으면 IK·충돌 검사·조작 시나리오 전부 무의미 | ~~David → X1Exc 소스 내 각도 clamp 상수~~ **09-14: 펌웨어에 없음이 확정** (clamp 는 붐 q≤0, 암 q≥0 두 개뿐). **Teo** → 실린더 스트로크(retracted/extended) 또는 2D working-range 도면. 없으면 **Olivia** → 실장비에서 각 축 full-stroke 실측 1회 | 캘리브레이션 영상 7종에서 극한 자세 프레임 추출해 역산 (영상 3.03GB, 필요 시점에 선택 다운로드) |
| **3D 메쉬** | 시각적 검증, 정확한 collision, 카메라 FOV 가림 판단 | **Olivia** → ECR88 3D 모델. 포맷 우선순위: STEP > FBX > OBJ. **좌표 원점을 Chs 프레임에 맞춰 주면** 변환 공수 대폭 절감 | LenBottom\*/LenUpp\* 바운더리 수치 기반 primitive box (§3.1) |
| **센서 구성 확정본** | 어떤 센서를 몇 개 어디에 — asset의 최종 형태를 결정 | **Olivia, 이찬호** → §5 후보 프레임 표에 체크. 모델명/FOV/해상도/마운트 브래킷 유무 | 후보 프레임에 empty frame만 배치, 센서는 xacro include로 나중에 부착 |
| **Undercarriage → Chassis 오프셋** | swing 축의 실제 높이. `distUcTo*` 행은 전부 `Delete` 처리됨 | **Olivia** → Uc 기준 Chs 원점 높이 1회 실측 | `LenBottomZ1 = -1.445` (Chs 기준 하부 바운더리 최저점)에서 **지면~Chs ≈ 1.445 m** 로 역산. 하부 박스가 지면에 접한다는 가정 필요 → `ESTIMATE` |
| **조인트 회전축 방향 (`<axis xyz>`)** | 부호가 틀리면 전 자세가 뒤집힘 | 측정 덱에 X(+)/Y(+)/Z(+) 화살표가 **그림으로만** 존재. **Olivia** → 축 방향 그림 1장 또는 원본 pptx 슬라이드 10 | 표준 굴착기 관례(boom/arm/bkt pitch = Y축)로 가정 후 캘리브레이션 영상과 부호 대조 |
| **상부 하우스 높이 (Z 상한)** | 상부 collision box 완성 | LenUppZ1(-0.67)은 원점만 주고 Z 크기는 없음. **Olivia** → 캐빈 상단 높이 | ECR88 전고 스펙 기반 `ESTIMATE` |

### 2.3 문서 간 수치 불일치 — 사용 전 확인 필요

| 항목 | KinematicPara_new (**채택**) | Length Measurement 덱 | old 시트 | 비고 |
|---|---|---|---|---|
| lenBm1 | **3.55** | 3.551 | 3.551 | 릴리스 반올림. 무시 가능(1mm) |
| lenArm | **1.7** | 1.7 | 1.699 | 무시 가능 |
| lenInpLink | **0.42** | 0.423 | 0.423 | 무시 가능(3mm) |
| lenConnRod | **0.4** | 0.402 | 0.402 | 무시 가능 |
| lenGndLink | **0.2597** | 0.259 | 0.259 | 무시 가능 |
| distAntMainToChs | **(0.57, -0.087, -1.416)** | (0.575, -0.08, -1.398) | — | **Z 18mm 차이**. GNSS lever arm이라 heading 정확도에 영향. 릴리스 열 채택하되 David 확인 권장 |
| distAntMainToAntAux | **(0.682, 0.959, 0)** | (0.678, 0.958, 0.001) | — | baseline 길이 4mm 차. 듀얼안테나 heading 기선이므로 확인 권장 |
| distTiltMntToTilt X | **0.28** | 0.28 | 0.221 (LenTRX) | **6cm 차이 — 무시 못 함.** old→new 사이 설계 변경 여부 확인 필요 |
| angTiltMntToTilt | **pi/180\*1.736** | 1.736° | 0.0339 rad = 1.942° | 0.2° 차. new 채택 |
| **lenToCylSmlBkt** | **0.42** | 0.42 | Len1BktX 0.256 에 매핑 | ⚠️ **미해결 모순**. 덱은 "lenGndLink(0.2597)와 같은 값"이라 주장하지만 릴리스 값 0.42는 **lenInpLink와 정확히 일치**. old 시트는 0.256(≈lenGndLink)을 가리킴. 세 문서가 서로 다름 → **버킷 실린더 모델링 전 David 확인 필수** |

> **부호 규약 주의**: Leica MC1 ↔ X1 PanelLift 는 일부 축의 기준 방향이 반대이며 `*(-1)` 로 처리된다.
> (`distAntMainToChs` X/Y/Z 전부, `distChsToBmMnt` Y). **URDF에는 X1 PanelLift 열 값만 사용**하고 Leica 원값은 쓰지 않는다.

### 2.4 URDF에 넣지 않는 행

`처리구분 = Delete` → **URDF 진입 금지**: `distUcToFork`, `distForkToPanelBottom`, `distUcToForkUpRef`,
`distUcToForkUpPnt1`, `distUcToForkDownPnt2`, `distUcToForkDownPnt3`, `angAttToContactSurface`

`처리구분 = Update after Calibration` → **하드코딩 금지, xacro 파라미터로 외부화**:
`distUcToForkBack`, `distForkBackToPanelTop`, `angForkUpLimit`
→ `assets/ecr88/config/calibration_params.yaml` 로 분리하여 캘리브레이션마다 값만 교체 가능하게 한다.

---

## 3. 파이프라인

```
  [1] xacro  ──►  [2] URDF  ──►  [3] USD  ──►  [4] 센서 부착  ──►  [5] ROS 2 브리지
   파라미터화      평탄화 XML     Isaac 임포트     카메라/LiDAR       토픽 입출력
```

| 단계 | 도구 | 산출물 경로 | 검증 방법 |
|---|---|---|---|
| **[1] xacro** | `xacro` (ROS 2) | `assets/ecr88/urdf/ecr88.urdf.xacro`<br>`assets/ecr88/urdf/ecr88_params.xacro`<br>`assets/ecr88/urdf/ecr88_macros.xacro`<br>`assets/ecr88/urdf/ecr88_sensors.xacro` | 파라미터 값이 `KinematicPara_new` 표와 **1:1로 대조**되는지 스크립트 검사 |
| **[2] URDF** | `xacro ecr88.urdf.xacro > ecr88.urdf` | `build/ecr88.urdf` | `scripts/validate_urdf.py` — ① XML 파싱 ② `check_urdf` ③ 트리 무결성(고아 링크/중복 조인트) ④ **FK 검증**: 전 조인트 0도에서 `contact_surface_link` 위치 계산 → 손계산 값과 대조 ⑤ inertia 양정치/삼각부등식 |
| **[3] USD** | `scripts/urdf_to_usd.py` → `isaacsim.asset.importer.urdf` (Isaac Sim 4.x/5.x) | `assets/ecr88/usd/ecr88.usd` | Isaac Sim GUI에서 로드 → Articulation Inspector로 DOF 수 확인 → 각 조인트 수동 구동 시 **자기충돌/링크 분리 없음** 확인 |
| **[4] 센서** | Isaac Sim sensor prim (RTX Lidar / Camera) | `assets/ecr88/usd/ecr88_sensors.usd` (레이어 오버레이) | 각 센서 뷰포트에서 **작업 영역이 실제로 보이는지**, 붐/암에 가려지지 않는지 육안 확인 |
| **[5] ROS 2** | Isaac Sim ROS 2 Bridge (Action Graph) | `assets/ecr88/config/ros2_bridge.yaml` | `ros2 topic hz` 로 발행 주기, `tf2_echo base_link contact_surface_link` 로 TCP 프레임 일치 확인 |

### 3.1 메쉬 없이 시작하기 — primitive geometry

메쉬 확보 전까지 collision/visual은 `old` 시트 바운더리 수치로 만든 box를 쓴다. **실측 기반이므로 근거가 있다.**

```
하부 (undercarriage, base_link):
  원점 (LenBottomX1, LenBottomY1, LenBottomZ1) = (-0.44, -1.08, -1.445)   [Chs 프레임 기준]
  크기  X = LenBottomX 2.82,  Y = LenBottomY 2.32,  Z = 0.775  ← ESTIMATE
        (Z는 LenUppZ1(-0.67) - LenBottomZ1(-1.445) = 0.775 로 역산. 하부와 상부가 접한다는 가정)
  범위  X ∈ [-0.44, 2.38],  Y ∈ [-1.08, 1.24]

상부 (house, house_link):
  원점 (LenUppX1, LenUppY1, LenUppZ1) = (-0.41, -1.01, -0.67)
  크기  X = LenUpX 2.28,  Y = LenUpY 2.46,  Z = ESTIMATE (캐빈 상단 높이 미확보)
  범위  X ∈ [-0.41, 1.87],  Y ∈ [-1.01, 1.45]

붐/암/링크: 길이는 실측(lenBm1 3.55, lenArm 1.7 …), 단면은 ESTIMATE 로 가는 원통/박스
```

**메쉬 전환은 플래그 하나로**:
```xml
<xacro:arg name="use_meshes" default="false"/>
<!-- true 로 바꾸면 assets/ecr88/meshes/*.stl 참조, false 면 primitive box -->
```
→ Olivia의 3D 모델이 도착하면 `assets/ecr88/meshes/` 에 넣고 `use_meshes:=true` 만 주면 된다. URDF 구조 변경 불필요.

### 3.2 디렉토리 구조 (계획)

```
xpanner-sim/
├── README.md
├── docs/ECR88_ASSET_PLAN.md            ← 이 문서
├── assets/ecr88/
│   ├── urdf/                           ← xacro 소스 (ecr88_params.xacro 외)
│   ├── config/                         ← joint_limits / calibration_params / sensors yaml
│   ├── meshes/                         ← (비어 있음) Olivia 3D 모델 대기
│   └── usd/                            ← 변환 산출물 (git 미추적)
├── scripts/                            ← urdf_to_usd.py, validate_urdf.py
└── resources/ (리포 상위 ../resources)  ← 추출된 원본 문서, 참조 전용
```

---

## 4. 4절 링크(bucket 4-bar) 문제

### 4.1 문제

버킷 링크는 **닫힌 루프(closed kinematic loop)** 다.

```
        arm_link
     ●─────────────────●  ← ground link (0.2597 m, arm 축 대비 4.1°)
     │ inp pin         │ bkt pin
     │                 │
  input_link 0.42   output_link 0.33
     │                 │
     ●───── conn_rod ──●
            0.40 m
```

**URDF는 트리 구조만 표현할 수 있다.** 루프를 닫는 조인트를 URDF 파일 안에 쓸 방법이 없다.
어딘가 한 핀을 끊어서 열린 사슬로 만든 뒤, 그 핀을 **USD 단계에서 다시 닫아야** 한다.

### 4.2 두 가지 접근

#### 옵션 A — USD 단계 loop closure joint (물리적으로 정확)

conn_rod ↔ output_link 핀을 끊고 URDF를 열린 사슬로 임포트한 뒤,
USD에서 그 자리에 **D6 joint**(또는 축 하나만 푼 revolute)를 추가해 루프를 닫는다.

```python
# scripts/urdf_to_usd.py 내 후처리 (개념)
#  - 6 DOF 중 1개 회전축만 자유, 나머지 5개 lock
#  - articulation root 바깥의 loop joint 로 선언해야 solver가 처리 가능
```

| 장점 | 단점 |
|---|---|
| 실제 링크 간 **하중/힘 전달**이 물리적으로 재현됨 | 임포트 후 **파이썬 후처리 스크립트 필수** — URDF만으로 완결되지 않음 |
| 실린더 힘 → 버킷 토크 변환이 자동으로 맞음 | solver 안정성 튜닝 필요 (loop는 수렴이 까다로움, TGS solver / iteration 증가) |
| 굴착 반력 시뮬레이션의 **전제 조건** | 조립 초기 자세가 루프 구속과 불일치하면 임포트 순간 링크가 튐 → 초기 자세를 §2.1 폐합 계산으로 정확히 맞춰야 함 |

#### 옵션 B — mimic joint 근사 (기하학적으로만 정확)

루프를 아예 만들지 않고, output_link 각도를 input_link 각도의 함수로 **종속 구동**한다.

| 장점 | 단점 |
|---|---|
| URDF 하나로 완결, 임포트가 단순하고 안정적 | **힘 전달이 물리적으로 틀림** — 링크는 "따라 움직이는 장식"이 됨 |
| 시각/기하 검증에는 충분 (핀 위치·작업 반경 정확) | `mimic`은 **선형 관계(multiplier/offset)** 만 표현. 실제 4절은 **비선형** → 전 구동 범위에서 오차 발생 |
| solver 부담 없음, 실시간성 좋음 | Isaac Sim URDF importer의 mimic 지원은 버전별 편차가 있음 → **임포트 직후 실제 동작 확인 필수** |

> 비선형 오차는 구동 범위 중앙 근처에서 multiplier를 선형화하면 작게 유지되지만,
> 극한 자세에서는 수 도(°) 단위로 벌어진다. **정량 오차는 joint limit이 확정되어야 계산 가능**하다.

### 4.3 권고 — 단계 분리

| 단계 | 방식 | 근거 |
|---|---|---|
| **Phase 1 (지금)** | **옵션 B — mimic 근사** | 1단계 목표는 **센서 위치 결정**이다. 카메라/LiDAR 시야 판단에 필요한 것은 링크의 **기하학적 위치**뿐이고, mimic은 그것을 정확히 준다. 루프 solver 튜닝에 시간을 쓸 이유가 없다. |
| **Phase 2 (굴착 진입 시)** | **옵션 A — D6 loop closure** | 반력·하중이 개입하는 순간 mimic은 무효. 이때 `scripts/urdf_to_usd.py` 에 후처리를 추가한다. |

구조적으로는 **두 방식을 xacro 플래그로 동시 지원**한다:
```xml
<xacro:arg name="linkage_mode" default="mimic"/>   <!-- mimic | open_loop -->
```
`open_loop` 로 빌드하면 핀이 끊긴 URDF가 나오고, `scripts/urdf_to_usd.py` 의 후처리가 D6로 닫는다. Phase 2 전환 시 재작성 불필요.

---

## 5. 센서 부착 계획 — **Olivia / 이찬호 결정용**

> **이 절이 "1단계: 카메라/라이다 위치를 결정할 수 있게 한다"에 해당한다.**
> 아래 프레임들은 URDF에 **빈 frame으로 미리 배치**된다. 센서를 붙이는 건 xacro 한 줄이다.
> 표에 원하는 위치를 체크하고 모델명만 알려주면 그 구성으로 빌드한다.

### 5.1 후보 마운트 프레임

| 후보 프레임 (URDF 이름) | 물리적 위치 | 적합 센서 | 장점 | 유의점 | 선택 |
|---|---|---|---|---|---|
| `house_link` 상단 전방 | 캐빈 지붕 앞쪽 | 광각 카메라, LiDAR | 작업 영역 전체 조망. 붐 하강 시에도 시야 확보 | 붐 실린더가 하단 시야 일부 가림 | ☐ |
| `house_link` 상단 후방 | 카운터웨이트 위 | LiDAR (360°) | 선회 전 주변 감시, 후방 사각 해소 | 장비 자체 자기가림 최대 | ☐ |
| `house_link` 좌/우 측면 | 캐빈 측면 | 스테레오 카메라 쌍 | 패널 파지 시 측면 정렬 관측 | 선회와 함께 회전 → 정지좌표 변환 필요 | ☐ |
| `boom_link` 측면 (근위) | 붐 뿌리 쪽 측면 | 카메라 | 붐 각도와 무관하게 암/버킷 추적 | 붐 진동 직접 전달 | ☐ |
| `boom_link` 측면 (원위) | 붐 끝 쪽 측면 | 카메라, 소형 LiDAR | 작업점에 가까워 해상도 유리 | 진동·충격 가장 큼, 케이블 라우팅 난이도 | ☐ |
| `arm_link` 끝단 | 암 선단 (버킷 핀 근처) | 근접 카메라, ToF | **패널 접촉면 직시** — PanelLift 정밀 작업에 최적 | 작업물에 의한 가림, 파손 위험 | ☐ |
| `attachment_link` | 틸트로테이터 하단 회전체 | 카메라, ToF | 어태치먼트와 함께 회전 → 항상 툴 기준 관측 | 회전 슬립링/케이블 꼬임 | ☐ |
| `contact_surface_link` (TCP) | 흡착 접촉면 | 근접·압력 센서 | TCP 기준 관측 | 접촉면이라 물리적 공간 거의 없음 | ☐ |

### 5.2 이미 파라미터가 확정된 센서 프레임 (즉시 URDF 반영)

| 프레임 | parent | origin xyz | 근거 |
|---|---|---|---|
| `gnss_main_link` | `house_link` | **(-0.57, 0.087, 1.416)** | `distAntMainToChs` (0.57, -0.087, -1.416) 의 **역변환**. 표 값은 Ant→Chs 방향이므로 URDF(Chs→Ant)에서는 부호 반전 |
| `gnss_aux_link` | `gnss_main_link` | (0.682, 0.959, 0) | `distAntMainToAntAux` — 듀얼안테나 heading 기선 |
| `probe_link` | `attachment_link` | (0.35, 0, -0.572) | `distAttToProbe` |
| `contact_surface_link` | `probe_link` | (0.1, 0, -0.264) | `distProbeToContactSurface` — **TCP로 발행 권장** |

### 5.3 문서에 존재하지만 마운트 정보가 없는 센서

| 센서 | 문서상 언급 | 부족한 것 |
|---|---|---|
| BoomSwing 센서 | Slide 2 "Sensor on BmMnt, Enable/Disable". `distBmMntToBm1`이 이 옵션 때문에 존재 | 현 사양에서 (0,0,0) = **미장착**. 장착 여부 확인 필요 |
| Bkt IMU | Slide 2 "IMU sensor: On Link / On Bkt" — **선택식** | 어느 쪽인지, 오프셋/자세 전부 미기재 → **이찬호 확인** |
| 차체 Roll/Pitch 소스 | 캘리브레이션 전 단계마다 "Roll = 0°, Pitch ＞ 5°" 를 읽음 | **어떤 장치가 읽는지 문서에 없음.** 경사계/IMU 추정되나 단정 금지 |
| Rotator Roll/Pitch 소스 | Slide 14 "Set Rotator's Roll angle = 0° & Pitch angle = 0°" | 동일 — 장치명 없음 |
| 각 링크 각도 센서 | CalibChs~CalibRot 10단계가 **모든 링크**를 캘리브레이션 | 절차만 있고 **센서 장착 위치·모델 전무.** Isaac Sim용 추론 금지 |

> 즉 캘리브레이션 문서는 "무엇을 보정하는가"는 말하지만 **"무엇으로 재는가"는 말하지 않는다.**
> 실장비 사진 1회 또는 이찬호 확인으로 메울 수 있는 공백이다.

---

## 6. 리포지토리 경계

### 6.1 현황 — 실제 확인됨

`XpannerLab/X1Exc` 는 현재 **클론 불가**하다. WASM 기반 접근만 열려 있고 **git 인증이 실패**한다 (실제 시도 확인).
따라서 X1Exc 소스를 직접 참조하는 구조는 **지금 성립하지 않는다.**

### 6.2 제안 — 2-리포 분리

| 리포 | 역할 | 담당 | 의존 방향 |
|---|---|---|---|
| **`xpanner-sim`** (이 리포) | Asset + 시뮬레이션 전용. URDF/xacro, USD 변환 스크립트, 센서 구성, Isaac Sim 씬 | jude | X1Exc를 **참조하지 않음** |
| **`XpannerLab/X1Exc`** | 실장비 제어 로직, 캘리브레이션, 패널리프팅 시퀀스 | David / 신흥주 | sim 결과를 **소비**하는 쪽 |

**분리 근거**
1. 라이프사이클이 다르다 — asset은 장비 형상이 바뀔 때만, 제어 로직은 매일 바뀐다.
2. 접근 권한이 다르다 — 시뮬레이션 협업자에게 제어 소스 권한까지 줄 필요가 없다. (현재 애초에 불가)
3. 의존을 단방향으로 유지해야 asset이 제어 구현에 오염되지 않는다.

**두 리포의 계약(contract)은 코드가 아니라 인터페이스로 둔다**: ROS 2 토픽/TF 프레임 이름, joint 이름, TCP 프레임 정의.
이 이름들만 합의되면 각자 독립 개발이 가능하다. → **D4 승인 요청.**

### 6.3 X1Exc에서 필요한 것 (코드가 아니라 값)

클론이 안 되어도 아래 세 가지만 **값으로** 받으면 URDF는 완성된다:
1. 각 조인트 각도 **min/max clamp 상수**
2. 실린더 **스트로크 길이** (retracted / extended)
3. 제어기가 쓰는 **joint 이름 / 프레임 이름** 규약 (sim과 일치시키기 위함)

---

## 7. AGX Dynamics (openPLX) vs Isaac Sim — **열린 결정 사항**

미팅 요약 기준 정리이며, **어느 쪽으로도 확정하지 않는다.**

| | Isaac Sim | AGX Dynamics (openPLX) |
|---|---|---|
| 비용 | 현재 사용 중, 추가 비용 없음 | **8,000 ~ 9,000 EUR/년**, **30일 체험 가능** |
| 강점 | RTX 센서 시뮬레이션(카메라/LiDAR) 품질, ROS 2 브리지, 학습 파이프라인 연결 | **굴착 반력 / 토양 상호작용** — 실제 굴착력 센서 데이터 기반 검증 자산 보유 |
| 약점 | 토양·굴착 물리 약함 | 센서 시뮬레이션/학습 생태계는 상대적으로 약함 |
| 현 단계 적합성 | **1단계(센서 배치 결정)에 정확히 부합** | 1단계에는 과잉. 굴착 단계에서 가치 발생 |

### 현재 정리 (제안이며 확정 아님)

> **Isaac Sim asset화를 먼저 끝내고, AGX 30일 체험은 굴착(trenching) 단계 진입 시점에 판단한다.**

근거:
- 30일 체험은 **한 번뿐인 카드**다. 지금 써 버리면 정작 굴착 물리를 평가해야 할 때 체험이 없다.
- URDF/USD asset은 **두 엔진 모두에서 재사용 가능한 자산**이다. 어느 쪽을 고르든 지금 만드는 asset은 버려지지 않는다.
- 1단계 산출물(센서 배치)은 AGX가 특별히 유리하지 않다.

**남는 질문 (팀 논의 필요)**
- 굴착 반력이 우리 로드맵에서 **언제** 필수가 되는가? (그 시점이 체험 시작 시점)
- 두 엔진 병행 운용의 유지 비용을 감당할 것인가, 한쪽으로 수렴할 것인가?
- AGX의 openPLX가 우리 URDF를 얼마나 손실 없이 받아들이는가? → **체험 기간 중 첫 검증 항목으로 둘 것**

---

## 8. 다음 2주 액션 아이템

### jude
- [ ] `assets/ecr88/urdf/ecr88.urdf.xacro` 작성 — `KinematicPara_new` 기장 열 **전량 반영**, Delete 행 제외
- [ ] `Update after Calibration` 3개 항목을 `assets/ecr88/config/calibration_params.yaml` 로 외부화
- [ ] `scripts/validate_urdf.py` — 파싱/트리/FK/inertia 검증 + 파라미터 표 대조
- [ ] primitive collision box 생성 (LenBottom\*/LenUpp\* 기반), `use_meshes` 플래그 구조 완성
- [ ] Isaac Sim URDF importer로 USD 변환, Articulation DOF 확인
- [ ] mimic 방식 4절 링크 구현 (Phase 1) + `linkage_mode` 플래그 골격
- [ ] 후보 센서 프레임 8종을 빈 frame으로 배치 → **스크린샷으로 Olivia/이찬호에게 제시**
- [ ] 질량/관성 `ESTIMATE` 값과 가정을 파일 상단 TODO 블록에 전량 명기

### Olivia
- [ ] **ECR88 3D 모델 전달** (STEP 우선). 가능하면 원점을 Chs 프레임 기준으로 맞춰서 — 최우선 블로커
- [ ] CAD mass property 추출 가능 여부 회신 (링크별 질량/CoG/관성)
- [ ] §5.1 센서 후보 표 체크 — 어떤 센서 **몇 개, 어느 프레임에**
- [ ] 조인트 회전축 방향 그림 1장 (또는 Length Measurement 원본 pptx **슬라이드 10** — 실린더 기준 자세, 텍스트 추출 불가)
- [ ] Undercarriage → Chassis 원점 높이 실측 (현장 접근 시 1회)

### David / 신흥주
- [ ] X1Exc 내 **joint 각도 min/max clamp 상수** 공유 (코드 아닌 값으로 가능)
- [ ] 실린더 **스트로크 길이** (retracted / extended) 공유
- [ ] §2.3 **`lenToCylSmlBkt` 0.42 vs 0.2597 vs 0.256 모순 판정** — 버킷 링크 모델링 블로커
- [ ] `distTiltMntToTilt` X: old 0.221 → new 0.28, 설계 변경인지 확인
- [ ] §6.2 리포지토리 분리안 승인 / 조인트·프레임 이름 규약 합의

### 이찬호
- [ ] Bkt IMU 실제 장착 위치 확인 (**On Link / On Bkt** 중 어느 쪽)
- [ ] 차체 및 rotator Roll/Pitch를 **어떤 장치가** 읽는지 확인
- [ ] 각 링크 각도 센서 장착 위치 — 실장비 사진으로 대체 가능
- [ ] §5.1 센서 후보 표 Olivia와 공동 검토

---

## 9. 부산 방문 / 미팅 메모

### 일정 제약
- **Olivia**: 차주 **현장 일정** → 이어서 **텍사스 출장**. **대면은 10월 2주차부터 가능.**
- **Teams call은 그 전에도 가능.**

### 제안

> **차주 부산 방문을 10월로 미루고, 대신 Teams call로 센서 스펙만 먼저 확정한다.**

근거:
- 지금 방문해도 **3D 모델과 센서 결정 없이는 현장에서 할 수 있는 일이 거의 없다.** 실측은 이미 파라미터 표로 확보되어 있다.
- 반대로 **센서 스펙 확정은 화면 공유로 충분하다** — §5.1 표에 체크하는 작업이다.
- 2주 뒤면 primitive 기반 asset이 Isaac Sim에서 움직인다. **그 화면을 띄워 놓고 통화**하면 "여기에 카메라를 달면 이게 보인다"를 즉시 확인할 수 있고, 결정 품질이 훨씬 높다.
- 10월 대면은 **결정된 센서를 실장비에 대조하고 미확보 실측치(Uc→Chs 높이, 하우스 높이, 축 방향)를 한 번에 처리**하는 자리로 쓴다.

### Teams call 안건 (30~40분)
1. §5.1 센서 후보 프레임 표 — 체크 (15분)
2. 3D 모델 전달 경로/포맷 합의 (5분)
3. §2.3 수치 불일치 3건 판정 — 특히 `lenToCylSmlBkt` (10분)
4. 10월 부산 방문 시 실측 항목 리스트 확정 (5분)

### 10월 부산 방문 시 처리할 것 (미리 준비)
- [ ] Uc → Chs 원점 높이 실측
- [ ] 상부 하우스 상단 높이 실측 (collision box Z 상한)
- [ ] 각 조인트 회전축 부호 확인 (실기 동작 ↔ 시뮬레이션 대조)
- [ ] 센서 후보 위치 실물 확인 — 브래킷 공간, 케이블 라우팅 가능 여부
- [ ] 각 축 full-stroke 실측 (joint limit 확보의 최후 수단)

---

## 부록 A. URDF 링크/조인트 트리 (설계안)

```
base_link  (undercarriage)
 └─ swing_joint            [revolute, Z]        ← 원점 높이 ESTIMATE (§2.2)
    └─ house_link  (Chs)
       ├─ gnss_main_link   [fixed]  (-0.57, 0.087, 1.416)      ← distAntMainToChs 역변환
       │   └─ gnss_aux_link[fixed]  (0.682, 0.959, 0)
       └─ boom_mount_joint [fixed]  (0.94, -0.15, 0)           ← distChsToBmMnt
          └─ boom_mount_link (BmMnt)      ※ BoomSwing 옵션 시 revolute(Z)로 승격
             └─ boom1_joint [revolute]  (0, 0, 0)              ← distBmMntToBm1
                └─ boom1_link
                   │  ※ boom2: lenBm2 = 0 → use_dual_boom=false 시 생성 안 함
                   └─ arm_joint [revolute]  (3.55, 0, 0)       ← lenBm1
                      └─ arm_link
                         └─ inp_link_joint [revolute] (1.441, 0, 0.0185)  ← distArmToInpLink
                            └─ input_link
                               └─ conn_rod_joint [revolute] (0.42, 0, 0)  ← lenInpLink
                                  └─ conn_rod_link
                                     └─ output_joint [revolute] (0.4, 0, 0)  ← lenConnRod
                                        │   ※ 여기가 루프 절단점 (§4)
                                        └─ output_link  (0.33 = lenOutpLink)
                                           └─ tilt_mount_joint [fixed] (0,0,0)  ← distOutpLinkToTiltMnt
                                              └─ tilt_mount_link
                                                 └─ tilt_joint [revolute]
                                                    │  (0.28, 0, -0.22), rpy pitch = pi/180*1.736
                                                    └─ tilt_link
                                                       └─ rotator_joint [continuous, Z]
                                                          │  (0,0,0)  ※ "Tilt = Rot" 동일점
                                                          └─ rotator_link
                                                             └─ att_joint [fixed] (0,0,-0.2615)
                                                                └─ attachment_link
                                                                   └─ probe_joint [fixed] (0.35,0,-0.572)
                                                                      └─ probe_link
                                                                         └─ tcp_joint [fixed] (0.1,0,-0.264)
                                                                            └─ contact_surface_link  ★ TCP
```

**루프 폐합용 ground link** (§4): `arm_link` 의 inp 핀(1.441, 0, 0.0185) ↔ 버킷 핀(1.7, 0, 0),
길이 0.2597, arm 축 대비 4.1° — `lenGndLink` / `angArmToGndLink` 와 **완전 일치 검증됨**.

**미확정 항목 (전부 `ESTIMATE` 또는 TODO로 표기)**
- 모든 `<axis xyz>` 방향 — 문서에 그림으로만 존재
- 모든 `<limit lower upper effort velocity>` — 어떤 문서에도 없음
- 모든 `<inertial>` (mass / origin / inertia) — 어떤 문서에도 없음
- `swing_joint` 원점 Z (Uc→Chs)
- 상부 하우스 collision box Z 크기

---

## 부록 B. 참조 문서

| 문서 | 경로 | 비고 |
|---|---|---|
| Kinematic 파라미터 (ground truth) | `resources/ECR88_kinematic_parameters.md` | `KinematicPara_new` / `ECR88 기장` 열 사용 |
| Length Kinematic Measurement V1 | `resources/Length_Kinematic_Measurement_ProductionV1.md` | 13p 전량 추출. **슬라이드 10은 전부 그림 → 텍스트 0** |
| Sensor & Valve Calibration V1 | `resources/Sensor_and_Valve_Calibration_ProductionV1.md` | 19p 전량 추출. 절차만 있고 센서 장착 정보 없음 |
| 캘리브레이션 영상 7종 (3.03 GB) | SharePoint `.../Kinematic/260801/CalibrationVideo/` | 미다운로드. **joint limit 역산이 필요해지는 시점에 선택 취득** |

> 두 pptx는 **텍스트 추출본**이며 원본 바이너리가 아니다. 슬라이드의 약 80%가 CAD 도면/사진이고
> 치수 라벨이 순서 없는 토큰으로 추출되어 **화살표 방향과 행/열 대응이 소실**되었다.
> 축 방향처럼 그림에만 있는 정보는 원본을 열어야 한다.
