# xpanner-sim

Volvo **ECR88** short-swing excavator + Xpanner **X1 PanelLift** front end 의
Isaac Sim asset (URDF → USD) 및 시뮬레이션 리포지토리.

제어 로직은 여기 없다. `XpannerLab/X1Exc` 가 담당하며 두 리포는 ROS 2 토픽·TF 프레임 이름으로만
연결된다 → [docs/ECR88_ASSET_PLAN.md](docs/ECR88_ASSET_PLAN.md) §6

## 현재 상태

URDF 는 **빌드되고 검증을 통과한다.**

```
checks: 18 pass, 0 fail, 1 warn, 1 skip   |   0 hard failures
22 links, 21 joints, single root (base_link), acyclic, 7/7 revolute joints limited
```

체인: `base_link` → house(swing) → boom → arm → 버킷 4절(ground/input/conn_rod/output)
→ tilt_mount → tilt → rotator → attachment → probe → **contact_surface** (TCP).
GNSS 안테나 2개와 유압 실린더 앵커 프레임은 고정 프레임으로 별도 유지.

기하 수치는 전부 실측 기반이다 — `resources/ECR88_kinematic_parameters.md`
(sheet `KinematicPara_new`, **`ECR88 기장` 열**). 링크/조인트 정의에는 `0`·`1` 외의
숫자 리터럴이 하나도 없고, 모든 값은 `ecr88_params.xacro` 의 property 를 거친다.
property 이름은 스프레드시트 이름을 그대로 쓴다 (`lenBm1`, `distChsToBmMnt` …).

### 아직 추정치인 것

**질량·관성·조인트 limit·effort/velocity 는 어떤 원본 문서에도 없다.** 제조사가 공개하지 않고
사내에도 없을 가능성이 높다 (Olivia 확인). 따라서 공개 스펙과 동급 장비 문헌에서 유도한
값을 넣되, **모든 수치에 신뢰도 태그를 단다**:

| 태그 | 의미 |
|---|---|
| `PUBLISHED` | 공개 출처 있음 (URL 필수) |
| `DERIVED` | 위 출처 + 우리 실측 기하로 계산 (계산식 명시) |
| `CLASS-TYPICAL` | 8톤급 굴착기 통상값 |
| `GUESS` | 근거 없음, 자리만 채움 |

근거 문서는 `resources/ECR88_estimated_dynamics.md` (신뢰도 원장 포함),
값 자체는 `assets/ecr88/urdf/` 의 xacro property 에 인라인 주석과 함께 들어간다.
**추정치를 실측값으로 오인하지 말 것.** 더 나은 값이 들어오면 해당 property 만 교체하면 된다.

## TODO

* **메쉬 부재** — `use_meshes:=true` 경로는 검증 불가. 현재는 `LenBottom*`/`LenUpp*`
  바운더리 기반 primitive. Olivia 측 ECR88 CAD 대기.
* **버킷 4절 mimic 계수** (`bkt_mimic_mult`/`bkt_mimic_off`) — URDF `<mimic>` 은 선형이라
  실제 4절의 비선형 관계를 재현하지 못한다. 실린더 스트로크 확보 후 fit 하거나,
  mimic 을 버리고 USD 단계의 loop constraint 로 구동할 것.
* **조인트 limit 확정** — 캘리브레이션 영상(SharePoint `CalibrationVideo/`, 3.03 GB, 미다운로드)
  또는 실린더 bore/stroke 로 교체.
* **센서 구성 미정** — 종류·개수·위치는 Olivia/이찬호 결정 대기.
  현재는 마운트 후보 프레임만 제공 → [ECR88_ASSET_PLAN.md](docs/ECR88_ASSET_PLAN.md) §5.

## 디렉토리 구조

```
xpanner-sim/
├── docs/                 ECR88_ASSET_PLAN.md
├── assets/ecr88/
│   └── urdf/             ecr88.urdf.xacro, ecr88_params.xacro
└── scripts/              urdf_to_usd.py, validate_urdf.py
```

메쉬(`assets/ecr88/meshes/`)와 USD 산출물(`assets/ecr88/usd/`)은 아직 없다 — 위 TODO 참조.

## 빠른 시작

### 1. URDF 빌드 및 검증

```bash
python3 scripts/validate_urdf.py --xacro assets/ecr88/urdf/ecr88.urdf.xacro
```

xacro 확장 → XML 파싱 → 트리 무결성 → inertia 유효성 → zero-pose FK →
파라미터 시트 대조를 한 번에 한다. Isaac Sim / ROS 없이 stdlib(+numpy)만으로 돈다.
`xacro` 바이너리는 `$XACRO` → PATH → python `xacro` 모듈 → 내장 축소 확장기 순으로 찾는다.

중간 URDF 가 필요하면:
```bash
mkdir -p build && xacro assets/ecr88/urdf/ecr88.urdf.xacro > build/ecr88.urdf
python3 scripts/validate_urdf.py --urdf build/ecr88.urdf
```

주요 빌드 플래그 (`ecr88.urdf.xacro` 의 `<xacro:arg>` 가 전부다):

| 플래그 | 기본값 | 설명 |
|---|---|---|
| `use_meshes` | `false` | `true` 면 `assets/ecr88/meshes/*`, `false` 면 primitive (메쉬 미확보) |
| `model_bucket_fourbar` | `true` | 버킷 4절 분기(visual only). 루프를 풀지 않는 물리 실행에서는 `false` 권장 — 툴에 매달린 자유 링크 2개는 solver 부담 |
| `use_ros2_control` | `false` | `true` 면 `<ros2_control>` 블록 방출 |
| `ros2_control_plugin` | `mock_components/GenericSystem` | 위가 `true` 일 때의 하드웨어 플러그인 |
| `mesh_package` | `xpanner_sim` | 메쉬 `package://` 경로의 패키지명 |
| `mesh_dir` | `assets/ecr88/meshes` | 패키지 기준 메쉬 디렉토리 |
| `mesh_visual_ext` / `mesh_collision_ext` | `dae` / `stl` | visual·collision 메쉬 확장자 |

기종 변형(`기장` / `2.1m 미국 #1` / `신규흡착기`)은 CLI 인자가 아니라
`ecr88_params.xacro` 의 `machine_variant` 로 고른다. dual-boom 도 마찬가지로
`cfg_dual_boom_enable`(현재 `false`, `lenBm2 = 0`).

### 2. Isaac Sim USD 변환

Isaac Sim 자체 python 환경에서 실행해야 한다 (`isaacsim` / `omni.*` 가 시스템 python3 에 없음).

```bash
cd ~/isaacsim && ./python.sh /home/ubuntu/jude/xpanner-sim/scripts/urdf_to_usd.py \
    --xacro  /home/ubuntu/jude/xpanner-sim/assets/ecr88/urdf/ecr88.urdf.xacro \
    --output /home/ubuntu/jude/xpanner-sim/assets/ecr88/usd/ecr88.usd
```

변환 후 Articulation Inspector 에서 DOF 수를 확인하고, 각 조인트를 수동 구동해
링크 분리나 자기충돌이 없는지 점검한다.

> **"Merge Fixed Joints" 는 반드시 끈다.** 켜면 `gnss_*_link`, 실린더 앵커,
> `boom_mount_link`, `probe_link`, `contact_surface_link`(TCP) 프레임이 USD 에서 사라진다 —
> 전부 센서 마운트 후보 지점이다.

## 문서

- [ECR88 Asset-ization Plan](docs/ECR88_ASSET_PLAN.md) — 목표, 확보/미확보 자산과 요청처, 파이프라인, 4절 링크, 센서 마운트 후보, 리포 경계, AGX 대비 검토, 액션 아이템
- Ground truth: `resources/ECR88_kinematic_parameters.md` (sheet `KinematicPara_new`, **`ECR88 기장` 열**)
- 측정 절차: `resources/Length_Kinematic_Measurement_ProductionV1.md` (Leica MC1 → X1 PanelLift 변환, 부호 규약)
- 센서·밸브 캘리브레이션: `resources/Sensor_and_Valve_Calibration_ProductionV1.md`
- 추정 동역학 근거: `resources/ECR88_estimated_dynamics.md` (신뢰도 원장)
