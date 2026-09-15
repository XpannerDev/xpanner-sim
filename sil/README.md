# sil/ — X1Exc 펌웨어를 시뮬레이션에 물리는 하네스

`XpannerLab/X1Exc` 의 생성 C(`MdlApp`)를 **이 서버에서 그대로 컴파일해 프로세스 안에서 돌린다.**
Isaac Sim 없이도 상태머신·인히빗·캘리브 진입 같은 펌웨어 로직을 검사할 수 있고, 나중에 Isaac 이
`plant` 훅으로 센서 입력을 채우면 그대로 SIL 이 된다.

설계 근거와 모든 수치의 출처: 서버의 `resources/X1Exc_SIL_spec.md`.

## 쓰는 법

```bash
cd ~/jude/xpanner-sim
python3 sil/build.py                                   # X1Exc 는 읽기만. 결과는 build/sil/
python3 -m unittest discover -s sil/tests -t . -v      # 시나리오 테스트
```

```python
from sil.harness import Harness
h = Harness().reset().nominal_inputs()
h.pulse("u.isSwingAligned")          # 스윙 정렬 근접스위치 (없으면 자동 모드가 부팅부터 막힘)
h.gnss_rtk_fixed()
h.set_target_panel(panel_id=7)
h.tick(3)
h.request_step("Standby")            # autoReqStep 은 값이 '바뀌어야' 발화
h.run_until(lambda h: h.curr_step() == "Standby", timeout_s=0.5)
print(h.describe())
```

신호 이름은 펌웨어 그대로다: `u.*` = `MdlApp_U`(입력), `y.*` = `MdlApp_Y`(출력),
`par.*` = `parLocalTest`(기구학·IMU 장착·속도표). `fw.paths("u.tarPanel")` 로 찾는다.

## 알아야 할 것

- **레포에 든 빌드는 ShortArm(1.7 m)** 이다. `par.parKin.lenArm == 1.7`. LongArm 파라미터 파일은 구 스키마라
  `par.*` 로 전부 옮길 수 없다 (명세 A0.2).
- **32 비트 타깃용 코드**를 64 비트에서 돌린다. 워드 크기 가드만 우회하고 의미는 같다 (`build.py` 문서 참고).
- **프로세스당 펌웨어 하나.** 상태가 C 전역이다. 병렬 실행은 프로세스를 나눌 것.
- `y.autoCtrl_InhibitSts` 의 **비트 1 은 POOR_ACCURACY 가 아니라 "자동 인히빗 전체" 집계**다.
  `h.auto_inhibited()` / `h.inhibit_names()` 가 그렇게 해석한다.
- 정확도 임계값 `u.verticalAccuracyGood/PoorThld` 는 **입력 포트**다. 안 넣으면 GNSS 가 영원히 불량이다.
  `nominal_inputs()` 가 모델 기본값(0.02 / 0.04 m)을 넣는다.
- 파생 플래그는 전부 **1 틱 늦게** 상태머신에 도달한다. 같은 틱 인과를 가정하지 말 것.

## Isaac Sim 에 물리기 — 캘리브 step 28 (첫 SIL 시나리오)

펌웨어는 시뮬이 내보내는 **IMU 만** 보고 자세를 계산한다. step 28 은 동작·밸브·GNSS 없이 포크 기준을 기록하는
순수 FK 읽기라서, IMU 체인과 기하를 가장 싸게 검증한다. 판정 = 펌웨어가 기록한 값 vs 같은 식을 Isaac 정답에 적용한 값.

```bash
cd ~/jude/xpanner-sim
python3 sil/build.py
python3 scripts/sil_step28_prepare.py     # 1.7 m URDF 전개 + 자세 계획 + 아래 두 docker 명령 출력
# 출력된 두 줄(USD 변환, 실행)을 그대로 실행 -> build/isaac/step28_report.json
```

2026-09-15 결과: 관절각 ≤ 3e-5°, `angForkUpLimit` 1e-4°, `distUcToForkBack` 0.008 mm.
**1.7 m 변형을 쓰는 이유**: 레포 바이너리가 `ECR88D_ShortArm.m` 으로 컴파일돼 있고, 그 변형의 URDF 가 같은 유닛의
IMU 장착행렬·흡착기 치수를 쓴다. 2.1 m 로 돌리려면 `h.load_imu_mounts("ECR88D_LongArm.m")` 와 LongArm 기하를 `par.*` 에 넣어야 한다.
**4절링크**: URDF 입력링크는 1:1 mimic 자리표시자라, bktImu 는 Isaac 출력링크 각의 실제 4절 역해(`kinematics.fourbar_input`)로 발행한다.

## Isaac Sim 에 물리기 — 펌웨어가 기계를 움직이는 첫 시나리오 (Picking)

`sil/isaac_plant.py` 의 `IsaacPlant` 는 `KinematicPlant` 와 밸브 모델·센서 발행이 **완전히 같고**, 적분만 Isaac 에 맡긴다:
밸브 → 관절 속도 → articulation **속도 목표** → 물리 10 ms → 관절 상태를 읽어 IMU 발행. 중력·드라이브 한계가 기계에 걸린다.
속도 드라이브 감쇠 `kd` 는 GUESS (붐/암/버킷 1e8, 스윙 1e7, 틸트/로테이터 1e6) — 유량원인 유압축에 가깝게 강한 속도루프로 뒀다.

```bash
python3 scripts/sil_step28_prepare.py      # USD 가 없으면 먼저 (같은 1.7 m 무실린더 USD 를 쓴다)
docker exec isaac-sim-jude /isaac-sim/python.sh /work/xpanner-sim/scripts/sil_isaac_prepare_pick.py \
    --usd /work/xpanner-sim/build/isaac/ecr88_kijang_step28.usd --report /work/xpanner-sim/build/isaac/prepare_pick_report.json
```

시나리오: Standby → Picking 점프 → **PreparePick** (붐/암/링크 관절공간) → **ApproachPanel** (작업공간) 15 초 → 일시정지 2 초 → 정지 비교.

2026-09-15 결과 (3 분, GPU 여유 있음; 마찰·커넥팅로드 수정 후 재생성한 USD 로 재확인):
- PreparePick → ApproachPanel **3.2 초** (틱 320). 이동 붐 9.8° / 암 35.4° / 링크 53.8°, 목표 스트로크 오차 붐 0.16° / 암 0.001° / 링크 0.03° (허용 2/1/1°).
- **일시정지 후 정지 상태**: 펌웨어가 믿는 툴 위치 vs Isaac 실제 위치 **0.16 / 0.05 / 0.10 mm**, 관절 ≤ 0.02°.
- 이동 중 펌웨어 관절 추정은 최대 3.2° 뒤처진다 — **Isaac 탓이 아니다**: 완전 적분 플랜트에서도 3.3° (펌웨어 추정 지연).
- ApproachPanel 목표는 `panelBottom` 이고 `armIn`/`bm1Down` 에 최소출력 유지가 걸린다 (`MdlApp.c:6882-6893, 8380-8402`).
  씬에 패널·컵이 없으니 **15 초 동안 계속 눌러 내려갔다** (툴 z −0.19 → −0.88 m). 컵 4 개 접촉 말고는 멈출 조건이 없다 — 설계 의도인지 David 확인.
- 주행 밸브가 계속 열려 있다 (`trvlRiFwd` 70 %): 하부체 정렬 제어인데 이 플랜트는 하부체를 움직이지 않는다 (플랜트 한계, 펌웨어 결함 아님).

### Positioning — 펌웨어가 상부를 돌린다 (관성이 처음 들어가는 시나리오)

```bash
docker exec isaac-sim-jude /isaac-sim/python.sh /work/xpanner-sim/scripts/sil_isaac_positioning.py \
    --usd /work/xpanner-sim/build/isaac/ecr88_kijang_step28.usd --report /work/xpanner-sim/build/isaac/positioning_report.json
```

`test_valve_plant.test_positioning_raise_swing_and_align_converge` 와 같은 순서: 스윙 0 에서 부팅(근접스위치 래치) → 리모컨 레버 -30 % 3 초로
상부를 ~13° 틀기 → Standby → Auto → Raise(작업공간, 툴을 섀시 위 0.8 m 로) → Swing(스윙·틸트·로테이터 관절공간 → 0) → Align → 일시정지 → 정지 비교.

2026-09-15 결과: 레버 13.0° + 놓은 뒤 **관성으로 0.32° 더** 돎 → Raise 1.85 s (툴 높이 0.809 m ≥ 0.8) → Swing 2.56 s
(최고 15.9°/s, **오버슈트 0**, 끝 0.08°, 허용 1°) → Align 20 초 → 일시정지. 정지 비교 툴 위치 0.01 / 0.02 / 0.18 mm.
정지 시 스윙 추정만 0.05° 차이 — 근접스위치 에지 재래치의 틱 양자화(교차 속도 ~2.5°/s × 약 2 틱)로 보인다 (추정, 미검증).
Align 20 초 뒤 틸트 0.29° (허용 0.5°), 로테이터 -0.58° 로 최소속도(~0.06°/s)로 접근 중 — 운동학 플랜트와 같은 성질.

**🚫 조인트 마찰 = 0 이어야 한다.** URDF `<dynamics friction>` 을 Isaac 임포터가 `physxJoint:jointFriction` 으로 옮기는데, PhysX 에선
**단위 없는 계수(조인트 구속력 × 계수)** 다. 예전 값 10 으로는 집 전체 하중을 받는 스윙이 잠겨, 0.3 rad/s 목표에 1 초 동안 0.06° 움직이고
22.9 kN·m 드라이브가 6 N·m 만 냈다. 붐·암은 움직여서 PreparePick 까지는 안 드러났다. `ecr88_dynamics.xacro` 의 `dyn_friction` 을 0 으로
고쳤고, `IsaacPlant.configure_drives` 가 예전 USD 에서도 구동 조인트 마찰계수를 0 으로 둔다 (보고서 `usd_joint_friction_before_zeroing`).
마찰 0 이 되자 4 절 장식 가지의 **끊긴 부재 `conn_rod_joint`**(드라이브 한계 1 N·m 토큰)가 제 무게(31 N·m)를 못 버티고 진자처럼 흔들려
`check_stability` 가 미수렴 1/10 을 냈다 → `eff_conn_rod` 200 N·m (GUESS, 토크 읽지 말 것). 재생성 후 0/10, step 28 도 ≤0.009 mm 로 그대로.

함정: articulation 에 자세를 순간이동시킬 때 **mimic 조인트(`input_link_joint`)도 같이 옮길 것.** 안 옮기면 mimic 구속이
한 스텝에 끌어당겨 암 −8°, 링크 +26° 가 튄다 (`IsaacPlant.push_pose` 가 처리). step 28 은 위치 드라이브가 되돌려서 안 드러났다.
