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
mkdir -p build/isaac && chmod 777 build/isaac          # 컨테이너 uid 1234 가 쓸 수 있게
xacro assets/ecr88/urdf/ecr88.urdf.xacro machine_variant:=ECR88_KIJANG model_cylinders:=false \
      model_panel_stack:=false -o build/isaac/ecr88_kijang_step28.urdf
# 자세 계획(백레스트 0.5 m 앞, 스윙 0) -> build/isaac/step28_pose.json : scripts/sil_isaac_step28.py 문서 참고
docker exec isaac-sim-jude /isaac-sim/python.sh /work/xpanner-sim/scripts/urdf_to_usd.py \
      --urdf /work/xpanner-sim/build/isaac/ecr88_kijang_step28.urdf \
      --output /work/xpanner-sim/build/isaac/ecr88_kijang_step28.usd --rest-pose ...
docker exec isaac-sim-jude /isaac-sim/python.sh /work/xpanner-sim/scripts/sil_isaac_step28.py \
      --usd /work/xpanner-sim/build/isaac/ecr88_kijang_step28.usd \
      --pose /work/xpanner-sim/build/isaac/step28_pose.json \
      --report /work/xpanner-sim/build/isaac/step28_report.json
```

2026-09-15 결과: 관절각 ≤ 3e-5°, `angForkUpLimit` 1e-4°, `distUcToForkBack` 0.008 mm.
**1.7 m 변형을 쓰는 이유**: 레포 바이너리가 `ECR88D_ShortArm.m` 으로 컴파일돼 있고, 그 변형의 URDF 가 같은 유닛의
IMU 장착행렬·흡착기 치수를 쓴다. 2.1 m 로 돌리려면 `h.load_imu_mounts("ECR88D_LongArm.m")` 와 LongArm 기하를 `par.*` 에 넣어야 한다.
**4절링크**: URDF 입력링크는 1:1 mimic 자리표시자라, bktImu 는 Isaac 출력링크 각의 실제 4절 역해(`kinematics.fourbar_input`)로 발행한다.
