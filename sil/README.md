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
