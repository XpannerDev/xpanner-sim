# X1Exc 펌웨어 확인 사항 — SIL 에서 나온 것 (2026-09-15)

대상: `XpannerLab/X1Exc` 레포의 생성 C (`Asw/GeneratedCode/MdlApp_ert_rtw/MdlApp.c`, ShortArm 빌드) 와 ECU 래퍼.
방법: 펌웨어를 호스트에서 그대로 컴파일해 `sil/` 하네스로 10 ms 락스텝 실행 (테스트 310 개), 일부는 Isaac Sim 물리 위에서 폐루프.
모든 항목은 코드 위치를 달았고, 대부분은 재현 테스트가 있다 (`python3 -m unittest sil.tests.<모듈>.<클래스>.<테스트>`).
각 발견은 수정자 → 적대적 검증자 두 단계를 거쳤고, 검증자가 분류를 바꾼 곳은 바뀐 쪽으로 적었다.

**이 문서는 "결함 확정" 목록이 아니다.** 의도된 설계일 수 있는 것은 질문으로 적었다. 분류:

| 분류 | 뜻 |
|---|---|
| **빌드 설정** | 소스가 아니라 이 레포 빌드의 설정 때문에 생기는 것 |
| **펌웨어 결함** | 소스 수준, 플랜트와 무관하게 재현. 의도 확인 필요 |
| **이 빌드에선 잠재** | 소스 결함이지만 이 빌드 설정(EnTestPar)에선 런타임에 영향 없음. 필드 빌드 설정에 따라 살아남 |
| **절차 의존** | 오퍼레이터 절차가 결과를 좌우하는데 펌웨어가 확인하지 않음 |
| **플랜트 의존** | 시뮬 장비의 가정(IMU 노이즈, 밸브 곡선)에 따라 달라짐. **실기 동작이라고 말하지 말 것** |

---

## 0. 가장 먼저 물어볼 것 — `EnTestPar = true` (빌드 설정)

`ControlModel/Data/SysPar.m:5` `EnTestPar = true;` → `<S1>/Switch6~17` 이 전부 `parLocalTest` 쪽으로 상수 선택되고
Coder 가 `*Stored` 인포트를 접어 없앴다 (예: `MdlApp.c:41963` "Switch generated from: '<S1>/Switch12'" → `parLocalTest.imuTilt`).

해당 스위치: Switch6 `parKinStored`, Switch7~12 섀시/붐1/붐2/암/링크/틸트 IMU 장착행렬, Switch16 `jntAngRotZeroOffsStored`, Switch17 `tblReqSpdToActCmdStored`.

결과: **이 빌드는 캘리브레이션 결과를 런타임에 하나도 쓰지 않는다.** `AppCtrlIf.c` 가 부팅 때 NVM 을 `*Stored` 인포트로
복원하지만(장착행렬 `:565ff`/`:630-638`, 속도표 `:432ff`, 로테이터 영점 `:244`) 모델이 읽지 않는다.
캘리브를 돌리면 NVM 이미지만 바뀌고, 장비는 계속 컴파일된 값으로 돈다.
테스트: `test_harness_review` … `test_par_patch_reaches_firmware_and_stored_inport_does_not`, `test_calib_tilt_plant` (NVM 인포트를 채워도 틸트가 10° 틀린 채).

> **질문 (David):** 현장 빌드도 `EnTestPar = true` 인가? 그렇다면 현장 캘리브는 효과가 없다.
> `false` 라면 아래 "이 빌드에선 잠재" 항목이 **전부 살아난다.**

로테이터 영점도 같은 원인이다. CalibRot 이 잰 영점(`-angRefRot`)은 로그 틱 한 번만 출력되고(`MdlApp.c:43359-43361`),
저장 시점엔 live `parLocalTest.jntAngRotZeroOffs` 가 실린다(`:43371`, 제어 경로 `:42032`). 스위치가 제대로 연결되면
`AppCtrlIf.c:882 → :244` 로 래치될 설계다 → **스위치를 고칠 문제지 차트를 고칠 문제가 아니다.**
테스트: `test_calib_rot_plant` (live par 0.3 rad 가 저장되고 측정값 -10° 는 버려짐).

---

## 1. 펌웨어 결함 (소스 수준, 의도 확인 필요)

### 캘리브레이션

| # | 내용 | 근거 | 테스트 |
|---|---|---|---|
| C1 | **`_Min` 상태(최소 명령 탐색)에 타임아웃이 없다.** 0.5° 움직임이 보일 때까지 명령이 2 초마다 0.2 %씩 100 %까지 오른다. 축이 스톱에 걸렸거나 유압이 꺼졌거나 IMU 가 멈추면 오퍼레이터가 멈출 때까지 밸브를 연다. ChsLe/RiMin, ArmIn/OutMin, LinkIn/OutMin, RotPosi/NegaMin, Trvl*Min 전부 | chart_1210 가드 `[isMotionOnset]` 뿐, `PropVlvCmdMinCalib_limit` 100 | 소스 확인 (800 s 머신타임이라 전 구간 실행은 안 함) |
| C2 | **`_ToPnt` 구간(각도 도달 구간)의 10 초 타임아웃을 각도 도달과 똑같이 취급한다.** 알람·플래그 없이 정상 저장. 조인트 스톱에 걸려 끝나도 같다. 섀시는 Le 구간이 80° 를 못 넘으면 `ChsRiMoveToPnt2` 의 `angCalib.chs < 80°` 가드가 이미 참이라 첫 평가에서 끝나 **Ri 최고속도가 측정되지 않는다** | chart_1210 T599/T624 (`MdlApp.c:36320/35494`), `CntCalib_timeout` 1000 (`SysPar.m:104`) | `test_calib_tilt_plant`, `test_calib_rot_plant`, `test_calib_chs_plant` |
| C3 | **식별 속도표 X = [0, 0.01, max(MinTblReqSpd, peak)]** → 최고속도가 0.01 rad/s 미만인 구간(예: 속도 구간에서 일시정지)은 **단조가 아닌 브레이크포인트** [0, 0.01, 0.002] 를 저장한다. `MinTblReqSpd` 주석은 "단조 증가를 위해" 인데 0.002 는 옛 무릎 0.001 에 맞춘 값이고 무릎은 0.01 리터럴이다 | chart_2338 l.68-116, `SysPar.m:112` | `test_calib_tilt_plant` (중단된 TiltPosiToPnt1) |
| C4 | **데드밴드를 20 % 아래로는 식별할 수 없다.** 계단이 `PropVlvCmdInitOffs` 20 % 에서 시작하고 onset − 0.5 를 저장하므로 저장값은 항상 ≥ 19.5 %. 컴파일된 `tiltPosi_Y[1]` 이 정확히 19.5 = 바닥값이라 실제 데드밴드를 알려주지 않는다 | `SysPar.m:174-175`, `MdlApp.c:45610/45618`, `ECR88D_ShortArm.m:331` | `test_calib_tilt_plant` |
| C5 | **CalibRot 은 로테이터 방향을 모른다.** 배관이 반대거나 `INTP.trRotDir` 부호가 틀려도 똑같은 표를 저장, 인히빗 없음 | chart_2383, fabsf 속도 | `test_calib_rot_plant` |
| C6 | **Rot_save 가 100 % RotNega 구간 바로 뒤에 와서 1 초 램프다운이 잘린다.** `propVlvCmd.rotNega` 가 99.7 % → 0 을 한 틱에 | chart_1210, `SmoothPropVlvCmd` | `test_calib_rot_plant` |
| C7 | **로테이터 명령이 CAN 에서 0.4 % 단위로 절사된다** `(uint8)(x * 2.5f)`. CalibRot 계단은 0.2 % 단위라 홀수 계단은 TR 컨트롤러에 도달하지 않는데, 펌웨어는 절사 전 값을 기록한다 → 전송되는 최소 유지 명령은 전송된 onset 명령보다 항상 ≥ 0.4 % 낮다 | `CanCtrl.c:1090`, `CAN4_X1Exc.dbc:189` (0.4 %/bit) | `test_calib_rot_plant` |
| C8 | **로테이터 각도를 잃어도 MdlApp 는 고장을 모른다.** TR 컨트롤러 CAN 두절 시 PreProc 가 0.0° 로 대체(`PrePostProc_If.c:1049`)하고, `u.isJntAngRotFault` 를 쓰는 코드가 없다(모델은 `:39616` 에서 읽음). ECU 가 감지하는 것(`lostComm_CAN4TrCtrlDiagL`, `TrRotSensFault_DiagL`)은 서비스 진단으로만 간다. RotPosiMin 은 C1 대로 타임아웃도 없어 계단이 계속 오른다 | 위 | `test_calib_rot_plant` |
| C9 | 섀시 IMU 장착행렬의 **평면각 자체에 대한 타당성 검사가 없다.** `BIT_IMU_CALIB_ERR` 는 이 빌드에서 켜질 수 없고(SanityCheckImuCalib 코드 생성 0), 모델에 있는 그 검사도 mast/base/uc 롤·피치 창만 보므로 스윙축 둘레 요 오차는 원리상 못 잡는다 | chart_2516, `SysPar_ref.m` | `test_calib_chs_plant` |
| C10 | (사소) ChsRiMin 이 `calibStep` 을 entry 가 아니라 during 에서 대입 → 태블릿이 한 틱 `ChsPnt1_log` 를 102 틱 봄 | chart_1210 | `test_calib_chs_plant` |
| C11 | `CalibTrvl` 데드락 · `CalibBm2` 가드가 bm1 을 읽음 · IMU NVM 슬롯 72/76 교차 | `resources/X1Exc_SIL_spec.md` A7 | 소스 확인 |

### 자동 사이클

| # | 내용 | 근거 | 테스트 |
|---|---|---|---|
| A1 | **인히빗 상태에서 Auto 를 누르면 밸브가 1 틱(10 ms) 움직인다.** Standby→Positioning(T281), Complete 재시작, 인히빗이 생기는 틱의 재개, ReadyToPlace(T245) 에 인히빗 항이 없다 | `MdlApp.c:25278-25285`, T180 `:23921-23925` | `test_main_cycle.*runs_one_tick` 3 개 |
| A2 | **일시정지 중 목표 id 를 지워도 재개하면 달린다.** `BIT_NO_TARGET` 은 자동 인히빗 마스크 밖. 게다가 목표가 없으면 주행 목표가 **현재 위치 + 그리드 동쪽**으로 붕괴해, 동쪽 ±3.5° 를 보고 있으면 **목표 없이 Picking 진입**, 북쪽을 보고 있으면 동쪽으로 궤도를 돌린다 | `MdlApp.c:39675, 9395-9446, 10106-10125, 47676-47708` | `test_main_cycle.test_target_cleared_while_paused_resumes_into_picking_when_facing_grid_east` |
| A3 | **흡착 중 일시정지·취소해도 진공 펌프가 계속 돈다.** 수동 해제(NoTarget/Standby 에서만 가능)만 멈춘다. 패널을 잡고 있으려는 의도일 수 있으나 패널이 없어도 같다 | `MdlApp.c:49041-49146` | `test_vacuum_logic.test_pause_and_cancel_do_not_stop_the_pump` |
| A4 | **EngagingVacuum 에 타임아웃이 없고 컵 접촉 상실도 무시한다.** 진공센서 진단 고장은 0.0 bar 로 읽혀(`PrePostProc_If.c:739-756`) 영원히 대기 | `MdlApp.c:40160-40203` | `test_vacuum_logic.test_engaging_vacuum_has_no_timeout_and_ignores_contact_loss` |
| A5 | **흡착 시작 시 잔류 진공이 -0.05 bar 이하면 펌프가 안 켜지고 타임아웃·알람도 없다.** 센서 오프셋 -0.05 bar 하나로 멈춘다 | `MdlApp.c:48921` | `test_vacuum_logic.test_residual_vacuum_at_request_leaves_the_pump_off_with_no_timeout` |
| A6 | **ApproachPanel 에 컵 4 개 접촉 외의 정지 조건이 없다.** 목표가 `panelBottom` 이고 armIn/bm1Down 에 최소출력 유지가 걸려, 컵 센서 하나가 안 들어오면 계속 누른다. **Isaac 물리에서 관찰:** 패널 없는 씬에서 15 초 동안 공구가 0.7 m 내려감 | `MdlApp.c:6882-6893, 7832-7873, 8380-8402` | `scripts/sil_isaac_prepare_pick.py` |
| A7 | **Picking 중 진공이 먼저 걸리면 PickingInhibited 에 래치**되고 Auto 로는 안 풀린다 (bit 9 만 보임). 탈출은 Cancel/Standby/EngagingVacuum 요청뿐이고, **EngagingVacuum 요청은 펌프를 켜지 않은 채 ReadyToPlace 로 건너뛴다** ("잡은" 패널에 펌프가 꺼져 있음) | `MdlApp.c:23002-23200, 40199` | `test_vacuum_logic.test_vacuum_during_picking_latches_picking_inhibited`, `test_picking_inhibited_escapes` |
| A8 | `ReadyToReleaseInhibited` 에 취소 경로가 없다 | 명세 A8 | 소스 확인 |
| A9 | 붐 스윙 각을 0 으로 고정(센서 없음) → 붐이 옆으로 가 있으면 최대 860 mm 를 모른 채 계산 | `SysPar.m:72` | 소스 확인 |

**정정:** 예전 목록의 "릴리즈 중 일시정지하면 블로오프가 알람 없이 취소" 는 **틀렸다.** 일시정지해도 `CurrStep` 이 Releasing 으로
남아 벤팅이 계속된다 (`MdlApp.c:48774-48793, 40367`, `test_vacuum_logic.test_pause_during_blow_off_keeps_venting`).
"리모컨 링크 끊김이 자동을 막지 않는다" 는 **MdlApp 수준에서만** 사실이다 — ECU 래퍼가 리모컨 Auto 버튼·조이스틱을 0 으로 만든다
(`PrePostProc_If.c:275-285`).

---

## 2. 이 빌드에선 잠재 (EnTestPar = false 빌드에서 살아남)

| # | 내용 | 근거 |
|---|---|---|
| L1 | **어떤 캘리브를 저장해도 20 개 속도표 전부가 캘리브 형태로 다시 써진다.** 건드리지 않은 축도 무릎 0.001 → 0.01 (최소속도 유지가 10 배 빨라짐), Y[2] → `PropVlvRefCmd` (주행 100→60 %, 붐 80→70 %, 로테이터 80→100 %, 블레이드 데드밴드 25→0.01 %). X[2] 는 그대로라 %당 속도가 달라진다 | chart_2338 l.68-116, `AppCtrlIf.c:802-1012` |
| L2 | 중단된 속도 구간이 남긴 단조 아닌 표(C3)를 **다음에 저장되는 아무 캘리브**(예: 아무것도 안 움직이는 step 28)가 NVM 에 영구화 | `test_calib_tilt_plant` |
| L3 | 전원을 껐다 켜면 식별 결과가 사라지고, 그 뒤 **아무 캘리브**를 저장하면 CalibRot 이 식별한 로테이터 표를 컴파일 표로 덮어쓴다 | `test_calib_rot_plant.test_after_a_power_cycle_any_calibration_erases_the_identified_rotator_table` |
| L4 | **속도가 저장돼 있던(옛) 장착행렬로 식별된다.** 장착행렬은 이번 실행이 끝나야 새로 생기므로, 보드가 틀어진 유닛은 장착행렬은 맞게 복원되지만 속도표는 틀린다 (링크 12° 오차에 +9~11 %, 틸트는 1−R₀₀ 만큼 낮게, 스윙은 (MᵀM)₂₂). **같은 캘리브를 한 번 더** 돌려야 맞다 | `test_calib_link_plant`, `test_calib_tilt_plant`, `test_calib_chs_plant` |
| L5 | 런타임 보정 `u.parMotionOnsetCmp.<port>` 을 ECU 가 안 쓴다 → 0. 저장된 최소 명령이 틀려도 현장에서 보정할 수단이 없다 | `test_calib_link_plant` |

---

## 3. 절차 의존 (절차서가 결과를 좌우하는데 펌웨어가 확인하지 않음)

| # | 내용 | 수치 (SIL) |
|---|---|---|
| P1 | **섀시(CalibChs): 도저 잭업의 방향·크기를 확인하지 않는다.** 평지면 외적이 노이즈 크기라 `|v1×v2|>1e-6` 가드가 **조용히 옛 장착행렬을 유지**하고, 밸브가 열린 적도 없는 장비에서 식별한 스윙 표와 함께 저장한다. 기울기 방향이 틀리면 장착행렬이 스윙축 둘레로 180°/±90° 돌아간다. 연속적으로는 **요 오차 = atan(tan roll / tan pitch)**: 피치 6° 에서 **남은 롤 1° 당 9.4°**, 피치 5° 면 11.3°. 또 잭업이 **IMU 보드 기울기보다 작으면** 평면각이 170° 에 못 닿아 Le 구간은 타임아웃(C2), Ri 구간은 1 틱에 끝난다 | 롤 ±1° → 저장 장착행렬 Rz(±9.428°), 알람 없음 |
| P2 | **암(CalibArm): 기준자세에서 암이 중력에 수직(다림추)이어야 한다.** δ 만큼 틀리면 저장 장착행렬이 Ry(δ) 만큼 돌아가고, 로드 후 모든 암 각이 δ 만큼 높게 읽힌다. 잭업한 섀시에 직각으로 맞추면 6.01° | 수평 붐 -40/암 100 → +30.00° 로 읽힘 |
| P3 | **암·링크: 움직인 방향을 확인하지 않는다.** 밸브 배관이 반대면 암은 diag(1,−1,−1), 링크는 보드 Z 둘레 180° 로 돌아간 장착행렬을 정상 저장 → 로드 후 각이 기준을 중심으로 **거울상**(피드백 부호는 맞아서 루프는 닫힌다) | 참 90° → 140°, 참 135° → 95° |
| P4 | **링크(CalibLink): 기준자세에서 입력링크 현이 기계가 아니라 중력에 수평이어야 한다.** 잭업 6° 에서 태블릿 각으로 수평을 맞추면 정확히 6.00° 틀어진 장착행렬 | 6.00° |
| P5 | **덱 11 장은 "커넥팅로드" 를 수평으로 맞추라고 한다.** 펌웨어 4 절 해로는 입력링크 −130~−10° 에서 커넥팅로드가 입력링크와 **항상 45° 이상** 벌어져 둘을 동시에 수평으로 만들 수 없다. 글자대로 따르면 45~90° 가 장착행렬에 들어간다 → **질문: 입력링크를 말하는 것인가?** | |
| P6 | **틸트(CalibTilt): 기준자세에서 공구가 중력에 수평이어야 한다.** 롤 φ 가 조용한 틸트 영점 오프셋이 된다 (로드 후 tilt − φ) | |
| P7 | 로테이터 센서 고장 비트는 캘리브 인히빗 마스크에만 있어, 태블릿이 `u.isMachCalib` 을 들고 있을 때만 캘리브를 멈춘다 | |

---

## 4. 데이터 출처 질문

- **컴파일된 ShortArm 링크 속도표는 CalibLink 원출력이 아니다.** `linkIn` [0; 0.001; 0.588] / [0; 30.5; 70] 은 캘리브 결과에서 무릎만
  0.001 로 되돌린 모양이고, `linkOut` (90 % 에 0.85 m/s) 은 캘리브 형태가 아예 아니다 (`ECR88D_ShortArm.m:326-329`, chart_2338 l.98-101).
  → 파라미터 파일이 캘리브 뒤에 어떻게 편집됐는가?
- 레포 빌드가 ShortArm(1.7 m)이다. 현장 빌드도 그런가? `ECR88D_LongArm.m` 은 구 스키마라 2.1 m 장비 SIL 이 막혀 있다 → 현행 스키마 파일.

---

## 5. 플랜트 의존 (시뮬 가정에 따른 것 — 실기 동작으로 인용하지 말 것)

코드 사실과 결과를 나눠 적는다. 코드 사실은 확정, 결과는 IMU 노이즈·필터·밸브 곡선에 달렸다.

- **동작 시작 감지 = 필터 없는 가속도 샘플 1 개 vs 0.5° (8.7 mg)** — 코드 사실 (`MdlApp.c:11102-11105`, 기준은 비-`_Min` 상태에서 래치).
  시뮬에서 3~5 mg rms 노이즈면 첫 계단에서 오작동해 **두 최소 명령이 19.5 % 로 저장**(밸브는 안 열림). 1 mg 에선 안 생김. 실기 IMU 의 노이즈·내부 필터는 자료가 없다.
- 엔진 진동 30 mg(GUESS) 에서 잭업 캘리브가 섀시 장착행렬을 11~34° 틀리게 저장 (v1·v2 가 6° 떨어져 요 방향 ~9.6 배 증폭).
- `PropVlvCmdMotionOnsetDlyCmp` 0.5 % 를 빼는 규칙 때문에, 밸브 곡선이 여는 점 바로 위에서 가파르면 저장 최소 명령이 실제 여는 점 아래가 되어 최소속도 유지가 흐름을 못 만든다.
- 저장 데드밴드(30 % 전후)에서 캘리브 한 축에 4~5 분, CalibArm 은 ≥ 79 % 가 계단 대기 (산수는 펌웨어, 데드밴드 크기는 가정).
- 링크 구간 뒤 1 초 램프다운이 저장 속도에서 링크를 구간 끝보다 ~53° 더 보낸다 (시뮬 스톱이 GUESS).

---

## 재현

```bash
cd ~/jude/xpanner-sim && python3 sil/build.py
python3 -m unittest discover -s sil/tests -t .                      # 310 tests, ~3.5 min
python3 -m unittest sil.tests.test_calib_rot_plant -v               # 모듈 하나
```
Isaac 시나리오는 `sil/README.md`.
