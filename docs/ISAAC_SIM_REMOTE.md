# Isaac Sim 원격 GUI 런북 (EC2 L4 / 컨테이너 6.0.1)

> 이 문서의 모든 명령은 **2026-09-11 이 서버에서 실제로 실행해 검증**했다.
> 검증하지 못한 항목은 본문에 인라인으로 표시했고 §8 에 모아 두었다.
> 서버: `i-027baebf60af5ad06` (ap-northeast-2), NVIDIA **L4 23034 MiB 한 장**,
> public IP **3.37.129.139** (2026-09-14 기준). ⚠️ **Elastic IP 가 없어서 인스턴스를 재시작하면 IP 가 바뀐다**
> (09-11 은 `3.37.123.120` 이었다). 이 문서의 IP 를 믿지 말고 항상 §3.0.2 로 확인할 것.
> 재부팅하면 컨테이너도 사라진다: `ISAAC_CONTAINER=isaac-sim-jude ./scripts/run_isaac.sh stream`.
> Isaac Sim 은 **Docker 이미지 `nvcr.io/nvidia/isaac-sim:6.0.1` 로만** 존재한다 (native 설치도 `python.sh` 도 호스트에 없음).

---

## 1. 한 줄 요약

서버에서 Isaac Sim 을 **headless WebRTC streaming 모드**로 띄우고, 노트북에 설치한
**Isaac Sim WebRTC Streaming Client** 로 서버 public IP(§3.0.2)에 접속해 GUI 를 본다.
X11/VNC 가 아니라 NVENC 하드웨어 인코딩 기반 WebRTC 이며, **TCP 49100 (signaling) + UDP 47998 (media)**
두 포트가 모두 노트북까지 도달해야 한다. `ssh -L` 은 TCP 만 나르므로 **단독으로는 화면이 안 나온다**(§5).

전체 흐름:

```
[노트북] WebRTC Client ──TCP 49100 signaling──▶ [EC2 <PUBLIC_IP>]  docker run --network=host
                       ◀──UDP 47998 media───── Isaac Sim 6.0.1 (isaacsim.exp.full.streaming.kit)
```

---

## 2. 사전 확인 — 다른 사람 방해하지 않기

이 서버는 **공유 머신이고 GPU 는 L4 한 장**이다. Isaac Sim 은 `--network=host` 로만 동작하므로
포트 49100/47998 은 호스트 전역이고 **선점한 사람이 독점**한다. 시작 전에 아래 3 개를 반드시 돌린다.

> **가장 쉬운 방법**: `scripts/run_isaac.sh` 가 아래 세 가지를 전부 대신 해 준다.
> 다른 사람의 컨테이너·GPU·포트를 발견하면 **아예 시작하지 않는다**(`--force` 로도 안 뚫린다).
> 아래 수동 확인은 래퍼를 안 쓰거나 래퍼가 왜 거절했는지 확인할 때 쓴다.

```bash
# (1) GPU 점유 확인
nvidia-smi --query-gpu=memory.used,memory.total,utilization.gpu --format=csv
nvidia-smi --query-compute-apps=pid,process_name,used_memory --format=csv

# (2) 이미 떠 있는 컨테이너 확인 (Exited 까지 보려면 -a)
docker ps --format 'table {{.Names}}\t{{.Image}}\t{{.Status}}\t{{.Ports}}'
docker ps -a --format 'table {{.Names}}\t{{.Image}}\t{{.Status}}'

# (2b) 이름/이미지로 거르지 말고, GPU 를 실제로 쥔 컨테이너를 찾는다
for c in $(docker ps -q); do
  docker inspect -f '{{.Name}} gpus={{if .HostConfig.DeviceRequests}}{{len .HostConfig.DeviceRequests}}{{else}}0{{end}} runtime={{.HostConfig.Runtime}} net={{.HostConfig.NetworkMode}} image={{.Config.Image}}' "$c"
done

# (3) 스트리밍 포트 충돌 확인
ss -tuln | grep -E '49100|47998|8210' || echo 'ports free'
```

### ⚠️ 이름이나 이미지 태그로 거르면 놓친다

`docker ps --filter name=isaac-sim` 도, `docker ps --filter ancestor=nvcr.io/nvidia/isaac-sim:6.0.1` 도
**믿을 수 없다.** 이 호스트에서 확인한 두 가지 이유:

* **이름 필터**는 단순 부분문자열이다. 이 서버의 `vpick-stream` 은 Isaac Sim 이미지로 만들어졌지만
  이름에 `isaac-sim` 이 없다 → `--filter name=isaac-sim` 결과에 안 나온다(확인함).
* **`ancestor=` 필터는 이미지 ID 하나로 해석된다.** 누가 `:latest`(지금은 6.1.0, §8 #10)를
  pull 해서 띄우면 ID 가 다르므로 안 잡힌다.

그래서 (2b) 처럼 **돌고 있는 컨테이너를 전부 훑어서 성질로 판정**해야 한다.
`run_isaac.sh` 의 `check_no_other_isaac()` 가 정확히 그렇게 한다(이미지 참조·이미지 RepoTags·
GPU 보유·포트 바인딩).

### 판단 기준

| 관측 | 해석 | 할 일 |
|---|---|---|
| `memory.used` < **300 MiB**, compute-apps 비어 있음, `docker ps` 비어 있음 | 아무도 안 쓴다 | 그냥 진행 |
| `memory.used` **1000 MiB 이상** 또는 compute-apps 에 프로세스 존재 | 누군가 작업 중 | **띄우지 말 것.** 기다리거나 당사자에게 물어본다 |
| `docker ps` 에 **`Up` 인 컨테이너가 GPU(`gpus=`>0 또는 `runtime=nvidia`)를 쥐고 있음** — 이름이 무엇이든 | 남이 실행 중 | **절대 `docker stop/rm/rm -f` 하지 말 것.** 대기 |
| `ss` 결과에 49100 LISTEN | 남이 스트리밍 중 | 포트를 옮겨서 띄운다 (§3 의 B 안) |
| `docker ps -a` 에 `Exited` 상태의 `isaac-sim`, `vpick-stream` | **남이 남긴 흔적**(6주 전 / 3주 전 종료). GPU·포트는 안 잡고 **이름만 점유** | 건드리지 말고, 내 컨테이너는 다른 이름으로 (§3 의 이름 규칙) |

참고로 **아무 씬도 안 연 idle 스트리밍 컨테이너 하나가 VRAM 약 1.5 GiB** 를 쓴다(오늘 측정).
ECR88 같은 씬을 올리면 더 늘어난다. L4 23 GiB 한 장이므로 두 명이 동시에 돌리면
용량은 버틸 수도 있지만 **렌더 프레임레이트를 서로 나눠 갖고**, 무거운 씬 두 개면 CUDA OOM 으로
**둘 다 죽는다**. NVIDIA 의 다중 인스턴스 가이드도 "인스턴스당 GPU 한 장"을 전제하므로
이 서버에서 동시 실행은 권장되지 않는다. 포트만 분리해도 GPU 는 분리되지 않는다.

끝나면 반드시 §3 의 정리 명령으로 컨테이너를 내려서 GPU 와 포트를 돌려준다.

---

## 3. 서버 쪽 — 스트리밍 컨테이너 띄우기

### 3.0 권장: 래퍼로 띄우기 (가드가 붙은 경로)

아래 3.1 / 3.2 의 날 `docker run` 은 **가드가 하나도 없다.** 평소에는 래퍼를 쓴다:

```bash
cd /home/ubuntu/jude/xpanner-sim
./scripts/run_isaac.sh stream          # public IP 도 IMDSv2 로 알아서 잡는다
./scripts/run_isaac.sh shell
./scripts/run_isaac.sh convert --xacro ... --output ...
```

래퍼가 시작 전에 거절하는 조건(§2 와 같은 내용):

| 검사 | `--force` 로 건너뛰나? |
|---|---|
| 다른 컨테이너가 Isaac Sim / GPU / 49100·47998 을 쥐고 있음 | **아니오** |
| 49100 또는 47998 이 이미 LISTEN 중 (`stream` 한정) | **아니오** |
| `nvidia-smi` 상 GPU 메모리가 임계치 초과 | 예 (그 메모리가 내 것일 때만) |
| 내가 쓰려는 컨테이너 이름이 이미 존재 | 아니오 |

`--force` 는 **"저 GPU 메모리는 내 것이다"** 라는 뜻일 뿐이다.
남이 이미 잡은 포트나 남이 돌리는 컨테이너에 대해서는 어떤 의미도 갖지 않으므로 그 검사들은 못 건너뛴다.

### 3.0.1 컨테이너 이름 규칙 (중요)

**이 서버의 로그인 계정은 `ubuntu` 하나뿐이다**(`ls -l /home/` 확인). 즉 `$USER` 는 모두에게 같은 값이고
`isaac-sim-$USER` 같은 이름은 **충돌 방지에 아무 도움이 안 된다.** 그래서 래퍼의 기본 이름은
`isaac-sim-$USER-$$` (`$$` = 그 셸의 PID) 이고, 고정 이름을 쓰고 싶으면 명시적으로 준다:

```bash
ISAAC_CONTAINER=isaac-sim-jude ./scripts/run_isaac.sh stream
```

아래 3.1/3.2 예제의 `isaac-sim-jude` 도 **자리표시자**다. 같은 이름으로 두 세션을 돌리면 충돌하니
`isaac-sim-jude-$$` 처럼 자기만의 값으로 바꿔 쓴다.

### 3.0.2 public IP 잡기

컨테이너 안에서는 EC2 의 public IP 가 안 보인다(AWS 1:1 NAT). `hostname -I` 는 **사설 IP** 라
그걸 쓰면 노트북에서 화면이 안 온다. IMDSv2 로 가져온다.

```bash
PUBLIC_IP=$(TOKEN=$(curl -s -X PUT "http://169.254.169.254/latest/api/token" \
  -H "X-aws-ec2-metadata-token-ttl-seconds: 21600") && \
  curl -s -H "X-aws-ec2-metadata-token: $TOKEN" \
  http://169.254.169.254/latest/meta-data/public-ipv4)
echo "$PUBLIC_IP"     # 재시작마다 바뀐다. 09-11: 3.37.123.120 / 09-14: 3.37.129.139
```

### 3.1 A안 (기본) — 기본 포트로 띄우기

포트가 비어 있을 때 쓴다. **`-p` 포트 퍼블리시는 쓰지 않는다** — NVIDIA 문서가 명시적으로
"bridge networking 에서는 WebRTC media 가 동작하지 않는다"고 못박았다. `--network=host` 가 필수다.

```bash
PUBLIC_IP=$(TOKEN=$(curl -s -X PUT "http://169.254.169.254/latest/api/token" \
  -H "X-aws-ec2-metadata-token-ttl-seconds: 21600") && \
  curl -s -H "X-aws-ec2-metadata-token: $TOKEN" \
  http://169.254.169.254/latest/meta-data/public-ipv4)

docker run -d --name isaac-sim-jude --gpus all --network=host \
  -e "ACCEPT_EULA=Y" \
  -e "PRIVACY_CONSENT=Y" \
  -e "ISAACSIM_HOST=$PUBLIC_IP" \
  -v /home/ubuntu/docker/isaac-sim/cache/main:/isaac-sim/.cache:rw \
  -v /home/ubuntu/docker/isaac-sim/cache/computecache:/isaac-sim/.nv/ComputeCache:rw \
  -v /home/ubuntu/docker/isaac-sim/logs:/isaac-sim/.nvidia-omniverse/logs:rw \
  -v /home/ubuntu/docker/isaac-sim/config:/isaac-sim/.nvidia-omniverse/config:rw \
  -v /home/ubuntu/docker/isaac-sim/data:/isaac-sim/.local/share/ov/data:rw \
  -v /home/ubuntu/docker/isaac-sim/pkg:/isaac-sim/.local/share/ov/pkg:rw \
  -v /home/ubuntu/.cache/ov/hub:/var/cache/hub:rw \
  -v /home/ubuntu/jude/xpanner-sim:/work/xpanner-sim:rw \
  -u 1234:1234 \
  nvcr.io/nvidia/isaac-sim:6.0.1 -v
```

알아 둘 것:

* 이미지의 기본 ENTRYPOINT 가 이미 `/isaac-sim/runheadless.sh` 다 → `--entrypoint bash` 나
  `-c "./runheadless.sh"` 를 붙일 필요가 없다. 맨 끝의 `-v` 는 **볼륨이 아니라 kit 의 verbose 플래그**다.
* `runheadless.sh` 가 `ISAACSIM_HOST` / `ISAACSIM_SIGNAL_PORT` / `ISAACSIM_STREAM_PORT` 를
  `--/exts/omni.kit.livestream.app/primaryStream/{publicIp,signalPort,streamPort}` 로 바꿔 준다.
  6.0 에서 setting 네임스페이스가 바뀌었으므로 **4.x/5.x 예제의 `--/app/livestream/*` 플래그는 조용히 무시된다.**
* 마운트 경로는 이 머신의 실제 디렉토리다. `cache/main`, `cache/computecache`, `config`, `data`,
  `logs`, `pkg` 는 이미 uid 1234 소유라 `chown` 불필요.
  (`cache/{glcache,kit,ov,pip}` 는 4.x 시절 잔재로 root 소유이지만 6.0 은 안 쓴다. 건드리지 말 것.)
* `-v /home/ubuntu/jude/xpanner-sim:/work/xpanner-sim:rw` 로 프로젝트 레포가 컨테이너 안
  `/work/xpanner-sim` 에 보인다. GUI 의 File > Open 에서 이 경로를 쓴다.
  (`run_isaac.sh` 도 같은 `/work/<레포이름>` 으로 마운트한다. 예전 판의 `/workspace/...` 는 틀린 경로였다.)

**로딩 대기** (이 서버에서 warm cache 기준 **약 50초** 걸렸다):

```bash
docker logs -f isaac-sim-jude | grep -m1 "Full Streaming App is loaded"
# -> [50.540s] Isaac Sim Full Streaming App is loaded.
```

이 줄이 뜨기 전에 클라이언트를 붙이면 실패한다. 이어서 소켓 확인:

```bash
ss -tuln | grep 49100      # tcp LISTEN 0.0.0.0:49100  <- 정상
ss -uln  | grep 47998      # 아무것도 안 나오는 게 정상 (클라이언트 연결 전엔 UDP 미바인딩)
docker ps --format '{{.Names}}\t{{.Status}}'   # Up ... (healthy)
```

### 3.2 B안 — 누가 기본 포트를 쓰고 있을 때

```bash
docker run -d --name isaac-sim-jude --gpus all --network=host \
  -e "ACCEPT_EULA=Y" -e "PRIVACY_CONSENT=Y" \
  -e "ISAACSIM_HOST=$PUBLIC_IP" \
  -e "ISAACSIM_SIGNAL_PORT=49200" \
  -e "ISAACSIM_STREAM_PORT=48100" \
  ... (나머지 -v, -u 는 A안과 동일) ...
  nvcr.io/nvidia/isaac-sim:6.0.1 -v
```

**주의**: 네이티브 WebRTC 클라이언트 UI 에는 **포트 입력란이 없다**(IP 만 받는다).
포트를 옮기면 클라이언트가 기본 49100 으로 붙으므로 §5 의 SSH 터널 트릭
(`ssh -L 49100:127.0.0.1:49200`)으로 signaling 을 우회해야 한다. 그런데 media(UDP 48100)는
터널을 못 타므로 **결국 SG 에 UDP 48100 도 열어야 한다.** 즉 B안은 GPU 를 나눠 쓰는 상황 자체가
권장되지 않으므로 (§2) 되도록 **대기**를 택하는 편이 낫다.

### 3.3 종료 / 정리

**내가 띄운 이름에만** 쓴다. 남의 컨테이너에는 절대 쓰지 않는다(§7 의 판정표).

```bash
docker stop isaac-sim-jude && docker rm isaac-sim-jude
# 한 방에:  docker rm -f isaac-sim-jude
# (래퍼로 띄웠다면 --rm 이라 컨테이너가 끝나면서 스스로 사라진다)

# 정리 확인
ss -tuln | grep -E '49100|47998' || echo 'ports free'
nvidia-smi --query-gpu=memory.used --format=csv,noheader     # 0 MiB 로 돌아와야 정상
```

로그를 나중에 보려면 컨테이너를 지우기 전에:

```bash
docker logs isaac-sim-jude > /home/ubuntu/jude/isaac-sim-$(date +%Y%m%d-%H%M).log 2>&1
```

---

## 4. 노트북 쪽 — 클라이언트

**이름**: `Isaac Sim WebRTC Streaming Client` (버전 **2.0.0**, 2026-06).
Isaac Sim 은 6.0.1 인데 클라이언트 버전은 2.0.0 이 맞다 — 별개 버전 체계다.

다운로드 (NVIDIA Isaac Sim 6.0.1 download 페이지 기준. *URL 은 문서에서 가져왔고 실제로 받아보진 않았다 — §8*):

| OS | URL |
|---|---|
| macOS (Apple Silicon) | `https://downloads.isaacsim.nvidia.com/isaacsim-webrtc-streaming-client-2.0.0-macos-aarch64.dmg` |
| macOS (Intel) | `https://downloads.isaacsim.nvidia.com/isaacsim-webrtc-streaming-client-2.0.0-macos-x86_64.dmg` |
| Windows | `https://downloads.isaacsim.nvidia.com/isaacsim-webrtc-streaming-client-2.0.0-windows-x86_64.exe` |
| Linux x86_64 | `https://downloads.isaacsim.nvidia.com/isaacsim-webrtc-streaming-client-2.0.0-linux-x86_64.deb` |
| Linux aarch64 | `https://downloads.isaacsim.nvidia.com/isaacsim-webrtc-streaming-client-2.0.0-linux-aarch64.deb` |

6.0 부터 Linux 는 **.deb** 이다. 4.5/5.0 시절의 AppImage + `libfuse2` 요구사항은 없어졌다.

```bash
# Linux 노트북
wget https://downloads.isaacsim.nvidia.com/isaacsim-webrtc-streaming-client-2.0.0-linux-x86_64.deb
sudo dpkg -i ./isaacsim-webrtc-streaming-client-2.0.0-linux-x86_64.deb
sudo apt -f install
isaacsim-webrtc-streaming-client

# Ubuntu 24.04+ 에서 Electron sandbox 가 막히면
sudo sysctl -w kernel.apparmor_restrict_unprivileged_userns=0
```

**접속 주소 입력법**

1. 서버 로그에 `Isaac Sim Full Streaming App is loaded.` 가 뜬 걸 먼저 확인한다.
2. 클라이언트를 실행하면 주소 칸에 기본값 `127.0.0.1` 이 들어 있다. 이걸 지우고
   **§3.0.2 의 `$PUBLIC_IP`** (09-14 기준 `3.37.129.139`) 를 넣는다. `http://` 도, `:49100` 도 붙이지 않는다. **IP 만.**
3. Connect.

동시 접속은 **한 명만** 가능하다(인스턴스당 클라이언트 1개). 둘째 사람은 붙지 않는다.

---

## 5. 네트워크 — 포트와 두 가지 방법

### 포트 표

| 포트 | 프로토콜 | 용도 | 필요 여부 |
|---|---|---|---|
| 49100 | **TCP** | WebRTC signaling | 필수 |
| 47998 | **UDP** | WebRTC media (실제 영상) | **필수** |
| 8210 | TCP | 브라우저 web viewer | Docker Compose 방식을 쓸 때만 (§8) |

NVIDIA 문서 원문: *"Firewalls must allow both TCP 49100 and UDP 47998; opening only TCP ports is
not sufficient for WebRTC media."*

호스트 방화벽은 이미 무관하다 — 이 서버의 `ufw` 는 **inactive** 다. 유일한 관문은 **EC2 security group** 이다.

### 방법 A — SSH 터널 (되지 않는다, 정직하게)

```bash
# 노트북에서
ssh -L 49100:127.0.0.1:49100 ubuntu@<PUBLIC_IP>
```

이렇게 하면 클라이언트에 `127.0.0.1` 을 넣고 **연결은 된다**. 그리고 **화면은 영원히 검다.**
이유: `ssh -L` 은 **TCP 만** 포워딩한다. 영상은 UDP 47998 로 오는데 그 경로가 없다.
같은 증상이 NVIDIA IsaacSim 저장소 discussion #597(6.0, NAT 클라우드 호스트)에 그대로 보고돼 있고,
유지보수자의 답도 "터널이 아니라 실제 UDP 경로를 열어라"였다.

* `sshuttle` 도 구제책이 아니다. UDP 는 Linux 전용 `--method=tproxy` 에서만 되고 macOS(pf)는 UDP 미지원.
* `ssh -w`(진짜 L3 VPN)는 UDP 를 나르긴 하지만 이 서버의 `/etc/ssh/sshd_config` 는
  `#PermitTunnel no` (주석 → 기본값 `no`)라 **현재 꺼져 있고**, 켜려면 root 로 공용 sshd 설정을 바꿔야 한다.
  게다가 TCP-over-TCP 라 영상 스트림에는 최악이다. 권장하지 않는다.

**SSH 터널이 유용한 유일한 경우**: §3.2 처럼 포트를 옮겼는데 클라이언트에 포트 입력란이 없을 때,
signaling 만 `ssh -L 49100:127.0.0.1:49200` 로 우회하는 용도. **영상 문제는 여전히 해결 못 한다.**

### 방법 B — EC2 security group (권장안) ✅

NVIDIA 가 AWS 배포 문서에서 공식적으로 제시하는 유일한 원격 접속 방법이다.

1. 노트북에서 **자기 공인 IP** 를 확인한다. **SSH 접속 출발지로 보이는 `13.209.1.61` 을 쓰면 안 된다** —
   그건 AWS 서울 리전 대역(bastion / EC2 Instance Connect 홉)이지 노트북 IP 가 아니다.

   ```bash
   # 반드시 노트북에서 실행
   curl -s https://checkip.amazonaws.com
   ```

2. security group 에 인바운드 2줄을 추가한다. Source 는 **`<노트북IP>/32`**.

   | Type | Protocol | Port | Source |
   |---|---|---|---|
   | Custom TCP | TCP | 49100 | `<노트북IP>/32` |
   | Custom UDP | **UDP** | 47998 | `<노트북IP>/32` |

   AWS CLI 로 한다면 (**이 서버에는 AWS 자격증명이 없다** — 노트북이나 콘솔에서 실행):

   ```bash
   SG=sg-xxxxxxxx                       # 아래 명령으로 찾는다
   aws ec2 describe-instances --region ap-northeast-2 \
     --instance-ids i-027baebf60af5ad06 \
     --query 'Reservations[].Instances[].SecurityGroups' --output table

   MYIP=$(curl -s https://checkip.amazonaws.com)
   aws ec2 authorize-security-group-ingress --region ap-northeast-2 \
     --group-id $SG --protocol tcp --port 49100 --cidr $MYIP/32
   aws ec2 authorize-security-group-ingress --region ap-northeast-2 \
     --group-id $SG --protocol udp --port 47998 --cidr $MYIP/32
   ```

3. **`0.0.0.0/0` 는 절대 쓰지 말 것.** 이 스트림에는 **인증도 암호화도 없다.**
   49100 에 닿을 수 있는 사람은 누구나 GUI 를 마우스로 조작할 수 있다 — 공유 서버를 인터넷에 내주는 셈이다.
   NVIDIA 원문: *"The streaming endpoints do not include authentication or encryption."*

4. 집/사무실 IP 는 보통 유동이다. ISP 임대가 갱신되면 규칙이 죽으므로, 자주 쓸 거면
   Tailscale 같은 **WireGuard 계열 VPN**(UDP 터널이라 47998 이 그대로 통과)로 바꾸는 게 낫다.
   *다만 이 경로는 여기서 검증하지 않았다 — §8.*

---

## 6. ECR88 asset 실제로 띄워보기

> **2026-09-14 에 전면 갱신.** 예전 판은 지금은 없는 `scripts/urdf_to_usd_isaac60.py` 와
> `assets/ecr88/usd/ecr88/ecr88.usda` 출력 경로를 가리켰다. 아래는 현재 레포에서 반복해서 쓰고 있는 절차다.

### 6.0 먼저 알아야 할 것

* **호스트에는 Isaac Sim 도 `python.sh` 도 없다.** Isaac 쪽 스크립트는 전부 컨테이너 안에서 돈다.
  반대로 **`xacro` 는 컨테이너에 없고 호스트에만 있다.** 그래서 순서가 늘 "호스트에서 xacro → 컨테이너에서 변환" 이다.
* `scripts/urdf_to_usd.py` 는 6.0.1 의 `URDFImporter` / `URDFImporterConfig` API 로 작성돼 있고
  **이 서버에서 반복 검증됐다** (2026-09-11 ~ 09-14, 매 URDF 변경마다). 옛 `ImportError: cannot import name '_urdf'`
  는 재작성 이전의 기록이다.
* 이미 떠 있는 스트리밍 컨테이너(`isaac-sim-jude`)가 있으면 **`docker exec` 로 그 안에서 돌린다.**
  컨테이너를 하나 더 띄우면 GPU 를 두 번 잡는다.

### 6.1 호스트에서 xacro → flat URDF

```bash
cd /home/ubuntu/jude/xpanner-sim
xacro assets/ecr88/urdf/ecr88.urdf.xacro -o build/ecr88.urdf
xacro assets/ecr88/urdf/ecr88.urdf.xacro model_cylinders:=false -o build/ecr88_nocyl.urdf   # 물리용

# 검증 (Isaac Sim 없이 stdlib + numpy)
python3 scripts/validate_urdf.py --xacro assets/ecr88/urdf/ecr88.urdf.xacro --variant ECR88_US1_2P1M
```

### 6.2 출력 디렉토리 권한

컨테이너는 uid **1234**, 레포는 `ubuntu`(uid 1000) 소유다. 출력 디렉토리만 열려 있으면 된다(이미 열려 있음):

```bash
chmod 777 assets/ecr88/usd assets/site
```

### 6.3 URDF → USD, 현장 씬, 물리 확인 (컨테이너 안)

```bash
C=isaac-sim-jude
RP="--rest-pose boom_joint=-30 --rest-pose arm_joint=110 --rest-pose bucket_joint=20"
W=/work/xpanner-sim

docker exec $C /isaac-sim/python.sh $W/scripts/urdf_to_usd.py \
    --urdf $W/build/ecr88.urdf       --output $W/assets/ecr88/usd/ecr88.usd $RP
docker exec $C /isaac-sim/python.sh $W/scripts/urdf_to_usd.py \
    --urdf $W/build/ecr88_nocyl.urdf --output $W/assets/ecr88/usd/ecr88_physics.usd $RP

# 현장 씬: 순서가 중요하다 (site 가 robot 을 참조하고, 뒤 두 개가 site 를 덮어쓴다)
rm -f assets/site/solar_site.usd
docker exec $C /isaac-sim/python.sh $W/scripts/build_site.py    --robot $W/assets/ecr88/usd/ecr88.usd --output $W/assets/site/solar_site.usd
docker exec $C /isaac-sim/python.sh $W/scripts/animate_cycle.py --stage $W/assets/site/solar_site.usd
docker exec $C /isaac-sim/python.sh $W/scripts/add_cameras.py   --stage $W/assets/site/solar_site.usd

# 물리 안정성 (관절마다 수렴 여부)
docker exec $C /isaac-sim/python.sh $W/scripts/check_stability.py $W/assets/ecr88/usd/ecr88_physics.usd
```

**⚠️ 변환의 종료 코드 1 은 무시해도 되는 경우가 있다.** 변환을 끝낸 뒤 `simulation_app.close()` 에서
Isaac Sim 자체가 `carb::tasking::TaskGroup::~TaskGroup(): Destroying busy TaskGroup!` 로 abort 할 때가 있다
(09-14 에도 재현). 로그에 `[usd] PhysicsFixedJoint ...` 요약이 다 찍힌 뒤라면 **파일은 정상**이다.
확실히 하려면 USD 를 열어 관절 수와 `physxJoint:maxJointVelocity` 를 확인한다.

기대값 (09-14): `check_stability.py` 가 **10 개 관절** 을 보고 `미수렴 0개 / 10`.
`merge_fixed_joints` 는 기본 OFF — 켜면 `contact_surface_link`, `gnss_*_link`, IMU·카메라 마운트 프레임이 사라진다.

### 6.4 GUI 에서 열기

1. §3 으로 스트리밍 컨테이너를 띄운다 (래퍼든 수동이든 레포는 `/work/xpanner-sim` 에 마운트된다).
2. 노트북 클라이언트로 **§3.0.2 의 `$PUBLIC_IP`** 에 접속.
3. **File ▸ Open…** (`Ctrl+O`) → 컨테이너 안 경로:

   ```
   /work/xpanner-sim/assets/site/solar_site.usd      # 현장 + 장비 + 카메라 + 작업 사이클
   /work/xpanner-sim/assets/ecr88/usd/ecr88.usd      # 장비만
   ```

4. 조작법·카메라·사이클 재생은 노션 "📋 시뮬레이션 tool" 페이지에 정리돼 있다.

---

## 7. 문제 해결

### 클라이언트가 연결됐다는데 화면이 검다 (가장 흔함)

거의 항상 **UDP 47998 이 안 오는 것**이다. 순서대로:

```bash
# 1. 앱이 진짜 로드됐나
docker logs isaac-sim-jude | grep "Full Streaming App is loaded"

# 2. host 네트워크로 떴나 (host 여야 한다)
docker inspect isaac-sim-jude --format '{{.HostConfig.NetworkMode}}'

# 3. 클라이언트에 넣은 IP 와 컨테이너에 준 IP 가 같나
docker logs isaac-sim-jude | grep -o 'publicIp=[0-9.]*'

# 4. signaling 소켓
ss -tuln | grep 49100
```

여기까지 다 정상인데 검다면 → **SG 의 UDP 47998 규칙이 없거나, 노트북 네트워크가 인바운드 UDP 를 막는다.**
`ssh -L` 만 쓰고 있다면 이게 원인이다(§5). 회사망/호텔 NAT 가 UDP 를 죽이는 경우도 있는데,
그때는 SG 를 아무리 열어도 안 되고 VPN(WireGuard/Tailscale)이 답이다.

> `ss -uln | grep 47998` 이 **비어 있는 것은 정상이다.** UDP 소켓은 클라이언트가
> 협상을 시작해야 바인딩된다. 이걸 서버 고장으로 오해하지 말 것.

### 연결 자체가 안 된다 (Connect 눌러도 무반응 / timeout)

* 앱이 아직 로딩 중 → `Full Streaming App is loaded.` 대기 (이 서버 ~50초).
* SG 의 TCP 49100 규칙 없음 → §5 방법 B.
* 노트북 IP 가 바뀌었다 → `curl -s https://checkip.amazonaws.com` 다시 찍고 SG 갱신.
* 다른 사람이 이미 붙어 있다 → 인스턴스당 클라이언트 **1개**만 가능.
* 클라이언트에 `http://` 나 `:49100` 을 같이 넣었다 → **IP 만** 넣는다.

### 컨테이너 이름 충돌

```
docker: Error response from daemon: Conflict. The container name "/isaac-sim" is already in use
```

**먼저 그 이름이 살아 있는지 죽어 있는지 확인한다. 이 둘은 완전히 다른 상황이다.**

```bash
docker inspect -f '{{.State.Running}}' <이름>
```

| 결과 | 뜻 | 할 일 |
|---|---|---|
| `true` | **누가 지금 쓰는 중일 수 있다.** 이 서버는 로그인 계정이 `ubuntu` 하나라서 이름만 보고 "내 것"이라고 단정할 수 없다 | **`docker stop` / `docker rm` / `docker rm -f` 금지.** `docker logs --tail 50 <이름>` 으로 보기만 하고, 내 컨테이너는 다른 이름으로 띄운다 |
| `false` | GPU 도 포트도 안 잡고 **이름만** 차지한 껍데기 | `docker inspect -f '{{.State.FinishedAt}}' <이름>` 으로 언제 끝났는지 본다. 내 것이면 `docker rm <이름>`, 기억에 없으면 그냥 다른 이름을 쓴다 |

이 서버에는 이미 **다른 사람이 남긴** `isaac-sim`, `vpick-stream`(둘 다 Exited) 컨테이너가 있다.
그 이름들은 재사용하지도, 지우지도 말 것.

`run_isaac.sh` 는 이 판정을 대신 해 준다. 살아 있는 컨테이너에 대해서는 **`docker rm -f` 를 절대 제안하지 않고**,
Exited 인 경우에만 종료 시각과 함께 `docker rm` 을 제안한다. (예전 버전은 `docker ps -a` 결과만 보고
살아 있든 죽어 있든 `docker rm -f` 를 안내했다 — 남의 세션을 죽이는 안내였다.)

### GPU OOM / 렌더가 극도로 느림

```bash
nvidia-smi --query-compute-apps=pid,process_name,used_memory --format=csv
docker ps --format 'table {{.Names}}\t{{.Status}}'
```

* 다른 Isaac Sim 컨테이너가 같이 떠 있으면 **L4 한 장을 나눠 쓰는 중**이다. 하나를 내려야 한다
  (내 것을 내리거나, 상대와 합의).
* 좀비 컨테이너가 VRAM 을 쥐고 있으면 `docker ps` 로 `Up` 인 것만 확인한다.
  **정리해도 되는 건 내가 띄운 것뿐이다** — 이름만으로는 내 것인지 알 수 없다(계정이 하나뿐이므로).
  `Exited` 는 GPU 를 잡지 않으니 건드릴 이유가 없다.
* 씬이 무거우면 `--merge-fixed-joints` 빌드로 링크 수를 줄이거나 4절 분기를
  끄는 것(`-D model_bucket_fourbar:=false`)을 고려. 단 TCP/GNSS 프레임이 사라지는 대가가 있다(§6).

### 스테일 컨테이너 / 캐시 꼬임

증상: 컨테이너가 곧바로 죽는다, config 오류, 스트리밍이 붙었다 끊긴다.

```bash
docker ps -a --format 'table {{.Names}}\t{{.Status}}\t{{.RunningFor}}'
docker logs isaac-sim-jude 2>&1 | tail -50
```

NVIDIA 문서는 *"Stale volume mounts: Old cached data ... can cause crashes, config errors, or
livestream failures"* 라고 경고한다. **최후의 수단**(셰이더 캐시가 날아가서 다음 실행이 매우 느려지고,
**공용 디렉토리라 다른 사용자에게도 영향**을 준다 — 반드시 합의 후):

```bash
# sudo rm -rf /home/ubuntu/docker/isaac-sim
# mkdir -p /home/ubuntu/docker/isaac-sim/{cache/main,cache/computecache,config,data,logs,pkg}
# sudo chown -R 1234:1234 /home/ubuntu/docker/isaac-sim /home/ubuntu/.cache/ov/hub
```

### 로그에 뜨지만 **무시해도 되는** 경고들 (전부 오늘 실제 로그에서 확인)

| 메시지 | 판정 |
|---|---|
| `OmniHub: Hub failed to launch ... retry_reason` (수십 줄 반복) | 무해. Nucleus/Hub 미사용 |
| `failed to open the default display. Can't verify X Server version` | 무해. headless 라 정상 |
| `GLFW initialization failed` / `Failed to startup plugin carb.windowing-glfw.plugin` | 무해. 창이 없는 게 정상 |
| `ECC is enabled on physical device 0` | 무해. L4 특성 |
| `PCIe link width current (8) and maximum (16) don't match` | 무해(대역폭 절반이지만 동작함) |
| `pxr.Semantics is deprecated` | 무해 |
| `Destroying busy TaskGroup!` + exit 1 (**변환 스크립트 종료 시에만**) | 무해. `[usd] ...` 요약이 다 찍힌 뒤라면 파일은 정상 (§6.3) |

### ImportError: cannot import name '_urdf'

4.5/5.x 시절의 importer API 를 6.0.1 에서 불렀다는 뜻이다.
현재 `scripts/urdf_to_usd.py` 는 6.0.1 API 로 재작성돼 반복 검증됐다(§6.3). 이 오류가 다시 나면
**다른 태그의 이미지(`:latest` = 6.1.0 등)** 를 쓰고 있지 않은지부터 확인한다(§8 #10).

---

## 8. 확실하지 않은 것 (숨기지 않고 모아 둠)

1. ~~노트북 네트워크가 인바운드 UDP 47998 을 실제로 통과시키는지 검증 못 했다.~~ **해결 (09-11): 사용자 노트북에서 WebRTC 화면 수신 확인.**
   (아래는 다른 사람 노트북에서 막힐 때를 위해 남겨 둔다.)
   서버 쪽(TCP 49100 LISTEN, 앱 로드, NVENC 존재)은 전부 확인했지만 노트북이 없어 핸드셰이크를 못 돌렸다.
   "연결됨 + 검은 화면"이 나오면 설정이 아니라 **UDP 경로**를 의심할 것.
2. **이 인스턴스의 security group ID 와 현재 인바운드 규칙을 확인하지 못했다.**
   호스트에 AWS 자격증명이 없고(`aws: NoCredentials`), AWS MCP 토큰도 만료였다.
   §5 의 `describe-instances` 를 노트북/콘솔에서 돌려 SG 를 직접 확인해야 한다. **스트림을 막는 1순위 용의자다.**
3. **노트북의 진짜 공인 IP 를 모른다.** SSH 출발지 `13.209.1.61` 은 AWS 서울 대역이라 노트북 IP 가 아니다.
   반드시 노트북에서 `curl -s https://checkip.amazonaws.com` 으로 확인할 것.
4. **WebRTC 클라이언트 2.0.0 다운로드 URL 은 NVIDIA 6.0.1 문서에서 옮겨 적은 것이고 실제로 받아보지 않았다.**
   (참고: 커뮤니티 글에는 1.1.5 를 쓰는 사례도 보인다. 다운로드 페이지의 최신 링크를 우선한다.)
5. **브라우저 전용 web viewer(:8210, Docker Compose) 경로는 시도하지 않았다.**
   NGC 이미지에 web viewer 가 들어 있지 않아 `github.com/isaac-sim/IsaacSim` 를 `v6.0.1` 태그로
   clone 하고 npm/vite 빌드를 해야 한다. 게다가 **8210 을 열어도 영상은 여전히 UDP 47998 로 온다고 보는 게
   내 해석**이며, 문서가 이를 명시하지 않았다. 즉 "8210 만 터널링하면 해결"은 **검증되지 않은 희망**이다.
6. **Tailscale/WireGuard 경로는 제3자 블로그 사례에 근거한 것이고 여기서 검증하지 않았다.**
   원리상(UDP 터널) 맞지만, Tailscale 이 직접 UDP 경로를 못 뚫고 DERP(TCP/443) 릴레이로 폴백하면
   지연이 커진다.
7. **드라이버 595.71.05 로 스트리밍이 깨끗하게 되는지는 미확인.** 이미지의 `MIN_DRIVER_VERSION=570.169`
   보다 높고 6.0.1 요구사항의 테스트 드라이버(595.58.03)와 같은 브랜치지만, NVIDIA 는
   *"The latest NVIDIA drivers may not be fully supported for some features like livestreaming"* 라는
   단서를 달아 두었다.
8. ~~USD 의 DOF 수를 GUI 에서 직접 세어보지 못했다.~~ **해결: `check_stability.py` 가 articulation API 로 10 개 관절을 확인** (GUI 대신). 아래 "revolute 7개" 는 도저·붐스윙 관절 추가 전 수치다. 변환 성공과 링크·articulation root 존재는 확인했지만
   Articulation Inspector 는 GUI 가 필요하다. URDF 검증기 기준 revolute 7개가 기대값이다.
9. **`impl/_urdf.py` 호환 shim** 이 6.0.1 에 남아 있어 `from ...urdf.impl import _urdf` 로
   기존 스크립트를 살릴 수 있을지도 모르나, 그 shim 은 클래스 형태이고 API 시그니처가 달라
   **시도하지 않았다.** 새 스크립트를 쓰는 편이 확실하다.
10. **NGC 의 `latest` 태그는 이제 6.1.0 이다.** 이 문서의 모든 내용은 **6.0.1 기준**으로만 검증됐다.
    누군가 `:latest` 를 pull 하면 6.1.0 을 받게 되고, 플래그·확장 이름이 또 바뀌었을 수 있다.
    **항상 태그를 `6.0.1` 로 명시해서 실행할 것.**
11. `/home/ubuntu/docker/isaac-sim/config` 에 **다른 사용자가 남긴 설정**이 있을 수 있다.
    마지막 기록자는 6.0 'Isaac-Sim Streaming' 앱이라 버전 불일치는 아니지만, 내가 선택하지 않은
    설정(예: 고정된 publicIp, Nucleus 서버)이 남아 있을 가능성은 배제하지 못했다.
12. 4.x 잔재인 `cache/{glcache,kit,ov,pip}` (root 소유, 8월 11일자)는 6.0 이 **쓰지 않으므로** 그대로 뒀다.
    지우는 게 안전하다고 보지만 **공용 디렉토리라 건드리지 않았다.**
13. ~~`scripts/urdf_to_usd.py` 의 6.0.1 재작성본을 실제로 돌려보지 않았다~~ **해결: 09-11 ~ 09-14 반복 사용, §6.3 이 그 절차다.** 아래 원문은 기록용.
    `run_isaac.sh convert` 가 부르는 건 이 파일이다. 성공/실패를 확인한 뒤 §6.0 과 §7 의
    `ImportError` 항목을 갱신할 것. 그때까지는 §6.3 의 `urdf_to_usd_isaac60.py` 경로가
    **실제로 성공을 확인한** 유일한 경로다.
14. **`run_isaac.sh` 의 컨테이너 검사는 "데몬 기본 런타임이 nvidia + `NVIDIA_VISIBLE_DEVICES` 만으로
    GPU 를 받은 컨테이너"는 못 잡는다.** 지금 이 호스트는 `docker info` 상 `default-runtime=runc` 라
    해당 경로가 없어서 문제가 안 되지만, 데몬 설정이 바뀌면 `nvidia-smi` 기반 검사만 남는다.
