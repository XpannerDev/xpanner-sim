#!/usr/bin/env bash
#
# run_isaac.sh -- thin wrapper around the Isaac Sim 6.0.1 docker invocation.
#
# This machine has NO native Isaac Sim and no host-side python.sh: Isaac Sim exists
# only as the image nvcr.io/nvidia/isaac-sim:6.0.1. This wrapper encodes the one
# documented `docker run` line (NVIDIA, Isaac Sim 6.0.1 "Container Installation")
# plus the guard rails this box needs, because the GPU is SHARED with other people.
#
# SUBCOMMANDS
#   shell     interactive bash inside the container (you drive python.sh yourself)
#   stream    headless Isaac Sim + WebRTC livestream (runheadless.sh)
#   convert   run scripts/urdf_to_usd.py under /isaac-sim/python.sh; all remaining
#             arguments are forwarded to it verbatim
#
# GUARD RAILS
#   * the container name defaults to isaac-sim-$USER-$$ -- the $$ (this shell's PID)
#     is what actually makes it unique.  EVERY human on this box logs in as the same
#     unix account (`ubuntu`), so isaac-sim-$USER alone collides between people and
#     is NOT a uniqueness guarantee.  Set ISAAC_CONTAINER to pin a stable name.
#   * refuses to start if ANY other container is holding Isaac Sim, the GPU, or the
#     WebRTC ports -- enumerated positively over every running container, not by
#     image tag or name substring -- and prints who/what is holding it.
#     NOT overridable by --force: that container is probably somebody else's.
#   * refuses to start if the WebRTC ports are already bound (stream only).
#     NOT overridable by --force either -- stealing a bound port can only break the
#     person who got there first.  Move your own ports with the env vars below.
#   * refuses to start if nvidia-smi reports significant GPU memory already in use
#     (this one IS overridable with --force, for when the memory in use is yours)
#   * refuses to start if a container with our own name already exists.  If that
#     container is RUNNING it is treated as somebody else's live session and the
#     script never suggests removing it; only an Exited one gets a removal hint.
#
# WHY --network=host AND NO -p FLAGS
#   NVIDIA, Isaac Sim 6.0.1 container docs: "--network=host is required for WebRTC
#   livestreaming. ... Docker bridge networking (-p port publishing) does not work
#   because the host IP is not available inside the container's network namespace --
#   signaling may connect, but the video stream will not." Ports are opened in the
#   AWS security group, never with -p.
#
# ENVIRONMENT OVERRIDES
#   ISAAC_IMAGE            default nvcr.io/nvidia/isaac-sim:6.0.1
#   ISAAC_CONTAINER        default isaac-sim-$USER-$$ (the PID keeps it unique on a
#                          single-account box); set it to pin a stable name
#   ISAACSIM_HOST          default: the EC2 public IP via IMDSv2 (stream only)
#   ISAACSIM_SIGNAL_PORT   default 49100  (TCP, WebRTC signaling)
#   ISAACSIM_STREAM_PORT   default 47998  (UDP, WebRTC media)
#   GPU_MEM_LIMIT_MIB      default 1024 -- above this, refuse without --force
#
set -euo pipefail

IMAGE="${ISAAC_IMAGE:-nvcr.io/nvidia/isaac-sim:6.0.1}"
# NOTE: the $$ is load-bearing.  This box has exactly one login account (`ubuntu`),
# so `isaac-sim-${USER}` is the SAME string for every human here -- deriving
# uniqueness from $USER is a no-op.  The shell PID is what actually separates two
# concurrent sessions.  Pin a stable name with ISAAC_CONTAINER when you want one.
CONTAINER="${ISAAC_CONTAINER:-isaac-sim-${USER:-$(id -un)}-$$}"
SIGNAL_PORT="${ISAACSIM_SIGNAL_PORT:-49100}"
STREAM_PORT="${ISAACSIM_STREAM_PORT:-47998}"
GPU_MEM_LIMIT_MIB="${GPU_MEM_LIMIT_MIB:-1024}"

# Host paths. REPO_DIR is this repo; it is mounted at /work/xpanner-sim.
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
REPO_NAME="$(basename -- "${REPO_DIR}")"
ISAAC_DATA="${ISAAC_DATA:-/home/ubuntu/docker/isaac-sim}"
HUB_CACHE="${HUB_CACHE:-/home/ubuntu/.cache/ov/hub}"

FORCE=0

die() { printf '%s\n' "$*" >&2; exit 1; }
note() { printf '[run_isaac] %s\n' "$*" >&2; }

usage() {
    cat >&2 <<EOF
usage: $(basename -- "$0") {shell|stream|convert} [--force] [args...]

  shell              interactive bash in ${IMAGE}
  stream             headless Isaac Sim + WebRTC livestream (runheadless.sh)
  convert [args...]  /isaac-sim/python.sh /work/${REPO_NAME}/scripts/urdf_to_usd.py [args...]

  --force            skip ONLY the "GPU is busy" refusal (nvidia-smi threshold).
                     It does NOT skip the other-container check or the port check:
                     those protect other people, not you.

  container name : ${CONTAINER}   (override: ISAAC_CONTAINER=...)
  repo mount     : ${REPO_DIR} -> /work/${REPO_NAME}

examples:
  $(basename -- "$0") convert --xacro /work/${REPO_NAME}/assets/ecr88/urdf/ecr88.urdf.xacro \\
                              --output /work/${REPO_NAME}/assets/ecr88/usd/ecr88.usd
  $(basename -- "$0") stream
  $(basename -- "$0") shell
EOF
    exit 2
}

# --------------------------------------------------------------------------- #
# Guard: is somebody else already holding Isaac Sim, the GPU, or our ports?
#
# This used to be two `docker ps --filter` calls, and BOTH of them missed the case
# that actually matters:
#
#   --filter "ancestor=${IMAGE}"  resolves to exactly one image ID.  A container
#       started from a different tag -- e.g. `:latest`, which on NGC is now 6.1.0
#       (docs/ISAAC_SIM_REMOTE.md section 8 item 10) -- has a different ID and was
#       invisible.
#   --filter 'name=isaac-sim'     is a NAME SUBSTRING.  This very host carries a
#       container called `vpick-stream` built from the Isaac Sim image; that name
#       does not contain "isaac-sim", so it was invisible too.
#
# A container that is both differently named AND from a different image ID slipped
# past both filters, and the wrapper then ran `--gpus all --network=host` straight
# over a live session, taking tcp/49100 with it.
#
# So: enumerate EVERY running container and test it positively -- by image
# reference, by image repo tags, by our own image ID, by whether it holds a GPU,
# and by whether it binds our streaming ports.
#
# Known gap (deliberate): a container that gets a GPU purely through a daemon-level
# default runtime plus NVIDIA_VISIBLE_DEVICES, with no DeviceRequests and no
# per-container runtime, is not detected here.  `docker info` on this host reports
# default-runtime=runc, so that path does not apply; check_gpu_free (nvidia-smi) is
# the backstop if it ever does.
# --------------------------------------------------------------------------- #
check_no_other_isaac() {
    local ids
    ids="$(docker ps -q 2>/dev/null || true)"
    if [ -z "${ids}" ]; then
        note 'no other containers are running'
        return 0
    fi

    local our_image_id
    our_image_id="$(docker image inspect -f '{{.Id}}' "${IMAGE}" 2>/dev/null || true)"

    # Fields are '|'-separated; the only field that could contain a '|' is the
    # trailing command, and that one is never parsed.
    #
    # EVERY nil-able field is guarded with {{if}}. Go templates abort the WHOLE
    # `docker inspect` run on the first template error -- `len` of a nil
    # DeviceRequests and `join` of a nil Entrypoint both do that -- and a plain
    # `ubuntu:22.04` container has both. Losing the output that way used to make
    # this guard silently pass. It is inspected one container at a time so that one
    # unreadable container cannot blind us to the rest.
    # Go template braces are literal; they must not be expanded by the shell.
    # shellcheck disable=SC2016
    local fmt
    fmt='{{.Id}}|{{.Name}}|{{.Config.Image}}|{{.Image}}|{{.HostConfig.NetworkMode}}'
    fmt="${fmt}"'|{{if .HostConfig.DeviceRequests}}{{len .HostConfig.DeviceRequests}}{{else}}0{{end}}'
    fmt="${fmt}"'|{{.HostConfig.Runtime}}'
    # shellcheck disable=SC2016
    fmt="${fmt}"'|{{range $p, $v := .HostConfig.PortBindings}}{{$p}} {{end}}'
    fmt="${fmt}"'|{{.Config.User}}|{{.State.Status}} since {{.State.StartedAt}}'
    fmt="${fmt}"'|{{if .Config.Entrypoint}}{{join .Config.Entrypoint " "}}{{end}}'
    fmt="${fmt}"' {{if .Config.Cmd}}{{join .Config.Cmd " "}}{{end}}'

    local lines='' unreadable='' one id
    while IFS= read -r id; do
        [ -n "${id}" ] || continue
        one="$(docker inspect --format "${fmt}" "${id}" 2>/dev/null || true)"
        if [ -z "${one}" ]; then
            # Fail CLOSED: a running container we cannot read is counted as a
            # conflict rather than waved through. This guard protects other people,
            # so "I could not tell" has to mean "no".
            unreadable="${unreadable}$(printf '%-14s %-26s %-24s %-9s %-28s %s' \
                "${id:0:12}" '?' 'could not inspect it' '?' 'running' '?')"$'\n'
            continue
        fi
        lines="${lines}${one}"$'\n'
    done <<<"${ids}"

    local -A TAGS_OF=()
    local conflicts="${unreadable}" host_net_only=''
    local cid name cfg_image img_id netmode devreq runtime ports cuser status cmd
    local tags reason haystack

    while IFS='|' read -r cid name cfg_image img_id netmode devreq runtime ports cuser status cmd; do
        [ -n "${cid}" ] || continue
        name="${name#/}"

        # Repo tags of the image this container actually runs, so a container
        # started by digest or bare image ID is still identifiable.
        if [ -z "${TAGS_OF[${img_id}]+set}" ]; then
            TAGS_OF[${img_id}]="$(docker image inspect -f '{{join .RepoTags ","}}' "${img_id}" 2>/dev/null || true)"
        fi
        tags="${TAGS_OF[${img_id}]}"

        reason=''
        haystack="$(printf '%s %s %s' "${cfg_image}" "${tags}" "${name}" | tr '[:upper:]' '[:lower:]')"
        case "${haystack}" in
            *isaac-sim*|*isaac_sim*|*isaacsim*) reason='runs an Isaac Sim image' ;;
        esac
        if [ -z "${reason}" ] && [ -n "${our_image_id}" ] && [ "${img_id}" = "${our_image_id}" ]; then
            reason='runs the same image we would'
        fi
        if [ -z "${reason}" ]; then
            case " ${ports}" in
                *" ${SIGNAL_PORT}/tcp "*) reason="binds tcp/${SIGNAL_PORT}" ;;
                *" ${STREAM_PORT}/udp "*) reason="binds udp/${STREAM_PORT}" ;;
            esac
        fi
        if [ -z "${reason}" ] && [ -n "${devreq}" ] && [ "${devreq}" != "0" ]; then
            reason='holds a GPU (--gpus)'
        fi
        if [ -z "${reason}" ] && [ "${runtime}" = "nvidia" ]; then
            reason='holds a GPU (--runtime=nvidia)'
        fi

        if [ -n "${reason}" ]; then
            conflicts="${conflicts}$(printf '%-14s %-26s %-24s %-9s %-28s %s' \
                "${cid:0:12}" "${name:0:26}" "${reason:0:24}" "${cuser:-?}" "${status:0:28}" "${cmd}")"$'\n'
        elif [ "${netmode}" = "host" ]; then
            host_net_only="${host_net_only}  ${cid:0:12}  ${name}  (${cfg_image})"$'\n'
        fi
    done <<<"${lines}"

    if [ -n "${host_net_only}" ]; then
        note 'other host-network containers are running; they could still own a port:'
        printf '%s' "${host_net_only}" >&2
    fi

    if [ -z "${conflicts}" ]; then
        note 'no other Isaac Sim / GPU / streaming-port container is running'
        return 0
    fi

    printf '\n[run_isaac] REFUSING TO START: another container already holds Isaac Sim,\n' >&2
    printf '            the GPU, or the streaming ports on this host.\n\n' >&2
    printf '%-14s %-26s %-24s %-9s %-28s %s\n' 'CONTAINER' 'NAME' 'WHY IT CONFLICTS' 'USER' 'STATUS' 'COMMAND' >&2
    printf '%s' "${conflicts}" >&2
    printf '\nThis is a SHARED box with ONE L4 and one set of host ports. That container is\n' >&2
    printf 'probably not yours: do NOT stop or remove it. Talk to whoever owns it, or wait.\n' >&2
    printf 'To look without touching:  docker logs --tail 50 <CONTAINER>\n' >&2
    printf '%s\n\n' '--force does not bypass this check, by design.' >&2
    exit 1
}

# --------------------------------------------------------------------------- #
# Guard: is the GPU already busy?
# --------------------------------------------------------------------------- #
check_gpu_free() {
    command -v nvidia-smi >/dev/null 2>&1 || { note "nvidia-smi not found; skipping GPU check"; return 0; }

    local used total
    used="$(nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits | head -n1 | tr -d ' ')"
    total="$(nvidia-smi --query-gpu=memory.total --format=csv,noheader,nounits | head -n1 | tr -d ' ')"
    case "${used}" in ''|*[!0-9]*) note "could not parse nvidia-smi output; skipping GPU check"; return 0;; esac

    if [ "${used}" -le "${GPU_MEM_LIMIT_MIB}" ]; then
        note "GPU free: ${used} MiB / ${total} MiB in use"
        return 0
    fi

    printf '\n[run_isaac] REFUSING TO START: the GPU already has %s MiB of %s MiB in use\n' "${used}" "${total}" >&2
    printf '            (threshold GPU_MEM_LIMIT_MIB=%s).\n\n' "${GPU_MEM_LIMIT_MIB}" >&2
    nvidia-smi --query-compute-apps=pid,used_memory,process_name --format=csv >&2 || true
    printf '\nThis box has ONE L4 shared between people. Starting a second Isaac Sim will\n' >&2
    printf 'likely OOM both sessions. Re-run with --force only if you know that memory is yours.\n\n' >&2
    exit 1
}

# --------------------------------------------------------------------------- #
# Guard: are the WebRTC ports free? (stream only)
#
# Deliberately NOT skipped by --force. --force means "that GPU memory is mine";
# it can never mean "that bound socket is mine", and binding a port out from under
# whoever got there first only breaks them. The escape hatch is moving YOUR ports.
# --------------------------------------------------------------------------- #
check_ports_free() {
    command -v ss >/dev/null 2>&1 || { note "ss not found; skipping port check"; return 0; }
    local busy
    busy="$(ss -tulnH 2>/dev/null | awk -v s=":${SIGNAL_PORT}$" -v m=":${STREAM_PORT}$" \
            '$5 ~ s || $5 ~ m {print}')"
    if [ -z "${busy}" ]; then
        note "streaming ports free: tcp/${SIGNAL_PORT}, udp/${STREAM_PORT}"
        return 0
    fi

    printf '\n[run_isaac] REFUSING TO START: a streaming port is already bound.\n' >&2
    printf '%s\n' "${busy}" >&2
    printf '\nOnly one Isaac Sim can own tcp/%s + udp/%s on this host (--network=host).\n' \
        "${SIGNAL_PORT}" "${STREAM_PORT}" >&2
    printf 'Pick your own pair instead, e.g.:\n' >&2
    printf '  ISAACSIM_SIGNAL_PORT=49200 ISAACSIM_STREAM_PORT=48100 %s stream\n' "$0" >&2
    printf '(then open those in the AWS security group, sourced to your laptop IP /32).\n' >&2
    printf '%s\n\n' '--force does not bypass this check, by design.' >&2
    exit 1
}

# --------------------------------------------------------------------------- #
# EC2 public IP via IMDSv2. Cloud VMs do not carry their public IP on any local
# interface, so `hostname -I` returns the wrong (private) address.
# --------------------------------------------------------------------------- #
ec2_public_ip() {
    local token ip
    token="$(curl -s --max-time 2 -X PUT 'http://169.254.169.254/latest/api/token' \
             -H 'X-aws-ec2-metadata-token-ttl-seconds: 21600' || true)"
    if [ -z "${token}" ]; then
        return 1
    fi
    ip="$(curl -s --max-time 2 -H "X-aws-ec2-metadata-token: ${token}" \
          'http://169.254.169.254/latest/meta-data/public-ipv4' || true)"
    if [ -z "${ip}" ]; then
        return 1
    fi
    printf '%s\n' "${ip}"
}

# --------------------------------------------------------------------------- #
# The documented mount set for Isaac Sim 6.0 (SIX dirs + the hub cache), plus this
# repo. The 4.x-era cache/{kit,ov,pip,glcache} split is gone in 6.0; those dirs
# still exist on this host owned by root and are deliberately NOT mounted --
# NVIDIA's troubleshooting names stale volume mounts as a cause of livestream
# failures and config errors.
# --------------------------------------------------------------------------- #
docker_mounts() {
    printf '%s\0' \
        -v "${ISAAC_DATA}/cache/main:/isaac-sim/.cache:rw" \
        -v "${ISAAC_DATA}/cache/computecache:/isaac-sim/.nv/ComputeCache:rw" \
        -v "${ISAAC_DATA}/logs:/isaac-sim/.nvidia-omniverse/logs:rw" \
        -v "${ISAAC_DATA}/config:/isaac-sim/.nvidia-omniverse/config:rw" \
        -v "${ISAAC_DATA}/data:/isaac-sim/.local/share/ov/data:rw" \
        -v "${ISAAC_DATA}/pkg:/isaac-sim/.local/share/ov/pkg:rw" \
        -v "${HUB_CACHE}:/var/cache/hub:rw" \
        -v "${REPO_DIR}:/work/${REPO_NAME}:rw"
}

preflight() {
    command -v docker >/dev/null 2>&1 || die '[run_isaac] docker is not installed or not on PATH'
    docker image inspect "${IMAGE}" >/dev/null 2>&1 \
        || die "[run_isaac] image not present locally: ${IMAGE} (docker pull it first)"
    [ -d "${ISAAC_DATA}" ] || die "[run_isaac] missing bind-mount root: ${ISAAC_DATA}"
    local d
    for d in cache/main cache/computecache logs config data pkg; do
        [ -d "${ISAAC_DATA}/${d}" ] || die "[run_isaac] missing bind-mount dir: ${ISAAC_DATA}/${d}"
    done
    mkdir -p "${HUB_CACHE}"
    check_name_free
}

# --------------------------------------------------------------------------- #
# Guard: is our own container name taken?
#
# `docker ps -a` lists RUNNING containers too, so the old one-liner here told the
# user to `docker rm -f` a name that might well be somebody's live session -- and
# because this box has a single login account, `isaac-sim-$USER` was the same name
# for everyone, which made that the NORMAL path rather than an edge case.
# Running and exited are now treated as completely different situations.
# --------------------------------------------------------------------------- #
check_name_free() {
    local existing
    existing="$(docker ps -a --filter "name=^/${CONTAINER}$" --format '{{.ID}}' 2>/dev/null | head -n1 || true)"
    if [ -z "${existing}" ]; then
        return 0
    fi

    local running
    running="$(docker inspect -f '{{.State.Running}}' "${existing}" 2>/dev/null || echo 'unknown')"

    if [ "${running}" != "false" ]; then
        # true, or we could not tell -- either way, assume it is alive and NOT ours.
        printf '\n[run_isaac] REFUSING TO START: a container named %s is already RUNNING.\n\n' "${CONTAINER}" >&2
        docker ps --filter "name=^/${CONTAINER}$" \
            --format 'table {{.ID}}\t{{.Names}}\t{{.Image}}\t{{.Status}}' >&2 || true
        printf '\nEveryone on this box logs in as the same unix account, so this name may well\n' >&2
        printf 'belong to somebody else, mid-session. Do NOT stop or remove it.\n' >&2
        printf 'Look:      docker logs --tail 50 %s\n' "${CONTAINER}" >&2
        printf 'Or just pick your own name and start again:\n' >&2
        printf '  ISAAC_CONTAINER=%s-%s %s ...\n\n' "${CONTAINER}" "$$" "$0" >&2
        exit 1
    fi

    # Exited: it holds no GPU and no port, only the name.
    local status image
    status="$(docker inspect \
        -f '{{.State.Status}}{{if eq .State.Status "exited"}}, exited {{.State.FinishedAt}}{{end}}' \
        "${existing}" 2>/dev/null || true)"
    image="$(docker inspect -f '{{.Config.Image}}' "${existing}" 2>/dev/null || true)"
    printf '\n[run_isaac] a container named %s already exists but is NOT running.\n' "${CONTAINER}" >&2
    printf '            %s  (%s, %s)\n\n' "${existing:0:12}" "${image}" "${status}" >&2
    printf 'It holds no GPU and no port -- only the name. If it is yours, remove it:\n' >&2
    printf '  docker rm %s\n' "${CONTAINER}" >&2
    printf 'If you do not recognise it (check the timestamp above), leave it alone and use\n' >&2
    printf 'a different name instead:\n' >&2
    printf '  ISAAC_CONTAINER=%s-%s %s ...\n\n' "${CONTAINER}" "$$" "$0" >&2
    exit 1
}

# --------------------------------------------------------------------------- #
# main
# --------------------------------------------------------------------------- #
[ $# -ge 1 ] || usage
SUBCOMMAND="$1"; shift

# --force may appear anywhere before the forwarded args.
REST=()
for arg in "$@"; do
    if [ "${arg}" = "--force" ]; then
        FORCE=1
    else
        REST+=("${arg}")
    fi
done
set -- "${REST[@]}"

case "${SUBCOMMAND}" in
    shell|stream|convert) ;;
    -h|--help|help) usage ;;
    *) printf '[run_isaac] unknown subcommand: %s\n\n' "${SUBCOMMAND}" >&2; usage ;;
esac

# ORDER MATTERS. The checks that protect OTHER PEOPLE run first, so that the
# "somebody else is using this box" message is what the user actually sees. When
# preflight ran first it died on "a container named X already exists -- docker rm -f
# X" and the polite guard below never got a chance to speak.
command -v docker >/dev/null 2>&1 || die '[run_isaac] docker is not installed or not on PATH'

check_no_other_isaac

# Not `[ ... ] && check_ports_free`: a false test as the last command in a branch
# would trip `set -e`.
if [ "${SUBCOMMAND}" = "stream" ]; then
    check_ports_free
fi

if [ "${FORCE}" -eq 1 ]; then
    note 'WARNING: --force given; skipping the GPU-busy refusal only'
    note '         (the other-container and port checks are never skipped)'
else
    check_gpu_free
fi

preflight

mapfile -d '' -t MOUNTS < <(docker_mounts)

COMMON=(
    --name "${CONTAINER}"
    --rm
    --gpus all
    --network=host
    -e ACCEPT_EULA=Y
    -e PRIVACY_CONSENT=Y
    -u 1234:1234
    -w /isaac-sim
    "${MOUNTS[@]}"
)

case "${SUBCOMMAND}" in
    shell)
        note "starting interactive shell in ${CONTAINER} (repo at /work/${REPO_NAME})"
        exec docker run -it --entrypoint bash "${COMMON[@]}" "${IMAGE}"
        ;;

    convert)
        [ $# -ge 1 ] || die '[run_isaac] convert needs arguments, e.g. --xacro ... --output ...'
        note "converting via /isaac-sim/python.sh /work/${REPO_NAME}/scripts/urdf_to_usd.py"
        exec docker run -i --entrypoint /isaac-sim/python.sh "${COMMON[@]}" "${IMAGE}" \
            "/work/${REPO_NAME}/scripts/urdf_to_usd.py" "$@"
        ;;

    stream)
        HOST_IP="${ISAACSIM_HOST:-}"
        if [ -z "${HOST_IP}" ]; then
            HOST_IP="$(ec2_public_ip || true)"
        fi
        if [ -z "${HOST_IP}" ]; then
            die '[run_isaac] could not determine the public IP (IMDSv2 unreachable). Set ISAACSIM_HOST=<public-ip> explicitly.'
        fi
        note "livestream host ${HOST_IP}, signal tcp/${SIGNAL_PORT}, media udp/${STREAM_PORT}"
        note 'connect the Isaac Sim WebRTC Streaming Client 2.0.0 AFTER the log says:'
        note '  Isaac Sim Full Streaming App is loaded.'
        note 'the stream is UNAUTHENTICATED: open those ports to your laptop IP /32 only.'
        exec docker run -it --entrypoint /isaac-sim/runheadless.sh "${COMMON[@]}" \
            -e "ISAACSIM_HOST=${HOST_IP}" \
            -e "ISAACSIM_SIGNAL_PORT=${SIGNAL_PORT}" \
            -e "ISAACSIM_STREAM_PORT=${STREAM_PORT}" \
            "${IMAGE}" -v "$@"
        ;;
esac
