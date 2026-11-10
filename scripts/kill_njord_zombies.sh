#!/usr/bin/env bash
# Report, and optionally clean up, zombie processes associated with this ROS
# workspace.  A zombie cannot be killed directly; its parent must reap it.
set -u

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd -P)"
readonly SELF_PID="$$"
readonly CURRENT_UID="$(id -u)"

apply=0
force=0
wait_seconds=2

usage() {
    cat <<EOF
Usage: $(basename "$0") [--dry-run] [--apply] [--force]

Find zombie (STAT=Z) processes belonging to the current user and related to:
  ${WORKSPACE_DIR}

Default: report candidates only.
  --apply  send SIGCHLD to each eligible parent, then SIGTERM if necessary
  --force  after --apply, send SIGKILL if SIGTERM did not stop the parent
  --dry-run explicitly select report-only mode
EOF
}

log() {
    printf '[%s] %s\n' "$1" "$2"
}

is_pid() {
    [[ "$1" =~ ^[0-9]+$ ]] && (( 1 < $1 && $1 < 4194304 ))
}

proc_cmdline() {
    local pid=$1 value
    [[ -r "/proc/${pid}/cmdline" ]] || return 1
    value=$(tr '\0' ' ' < "/proc/${pid}/cmdline" 2>/dev/null) || return 1
    value=${value% }
    [[ -n "$value" ]] || value=$(ps -p "$pid" -o comm= 2>/dev/null || true)
    printf '%s' "$value"
}

proc_cwd() {
    local pid=$1
    readlink -e "/proc/${pid}/cwd" 2>/dev/null || true
}

proc_exe() {
    local pid=$1
    readlink -e "/proc/${pid}/exe" 2>/dev/null || true
}

path_is_workspace() {
    local path=$1
    [[ "$path" == "$WORKSPACE_DIR" || "$path" == "$WORKSPACE_DIR"/* ]]
}

# A process is related when its current executable/cwd/command, or one of its
# parents, identifies the workspace.  ROS_DISTRO alone is deliberately not a
# sufficient match: many unrelated ROS programs may run as the same user.
is_related() {
    local pid=$1 ppid=0 cmd cwd exe depth=0

    while is_pid "$pid" && (( depth < 32 )); do
        [[ "$pid" != "$SELF_PID" ]] || return 1

        ppid=$(ps -p "$pid" -o ppid= 2>/dev/null | awk '{print $1}')
        cmd=$(proc_cmdline "$pid")
        cwd=$(proc_cwd "$pid")
        exe=$(proc_exe "$pid")

        if path_is_workspace "$cwd" || path_is_workspace "$exe" ||
            [[ "$cmd" == *"$WORKSPACE_DIR"* ]]; then
            return 0
        fi

        [[ "$ppid" =~ ^[0-9]+$ ]] || break
        (( ppid > 1 )) || break
        pid=$ppid
        ((depth++))
    done
    return 1
}

parent_is_eligible() {
    local pid=$1 uid cmd
    [[ "$pid" =~ ^[0-9]+$ ]] || return 1
    (( pid > 1 && pid != SELF_PID )) || return 1
    uid=$(ps -p "$pid" -o uid= 2>/dev/null | awk '{print $1}')
    [[ "$uid" == "$CURRENT_UID" ]] || return 1
    cmd=$(proc_cmdline "$pid")
    [[ "$cmd" != *"kill_njord_zombies.sh"* ]] || return 1

    # A shell sitting in the workspace is not sufficient evidence that it
    # owns the ROS process.  Never terminate an interactive shell by default.
    case "$cmd" in
        bash|/bin/bash|-bash|zsh|/bin/zsh|-zsh|sh|/bin/sh|-sh|fish|/usr/bin/fish)
            return 1
            ;;
    esac

    is_related "$pid"
}

parent_still_exists() {
    local pid=$1
    kill -0 "$pid" 2>/dev/null
}

reap_parent() {
    local parent_pid=$1 zombie_pid=$2 cmd

    if ! parent_is_eligible "$parent_pid"; then
        log WARN "parent PID ${parent_pid} is not an eligible workspace-owned parent; no signal sent"
        return
    fi

    cmd=$(proc_cmdline "$parent_pid")
    log INFO "zombie PID ${zombie_pid} -> parent PID ${parent_pid}: ${cmd}"

    (( apply )) || return

    if kill -CHLD "$parent_pid" 2>/dev/null; then
        log INFO "sent SIGCHLD to parent PID ${parent_pid}"
    else
        log WARN "could not send SIGCHLD to parent PID ${parent_pid}"
        return
    fi

    sleep "$wait_seconds"
    if ! parent_still_exists "$parent_pid"; then
        log INFO "parent PID ${parent_pid} exited; init/systemd can reap the zombie"
        return
    fi

    if kill -TERM "$parent_pid" 2>/dev/null; then
        log WARN "parent PID ${parent_pid} did not reap; sent SIGTERM"
    else
        log WARN "could not send SIGTERM to parent PID ${parent_pid}"
        return
    fi

    sleep "$wait_seconds"
    if (( force )) && parent_still_exists "$parent_pid"; then
        if kill -KILL "$parent_pid" 2>/dev/null; then
            log WARN "parent PID ${parent_pid} still existed; sent SIGKILL"
        else
            log WARN "could not send SIGKILL to parent PID ${parent_pid}"
        fi
    fi
}

while (($#)); do
    case "$1" in
        --dry-run)
            apply=0
            ;;
        --apply)
            apply=1
            ;;
        --force)
            force=1
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            printf 'Unknown option: %s\n\n' "$1" >&2
            usage >&2
            exit 2
            ;;
    esac
    shift
done

if (( force && !apply )); then
    log WARN '--force has no effect without --apply'
fi

found=0
while read -r pid ppid uid stat rest; do
    [[ "$uid" == "$CURRENT_UID" ]] || continue
    [[ "$stat" == Z* || "$stat" == *Z* ]] || continue
    [[ "$pid" != "$SELF_PID" ]] || continue
    is_related "$pid" || continue

    found=1
    log CANDIDATE "PID=${pid} PPID=${ppid} STAT=${stat} CMD=${rest:-<defunct>}"
    reap_parent "$ppid" "$pid"
done < <(ps -e -o pid= -o ppid= -o uid= -o stat= -o args= 2>/dev/null)

if (( !found )); then
    log INFO "no workspace-related zombies found for UID ${CURRENT_UID}"
elif (( !apply )); then
    log INFO 'dry-run only; use --apply to signal eligible parents'
fi
