#!/usr/bin/env bash
set -euo pipefail

APP_ID="${STEAMVR_APP_ID:-250820}"
TERM_TIMEOUT="${STEAMVR_TERM_TIMEOUT:-8}"
START_TIMEOUT="${STEAMVR_START_TIMEOUT:-30}"
STEAM_START_TIMEOUT="${STEAM_START_TIMEOUT:-45}"
STEAM_READY_SECONDS="${STEAM_READY_SECONDS:-15}"
START_STABLE_SECONDS="${STEAMVR_START_STABLE_SECONDS:-8}"
RESTART_LOG="${STEAMVR_RESTART_LOG:-/tmp/steamvr-restart.log}"

log() {
  printf '%(%H:%M:%S)T %s\n' -1 "$*"
}

steam_cmd() {
  if [[ -n "${STEAM_CMD:-}" ]]; then
    printf '%s\n' "$STEAM_CMD"
    return
  fi
  if command -v steam >/dev/null 2>&1; then
    command -v steam
    return
  fi
  if [[ -x /usr/games/steam ]]; then
    printf '%s\n' /usr/games/steam
    return
  fi
  if [[ -x "$HOME/.steam/debian-installation/steam.sh" ]]; then
    printf '%s\n' "$HOME/.steam/debian-installation/steam.sh"
    return
  fi
  return 1
}

setup_gui_env() {
  if [[ -z "${DISPLAY:-}" && -S /tmp/.X11-unix/X0 ]]; then
    export DISPLAY=:0
  fi
  if [[ -z "${XAUTHORITY:-}" && -r "$HOME/.Xauthority" ]]; then
    export XAUTHORITY="$HOME/.Xauthority"
  fi
}

steam_pids() {
  ps -eo pid=,cmd= | awk -v self="$$" '
    {
      pid = $1
      $1 = ""
      cmd = substr($0, 2)
      if (pid == self) {
        next
      }
      if (cmd ~ /\/steam([[:space:]]|$)/ ||
          cmd ~ /\/steamwebhelper([[:space:]]|$)/ ||
          cmd ~ /steamwebhelper /) {
        print pid
      }
    }
  '
}

steam_is_running() {
  steam_pids | grep -q .
}

steamvr_pids() {
  ps -eo pid=,cmd= | awk -v self="$$" -v app="$APP_ID" '
    {
      pid = $1
      $1 = ""
      cmd = substr($0, 2)
      if (pid == self) {
        next
      }
      if (cmd ~ /\/steamapps\/common\/SteamVR\// ||
          cmd ~ ("SteamLaunch AppId=" app)) {
        print pid
      }
    }
  '
}

steamvr_is_running() {
  steamvr_pids | grep -q .
}

vrserver_pids() {
  ps -eo pid=,cmd= | awk -v self="$$" '
    {
      pid = $1
      $1 = ""
      cmd = substr($0, 2)
      if (pid == self) {
        next
      }
      if (cmd ~ /\/SteamVR\/bin\/linux64\/vrserver([[:space:]]|$)/) {
        print pid
      }
    }
  '
}

vrserver_is_running() {
  vrserver_pids | grep -q .
}

stop_steamvr() {
  mapfile -t pids < <(steamvr_pids | sort -n -u)
  if (( ${#pids[@]} == 0 )); then
    log "SteamVR is not running."
    return
  fi

  log "Stopping SteamVR (${#pids[@]} process(es)): ${pids[*]}"
  kill -TERM "${pids[@]}" 2>/dev/null || true

  local deadline=$((SECONDS + TERM_TIMEOUT))
  while (( SECONDS < deadline )); do
    if ! steamvr_is_running; then
      log "SteamVR stopped."
      return
    fi
    sleep 1
  done

  mapfile -t pids < <(steamvr_pids | sort -n -u)
  if (( ${#pids[@]} > 0 )); then
    log "SteamVR did not stop cleanly; forcing: ${pids[*]}"
    kill -KILL "${pids[@]}" 2>/dev/null || true
    sleep 1
  fi
  log "SteamVR stopped."
}

start_steamvr() {
  local steam
  steam="$(steam_cmd)" || {
    log "ERROR: steam command not found. Set STEAM_CMD=/path/to/steam."
    exit 1
  }

  mkdir -p "$(dirname "$RESTART_LOG")"

  if ! steam_is_running; then
    log "Steam client is not running; starting it first."
    setsid -f "$steam" -silent >>"$RESTART_LOG" 2>&1

    local steam_deadline=$((SECONDS + STEAM_START_TIMEOUT))
    while (( SECONDS < steam_deadline )); do
      if steam_is_running; then
        log "Steam client started."
        break
      fi
      sleep 1
    done
    if ! steam_is_running; then
      log "ERROR: Steam client did not appear within ${STEAM_START_TIMEOUT}s."
      log "Log: $RESTART_LOG"
      exit 1
    fi
    log "Waiting ${STEAM_READY_SECONDS}s for Steam client initialization."
    sleep "$STEAM_READY_SECONDS"
  fi

  log "Starting SteamVR via $steam steam://rungameid/$APP_ID"
  "$steam" "steam://rungameid/$APP_ID" >>"$RESTART_LOG" 2>&1 &

  local deadline=$((SECONDS + START_TIMEOUT))
  while (( SECONDS < deadline )); do
    if vrserver_is_running; then
      sleep "$START_STABLE_SECONDS"
      if vrserver_is_running; then
        log "SteamVR started."
        vrserver_pids | sort -n -u | xargs -r printf '  vrserver_pid=%s\n'
        steamvr_pids | sort -n -u | xargs -r printf '  steamvr_pid=%s\n'
        return
      fi
      log "SteamVR server appeared but exited before ${START_STABLE_SECONDS}s; waiting."
    elif steamvr_is_running; then
      log "SteamVR launch process appeared; waiting for vrserver."
    fi
    if ! steam_is_running; then
      log "ERROR: Steam client exited while launching SteamVR."
      log "Log: $RESTART_LOG"
      return
    fi
    sleep 1
  done

  log "WARNING: SteamVR launch requested, but vrserver did not stay up within ${START_TIMEOUT}s."
  log "Log: $RESTART_LOG"
}

main() {
  setup_gui_env
  if [[ -z "${DISPLAY:-}" ]]; then
    log "WARNING: DISPLAY is not set; Steam may be unable to start from this terminal."
  else
    log "Using DISPLAY=$DISPLAY"
  fi

  if steamvr_is_running; then
    log "SteamVR is running; restarting."
    stop_steamvr
  else
    log "SteamVR is not running; starting from scratch."
  fi
  start_steamvr
}

main "$@"
