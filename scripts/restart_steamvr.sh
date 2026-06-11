#!/usr/bin/env bash
set -euo pipefail

APP_ID="${STEAMVR_APP_ID:-250820}"
TERM_TIMEOUT="${STEAMVR_TERM_TIMEOUT:-8}"
START_TIMEOUT="${STEAMVR_START_TIMEOUT:-30}"
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

  log "Starting SteamVR via $steam steam://rungameid/$APP_ID"
  mkdir -p "$(dirname "$RESTART_LOG")"
  nohup "$steam" "steam://rungameid/$APP_ID" >>"$RESTART_LOG" 2>&1 &

  local deadline=$((SECONDS + START_TIMEOUT))
  while (( SECONDS < deadline )); do
    if steamvr_is_running; then
      log "SteamVR started."
      steamvr_pids | sort -n -u | xargs -r printf '  pid=%s\n'
      return
    fi
    sleep 1
  done

  log "WARNING: SteamVR launch requested, but no SteamVR process appeared within ${START_TIMEOUT}s."
  log "Log: $RESTART_LOG"
}

main() {
  if steamvr_is_running; then
    log "SteamVR is running; restarting."
    stop_steamvr
  else
    log "SteamVR is not running; starting from scratch."
  fi
  start_steamvr
}

main "$@"
