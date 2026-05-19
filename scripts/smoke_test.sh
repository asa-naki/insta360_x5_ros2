#!/usr/bin/env bash
# Smoke test for insta360_x5_ros2.
# Requires the camera connected and the workspace sourced.
set -u

DEVICE="${DEVICE:-/dev/video0}"
NS="${NS:-/insta360_x5}"
HZ_SEC="${HZ_SEC:-6}"
PKG="insta360_x5_ros2"

step() { echo; echo "==> $*"; }
ok()   { echo "  [OK] $*"; }
warn() { echo "  [WARN] $*" >&2; }
fail() { echo "  [FAIL] $*" >&2; FAILED=1; }

FAILED=0

# Kill any stale instance that may be holding the device.
pkill -9 -f insta360_x5_node 2>/dev/null || true
pkill -9 -f "ros2 launch ${PKG}" 2>/dev/null || true
sleep 1

step "Device present"
if [[ ! -e "$DEVICE" ]]; then
  fail "Device $DEVICE not found. Connect the Insta360 X5 and switch to UVC mode."
  exit 1
fi
ok "$DEVICE exists"

step "v4l2 formats"
if command -v v4l2-ctl >/dev/null; then
  v4l2-ctl --list-formats-ext -d "$DEVICE" | sed 's/^/    /' | head -25
else
  echo "  v4l2-ctl not installed (apt install v4l-utils) — skipping"
fi

step "Launch node"
NODE_LOG=/tmp/insta360_x5_node.log
ros2 launch "$PKG" insta360_x5.launch.py > "$NODE_LOG" 2>&1 &
NODE_PID=$!
trap "kill -INT $NODE_PID 2>/dev/null; sleep 1; kill -9 $NODE_PID 2>/dev/null; pkill -9 -f insta360_x5_node 2>/dev/null; true" EXIT
sleep 6

if ! kill -0 "$NODE_PID" 2>/dev/null; then
  fail "Node exited early — see $NODE_LOG"
  tail -n 40 "$NODE_LOG"
  exit 1
fi
ok "Node alive (pid=$NODE_PID)"

step "Topics advertised"
TOPICS=$(ros2 topic list)
for t in image_raw image_raw/compressed image_preview mode projection camera_info; do
  if echo "$TOPICS" | grep -q "^${NS}/${t}\$"; then
    ok "${NS}/${t}"
  else
    fail "Missing ${NS}/${t}"
  fi
done

probe_hz() {
  local topic="$1" minrate="$2" critical="$3"
  local log
  log=$(mktemp)
  ros2 topic hz "$topic" > "$log" 2>&1 &
  local hz_pid=$!
  sleep "$HZ_SEC"
  kill "$hz_pid" 2>/dev/null
  wait "$hz_pid" 2>/dev/null
  local rate
  rate=$(grep -oE 'average rate: [0-9.]+' "$log" | tail -1 | awk '{print $3}')
  if [[ -z "$rate" ]]; then
    if [[ "$critical" == "yes" ]]; then
      fail "$topic: no messages received"
    else
      warn "$topic: no messages received (may require DDS large-message tuning — see README)"
    fi
    rm -f "$log"
    return
  fi
  if awk -v r="$rate" -v m="$minrate" 'BEGIN{exit !(r>=m)}'; then
    ok "$topic: ${rate} Hz (>= ${minrate})"
  else
    warn "$topic: ${rate} Hz (< ${minrate})"
  fi
  rm -f "$log"
}

step "Topic rates"
# image_raw/compressed is the primary high-throughput path (MJPEG pass-through).
probe_hz "${NS}/image_raw/compressed" 25 yes
# image_raw / image_preview are bgr8 sensor_msgs/Image; large messages (~3-12 MB)
# need DDS tuning to flow reliably. Non-fatal in default DDS setup.
probe_hz "${NS}/image_raw" 25 no
probe_hz "${NS}/image_preview" 25 no

step "Latched 'mode' topic"
MODE=$(timeout 3 ros2 topic echo --once \
  --qos-durability transient_local --qos-reliability reliable \
  "${NS}/mode" 2>/dev/null | grep -oP 'data:\s*\K\S+' || true)
if [[ -n "$MODE" ]]; then
  ok "mode = $MODE"
else
  fail "no /mode message"
fi

step "Latched 'projection' topic"
PROJ=$(timeout 3 ros2 topic echo --once \
  --qos-durability transient_local --qos-reliability reliable \
  "${NS}/projection" 2>/dev/null | grep -oP 'data:\s*\K\S+' || true)
if [[ -n "$PROJ" ]]; then
  ok "projection = $PROJ"
else
  fail "no /projection message"
fi

step "Service: set_mode dual_fisheye"
if ros2 service call "${NS}/set_mode" "${PKG}/srv/SetCameraMode" \
    "{mode: 'dual_fisheye'}" 2>&1 | grep -q 'success=True'; then
  ok "switched to dual_fisheye"
  sleep 3
  probe_hz "${NS}/image_raw/compressed" 25 no
else
  fail "set_mode dual_fisheye failed"
fi

step "Service: set_mode equirectangular"
if ros2 service call "${NS}/set_mode" "${PKG}/srv/SetCameraMode" \
    "{mode: 'equirectangular'}" 2>&1 | grep -q 'success=True'; then
  ok "switched back to equirectangular"
  sleep 2
else
  fail "set_mode equirectangular failed"
fi

step "Diagnostics"
DIAG=$(timeout 4 ros2 topic echo --once /diagnostics 2>/dev/null || true)
if [[ -n "$DIAG" ]]; then
  echo "$DIAG" | grep -E 'name:|level:|message:|measured_fps|pipeline_state' | head -20 | sed 's/^/    /'
  LVL=$(echo "$DIAG" | grep -m1 -oP 'level:\s*"?\K[0-9]' || true)
  if [[ "$LVL" == "0" ]]; then
    ok "diagnostics level=OK"
  else
    warn "diagnostics level=$LVL (non-OK)"
  fi
else
  fail "no /diagnostics message"
fi

step "Shutdown"
kill -INT "$NODE_PID" 2>/dev/null
sleep 2
if kill -0 "$NODE_PID" 2>/dev/null; then
  warn "node did not exit on SIGINT; sending SIGKILL"
  kill -9 "$NODE_PID" 2>/dev/null
fi
sleep 1
# Verify device freed.
if command -v fuser >/dev/null && fuser "$DEVICE" 2>/dev/null | grep -q '[0-9]'; then
  warn "$DEVICE still held by a process after shutdown"
else
  ok "$DEVICE released cleanly"
fi

echo
if [[ $FAILED -ne 0 ]]; then
  echo "SMOKE TEST: FAILED"
  exit 1
fi
echo "SMOKE TEST: OK"
