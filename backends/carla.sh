#!/usr/bin/env bash
#
# backends/carla.sh — Carla 0.9.14 + evshary/autoware_carla_launch backend
#
# Implements the contract in backends/README.md. Sourced by the justfile
# orchestration recipes (just up / down / setup) which provide the
# `msg`, `warn`, `die`, `have` helpers.

# ── Configuration ───────────────────────────────────────────

BACKEND_NAME="carla"

BACKEND_ROOT="${BACKEND_ROOT:-${PROJECT_ROOT}/../autoware_carla_launch}"
CARLA_BIN="${CARLA_BIN:-${PROJECT_ROOT}/../carla-0.9.14/CarlaUE4.sh}"

# Source for auto-cloning autoware_carla_launch when BACKEND_ROOT is
# missing. Overridable so a contributor on a fork can point at their
# own remote without editing this file.
CARLA_LAUNCH_URL="${CARLA_LAUNCH_URL:-https://github.com/evshary/autoware_carla_launch.git}"
CARLA_LAUNCH_BRANCH="${CARLA_LAUNCH_BRANCH:-feat/fms-teleop}"

BACKEND_BRIDGE_CONTAINER="zenoh_bridge"
BACKEND_AUTOWARE_CONTAINER="zenoh_autoware"
BACKEND_EGO_CONTAINER="zenoh_ego"

_CARLA_BRIDGE_IMAGE="zenoh-carla-bridge-1.5.0"
_AUTOWARE_RAW_IMAGE="zenoh-autoware-1.5.0"

# Per-vehicle container names, keyed by an arbitrary scope (v1, v2, meow, ...).
_autoware_container() { printf '%s_%s' "$BACKEND_AUTOWARE_CONTAINER" "$1"; }
_ego_container()      { printf '%s_%s' "$BACKEND_EGO_CONTAINER" "$1"; }

# Lowest ROS_DOMAIN_ID not already claimed by a running vehicle. Each Autoware
# container records its domain in the fms.domain label, so domains are assigned
# dynamically (no dependence on the scope name) and freed when a vehicle stops.
_alloc_domain() {
    local used n=0
    used=$(docker ps -f label=fms.role=autoware --format '{{.Label "fms.domain"}}' 2>/dev/null)
    while printf '%s\n' "$used" | grep -qx "$n"; do n=$((n + 1)); done
    printf '%s' "$n"
}

# Inside-container path where rmw_zenoh's vendor prefix lives. autoware_manual_control's
# zenohcxx find_package() looks here at build time and at runtime.
_ZENOH_VP_INCONTAINER="/root/autoware_carla_launch/rmw_zenoh_ws/install/zenoh_cpp_vendor/opt/zenoh_cpp_vendor"

# Helper for one-shot commands in a Carla-bridge image (build steps).
_carla_run() {
    local img="$1"; shift
    docker run --rm --network host --privileged --ipc host --ulimit memlock=-1 \
        -v "${BACKEND_ROOT}:/root/autoware_carla_launch" \
        -w /root/autoware_carla_launch \
        "$img" bash -c "$*"
}

# ── Prereq checks ───────────────────────────────────────────

backend_check_runtime_prereqs() {
    [ -d "$BACKEND_ROOT" ] || die "missing autoware_carla_launch sibling at $BACKEND_ROOT
  run \`just setup\` first; see backends/carla.md"
    [ -f "$BACKEND_ROOT/install/setup.bash" ] \
        || die "Autoware not built (no install/setup.bash) — run \`just setup\`"
    [ -x "$BACKEND_ROOT/external/zenoh_carla_bridge/target/release/zenoh_carla_bridge" ] \
        || die "Carla bridge binary not built — run \`just setup\`"
    docker image inspect "$_CARLA_BRIDGE_IMAGE" >/dev/null 2>&1 \
        || die "Docker image missing: $_CARLA_BRIDGE_IMAGE
  see backends/carla.md for how to build via autoware_carla_launch's container/ scripts"
    docker image inspect "$_AUTOWARE_RAW_IMAGE" >/dev/null 2>&1 \
        || die "Docker image missing: $_AUTOWARE_RAW_IMAGE
  see backends/carla.md for how to build via autoware_carla_launch's container/ scripts"
    # Missing Carla binary is non-fatal; backend_start_sim will skip with a warn.
}

backend_check_bootstrap_prereqs() {
    # Everything else is auto-installed by backend_bootstrap. The only
    # piece we don't auto-install by default is the Carla binary (~4 GB);
    # opt-in via CARLA_AUTODOWNLOAD=1.
    if [ ! -f "$CARLA_BIN" ] && [ "${CARLA_AUTODOWNLOAD:-0}" != "1" ]; then
        warn "Carla binary not at $CARLA_BIN — \`just up\` will skip sim startup.
  Set CARLA_AUTODOWNLOAD=1 to auto-download (~4 GB), or place an extracted
  CARLA_0.9.14 tree at $(dirname "$CARLA_BIN")/. See backends/carla.md."
    fi
}

# ── Asset seeding ───────────────────────────────────────────

# Map a static asset from the backend tree into FMS frontend/public/ so the
# React Map View can fetch it. Idempotent: skips if already mirrored.
backend_seed_frontend_assets() {
    local src="$BACKEND_ROOT/carla_map/Town01/lanelet2_map.osm"
    local dst="$PROJECT_ROOT/frontend/public/carla_map/Town01/lanelet2_map.osm"
    [ -f "$dst" ] && return 0
    [ -f "$src" ] || { warn "$src missing — frontend Map View will be empty"; return 0; }
    mkdir -p "$(dirname "$dst")"
    cp -f "$src" "$dst"
    msg "  mirrored lanelet2_map.osm to frontend/public/carla_map/Town01/"
}

# Apply per-deployment overrides (autoware_data/custom_control/*) into the
# backend tree. No-op if PROJECT_ROOT/autoware_configs/custom_control doesn't exist.
backend_seed_custom_configs() {
    [ -d "${PROJECT_ROOT}/autoware_configs/custom_control" ] || return 0
    [ -d "$BACKEND_ROOT" ] || return 0
    msg "  applying custom speed control configurations..."
    mkdir -p "${BACKEND_ROOT}/autoware_data/custom_control"
    cp -r "${PROJECT_ROOT}/autoware_configs/custom_control/"* \
          "${BACKEND_ROOT}/autoware_data/custom_control/"
}

# Backend-specific runtime env; REACT_APP_* come from env.sh via `just run`.
backend_export_runtime_flags() {
    export USE_BRIDGE_ROS2DDS=True
}

# ── Lifecycle ───────────────────────────────────────────────

# Start Carla as a host process. `env -u DISPLAY` is required: UE4 4.26
# connects to X over SSH X11 forwarding even with -RenderOffScreen,
# blocking the main thread on do_poll().
backend_start_sim() {
    local i  # see run_steps in the justfile (dynamic scoping)
    if [ ! -f "$CARLA_BIN" ]; then
        warn "Carla binary not at $CARLA_BIN; skipping sim startup"
        return 0
    fi
    # A long-lived Carla degrades (5s RPC timeouts, stale actors) and then
    # silently breaks the engage handshake or spawns no ego vehicle, so
    # `just up` defaults to a deterministic restart: kill any running
    # Carla and start a known-good instance. Set CARLA_REUSE=1 to keep an
    # existing instance for fast iteration when it is known healthy.
    if [ "${CARLA_REUSE:-0}" = "1" ] && nc -z localhost 2000 2>/dev/null; then
        echo "  CARLA_REUSE=1 and :2000 open — reusing existing Carla (may be stale)."
        return 0
    fi
    if pgrep -f CarlaUE4 >/dev/null 2>&1; then
        echo "  Stopping existing Carla for a deterministic restart..."
        pkill -9 -f CarlaUE4 2>/dev/null
        fuser -k 2000/tcp 2>/dev/null || true
        for i in $(seq 1 15); do
            pgrep -f CarlaUE4 >/dev/null 2>&1 || break
            sleep 1
        done
    fi
    nohup env -u DISPLAY bash "$CARLA_BIN" -RenderOffScreen -nosound \
        > "${PROJECT_ROOT}/logs/carla.log" 2>&1 &
    echo -n "  Waiting for Carla on port 2000... "
    for i in $(seq 1 40); do
        nc -z localhost 2000 2>/dev/null && echo "port open." && break
        [ "$i" -eq 40 ] && echo "TIMEOUT." && return 1
        sleep 1
    done
    # Port 2000 opens before UE4 finishes loading the world. Bridge's
    # load_world() races and times out if it connects too early. Wait a
    # fixed duration; -RenderOffScreen's log doesn't reliably emit a
    # ready signal we could grep for.
    echo "  Giving Carla 15s to finish world load..."
    sleep 15
}

# Load the configured Carla world once. Per-vehicle agents then
# main.py --attach to it; if each loaded the world it would wipe the others.
backend_load_world() {
    [ -d "$BACKEND_ROOT" ] || return 0
    docker run --rm --network host --privileged --ipc host \
        -v "${BACKEND_ROOT}:/root/autoware_carla_launch" \
        -w /root/autoware_carla_launch \
        "$_CARLA_BRIDGE_IMAGE" \
        bash -c 'source ./env.sh && cd external/zenoh_carla_bridge/carla_agent && \
            ./.venv/bin/python3 set_world.py "${CARLA_SIMULATOR_IP:-127.0.0.1}"'
}

# Start the zenoh<->Carla bridge only. Egos are spawned per-vehicle by
# backend_start_ego so each vehicle has an independent lifecycle; the bridge
# discovers them dynamically.
backend_start_bridge() {
    [ -d "$BACKEND_ROOT" ] || { warn "skipping bridge: backend root missing"; return 0; }
    docker rm -f "$BACKEND_BRIDGE_CONTAINER" >/dev/null 2>&1 || true
    docker run -d --name "$BACKEND_BRIDGE_CONTAINER" \
        --network host --privileged --ipc host --ulimit memlock=-1 \
        -v "${BACKEND_ROOT}:/root/autoware_carla_launch" \
        -w /root/autoware_carla_launch \
        "$_CARLA_BRIDGE_IMAGE" \
        bash -c 'source ./env.sh && \
            RUST_LOG=z=info "${AUTOWARE_CARLA_ROOT}/external/zenoh_carla_bridge/target/release/zenoh_carla_bridge" \
                --mode ros2 --zenoh-listen tcp/0.0.0.0:7447 \
                --zenoh-config ${ZENOH_CARLA_BRIDGE_CONFIG} --carla-address ${CARLA_SIMULATOR_IP}'
    echo "  Waiting 10s for bridge..."
    sleep 10
}

# Spawn one vehicle's ego + sensors via main.py --attach: it attaches to the
# already-loaded world (--position random auto-picks a free spawn point) and
# holds the actors alive. --stop-signal=SIGINT so `docker stop` triggers the
# agent's World.destroy() for a clean teardown instead of orphaning actors.
backend_start_ego() {
    [ -d "$BACKEND_ROOT" ] || { warn "skipping ego: backend root missing"; return 0; }
    local scope="${1:-${VEHICLE:?scope required}}" container
    container="$(_ego_container "$scope")"
    docker rm -f "$container" >/dev/null 2>&1 || true
    docker run -d --name "$container" --stop-signal=SIGINT \
        --network host --privileged --ipc host --ulimit memlock=-1 \
        --label fms.role=ego --label fms.scope="$scope" \
        -v "${BACKEND_ROOT}:/root/autoware_carla_launch" \
        -w /root/autoware_carla_launch \
        "$_CARLA_BRIDGE_IMAGE" \
        bash -c "source ./env.sh && cd external/zenoh_carla_bridge/carla_agent && \
            ./.venv/bin/python3 main.py --attach --host \${CARLA_SIMULATOR_IP} --rolename ${scope}"
    echo "  Waiting 8s for ego (${scope})..."
    sleep 8
}

# Start Autoware ROS bringup container. The FMS clean_repo is mounted at
# the same path inside the container so manual_control source paths match.
backend_start_autoware() {
    [ -d "$BACKEND_ROOT" ] || { warn "skipping autoware: backend root missing"; return 0; }
    local vehicle="${VEHICLE:-v1}" container domain
    container="$(_autoware_container "$vehicle")"
    domain="$(_alloc_domain)"
    docker rm -f "$container" >/dev/null 2>&1 || true
    docker run -d --name "$container" \
        --network host --privileged --ipc host --ulimit memlock=-1 \
        --label fms.role=autoware --label fms.scope="$vehicle" --label fms.domain="$domain" \
        -v "${PROJECT_ROOT}:${PROJECT_ROOT}" \
        -v "${BACKEND_ROOT}:/root/autoware_carla_launch" \
        -w /root/autoware_carla_launch \
        "$_AUTOWARE_RAW_IMAGE" \
        bash -c "source install/setup.bash && source env.sh && \
                 export ROS_DOMAIN_ID=${domain} && \
                 ./script/autoware_ros2dds/run-autoware.sh ${vehicle}"
    echo "  Waiting 20s for Autoware stack (${vehicle}, domain ${domain})..."
    sleep 20
}

# Block until /api/operation_mode/state.is_remote_mode_available is true,
# or timeout. Returns 0 on ready, 1 on timeout. Optional argument: timeout
# in seconds (default 180).
backend_wait_ready() {
    local timeout=${1:-180} container domain
    container="$(_autoware_container "${VEHICLE:-v1}")"
    domain="$(docker inspect -f '{{index .Config.Labels "fms.domain"}}' "$container" 2>/dev/null)"
    local start=$SECONDS
    while [ $((SECONDS - start)) -lt "$timeout" ]; do
        local out
        out=$(docker exec "$container" bash -c \
            ". /opt/autoware/setup.bash >/dev/null 2>&1
             export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=${domain}
             timeout 2 ros2 topic echo --once /api/operation_mode/state 2>/dev/null" \
            2>/dev/null || true)
        if echo "$out" | grep -q "is_remote_mode_available: true"; then
            return 0
        fi
        sleep 5
    done
    return 1
}

# Run a command in the backend's Autoware ROS environment. Sources
# /opt/autoware/setup.bash + autoware_carla_launch/env.sh + sets
# ZENOH_VENDOR_PREFIX and LD_LIBRARY_PATH so zenohcxx is reachable for
# both build (find_package) and run (dynamic linking). env.sh hardcodes
# ROS_DOMAIN_ID=0, so re-pin it to this vehicle's domain (read back from the
# container's fms.domain label) or the teleop lands in the wrong Autoware.
#
# Usage:  backend_exec_in_ros 'cd ... && colcon build ...'
#         backend_exec_in_ros -d 'ros2 run ... > /tmp/log 2>&1'
backend_exec_in_ros() {
    local exec_opts="" container domain
    if [ "$1" = "-d" ]; then
        exec_opts="-d"
        shift
    fi
    container="$(_autoware_container "${VEHICLE:-v1}")"
    domain="$(docker inspect -f '{{index .Config.Labels "fms.domain"}}' "$container" 2>/dev/null)"
    docker exec $exec_opts "$container" bash -c \
        "export ZENOH_VENDOR_PREFIX=${_ZENOH_VP_INCONTAINER} && \
         export LD_LIBRARY_PATH=\${ZENOH_VENDOR_PREFIX}/lib:\${LD_LIBRARY_PATH:-} && \
         source /opt/autoware/setup.bash && \
         source /root/autoware_carla_launch/env.sh && \
         export ROS_DOMAIN_ID=${domain} && \
         $*"
}

# ── Shutdown ────────────────────────────────────────────────

backend_stop() {
    local stopped_any=0 c
    # name= is a substring match, so "zenoh_autoware" also catches the
    # per-vehicle "zenoh_autoware_v1"/"_v2"; two queries OR the two sets.
    for c in $(docker ps -aq -f "name=$BACKEND_AUTOWARE_CONTAINER" 2>/dev/null) \
             $(docker ps -aq -f "name=$BACKEND_EGO_CONTAINER" 2>/dev/null) \
             $(docker ps -aq -f "name=$BACKEND_BRIDGE_CONTAINER" 2>/dev/null); do
        echo "[backend:carla] Stopping container: $(docker inspect -f '{{.Name}}' "$c" 2>/dev/null)"
        docker rm -f "$c" >/dev/null 2>&1
        stopped_any=1
    done
    # force-kill: a degraded Carla (up, RPC port dead) ignores SIGTERM
    if pkill -9 -f CarlaUE4 2>/dev/null; then
        echo "[backend:carla] Stopped Carla."
        stopped_any=1
    fi
    # Carla & bridge port cleanup. FMS ports (3000, 8000) are owned by the
    # `just down` recipe and cleaned up there.
    fuser -k 2000/tcp 7447/tcp 7887/tcp 8080/tcp 2>/dev/null || true
    return 0
}

# Stop a single vehicle (its ego + Autoware; the teleop dies with Autoware),
# leaving other vehicles and the shared sim/bridge running. The ego is stopped
# with `docker stop` (SIGINT, per --stop-signal) so its agent runs World.destroy
# and cleans up the Carla actors instead of orphaning them.
backend_stop_vehicle() {
    local scope="${1:-${VEHICLE:-v1}}" a e
    a="$(_autoware_container "$scope")"
    e="$(_ego_container "$scope")"
    docker rm -f "$a" >/dev/null 2>&1 || true
    if docker ps -q -f "name=^${e}$" 2>/dev/null | grep -q .; then
        docker stop "$e" >/dev/null 2>&1 || true
    fi
    docker rm -f "$e" >/dev/null 2>&1 || true
    echo "[backend:carla] Stopped vehicle: $scope"
}

# ── Bootstrap (one-time first-run build) ────────────────────

backend_bootstrap() {
    # ── Step 0a: clone autoware_carla_launch sibling if missing ──
    # An empty dir from a partial earlier clone would block re-cloning;
    # rmdir it first.
    if [ ! -d "$BACKEND_ROOT" ] || [ -z "$(ls -A "$BACKEND_ROOT" 2>/dev/null)" ]; then
        [ -d "$BACKEND_ROOT" ] && rmdir "$BACKEND_ROOT" 2>/dev/null
        msg "  cloning $CARLA_LAUNCH_URL (branch: $CARLA_LAUNCH_BRANCH)"
        git clone --recurse-submodules -b "$CARLA_LAUNCH_BRANCH" \
            "$CARLA_LAUNCH_URL" "$BACKEND_ROOT"
    else
        msg "  autoware_carla_launch sibling: already present — skipping clone"
    fi

    # ── Step 0b: build the two upstream Docker images if missing ──
    # autoware_carla_launch's container/ scripts wrap docker build with
    # the right tags/args.
    skip_if_present "$_CARLA_BRIDGE_IMAGE Docker image" \
        "docker image inspect '$_CARLA_BRIDGE_IMAGE'" \
        "cd '$BACKEND_ROOT' && bash container/run-carla-bridge-docker.sh build"

    skip_if_present "$_AUTOWARE_RAW_IMAGE Docker image" \
        "docker image inspect '$_AUTOWARE_RAW_IMAGE'" \
        "cd '$BACKEND_ROOT' && bash container/run-autoware-docker.sh build"

    # ── Step 0c: download Carla binary if requested ──
    # Opt-in (CARLA_AUTODOWNLOAD=1) because it's a 4 GB pull and the
    # user may already have it positioned.
    if [ ! -f "$CARLA_BIN" ] && [ "${CARLA_AUTODOWNLOAD:-0}" = "1" ]; then
        local carla_dir tarball url
        carla_dir="$(dirname "$CARLA_BIN")"
        tarball="${carla_dir}/CARLA_0.9.14.tar.gz"
        url="https://github.com/carla-simulator/carla/releases/download/0.9.14/CARLA_0.9.14.tar.gz"
        msg "  downloading Carla 0.9.14 (~4 GB) to $carla_dir"
        mkdir -p "$carla_dir"
        if have wget; then
            wget --continue -O "$tarball" "$url"
        elif have curl; then
            curl -fL -C - -o "$tarball" "$url"
        else
            die "neither wget nor curl available — install one to use CARLA_AUTODOWNLOAD"
        fi
        tar xzf "$tarball" -C "$carla_dir"
        rm -f "$tarball"
        [ -f "$CARLA_BIN" ] || die "Carla extraction did not produce $CARLA_BIN"
    fi

    msg "  init backend submodules"
    git -C "$BACKEND_ROOT" submodule update --init --recursive

    # Probe `cargo` (real binary) — symlinks may point at a container-only path.
    skip_if_present "Rust toolchain (carla bridge)" \
        "[ -x '${BACKEND_ROOT}/rust/bin/cargo' ]" \
        "_carla_run '$_CARLA_BRIDGE_IMAGE' \
            'source env.sh && ./script/setup/dependency_install.sh rust'"

    skip_if_present "zenoh_carla_bridge binary" \
        "[ -x '${BACKEND_ROOT}/external/zenoh_carla_bridge/target/release/zenoh_carla_bridge' ]" \
        "_carla_run '$_CARLA_BRIDGE_IMAGE' \
            'source env.sh && cd external/zenoh_carla_bridge && \
             CARLA_VERSION=0.9.14 cargo build --release'"

    # poetry/pyenv installed into ${BACKEND_ROOT}/{poetry,pyenv}/.
    # Probe the venv binary, not bin/poetry which is an absolute
    # symlink to the in-container path.
    skip_if_present "poetry / pyenv (host installs)" \
        "[ -x '${BACKEND_ROOT}/poetry/venv/bin/poetry' ]" \
        "_carla_run '$_CARLA_BRIDGE_IMAGE' \
            'source env.sh && ./script/setup/dependency_install.sh python'"

    skip_if_present "carla_agent poetry venv" \
        "[ -f '${BACKEND_ROOT}/external/zenoh_carla_bridge/carla_agent/.venv/pyvenv.cfg' ]" \
        "_carla_run '$_CARLA_BRIDGE_IMAGE' \
            'source env.sh && poetry config virtualenvs.in-project true && \
             cd external/zenoh_carla_bridge/carla_agent && poetry install --no-root'"

    skip_if_present "Town01 map + perception models" \
        "[ -f '${BACKEND_ROOT}/carla_map/Town01/lanelet2_map.osm' ] && \
         [ -f '${BACKEND_ROOT}/carla_map/Town01/pointcloud_map.pcd' ]" \
        "_carla_run '$_AUTOWARE_RAW_IMAGE' \
            'source env.sh && \
             ./script/setup/download_map.sh && ./script/setup/download_models.sh'"

    skip_if_present "Autoware ROS workspace (colcon build)" \
        "[ -f '${BACKEND_ROOT}/install/setup.bash' ]" \
        "_carla_run '$_AUTOWARE_RAW_IMAGE' \
            'source /opt/autoware/setup.bash && source env.sh && \
             colcon build --symlink-install --base-paths src \
                 --cmake-args -DCMAKE_BUILD_TYPE=Release'"
}
