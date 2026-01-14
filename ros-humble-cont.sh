#!/usr/bin/env bash
set -euo pipefail

# ============================================================
# Lima Bridged LAN + ROS2 Humble Setup (One-Shot Optimized)
# ============================================================

LIMA_INSTANCE="ros2-bridged"
BRIDGE_IFACE="en0"
CONTAINER_NAME="ros2-humble"
ROS_IMAGE="ros:humble"

# Paths for host-side networking
SOCKET_VMNET_BIN="/opt/socket_vmnet/socket_vmnet"
LIMA_NET_VAR_RUN="/opt/socket_vmnet/var/run"
BRIDGED_SOCKET_PATH="${LIMA_NET_VAR_RUN}/socket_vmnet.bridged"
SUDOERS_FILE="/etc/sudoers.d/lima-socket-vmnet"

die() { echo "ERROR: $*" >&2; exit 1; }
need_cmd() { command -v "$1" >/dev/null 2>&1 || die "Missing command: $1"; }

# 1. Prerequisite Check
need_cmd limactl; need_cmd sudo; need_cmd qemu-system-aarch64

# Capture the host current working directory early for mounts handling
HOST_PWD="$(pwd)"

# Helper: ensure the Lima instance exists and is configured for bridged socket_vmnet
ensure_lima_vm() {
    local inst_dir="$HOME/.lima/${LIMA_INSTANCE}"
    local lima_yaml="${inst_dir}/lima.yaml"

    # Create the VM from docker template if it doesn't exist yet
    if [[ ! -d "${inst_dir}" ]]; then
        echo "--- Creating Lima VM '${LIMA_INSTANCE}' from template:docker ---"
        limactl create --name="${LIMA_INSTANCE}" template:docker
    fi

    # Ensure vmType and networks are set for bridged socket_vmnet
    if [[ -f "${lima_yaml}" ]]; then
        # If the socket path is not present, add/patch the section at the end of the file
        if ! grep -q "${BRIDGED_SOCKET_PATH}" "${lima_yaml}"; then
            echo "--- Patching ${lima_yaml} for bridged networking ---"
            # Ensure vmType: qemu line exists (append or replace)
            if grep -q '^vmType:' "${lima_yaml}"; then
                # Replace any existing vmType with qemu
                sed -i '' 's/^vmType:.*/vmType: qemu/' "${lima_yaml}"
            else
                printf '\nvmType: qemu\n' >> "${lima_yaml}"
            fi

            # Build desired block with networks and disable probes to avoid long startup waits
            tmpnets=$(mktemp)
            {
              echo "networks:"
              echo "  - socket: ${BRIDGED_SOCKET_PATH}"
              echo "probes: []"
            } >"${tmpnets}"

            # Remove any existing networks: or probes: blocks, then append ours (idempotent)
            awk '
              BEGIN {skip=0}
              {
                # Detect top-level keys
                if ($0 ~ /^[A-Za-z].*:/) { top=1 } else { top=0 }
                # Start skipping when encountering networks: or probes:
                if ($0 ~ /^networks:/ || $0 ~ /^probes:/) { skip=1; next }
                # Stop skipping on the next top-level key
                if (skip==1 && top==1) { skip=0 }
                if (skip==0) print $0
              }
            ' "${lima_yaml}" > "${lima_yaml}.tmp"

            cat "${tmpnets}" >> "${lima_yaml}.tmp"
            mv "${lima_yaml}.tmp" "${lima_yaml}"
            rm -f "${tmpnets}"
        fi

        # Ensure a writable mount for the current host directory
        if ! grep -q "location: ${HOST_PWD}" "${lima_yaml}"; then
            echo "--- Patching ${lima_yaml} mounts to include ${HOST_PWD} (writable) ---"
            tmpmnts=$(mktemp)
            cat >"${tmpmnts}" <<EOM
mounts:
  - location: "~"
    writable: false
  - location: "/tmp/lima"
    writable: true
  - location: "${HOST_PWD}"
    writable: true
EOM

            # Remove any existing mounts: block and append our consolidated one (idempotent)
            awk '
              BEGIN {skip=0}
              {
                if ($0 ~ /^[A-Za-z].*:/) { top=1 } else { top=0 }
                if ($0 ~ /^mounts:/) { skip=1; next }
                if (skip==1 && top==1) { skip=0 }
                if (skip==0) print $0
              }
            ' "${lima_yaml}" > "${lima_yaml}.tmp"
            cat "${tmpmnts}" >> "${lima_yaml}.tmp"
            mv "${lima_yaml}.tmp" "${lima_yaml}"
            rm -f "${tmpmnts}"
        fi
    else
        die "Expected Lima config not found at ${lima_yaml}"
    fi
}

echo "--- 1. Preparing Host (macOS) ---"
sudo -v # Authenticate once

# Ensure secure directories
sudo mkdir -p "${LIMA_NET_VAR_RUN}"
sudo chown root:daemon "${LIMA_NET_VAR_RUN}"
sudo chmod 775 "${LIMA_NET_VAR_RUN}"

# Copy socket_vmnet binary to secure root-owned path
if [[ ! -x "${SOCKET_VMNET_BIN}" ]]; then
    sudo mkdir -p "$(dirname "${SOCKET_VMNET_BIN}")"
    BREW_BIN="/opt/homebrew/opt/socket_vmnet/bin/socket_vmnet"
    [[ -x "${BREW_BIN}" ]] || die "socket_vmnet not found. Run: brew install socket_vmnet"
    sudo cp -f "${BREW_BIN}" "${SOCKET_VMNET_BIN}"
    sudo chown root:wheel "${SOCKET_VMNET_BIN}"
    sudo chmod 755 "${SOCKET_VMNET_BIN}"
fi

# Write paths-only global networks config
mkdir -p ~/.lima/_config
cat > ~/.lima/_config/networks.yaml <<YAML
paths:
  socketVMNet: ${SOCKET_VMNET_BIN}
  varRun: ${LIMA_NET_VAR_RUN}
YAML

# Install sudoers rules
echo "Ensuring sudoers rules..."
cat <<EOF | sudo tee "${SUDOERS_FILE}" >/dev/null
${USER} ALL=(root:wheel) NOPASSWD: /usr/bin/true
${USER} ALL=(root:wheel) NOPASSWD: /bin/mkdir -m 775 -p ${LIMA_NET_VAR_RUN}
${USER} ALL=(root:admin) NOPASSWD: ${SOCKET_VMNET_BIN}
${USER} ALL=(root:wheel) NOPASSWD: /bin/chmod 666 ${BRIDGED_SOCKET_PATH}
EOF
sudo chmod 440 "${SUDOERS_FILE}"

echo "--- 2. Starting Networking Daemon ---"
sudo pkill -f socket_vmnet || true
sudo rm -f "${BRIDGED_SOCKET_PATH}"
sudo nohup "${SOCKET_VMNET_BIN}" \
  --vmnet-mode=bridged \
  --vmnet-interface="${BRIDGE_IFACE}" \
  --socket-group=admin \
  "${BRIDGED_SOCKET_PATH}" >/tmp/socket_vmnet_bridged.log 2>&1 &

for _ in {1..30}; do [[ -S "${BRIDGED_SOCKET_PATH}" ]] && break; sleep 0.2; done
[[ -S "${BRIDGED_SOCKET_PATH}" ]] || die "Daemon failed to create socket."
sudo chmod 666 "${BRIDGED_SOCKET_PATH}"

echo "--- 3. Preparing Lima VM configuration ---"
ensure_lima_vm

echo "--- 4. Starting VM ---"
# Use --tty=false to avoid interactive prompts and speed up unattended start
# Stop first to ensure Lima applies any config changes (like mounts)
limactl stop "${LIMA_INSTANCE}" || true
limactl start "${LIMA_INSTANCE}" --tty=false || true

echo "--- 5. Setting up ROS2 Container in VM ---"
APT_DEPS=""
PY_DEPS=""

limactl shell "${LIMA_INSTANCE}" -- bash -lc "
set -euo pipefail

# A. Fix hostname resolution for sudo (ensure /etc/hosts maps the hostname)
HN=\$(hostname)
if ! grep -qE \"[[:space:]]\\\${HN}(\\\$|[[:space:]])\" /etc/hosts; then
  echo \"127.0.1.1 \${HN}\" | sudo tee -a /etc/hosts >/dev/null || true
fi

# B. Docker rootful
sudo systemctl enable --now docker
docker context use default || true

# C. Detect bridged LAN interface/IP (non-eth0)
# Detect without awk field refs to avoid `$1` expansion issues in nested quoting
LAN_IF=\$(ip -4 -br addr \
  | grep -E '([0-9]{1,3}\.){3}[0-9]{1,3}/' \
  | sed -E 's/^([^[:space:]]+).*$/\1/' \
  | grep -Ev '^(lo|eth0)$' \
  | head -n1)
LAN_IP=""
if [[ -n \"\${LAN_IF}\" ]]; then
  LAN_IP=\$(ip -4 -br addr show \"\${LAN_IF}\" \
    | sed -E 's/^[^[:space:]]+[[:space:]]+[^[:space:]]+[[:space:]]+([^[:space:]]+).*/\1/' \
    | cut -d/ -f1)
fi
echo \"[VM] LAN IF detected: \${LAN_IF:-<none>}, IP: \${LAN_IP:-<none>}\"

# D. Ensure container exists and is running (idempotent)
if ! sudo docker ps -a --format '{{.Names}}' | grep -qx ${CONTAINER_NAME}; then
  # Create the container if it doesn't exist
  sudo docker run -d --name ${CONTAINER_NAME} --network host -v '${HOST_PWD}:/workspace:rw' -w /workspace ${ROS_IMAGE} sleep infinity
else
  # Start it if it exists but is not running
  if ! sudo docker ps --format '{{.Names}}' | grep -qx ${CONTAINER_NAME}; then
    sudo docker start ${CONTAINER_NAME}
  fi
fi

# E. Ensure ROS environment is sourced in every shell of the container
sudo docker exec ${CONTAINER_NAME} bash -lc \"grep -qxF 'source /opt/ros/humble/setup.bash' /root/.bashrc || echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc\"

# F. Configure FastDDS to lock to LAN interface (if detected)
if [[ -n \"\${LAN_IP}\" ]]; then
  cat > /tmp/fastdds_lima.xml <<EOF
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <participant profile_name="bridged_profile" is_default_profile="true">
        <rtps>
            <userTransports><transport_id>udp_lan</transport_id></userTransports>
            <useBuiltinTransports>false</useBuiltinTransports>
        </rtps>
    </participant>
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>udp_lan</transport_id>
            <type>UDPv4</type>
            <interfaceWhiteList><address>\${LAN_IP}</address></interfaceWhiteList>
        </transport_descriptor>
    </transport_descriptors>
</profiles>
EOF
  sudo docker cp /tmp/fastdds_lima.xml ${CONTAINER_NAME}:/tmp/fastdds_lima.xml
else
  echo \"[VM] Warning: No bridged LAN IP detected; FastDDS profile will not be set.\"
fi

echo \"Done! Entering ROS 2 shell...\"
if [[ -n \"\${LAN_IP}\" ]]; then
  sudo docker exec -it -e FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fastdds_lima.xml ${CONTAINER_NAME} bash -lc 'source /opt/ros/humble/setup.bash && exec bash'
else
  sudo docker exec -it ${CONTAINER_NAME} bash -lc 'source /opt/ros/humble/setup.bash && exec bash'
fi
"