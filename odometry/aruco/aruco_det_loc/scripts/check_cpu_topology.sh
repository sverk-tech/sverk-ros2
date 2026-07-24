#!/usr/bin/env bash
# check_cpu_topology.sh
# ---------------------
# Identifies which CPU cores are Cortex-A76 "big" cores on Allwinner A733
# (and similar heterogeneous ARM SoCs).
#
# Run this on the Radxa Cubie A7Z (or any aarch64 board) to find the
# core IDs to pass as cpu_affinity_ids in the ROS2 launch files.
#
# Usage:
#   bash check_cpu_topology.sh
#
# Expected output example for Allwinner A733 (2xA76 + 6xA55):
#   CPU  CLUSTER  MAX_FREQ_MHz  PART_ID  LIKELY_CORE
#   0    0        1800          0xD0B    A76  ← use these IDs
#   1    0        1800          0xD0B    A76
#   2    1        1416          0xD05    A55
#   ...

set -euo pipefail

echo ""
echo "=== CPU Topology on $(hostname) ==="
echo ""

# Print lscpu --extended if available
if lscpu --extended &>/dev/null; then
    echo "--- lscpu --extended ---"
    lscpu --extended
    echo ""
fi

# Read per-core max frequency and CPU part number
echo "--- Per-core details ---"
printf "%-5s %-10s %-15s %-10s %-12s\n" "CPU" "CLUSTER" "MAX_FREQ_MHz" "PART_ID" "LIKELY_CORE"
printf "%-5s %-10s %-15s %-10s %-12s\n" "---" "-------" "------------" "-------" "-----------"

num_cpus=$(nproc --all)

for cpu in $(seq 0 $((num_cpus - 1))); do
    cpu_path="/sys/devices/system/cpu/cpu${cpu}"

    # Max frequency
    freq_file="${cpu_path}/cpufreq/cpuinfo_max_freq"
    if [ -r "${freq_file}" ]; then
        freq_khz=$(cat "${freq_file}" 2>/dev/null || echo "0")
        freq_mhz=$(( freq_khz / 1000 ))
    else
        freq_mhz="?"
    fi

    # Cluster (topology package)
    cluster_file="${cpu_path}/topology/physical_package_id"
    if [ -r "${cluster_file}" ]; then
        cluster=$(cat "${cluster_file}" 2>/dev/null || echo "?")
    else
        cluster="?"
    fi

    # CPU part number from /proc/cpuinfo (implementer 0x41 = ARM)
    # Part 0xD0B = Cortex-A76, 0xD05 = Cortex-A55
    part_id=$(grep -A 30 "^processor.*: ${cpu}$" /proc/cpuinfo 2>/dev/null \
              | grep "CPU part" | head -1 | awk '{print $NF}' || echo "?")

    case "${part_id}" in
        0xd0b|0xD0B) label="A76 ★ BIG" ;;
        0xd05|0xD05) label="A55" ;;
        0xd46|0xD46) label="A510" ;;
        0xd47|0xD47) label="A710" ;;
        0xd48|0xD48) label="X2" ;;
        "?"|"")      label="unknown" ;;
        *)           label="${part_id}" ;;
    esac

    printf "%-5s %-10s %-15s %-10s %-12s\n" \
        "${cpu}" "${cluster}" "${freq_mhz}" "${part_id}" "${label}"
done

echo ""

# Collect A76 IDs
a76_ids=()
for cpu in $(seq 0 $((num_cpus - 1))); do
    part_id=$(grep -A 30 "^processor.*: ${cpu}$" /proc/cpuinfo 2>/dev/null \
              | grep "CPU part" | head -1 | awk '{print $NF}' || echo "")
    case "${part_id,,}" in
        0xd0b) a76_ids+=("${cpu}") ;;
    esac
done

if [ ${#a76_ids[@]} -gt 0 ]; then
    ids_str=$(IFS=,; echo "${a76_ids[*]}")
    echo "✓ Cortex-A76 cores detected: ${a76_ids[*]}"
    echo ""
    echo "Use these IDs in ROS2 launch files:"
    echo ""
    echo "  cpu_affinity_ids:=[${ids_str}]"
    echo ""
    echo "Example (standalone nodes):"
    echo "  ros2 launch aruco_det_loc aruco_detect.launch.py cpu_affinity_ids:=[${ids_str}] opencv_num_threads:=${#a76_ids[@]}"
    echo "  ros2 launch aruco_det_loc aruco_loc.launch.py    cpu_affinity_ids:=[${ids_str}]"
    echo ""
    echo "Example (composed):"
    echo "  ros2 launch aruco_det_loc aruco_composed.launch.py cpu_affinity_ids:=[${ids_str}]"
else
    echo "⚠ Could not detect Cortex-A76 cores automatically."
    echo "  Check /proc/cpuinfo manually or rely on frequency (highest = big cores)."
fi

echo ""
echo "=== Done ==="
