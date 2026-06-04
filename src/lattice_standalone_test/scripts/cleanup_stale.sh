#!/bin/bash
# Cleanup stale processes and DDS shared memory before launching a new scenario.
# Uses pgrep + kill to avoid self-matching issues with pkill -f.

cleanup() {
    local me=$$
    for pattern in "$@"; do
        local pids=$(pgrep -f "$pattern" 2>/dev/null || true)
        for pid in $pids; do
            [ "$pid" = "$me" ] && continue
            kill -9 "$pid" 2>/dev/null || true
        done
    done
}

cleanup "lattice_test_node --ros-args"
cleanup "lattice_simulator_node --ros-args"
cleanup "robot_state_publisher --ros-args"
cleanup "joint_state_publisher --ros-args"

# Remove stale DDS shared-memory files (Fast-DDS fallback)
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true

exit 0
