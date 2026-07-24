"""
Backup and restore ALL PX4 parameters over pure uXRCE-DDS (no MAVLink cable).

Uses the param_dds_bridge firmware module via drone.fcu:
- backup_params()   -> in-memory dump (name/type/value)
- backup_to_file()  -> QGC-compatible .params file (loadable in QGC too)
- restore_from_file() -> write params back (DANGEROUS: needs a confirm phrase)

Reboot is a SEPARATE step (mandatory for some params) — call drone.fcu.reboot()
yourself, only when disarmed.
"""

import sverk_interfaces

BACKUP_PATH = "/home/sverk/params_backup.params"

drone = sverk_interfaces.init(Nodename="example_params_backup")
try:
    # 1. In-memory backup (reliable, by index, ~15-20 s for the full set).
    b = drone.fcu.backup_params()
    print(f"backup: success={b.success}  {len(b.params)}/{b.total}")

    # 2. Save to a QGC .params file.
    path = drone.fcu.backup_to_file(BACKUP_PATH)
    print("saved:", path)

    # 3. Restore (commented out — uncomment deliberately).
    #    Requires the confirm phrase; does NOT reboot.
    # ok, failed = drone.fcu.restore_from_file(
    #     BACKUP_PATH, confirm="I_UNDERSTAND_WHAT_I_AM_DOING")
    # print(f"restored {ok}, failed {len(failed)}")
    # for name, msg in failed:
    #     print("  FAIL", name, msg)
    # drone.fcu.reboot()   # MANDATORY for some params, only when disarmed
finally:
    drone.close()
