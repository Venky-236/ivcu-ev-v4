#!/usr/bin/env bash
# ============================================================================
# cleanup.sh -- remove superseded files before starting physical design.
#
#     ~/final_ivcu_project/scripts/cleanup.sh          # DRY RUN, shows only
#     ~/final_ivcu_project/scripts/cleanup.sh --delete # actually delete
#
# Everything here is either superseded or already in git. Nothing that can be
# regenerated in under a minute, and nothing needed for the flow, is touched.
#
# WHY IT IS SAFE
#   Your last commit (43e208a) has every RTL file, script, SDC and STA report.
#   If you delete something you later want:
#       git show HEAD:path/to/file > recovered_file
#       git checkout HEAD -- path/to/file
#   The two things git does NOT have are the gate netlists -- they are large
#   and would bloat the repo -- so the CURRENT netlist is never deleted here.
# ============================================================================
set -u
PROJ=/home/venky/final_ivcu_project
cd "$PROJ" || exit 1

GO=0
[ "${1:-}" = "--delete" ] && GO=1

TOTAL=0
kill_it () {   # $1 = path, $2 = reason
  local p="$1" why="$2" sz
  [ -e "$p" ] || return 0
  sz=$(du -sk "$p" 2>/dev/null | cut -f1)
  TOTAL=$((TOTAL + sz))
  printf "  %8s KB  %-52s  %s\n" "$sz" "$p" "$why"
  [ $GO -eq 1 ] && rm -rf "$p"
}

echo "=============================================================="
[ $GO -eq 1 ] && echo " DELETING" || echo " DRY RUN -- nothing removed. Add --delete to act."
echo "=============================================================="
echo

echo "--- superseded netlist and its logs (synthesis run 1 of today) ---"
kill_it synth_out/ivcu_ev_v3_hybrid_top_gate_full_20260804_070225.v \
        "superseded by 160929 (the one the symlink points at)"
kill_it synth_out/synth_full_20260804_070225.log  "log of that run"
kill_it synth_out/synth_full_20260804_070225.ys   "script of that run"
kill_it synth_out/time_full_20260804_070225.txt   "timing of that run"

echo
echo "--- nohup captures: byte-for-byte duplicates of the .log files ---"
kill_it synth_out/run_0702.out "duplicate of synth_full_20260804_070225.log"
kill_it synth_out/run_1609.out "duplicate of synth_full_20260804_160929.log"

echo
echo "--- abandoned July 31 fast run ---"
kill_it synth_out/old "fast-mode experiment, superseded"

echo
echo "--- pre-fix netlist (1,189,022 um2, before the divider work) ---"
echo "    NOTE: the NUMBERS are recorded in IVCU_flow_explained.md and in"
echo "    the commit history. The 12 MB netlist itself has no further use."
kill_it synth_out/old_20260801 "pre-fix baseline, numbers already documented"

echo
echo "--- RTL backup superseded by git ---"
kill_it RTL_backup_20260804_065319 "every file is in commit 43e208a"

echo
echo "--- SDC and script backups superseded by git ---"
kill_it SDC/ivcu_ev_v3.sdc.orig      "in git history"
kill_it SDC/ivcu_ev_v3.sdc.prefix    "in git history"
kill_it scripts/run_synthesis_v2.sh.bak "in git history"
kill_it scripts/run_real_synthesis.sh   "replaced by run_synthesis_v2.sh"

echo
echo "--- superseded STA reports (kept: the two FINAL ones) ---"
kill_it sta_out/sta_prelayout.txt "first run, SDC was broken"
kill_it sta_out/sta_diag.txt      "pre-fix netlist"
kill_it sta_out/sta_mask.txt      "pre-fix netlist"
kill_it sta_out/sta_diag_new.txt  "synthesis run 1, superseded"
kill_it sta_out/sta_mask_new.txt  "synthesis run 1, superseded"

echo
echo "--- stale link-check log ---"
kill_it synth_out/link_check_20260803_094137.log "regenerate any time in seconds"

echo
echo "=============================================================="
printf " total: %d KB  (%.1f MB)\n" "$TOTAL" "$(echo "$TOTAL/1024" | bc -l)"
echo "=============================================================="
echo
echo "KEPT, and why:"
cat <<'EOM'
  synth_out/ivcu_ev_v3_hybrid_top_gate_full_20260804_160929.v
                                    the live netlist
  synth_out/ivcu_ev_v3_hybrid_top_gate_full.v -> (symlink to it)
                                    the fixed name every STA script reads
  synth_out/fault_log_sram_1024x32_netlist.v
                                    SRAM wrapper netlist, still current
  synth_out/synth_full_20260804_160929.{log,ys}
  synth_out/time_full_20260804_160929.txt
                                    provenance of the live netlist
  synth_out/wrapper_synth_20260803_094137.log
                                    provenance of the wrapper netlist
  sta_out/sta_mask_final.txt        the timing result we are acting on
  macros/*                          OpenRAM output, NOT regenerable quickly
  RTL/, SDC/, scripts/, sim/, lef/, libs/
EOM
echo
[ $GO -eq 0 ] && echo "Nothing was deleted. Re-run with --delete when happy."
echo
