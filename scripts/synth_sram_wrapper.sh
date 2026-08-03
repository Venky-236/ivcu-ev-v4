#!/usr/bin/env bash
# ============================================================================
# synth_sram_wrapper.sh
#
# Synthesises ONLY the fault_log_sram_1024x32 wrapper -- the small module that
# glues two OpenRAM 512x32 macros into the 1024x32 block the main netlist
# expects. Takes seconds, not hours.
#
# The main design netlist is NOT touched. It already instantiates
# fault_log_sram_1024x32 with exactly this port list.
#
#   ~/final_ivcu_project/scripts/synth_sram_wrapper.sh
# ============================================================================
set -u

PROJ=/home/venky/final_ivcu_project
SRAMDIR=/home/venky/OpenRAM/sram_512x32_2port_output
LIB=$PROJ/libs/sky130_fd_sc_hd__tt_025C_1v80.lib
OUT=$PROJ/synth_out
STAMP=$(date +%Y%m%d_%H%M%S)

mkdir -p "$OUT"

for f in "$PROJ/RTL/fault_log_sram_1024x32.v" "$LIB" "$SRAMDIR/sram_512x32_2port.v"; do
  [ -f "$f" ] || { echo "missing: $f"; exit 1; }
done

cat > /tmp/wrap.ys <<EOF
# The OpenRAM macro is a hard block -- read it as a blackbox so Yosys emits an
# instance instead of trying to synthesise 16384 bits of storage.
read_verilog -lib $SRAMDIR/sram_512x32_2port.v

read_verilog $PROJ/RTL/fault_log_sram_1024x32.v

hierarchy -check -top fault_log_sram_1024x32
proc
opt
techmap
opt
dfflibmap -liberty $LIB
abc -liberty $LIB
setundef -zero
opt_clean

stat -liberty $LIB
write_verilog -noattr $OUT/fault_log_sram_1024x32_netlist.v
EOF

yosys -l "$OUT/wrapper_synth_${STAMP}.log" /tmp/wrap.ys
RC=$?

echo
echo "exit code: $RC"
[ $RC -ne 0 ] && { tail -20 "$OUT/wrapper_synth_${STAMP}.log"; exit $RC; }

echo
echo "=== macro instances (want exactly 2) ==="
grep -c "sram_512x32_2port" "$OUT/fault_log_sram_1024x32_netlist.v"
echo "=== wrapper cell count ==="
grep -c "sky130_fd_sc_hd__" "$OUT/fault_log_sram_1024x32_netlist.v"
echo
echo "netlist: $OUT/fault_log_sram_1024x32_netlist.v"

# ---------------------------------------------------------------------------
# Independent verification: read the written netlist back into a clean Yosys
# together with the main design netlist, and check the whole thing links.
# This is the gate that actually works -- see the note in verify_netlist.ys.
# ---------------------------------------------------------------------------
cat > /tmp/wrap_vfy.ys <<EOF
read_liberty -lib $LIB
read_verilog -lib $SRAMDIR/sram_512x32_2port.v
read_verilog $OUT/fault_log_sram_1024x32_netlist.v
read_verilog $OUT/ivcu_ev_v3_hybrid_top_gate_full.v
hierarchy -check -top ivcu_ev_v3_hybrid_top
check
stat
EOF

echo
echo "=== whole-design link check (main netlist + wrapper + macro) ==="
yosys -l "$OUT/link_check_${STAMP}.log" /tmp/wrap_vfy.ys > /dev/null 2>&1
echo "exit code: $?"
grep -n "Found and reported" "$OUT/link_check_${STAMP}.log"
grep -iE "^ERROR|referenced in module" "$OUT/link_check_${STAMP}.log" | head -10
