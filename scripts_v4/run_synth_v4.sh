#!/usr/bin/env bash
#=============================================================================
# run_synth_v4.sh  -  Yosys synthesis for IVCU-EV V4, Sky130 HD
#
# EXPECTATIONS
#   V3 took 4:53:29 and peaked at 3.17 GB, and needed an 8 GB swap file to
#   avoid an OOM kill.  V4 is 7,295 generic cells against V3's 69,773 mapped
#   instances, so this should complete in minutes and well inside 3.7 GB.
#   If it starts swapping, something has gone wrong - stop and look.
#
# THE ONE V3 TRAP THIS SCRIPT DOES NOT REPEAT
#   Your notes record that running "check -assert -mapped" at the end of the
#   synthesis run reported thousands of FALSE problems and renamed the netlist
#   REJECTED_*.  That check is not run here.  Verification happens afterwards,
#   as a separate standalone pass, exactly as your V3 notes concluded it should.
#
# USAGE
#   bash scripts_v4/run_synth_v4.sh
#   bash scripts_v4/run_synth_v4.sh nohup     # background, survives logout
#=============================================================================

set -u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${HERE}/.." && pwd)"
RTL="${ROOT}/RTL_V4"
TOP="ivcu_ev_v4_top"
LIB="${ROOT}/libs/sky130_fd_sc_hd__tt_025C_1v80.lib"
SRAM_V="${ROOT}/macros/sram_512x32_2port.v"
OUT="${ROOT}/synth_out_v4"
STAMP="$(date +%Y%m%d_%H%M%S)"

mkdir -p "${OUT}"

NETLIST="${OUT}/${TOP}_netlist_${STAMP}.v"
LOGFILE="${OUT}/synth_${STAMP}.log"
TIMEFILE="${OUT}/time_${STAMP}.txt"

# clk_ai and clk_mcu are the fastest at 20 ns.  ABC optimises against one
# target, so it gets the tightest one.
ABC_DELAY_PS=20000

#-----------------------------------------------------------------------------
# Pre-flight.  Two of these caught real problems on the V3 runs.
#-----------------------------------------------------------------------------
echo "=============================================================="
echo " IVCU-EV V4 synthesis"
echo " top      : ${TOP}"
echo " liberty  : $(basename "${LIB}")"
echo " netlist  : $(basename "${NETLIST}")"
echo "=============================================================="
echo

if [ ! -f "${LIB}" ]; then
  echo "  liberty not found: ${LIB}"; exit 1
fi
if [ ! -f "${SRAM_V}" ]; then
  echo "  SRAM model not found: ${SRAM_V}"; exit 1
fi

RUNNING=$(pgrep -c yosys || true)
if [ "${RUNNING}" != "0" ]; then
  echo "  *** ${RUNNING} yosys process(es) already running."
  echo "  *** Two synthesis runs on a 3.7 GB machine is how you get OOM killed."
  echo "  *** Kill them first, or wait."
  exit 1
fi

#-----------------------------------------------------------------------------
# FILTERED LIBERTY
#
# The first V4 STA run had a 9.967 ns delay through a single
# sky130_fd_sc_hd__lpflow_isobufsrc_1 on the paddr decode path.  That cell is a
# power-gating isolation buffer - slow and weak by design - and ABC picked it
# because nothing told it not to.  Filtering it and its relatives out is what
# every serious Sky130 flow does.
#-----------------------------------------------------------------------------
LIB_FILTERED="${ROOT}/libs/sky130_fd_sc_hd__tt_025C_1v80__filtered.lib"

if [ ! -f "${LIB_FILTERED}" ] || [ "${LIB}" -nt "${LIB_FILTERED}" ]; then
  echo "  building filtered liberty ..."
  python3 "${HERE}/filter_liberty.py" "${LIB}" "${LIB_FILTERED}" || exit 1
fi
LIB="${LIB_FILTERED}"
echo "  using liberty: $(basename "${LIB}")"
echo

echo "  free memory before start:"
free -h | sed 's/^/    /'
echo

#-----------------------------------------------------------------------------
# Run
#-----------------------------------------------------------------------------
read -r -d '' YS_SCRIPT <<EOF || true
# --- read ------------------------------------------------------------------
read_liberty -lib ${LIB}

# The SRAM is a hard macro.  -lib reads only its interface, and blackbox makes
# certain that its behavioural model - which contains delays and \$display -
# never reaches synthesis.
read_verilog -lib ${SRAM_V}
blackbox sram_512x32_2port

read_verilog -I ${RTL} ${RTL}/*.v

# --- elaborate --------------------------------------------------------------
hierarchy -check -top ${TOP}

# --- generic synthesis ------------------------------------------------------
proc
flatten
opt_expr
opt_clean
check
opt -nodffe -nosdff
fsm
opt
wreduce
peepopt
opt_clean
alumacc
share
opt
memory -nomap
opt_clean

# --- map ---------------------------------------------------------------------
memory_map
opt -full
techmap
opt -fast

dfflibmap -liberty ${LIB}
abc -liberty ${LIB} -D ${ABC_DELAY_PS}
setundef -zero
clean -purge

# --- report -------------------------------------------------------------------
stat -liberty ${LIB}
write_verilog -noattr ${NETLIST}
EOF

echo "  starting yosys ..."
echo

/usr/bin/time -v -o "${TIMEFILE}" \
  yosys -p "${YS_SCRIPT}" > "${LOGFILE}" 2>&1
RC=$?

echo
if [ ${RC} -ne 0 ]; then
  echo "  *** SYNTHESIS FAILED (rc=${RC})"
  tail -60 "${LOGFILE}"
  exit 1
fi

#-----------------------------------------------------------------------------
# The numbers that matter
#-----------------------------------------------------------------------------
echo "--------------------------------------------------------------"
echo " RESULT"
echo "--------------------------------------------------------------"
grep "Chip area for" "${LOGFILE}" | sed 's/^/  /'
echo
grep -E "Elapsed \(wall|Maximum resident" "${TIMEFILE}" | sed 's/^/  /'
echo
echo "  most-used cells:"
# The stat section runs from the module header to the "Chip area" line; the
# earlier version stopped at the first blank line, which is the line directly
# after the header, so it printed nothing.
sed -n "/=== ${TOP} ===/,/Chip area/p" "${LOGFILE}" \
  | grep -E "sky130_fd_sc_hd__|sram_" \
  | sort -k1 -rn | head -15 | sed 's/^/    /'
echo
echo "  V3 baseline for comparison:"
echo "    standard cells        633,274 um2"
echo "    SRAM macros           572,456 um2   (two macros; V4 has one)"
echo "    total instances        69,773"
echo "    wall clock            4:53:29"
echo "    peak memory              3.17 GB"
echo
echo "  netlist : ${NETLIST}"
echo "  log     : ${LOGFILE}"
echo
echo "  NEXT: verify the netlist standalone before trusting it -"
echo "        bash scripts_v4/verify_netlist_v4.sh ${NETLIST}"
echo
