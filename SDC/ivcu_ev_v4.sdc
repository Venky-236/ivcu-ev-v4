#=============================================================================
# ivcu_ev_v4.sdc  -  timing constraints for IVCU-EV V4
#
# Design : ivcu_ev_v4_top
# PDK    : Sky130 HD, sky130_fd_sc_hd, tt_025C_1v80
# Flow   : Yosys + OpenROAD / OpenSTA
#
#-----------------------------------------------------------------------------
# WHAT CHANGED FROM THE V3 SDC, AND WHY
#
# 1. THE CLOCK TARGETS COME DOWN.
#
#    V3 asked for 200 MHz on the MCU domain and 100 MHz on the AI domain, and
#    reported WNS -1.389 ns, TNS -48.323 ns.  Sky130 HD standard cells do not
#    comfortably run general logic at 200 MHz; that number came from a 14 nm
#    mindset, the same origin as the "_14nm" module names.
#
#    V4 asks for 50 / 50 / 25 / 10 MHz.  The fastest thing this chip must do is
#    a 64-channel sensor sweep in 10.24 us, which is a hundred times faster
#    than any physical sensor needs.  There is no performance argument for the
#    old numbers, and closing timing with margin is worth far more to this
#    project than a 200 MHz figure that does not close.
#
#    If STA closes comfortably here, raise them afterwards.  Do not start where
#    V3 started.
#
# 2. THE PORT LISTS ARE SHORT ENOUGH TO READ.
#
#    V3's SDC had to enumerate 42 sensor_adc_in_*, 42 sensor_adc_valid_*, 42
#    sensor_digital_in_* and 42 sensor_digital_valid_* ports.  Most of those
#    pins were unconnected in the RTL.  V4 has 21 pins for all 64 channels, so
#    every constraint below fits on a screen and can be checked by eye.
#
# 3. set_clock_groups -asynchronous IS LOad-BEARING.  Read section 2.
#=============================================================================


#-----------------------------------------------------------------------------
# 1.  PRIMARY CLOCKS
#
# Four clock input pins.  There is no PLL, no divider and no clock gate
# anywhere in the RTL, so each input port IS the clock root and no
# create_generated_clock is required.
#
# That is a deliberate choice: V3 reached working synthesis and STA with four
# primary clocks, and introducing on-chip division would hand OpenROAD's CTS a
# new class of problem in a project that has already spent effort fighting the
# physical flow.  The cost is three extra pins out of 237.
#-----------------------------------------------------------------------------
create_clock -name clk_aon    -period 100.0 [get_ports clk_aon]     ;#  10 MHz
create_clock -name clk_sensor -period  40.0 [get_ports clk_sensor]  ;#  25 MHz
create_clock -name clk_ai     -period  20.0 [get_ports clk_ai]      ;#  50 MHz
create_clock -name clk_mcu    -period  20.0 [get_ports clk_mcu]     ;#  50 MHz


#-----------------------------------------------------------------------------
# 2.  CLOCK GROUPS  -  DO NOT REMOVE THIS
#
# The four domains are mutually asynchronous.  This one command declares that,
# and it implicitly creates false paths between every cross-domain pair, so no
# additional set_false_path -from clk_X -to clk_Y lines are needed or wanted -
# duplicating them produces conflicting-constraint warnings in OpenROAD.
#
# THIS CONSTRAINT IS LOAD-BEARING FOR CORRECTNESS, NOT JUST FOR TIMING.
#
# ivcu_ai_cluster, ivcu_apb_regs and several others use the quasi-static CDC
# pattern: wide buses (sensor_value_flat is 1024 bits, the mode masks are 64)
# cross domains unregistered, and nothing acts on them except on a
# synchronised pulse.  Registering them all would cost thousands of
# flip-flops to solve a problem those paths do not have.
#
# That pattern is only legal if STA does not try to time the data path.  If
# someone removes this line to "clean up" the constraints, every one of those
# paths becomes a real timing path between unrelated clocks, the tool will
# report enormous violations, and the natural response - adding pipeline
# stages - would break the handshakes that make the pattern correct.
#
# The one place a coherent registered snapshot IS paid for is
# ivcu_hv_sense_sync, because the precharge comparison spans two channels and
# a mixed snapshot welds a contactor.
#-----------------------------------------------------------------------------
set_clock_groups -asynchronous \
    -group [get_clocks clk_aon]    \
    -group [get_clocks clk_sensor] \
    -group [get_clocks clk_ai]     \
    -group [get_clocks clk_mcu]


#-----------------------------------------------------------------------------
# 3.  CLOCK UNCERTAINTY
#
# No on-chip PLL, so jitter is whatever the board supplies.  0.3 ns setup
# uncertainty is conservative for Sky130 HD at tt_025C_1v80 and leaves room
# for CTS skew; 0.05 ns hold is the usual figure.
#-----------------------------------------------------------------------------
set_clock_uncertainty -setup 0.3  [all_clocks]
set_clock_uncertainty -hold  0.05 [all_clocks]

set_clock_transition 0.15 [all_clocks]


#-----------------------------------------------------------------------------
# 4.  INPUT / OUTPUT DELAY BUDGETS
#
# Convention, applied consistently:
#     input  -max = 35 % of period   (setup budget for the external driver)
#     input  -min =  0.5 ns          (min clock-to-out of the external driver;
#                                     was 0 %, see the correction note below)
#     output -max = 45 % of period
#     output -min = 10 % of period
#
#   clk_aon    100 ns : in 35.0 / 0.5   out 45.0 / 10.0
#   clk_sensor  40 ns : in 14.0 / 0.5   out 18.0 /  4.0
#   clk_ai      20 ns : in  7.0 / 0.5   out  9.0 /  2.0   (no ports constrained)
#   clk_mcu     20 ns : in  7.0 / 0.5   out  9.0 /  2.0
#
# V3's SDC had clk_200mhz_mcu with -max 8.0 on a 5 ns period, which is
# impossible and was found and fixed in review.  Every number below is a
# fraction of its own clock period; none of them can be impossible.
#-----------------------------------------------------------------------------

#-----------------------------------------------------------------------------
# SYNTAX NOTE - THIS WAS WRONG IN THE V3 SDC TOO
#
# set_input_delay and set_output_delay take the delay as a POSITIONAL argument:
#
#     set_input_delay <delay> [-clock <clk>] [-max|-min] <port_list>
#
# -max and -min are boolean FLAGS, not value-takers.  Writing
#
#     set_input_delay -clock clk_aon -max 35.0 -min 0.0 [get_ports {...}]
#
# gives the parser three positional arguments where it expects two, and
# OpenSTA stops with "requires two positional arguments".
#
# Your V3 SDC uses that same form throughout.  Worth checking what it actually
# did on the V3 run - if read_sdc aborted at the first one, then the
# WNS -1.389 / TNS -48.323 figures were produced with NO input or output delay
# constraints at all, which would make the real boundary timing worse than
# those numbers, not better.
#
# Each budget therefore needs two commands: one for max, one for min.
#-----------------------------------------------------------------------------

#-----------------------------------------------------------------------------
# CORRECTED 15 Aug 2026 - THE -min VALUES WENT FROM 0.0 TO 0.5
#
# The 0 % convention above was written before there was a clock tree.  With an
# ideal clock it costs nothing: launch and capture both start at time zero, so
# asserting "the external driver may change data at the instant of the clock
# edge" is a check the design passes for free.
#
# After CTS it is unmeetable.  The clock takes 1.1-1.3 ns to cross this die, so
# a capture flop's hold window closes more than a nanosecond after the edge
# reaches the pin, while -min 0.0 says the data may already be moving.  That
# produced 278 violating endpoints at up to -1.094 ns:
#
#     clk_aon     192 endpoints   worst -0.920
#     clk_mcu      47 endpoints   worst -0.996
#     clk_sensor   39 endpoints   worst -1.094
#     clk_ai        0 endpoints   (the one domain with no set_input_delay)
#
# NOT ONE of them was register-to-register.  This command:
#
#   report_checks -path_delay min -from [all_registers -clock_pins] \
#                 -slack_max 0 -format end -digits 3
#
# returned "No paths found", so the clock tree itself holds.  Every violation
# was a boundary artefact of the constraint, not a defect in the design.
#
# 0.5 ns asserts that no device driving this chip has a clock-to-out faster
# than half a nanosecond.  A Sky130 flip-flop alone is about 0.3 ns, and any
# real companion part adds a pad driver and a board trace on top - so this
# claims almost nothing, unlike 0.0, which claims something no CMOS part can
# do.  It is a modelling correction, not a relaxation to make a number pass.
#
# The remaining ~0.6 ns was closed physically, not by constraint: repair_timing
# -hold inserted 302 hold buffers (+0.2 % area) and closed to +0.050 ns with
# zero setup lost, because every buffer landed on an input path carrying 13-65
# ns of unbudgeted period.
#
# NOTE: -min affects hold ONLY.  Raising it cannot improve setup, and the -max
# budgets in the table above are deliberately untouched.
#-----------------------------------------------------------------------------

# ---- 4a. Sensor domain, 40 ns -----------------------------------------------
# The whole 64-channel interface is these five signals.
set_input_delay 14.0 -clock clk_sensor -max \
    [get_ports {adc_data[*] adc_valid adc_busy}]
set_input_delay  0.5 -clock clk_sensor -min \
    [get_ports {adc_data[*] adc_valid adc_busy}]

set_output_delay 18.0 -clock clk_sensor -max \
    [get_ports {adc_chan[*] adc_req afe_sclk afe_sdata afe_latch}]
set_output_delay  4.0 -clock clk_sensor -min \
    [get_ports {adc_chan[*] adc_req afe_sclk afe_sdata afe_latch}]

# ---- 4b. Always-on domain, 100 ns -------------------------------------------
#
# CORRECTED 13 Aug 2026.  The discrete vehicle inputs used to be constrained
# here with set_input_delay.  That produced -0.036 ns HOLD violations on
# ignition_on, permit_ack, hazard_button, hvil_raw and crash_trig_side, and
# those violations were meaningless.
#
# Every one of those pins is a genuinely asynchronous signal from the vehicle
# harness - a key switch, a button, an interlock loop - and every one of them
# lands directly on an ivcu_cdc_bit_sync.  There is no clock relationship to
# check.  Asking STA to verify hold time on a signal whose entire design
# assumption is that it may violate setup and hold is asking the wrong
# question, and "fixing" a 36 ps violation on it would mean inserting delay
# buffers to satisfy a check that should never have run.
#
# They are false-pathed in section 5c.  The synchroniser is the constraint.
#
# gps_rx_* and sos_tx_ready ARE constrained, because they come from companion
# chips - but see the flagged CDC gap in section 5d.
set_input_delay 35.0 -clock clk_aon -max \
    [get_ports {gps_rx_data[*] gps_rx_valid sos_tx_ready}]
set_input_delay  0.5 -clock clk_aon -min \
    [get_ports {gps_rx_data[*] gps_rx_valid sos_tx_ready}]

# HV drives, safety actuators, powertrain and status.
set_output_delay 45.0 -clock clk_aon -max \
    [get_ports {hv_contactor_pos_en hv_contactor_neg_en \
                hv_precharge_en hv_discharge_en \
                pyro_fuse_arm pyro_fuse_fire \
                hv_isolated hv_state[*] hv_fault_code[*] \
                torque_cmd[*] regen_cmd[*] motor_enable \
                power_derate_pct[*] cooling_pump_pwm[*] cooling_fan_pwm[*] \
                charge_enable \
                airbag_trigger[*] belt_pretension[*] door_unlock \
                horn_en headlight_en hazard_lights_en \
                sos_tx_data[*] sos_tx_valid \
                system_health_score[*] vehicle_enable limp_home_active \
                speed_limit_kph[*] safety_state[*] warn_latched}]
set_output_delay 10.0 -clock clk_aon -min \
    [get_ports {hv_contactor_pos_en hv_contactor_neg_en \
                hv_precharge_en hv_discharge_en \
                pyro_fuse_arm pyro_fuse_fire \
                hv_isolated hv_state[*] hv_fault_code[*] \
                torque_cmd[*] regen_cmd[*] motor_enable \
                power_derate_pct[*] cooling_pump_pwm[*] cooling_fan_pwm[*] \
                charge_enable \
                airbag_trigger[*] belt_pretension[*] door_unlock \
                horn_en headlight_en hazard_lights_en \
                sos_tx_data[*] sos_tx_valid \
                system_health_score[*] vehicle_enable limp_home_active \
                speed_limit_kph[*] safety_state[*] warn_latched}]

# ---- 4c. MCU domain, 20 ns ---------------------------------------------------
set_input_delay 7.0 -clock clk_mcu -max \
    [get_ports {paddr[*] psel penable pwrite pwdata[*]}]
set_input_delay 0.5 -clock clk_mcu -min \
    [get_ports {paddr[*] psel penable pwrite pwdata[*]}]

set_output_delay 9.0 -clock clk_mcu -max \
    [get_ports {prdata[*] pready pslverr disp_sclk disp_sdata disp_cs}]
set_output_delay 2.0 -clock clk_mcu -min \
    [get_ports {prdata[*] pready pslverr disp_sclk disp_sdata disp_cs}]


#-----------------------------------------------------------------------------
# 5.  FALSE PATHS
#
# 5a. Asynchronous resets.  por_n and ext_rst_n are captured by the reset
#     synchronisers in ivcu_reset_manager; the boundary path itself has no
#     timing relationship to any clock.
#-----------------------------------------------------------------------------
set_false_path -from [get_ports {por_n ext_rst_n}]

#-----------------------------------------------------------------------------
# 5b. mode_strap is a bonded strap.  It is tied at manufacture and cannot
#     change while the chip is powered, so there is nothing to time - but it
#     still passes through a two-flop synchroniser, because it is the signal
#     that decides whether this vehicle has airbags and two flops are cheap.
#-----------------------------------------------------------------------------
set_false_path -from [get_ports mode_strap[*]]

#-----------------------------------------------------------------------------
# 5c. THE ASYNCHRONOUS VEHICLE INPUTS
#
# A key switch, a rider's button, an interlock loop and a crash accelerometer
# discrete have no phase relationship with a 10 MHz clock on this die.  They
# are asynchronous by nature, and every one of them terminates on an
# ivcu_cdc_bit_sync whose entire purpose is to tolerate that.
#
# Timing them tells you nothing, and the -0.036 ns hold "violations" they
# produced would be fixed by inserting delay into a path whose correctness
# does not depend on delay at all.
#
# The synchroniser is the constraint.  Not the SDC.
#-----------------------------------------------------------------------------
set_false_path -from [get_ports {crash_trig_front crash_trig_side hvil_raw \
                                 ignition_on permit_ack hazard_button}]

#-----------------------------------------------------------------------------
# 5d. FLAGGED GAP - gps_rx_* AND sos_tx_ready
#
# These are treated above as synchronous to clk_aon, and ivcu_gps_receiver
# samples gps_rx_data and gps_rx_valid directly in an always @(posedge
# clk_aon) block with no synchroniser.
#
# That is only correct if the companion GNSS module and the telematics modem
# are clocked from the same source as clk_aon.  If they are not - and in most
# real systems they would not be - this is an unsynchronised crossing on the
# path that carries the crash position, and it violates RULE R7.
#
# It is written down here rather than quietly constrained because it is a real
# design decision that has not been made yet.  Two ways to close it:
#   a) specify that the GNSS module is clocked from clk_aon, and say so in the
#      system spec, or
#   b) put gps_rx_valid through an ivcu_cdc_pulse_sync and treat gps_rx_data
#      as quasi-static, qualified by the synchronised valid
# Option (b) costs about a dozen flops and removes the assumption entirely.
#-----------------------------------------------------------------------------


#-----------------------------------------------------------------------------
# 6.  MULTICYCLE PATHS
#
# There are none, and that is worth stating explicitly rather than leaving as
# an absence.
#
# Three blocks - ivcu_health_scorer, ivcu_service_guidance and the scan in
# ivcu_fault_logger - walk all 64 channels one per clock.  It would be
# tempting to declare those multicycle.  They are not: each individual step is
# a single-clock register-to-register path, and the 64-clock duration is
# sequential behaviour, not a relaxed timing requirement.
#
# Declaring a multicycle there would tell STA to ignore real single-cycle
# paths, which is how a design passes timing and fails silicon.
#
# If a specific internal path fails after P&R, add a targeted constraint at
# that point with a comment explaining why it is genuinely multicycle.  Do not
# add one to make a number look better.
#-----------------------------------------------------------------------------


#-----------------------------------------------------------------------------
# 7.  DESIGN-RULE CONSTRAINTS  (Sky130 HD limits)
#
# max_transition : 0.4 ns.  The library's own max is around 0.5 ns at
#                  tt/1.8V/25C and 0.5 is marginal on HD cells; 0.4 leaves
#                  room for the difference between the extracted and the
#                  estimated parasitics.
# max_capacitance: 0.2 pF per net.
# max_fanout     : 32, which keeps buffer insertion at sensible intervals.
#-----------------------------------------------------------------------------
set_max_transition  0.4 [current_design]
set_max_capacitance 0.2 [current_design]
set_max_fanout      32  [current_design]


#-----------------------------------------------------------------------------
# 8.  OUTPUT LOADING AND INPUT DRIVE
#
# Without these, STA assumes an ideal driver and zero load, and every boundary
# path looks better than it is.  Two inverter loads is a reasonable stand-in
# for a bond wire and a receiver until there is a real package model.
#-----------------------------------------------------------------------------
set_load 0.05 [all_outputs]
set_driving_cell -lib_cell sky130_fd_sc_hd__inv_2 -pin Y [all_inputs]


#-----------------------------------------------------------------------------
# 9.  WHAT TO CHECK AFTER THE RUN
#
#   grep "Chip area for top module"  synth_out/synth_full_*.log
#   sta -no_splash -exit scripts/run_sta_diag.tcl | tee sta_out/diag_v4.txt
#
# Compare against the V3 baseline:
#     standard cell area   633,274 um2
#     u_diagnostic         118,908 um2   <- deleted in V4, expect 0
#     u_sensor_grace       105,193 um2   <- deleted, replaced by permit FSMs
#     u_adc_interface       62,111 um2   <- replaced by the scan sequencer
#     SRAM macros          572,456 um2   <- one macro now, expect about half
#     WNS                  -1.389 ns
#     TNS                 -48.323 ns
#
# The single number to watch is the SRAM total.  V4 instantiates one
# sram_512x32_2port instead of two, so if the macro area has not roughly
# halved, check that the second instance really is gone from the netlist.
#-----------------------------------------------------------------------------
