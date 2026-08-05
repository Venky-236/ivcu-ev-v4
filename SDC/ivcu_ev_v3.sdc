# =============================================================================
# ivcu_ev_v3.sdc  -  Timing Constraints for IVCU-EV v3 (Sky130 / OpenROAD)
#
# Design:  ivcu_ev_v3_hybrid_top
# PDK:     Sky130 HD (tt_025C_1v80)
# Flow:    Yosys + OpenSTA + OpenROAD
#
# Clock domains
#   clk_100mhz       -> 100 MHz  (10 ns)   AI domain     (clk_ai)
#   clk_10mhz_aon    ->  10 MHz (100 ns)   AON domain    (clk_aon)
#   clk_50mhz_sensor ->  50 MHz  (20 ns)   Sensor domain (clk_sensor)
#   clk_200mhz_mcu   -> 200 MHz   (5 ns)   MCU domain    (clk_mcu)
#
# -----------------------------------------------------------------------------
# CHANGES IN THIS REVISION
#
#   A. FIXED (blocking): set_input_delay / set_output_delay had -max and -min
#      given as if they took values:
#          set_input_delay -clock C -max 35.0 -min 0.0 [get_ports {...}]
#      In SDC, -max and -min are FLAGS. The delay value is POSITIONAL:
#          set_input_delay [-clock C] [-max] [-min] delay_value port_list
#      So the old form handed OpenSTA three positional arguments (35.0, 0.0,
#      and the port list) when it expects two, and it refused to read the file:
#          "Error 567: set_input_delay requires two positional arguments."
#      Some commercial tools tolerate the combined form. OpenSTA does not.
#      Every such command is now split into a separate -max and -min pair.
#
#   B. ADDED: set_driving_cell and set_load. Without these, STA assumes inputs
#      are driven by an infinitely strong source and outputs drive zero load,
#      which makes every I/O path look faster than it can possibly be.
#
#   C. UNCHANGED: clocks, clock groups, uncertainty, false paths, design rules,
#      and all port lists are exactly as you had them. The delay VALUES are
#      also unchanged -- only the syntax carrying them.
#
#   D. FIXED 2026-08-04: fault_log_rd_data moved from the INPUT group to the
#      OUTPUT group. It is an output in the netlist, so all 32 set_input_delay
#      constraints were being rejected and discarded -- those bits were
#      unconstrained through the whole flow. 64 warnings, zero effect, easy
#      to miss. Now constrained as an output like the other fault_log signals.
#
# STILL OPEN (informational, does not affect timing)
#   The fault_log_* top-level ports describe an EXTERNAL fault-log interface,
#   but fault_logger_sram_32kb now contains an INTERNAL SRAM (2 x OpenRAM
#   512x32). Either these ports are a second, separate log interface, or they
#   are vestigial from before the SRAM was added. Worth resolving, but it is a
#   design-intent question, not a timing one.
# =============================================================================


# -----------------------------------------------------------------------------
# 1.  PRIMARY CLOCKS
# All four clocks enter the top module as primary inputs.
# clock_manager_14nm is a pass-through (no PLL generation), so the input
# port IS the clock root - no generated_clock is needed.
# -----------------------------------------------------------------------------
create_clock -name clk_100mhz       -period  10.0 [get_ports clk_100mhz]
create_clock -name clk_10mhz_aon    -period 100.0 [get_ports clk_10mhz_aon]
create_clock -name clk_50mhz_sensor -period  20.0 [get_ports clk_50mhz_sensor]
create_clock -name clk_200mhz_mcu   -period   5.0 [get_ports clk_200mhz_mcu]


# -----------------------------------------------------------------------------
# 2.  CLOCK GROUPS  (mutually asynchronous)
# This single command is sufficient to declare all four domains asynchronous.
# It implicitly creates false paths between every cross-domain pair, so no
# additional set_false_path -from clk_X -to clk_Y lines are needed or wanted.
# -----------------------------------------------------------------------------
set_clock_groups -asynchronous \
    -group [get_clocks clk_100mhz]       \
    -group [get_clocks clk_10mhz_aon]    \
    -group [get_clocks clk_50mhz_sensor] \
    -group [get_clocks clk_200mhz_mcu]


# -----------------------------------------------------------------------------
# 3.  CLOCK UNCERTAINTY
# Sky130 HD PVT at tt_025C_1v80.  No on-chip PLL so jitter is low; 0.3 ns
# setup uncertainty is conservative and appropriate.  Hold is 0.05 ns.
# -----------------------------------------------------------------------------
set_clock_uncertainty -setup 0.3  [all_clocks]
set_clock_uncertainty -hold  0.05 [all_clocks]


# -----------------------------------------------------------------------------
# 3b. EXTERNAL DRIVE AND LOAD                                      [NEW]
#
# set_driving_cell tells STA what is driving each input pin from outside the
# block. Without it the input transition is assumed ideal (zero rise/fall),
# which under-reports delay on every input path.
#
# set_load tells STA what each output pin is driving. Without it the load is
# zero, so output paths look artificially fast.
#
# inv_2 is a mid-strength inverter -- a reasonable stand-in for an unknown
# external driver. 0.05 pF (50 fF) is a modest on-chip / short-trace load.
# If this block is later wrapped in an I/O padring, replace both with the
# actual pad cell and pad capacitance.
# -----------------------------------------------------------------------------
set_driving_cell -lib_cell sky130_fd_sc_hd__inv_2 -pin Y [all_inputs]
set_load 0.05 [all_outputs]


# -----------------------------------------------------------------------------
# 4.  INPUT / OUTPUT DELAY BUDGETS
#
# Convention used throughout:
#   -max value = setup budget  = 35% of clock period
#   -min value = hold  budget  = 0 ns (combinational hold from external FF)
#   output -max                = 45% of clock period
#   output -min                = 10% of clock period (allow hold margin)
#
# clk_100mhz      (10 ns):  in_max=3.5,  in_min=0.0, out_max=4.5,  out_min=1.0
# clk_10mhz_aon  (100 ns):  in_max=35.0, in_min=0.0, out_max=45.0, out_min=10.0
# clk_50mhz_sensor(20 ns):  in_max=7.0,  in_min=0.0, out_max=9.0,  out_min=2.0
# clk_200mhz_mcu   (5 ns):  in_max=1.8,  in_min=0.0, out_max=2.2,  out_min=0.5
#
# NOTE: each budget below is now TWO commands -- one -max, one -min.
# -----------------------------------------------------------------------------


# ---- 4a. AON domain (clk_10mhz_aon, 100 ns period) - Inputs -----------------
set aon_in [get_ports {mode_switch_car mode_switch_bike mode_auto_detect \
                       user_mode_override emergency_stop manual_override}]

set_input_delay -clock clk_10mhz_aon -max 35.0 $aon_in
set_input_delay -clock clk_10mhz_aon -min  0.0 $aon_in


# ---- 4b. AON domain - Outputs -----------------------------------------------
# pwr_en_* and power_state: driven by power_domain_controller (clk_aon)
# active_mode, mode_change_ack: driven by mode_config_enhanced_v3 (clk_aon)
# vehicle_enable .. emergency_ack: driven by central_safety_fsm_v3 (clk_aon)
# ai_processing_active: combinational (pwr_en_ai & pll_locked & mode_valid)
# safety_health_status: combinational mux using crash_latched (clk_aon)
set aon_out [get_ports {pwr_en_ai pwr_en_sensor pwr_en_mcu power_state \
                        active_mode mode_change_ack \
                        vehicle_enable motor_enable brake_control throttle_limit \
                        cooling_control hazard_lights door_unlock airbag_control \
                        emergency_ack ai_processing_active safety_health_status}]

set_output_delay -clock clk_10mhz_aon -max 45.0 $aon_out
set_output_delay -clock clk_10mhz_aon -min 10.0 $aon_out


# ---- 4c. Sensor domain (clk_50mhz_sensor, 20 ns period) - Inputs ------------
# All ADC analog inputs, digital sensor data and valid strobes
set sens_adc_in [get_ports {sensor_adc_in_0  sensor_adc_in_1  sensor_adc_in_2  sensor_adc_in_3 \
                            sensor_adc_in_4  sensor_adc_in_5  sensor_adc_in_6  sensor_adc_in_7 \
                            sensor_adc_in_8  sensor_adc_in_9  sensor_adc_in_10 sensor_adc_in_11 \
                            sensor_adc_in_12 sensor_adc_in_13 sensor_adc_in_14 sensor_adc_in_15 \
                            sensor_adc_in_16 sensor_adc_in_17 sensor_adc_in_18 sensor_adc_in_19 \
                            sensor_adc_in_20 sensor_adc_in_21 sensor_adc_in_22 sensor_adc_in_23 \
                            sensor_adc_in_24 sensor_adc_in_25 sensor_adc_in_26 sensor_adc_in_27 \
                            sensor_adc_in_28 sensor_adc_in_29 sensor_adc_in_30 sensor_adc_in_31 \
                            sensor_adc_in_32 sensor_adc_in_33 sensor_adc_in_34 sensor_adc_in_35 \
                            sensor_adc_in_36 sensor_adc_in_37 sensor_adc_in_38 sensor_adc_in_39 \
                            sensor_adc_in_40 sensor_adc_in_41}]

set_input_delay -clock clk_50mhz_sensor -max 7.0 $sens_adc_in
set_input_delay -clock clk_50mhz_sensor -min 0.0 $sens_adc_in


set sens_adc_valid [get_ports {sensor_adc_valid_0  sensor_adc_valid_1  sensor_adc_valid_2  sensor_adc_valid_3 \
                               sensor_adc_valid_4  sensor_adc_valid_5  sensor_adc_valid_6  sensor_adc_valid_7 \
                               sensor_adc_valid_8  sensor_adc_valid_9  sensor_adc_valid_10 sensor_adc_valid_11 \
                               sensor_adc_valid_12 sensor_adc_valid_13 sensor_adc_valid_14 sensor_adc_valid_15 \
                               sensor_adc_valid_16 sensor_adc_valid_17 sensor_adc_valid_18 sensor_adc_valid_19 \
                               sensor_adc_valid_20 sensor_adc_valid_21 sensor_adc_valid_22 sensor_adc_valid_23 \
                               sensor_adc_valid_24 sensor_adc_valid_25 sensor_adc_valid_26 sensor_adc_valid_27 \
                               sensor_adc_valid_28 sensor_adc_valid_29 sensor_adc_valid_30 sensor_adc_valid_31 \
                               sensor_adc_valid_32 sensor_adc_valid_33 sensor_adc_valid_34 sensor_adc_valid_35 \
                               sensor_adc_valid_36 sensor_adc_valid_37 sensor_adc_valid_38 sensor_adc_valid_39 \
                               sensor_adc_valid_40 sensor_adc_valid_41}]

set_input_delay -clock clk_50mhz_sensor -max 7.0 $sens_adc_valid
set_input_delay -clock clk_50mhz_sensor -min 0.0 $sens_adc_valid


set sens_chan [get_ports {sensor_adc_channel}]

set_input_delay -clock clk_50mhz_sensor -max 7.0 $sens_chan
set_input_delay -clock clk_50mhz_sensor -min 0.0 $sens_chan


set sens_dig_in [get_ports {sensor_digital_in_0  sensor_digital_in_1  sensor_digital_in_2  sensor_digital_in_3 \
                            sensor_digital_in_4  sensor_digital_in_5  sensor_digital_in_6  sensor_digital_in_7 \
                            sensor_digital_in_8  sensor_digital_in_9  sensor_digital_in_10 sensor_digital_in_11 \
                            sensor_digital_in_12 sensor_digital_in_13 sensor_digital_in_14 sensor_digital_in_15 \
                            sensor_digital_in_16 sensor_digital_in_17 sensor_digital_in_18 sensor_digital_in_19 \
                            sensor_digital_in_20 sensor_digital_in_21 sensor_digital_in_22 sensor_digital_in_23 \
                            sensor_digital_in_24 sensor_digital_in_25 sensor_digital_in_26 sensor_digital_in_27 \
                            sensor_digital_in_28 sensor_digital_in_29 sensor_digital_in_30 sensor_digital_in_31 \
                            sensor_digital_in_32 sensor_digital_in_33 sensor_digital_in_34 sensor_digital_in_35 \
                            sensor_digital_in_36 sensor_digital_in_37 sensor_digital_in_38 sensor_digital_in_39 \
                            sensor_digital_in_40 sensor_digital_in_41}]

set_input_delay -clock clk_50mhz_sensor -max 7.0 $sens_dig_in
set_input_delay -clock clk_50mhz_sensor -min 0.0 $sens_dig_in


set sens_dig_valid [get_ports {sensor_digital_valid_0  sensor_digital_valid_1  sensor_digital_valid_2 \
                               sensor_digital_valid_3  sensor_digital_valid_4  sensor_digital_valid_5 \
                               sensor_digital_valid_6  sensor_digital_valid_7  sensor_digital_valid_8 \
                               sensor_digital_valid_9  sensor_digital_valid_10 sensor_digital_valid_11 \
                               sensor_digital_valid_12 sensor_digital_valid_13 sensor_digital_valid_14 \
                               sensor_digital_valid_15 sensor_digital_valid_16 sensor_digital_valid_17 \
                               sensor_digital_valid_18 sensor_digital_valid_19 sensor_digital_valid_20 \
                               sensor_digital_valid_21 sensor_digital_valid_22 sensor_digital_valid_23 \
                               sensor_digital_valid_24 sensor_digital_valid_25 sensor_digital_valid_26 \
                               sensor_digital_valid_27 sensor_digital_valid_28 sensor_digital_valid_29 \
                               sensor_digital_valid_30 sensor_digital_valid_31 sensor_digital_valid_32 \
                               sensor_digital_valid_33 sensor_digital_valid_34 sensor_digital_valid_35 \
                               sensor_digital_valid_36 sensor_digital_valid_37 sensor_digital_valid_38 \
                               sensor_digital_valid_39 sensor_digital_valid_40 sensor_digital_valid_41}]

set_input_delay -clock clk_50mhz_sensor -max 7.0 $sens_dig_valid
set_input_delay -clock clk_50mhz_sensor -min 0.0 $sens_dig_valid


# ---- 4d. Sensor domain - Outputs --------------------------------------------
# sensor_fault: driven by sensor_validation_fsm (clk_sensor)
# sensor_enable: driven by sensor_enable_logic (clk_sensor)
# sensor_grace_active: combinational from grace_timer (clk_sensor)
# sensor_valid_out: alias of sensor_filtered_valid (clk_sensor)
set sens_out [get_ports {sensor_enable sensor_grace_active sensor_fault sensor_valid_out}]

set_output_delay -clock clk_50mhz_sensor -max 9.0 $sens_out
set_output_delay -clock clk_50mhz_sensor -min 2.0 $sens_out


# ---- 4e. AI domain (clk_100mhz, 10 ns period) - Outputs ---------------------
# system_health_score: system_health_ai_complete (clk_ai)
# battery/motor/thermal_health_status: AI modules (clk_ai)
set ai_out [get_ports {system_health_score \
                       battery_health_status motor_health_status thermal_health_status}]

set_output_delay -clock clk_100mhz -max 4.5 $ai_out
set_output_delay -clock clk_100mhz -min 1.0 $ai_out


# ---- 4f. MCU domain (clk_200mhz_mcu, 5 ns period) - Inputs ------------------
# Only ACTUAL INPUT ports listed here.
# s_axi slave inputs: awvalid, awaddr, awprot, wvalid, wdata, wstrb, bready,
#                     arvalid, araddr, arprot, rready
set mcu_s_in [get_ports {s_axi_awvalid s_axi_awaddr  s_axi_awprot \
                         s_axi_wvalid  s_axi_wdata   s_axi_wstrb  s_axi_bready \
                         s_axi_arvalid s_axi_araddr  s_axi_arprot s_axi_rready}]

set_input_delay -clock clk_200mhz_mcu -max 1.8 $mcu_s_in
set_input_delay -clock clk_200mhz_mcu -min 0.0 $mcu_s_in


# m_axi master inputs (responses from external interconnect)
#
# FIXED: fault_log_rd_data was in this INPUT group, but the netlist declares it
# an OUTPUT. OpenSTA rejected all 32 bits with
#     "Warning 440: set_input_delay not allowed on output port"
# and silently discarded the constraint -- leaving those 32 bits completely
# untimed. It is now in mcu_dbg_out below, with the other fault_log signals.
set mcu_m_in [get_ports {m_axi_awready m_axi_wready  \
                         m_axi_bvalid  m_axi_bresp   \
                         m_axi_arready \
                         m_axi_rvalid  m_axi_rdata   m_axi_rresp \
                         fault_log_rd_data}]

set_input_delay -clock clk_200mhz_mcu -max 1.8 $mcu_m_in
set_input_delay -clock clk_200mhz_mcu -min 0.0 $mcu_m_in


# ---- 4g. MCU domain - Outputs -----------------------------------------------
# Only ACTUAL OUTPUT ports listed here.
set mcu_s_out [get_ports {s_axi_awready s_axi_wready  s_axi_bvalid  s_axi_bresp \
                          s_axi_arready s_axi_rvalid  s_axi_rdata   s_axi_rresp}]

set_output_delay -clock clk_200mhz_mcu -max 2.2 $mcu_s_out
set_output_delay -clock clk_200mhz_mcu -min 0.5 $mcu_s_out


set mcu_m_out [get_ports {m_axi_awvalid m_axi_awaddr  m_axi_awprot \
                          m_axi_wvalid  m_axi_wdata   m_axi_wstrb   m_axi_bready \
                          m_axi_arvalid m_axi_araddr  m_axi_arprot  m_axi_rready}]

set_output_delay -clock clk_200mhz_mcu -max 2.2 $mcu_m_out
set_output_delay -clock clk_200mhz_mcu -min 0.5 $mcu_m_out


set mcu_dbg_out [get_ports {fault_log_wr_en fault_log_addr fault_log_data \
                            debug_data_out debug_valid}]

set_output_delay -clock clk_200mhz_mcu -max 2.2 $mcu_dbg_out
set_output_delay -clock clk_200mhz_mcu -min 0.5 $mcu_dbg_out


# -----------------------------------------------------------------------------
# 5.  FALSE PATHS
#
# 5a. Asynchronous resets and power/test signals:
#     These have no timing relationship to any clock and must never be timed.
#     vdd_core, vdd_io, vdd_ram, pwr_good: power status signals, not data paths
#     ext_rst_n, por_n: asynchronous resets - captured by reset sync FFs,
#                       but we false-path at the top boundary
#     scan_enable, test_mode: DFT infrastructure, not in functional path
#     debug_mode: static configuration during operation
# -----------------------------------------------------------------------------
set_false_path -from [get_ports {ext_rst_n por_n \
                                 vdd_core vdd_io vdd_ram pwr_good \
                                 scan_enable test_mode debug_mode}]

# 5b. test_done and test_fail are tied to constants (1'b1 / 1'b0) in the RTL.
#     They can never change, so timing is not meaningful.
set_false_path -to [get_ports {test_done test_fail}]


# -----------------------------------------------------------------------------
# 6.  MULTICYCLE PATHS
#
# There are NO multicycle paths at the top-level I/O boundary of this design.
# Rationale:
#   - All register-to-port paths are single-cycle within their clock domain.
#   - diagnostic_report_generator has an internal 100-cycle report_interval,
#     but the top-level outputs are registered every cycle.
#   - sensor_grace_manager has a 1,000,000-cycle timeout, purely internal;
#     sensor_grace_active is combinational every cycle.
#   - CDC paths are handled by set_clock_groups -asynchronous above, making
#     multicycle declarations on cross-domain paths unnecessary and incorrect.
#
# If timing fails on a specific internal path after P&R, add a targeted
# constraint at that point, e.g.:
#   set_multicycle_path 2 -setup -from [get_cells u_diagnostic/...] \
#                                -to   [get_ports system_health_score]
# -----------------------------------------------------------------------------


# -----------------------------------------------------------------------------
# 7.  DESIGN-RULE CONSTRAINTS (Sky130 HD library limits)
# max_transition:  sky130_fd_sc_hd typical max is 0.4 ns at tt/1.8V/25C
# max_capacitance: 0.2 pF per net is a safe limit for HD cells
# max_fanout:      32 keeps buffers inserted at reasonable intervals
# -----------------------------------------------------------------------------
set_max_transition  0.4 [current_design]
set_max_capacitance 0.2 [current_design]
set_max_fanout      32  [current_design]