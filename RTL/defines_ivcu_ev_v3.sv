// defines_ivcu_ev_v3.v  – single include guard works for both .v and .sv files
`ifndef DEFINES_IVCU_EV_V3_SV
`define DEFINES_IVCU_EV_V3_SV

// Project Constants
`define PROJECT_NAME "IVCU_EV_V3"
`define VERSION_MAJOR 3
`define VERSION_MINOR 0
`define VERSION_PATCH 1

// Clock Domains
`define CLK_AI_FREQ    100_000_000
`define CLK_AON_FREQ    10_000_000
`define CLK_SENSOR_FREQ 50_000_000
`define CLK_MCU_FREQ    200_000_000

// Power Domains
`define PD_AON     2'b00
`define PD_AI      2'b01
`define PD_SENSOR  2'b10
`define PD_MCU     2'b11

// Vehicle Modes
`define MODE_CAR   2'b00
`define MODE_BIKE  2'b01
`define MODE_AUTO  2'b10
`define MODE_SAFE  2'b11

// Sensor Counts
`define NUM_SENSORS         42
`define NUM_BATTERY_SENSORS 13
`define NUM_MOTOR_SENSORS   9
`define NUM_THERMAL_SENSORS 6
`define NUM_SAFETY_SENSORS  14

// Sensor Indices
`define SENSOR_BATT_CELL_TEMP    0
`define SENSOR_BATT_PACK_TEMP    1
`define SENSOR_MOTOR_TEMP        2
`define SENSOR_INVERTER_TEMP     3
`define SENSOR_AMBIENT_TEMP      4
`define SENSOR_BATT_CELL_VOLT    5
`define SENSOR_BATT_PACK_VOLT    6
`define SENSOR_CHARGING_VOLT     7
`define SENSOR_BATT_CURRENT      8
`define SENSOR_CHARGING_CURRENT  9
`define SENSOR_COOLANT_FLOW      10
`define SENSOR_SOC               11
`define SENSOR_SOH               12
`define SENSOR_CRASH_IMPACT      13
`define SENSOR_GYROSCOPE         14
`define SENSOR_IMU               15
`define SENSOR_WHEEL_SPEED       16
`define SENSOR_MOTOR_RPM         17
`define SENSOR_ROTOR_POS         18
`define SENSOR_THROTTLE_POS      19
`define SENSOR_BRAKE_PRESSURE    20
`define SENSOR_BRAKE_SWITCH      21
`define SENSOR_STEERING_ANGLE    22
`define SENSOR_SIDE_STAND        23
`define SENSOR_GEAR_POS          24
`define SENSOR_COOLING_PRESS     25
`define SENSOR_HUMIDITY          26
`define SENSOR_ENCLOSURE_PRESS   27
`define SENSOR_ULTRASONIC        28
`define SENSOR_CAMERA            29
`define SENSOR_RADAR             30
`define SENSOR_LIDAR             31
`define SENSOR_GPS               32
`define SENSOR_TPMS              33
`define SENSOR_CABIN_TEMP        34
`define SENSOR_SEAT_OCCUPANCY    35

// AI Thresholds
`define BATTERY_TEMP_CRITICAL  16'd1000
`define BATTERY_TEMP_WARNING   16'd900
`define MOTOR_TEMP_CRITICAL    16'd1200
`define MOTOR_TEMP_WARNING     16'd1000
`define RPM_CRITICAL           16'd12000
`define RPM_WARNING            16'd10000
`define CURRENT_CRITICAL       16'd1000
`define CURRENT_WARNING        16'd800

// Grace Period
`define GRACE_PERIOD_COUNT     5
`define GRACE_TIMEOUT          32'd1000000

// Fault Codes
`define FAULT_NONE             8'h00
`define FAULT_BATTERY_TEMP     8'h01
`define FAULT_BATTERY_VOLT     8'h02
`define FAULT_MOTOR_TEMP       8'h03
`define FAULT_MOTOR_SPEED      8'h04
`define FAULT_THROTTLE_BRAKE   8'h05
`define FAULT_CRASH            8'h06
`define FAULT_SENSOR_FAIL      8'h07
`define FAULT_THERMAL          8'h08
`define FAULT_PERCEPTION       8'h09

// Status Codes
`define STATUS_OK              4'b0000
`define STATUS_WARNING         4'b0001
`define STATUS_CRITICAL        4'b0010
`define STATUS_FAULT           4'b0100
`define STATUS_EMERGENCY       4'b1000

// Interface Parameters
`define AXI_DATA_WIDTH         64
`define AXI_ADDR_WIDTH         32
`define SRAM_DATA_WIDTH        32
`define SRAM_ADDR_WIDTH        10
`define ADC_WIDTH              12

// Timing Constraints
`define SETUP_TIME_PS          100
`define HOLD_TIME_PS           50
`define CLK_TO_Q_PS            150

// Power States
`define PWR_OFF                2'b00
`define PWR_STANDBY            2'b01
`define PWR_ACTIVE             2'b10
`define PWR_FULL               2'b11

// Debug Modes
`define DEBUG_OFF              3'b000
`define DEBUG_SENSORS          3'b001
`define DEBUG_AI               3'b010
`define DEBUG_FAULTS           3'b011
`define DEBUG_PERFORMANCE      3'b100

`endif // DEFINES_IVCU_EV_V3_SV
