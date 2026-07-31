// mcu_axi_lite_interface.sv
`timescale 1ns/1ps
`include "defines_ivcu_ev_v3.sv"
`default_nettype none

module mcu_axi_lite_interface (
    input  wire        clk_mcu,
    input  wire        rst_mcu_n,
    // AXI Master Interface (placeholder)
    output reg         m_axi_awvalid,
    input  wire        m_axi_awready,
    output reg  [31:0] m_axi_awaddr,
    output reg  [2:0]  m_axi_awprot,
    output reg         m_axi_wvalid,
    input  wire        m_axi_wready,
    output reg  [63:0] m_axi_wdata,
    output reg  [7:0]  m_axi_wstrb,
    input  wire        m_axi_bvalid,
    output reg         m_axi_bready,
    input  wire [1:0]  m_axi_bresp,
    output reg         m_axi_arvalid,
    input  wire        m_axi_arready,
    output reg  [31:0] m_axi_araddr,
    output reg  [2:0]  m_axi_arprot,
    input  wire        m_axi_rvalid,
    output reg         m_axi_rready,
    input  wire [63:0] m_axi_rdata,
    input  wire [1:0]  m_axi_rresp,
    // AXI Slave Interface
    input  wire        s_axi_awvalid,
    output reg         s_axi_awready,
    input  wire [31:0] s_axi_awaddr,
    input  wire [2:0]  s_axi_awprot,
    input  wire        s_axi_wvalid,
    output reg         s_axi_wready,
    input  wire [63:0] s_axi_wdata,
    input  wire [7:0]  s_axi_wstrb,
    output reg         s_axi_bvalid,
    input  wire        s_axi_bready,
    output reg  [1:0]  s_axi_bresp,
    input  wire        s_axi_arvalid,
    output reg         s_axi_arready,
    input  wire [31:0] s_axi_araddr,
    input  wire [2:0]  s_axi_arprot,
    output reg         s_axi_rvalid,
    input  wire        s_axi_rready,
    output reg  [63:0] s_axi_rdata,
    output reg  [1:0]  s_axi_rresp,
    // Internal Interface
    input  wire [15:0] sensor_data_0,
    input  wire [15:0] sensor_data_1,
    input  wire [15:0] sensor_data_2,
    input  wire [15:0] sensor_data_3,
    input  wire [15:0] sensor_data_4,
    input  wire [15:0] sensor_data_5,
    input  wire [15:0] sensor_data_6,
    input  wire [15:0] sensor_data_7,
    input  wire [15:0] sensor_data_8,
    input  wire [15:0] sensor_data_9,
    input  wire [15:0] sensor_data_10,
    input  wire [15:0] sensor_data_11,
    input  wire [15:0] sensor_data_12,
    input  wire [15:0] sensor_data_13,
    input  wire [15:0] sensor_data_14,
    input  wire [15:0] sensor_data_15,
    input  wire [15:0] sensor_data_16,
    input  wire [15:0] sensor_data_17,
    input  wire [15:0] sensor_data_18,
    input  wire [15:0] sensor_data_19,
    input  wire [15:0] sensor_data_20,
    input  wire [15:0] sensor_data_21,
    input  wire [15:0] sensor_data_22,
    input  wire [15:0] sensor_data_23,
    input  wire [15:0] sensor_data_24,
    input  wire [15:0] sensor_data_25,
    input  wire [15:0] sensor_data_26,
    input  wire [15:0] sensor_data_27,
    input  wire [15:0] sensor_data_28,
    input  wire [15:0] sensor_data_29,
    input  wire [15:0] sensor_data_30,
    input  wire [15:0] sensor_data_31,
    input  wire [15:0] sensor_data_32,
    input  wire [15:0] sensor_data_33,
    input  wire [15:0] sensor_data_34,
    input  wire [15:0] sensor_data_35,
    input  wire [15:0] sensor_data_36,
    input  wire [15:0] sensor_data_37,
    input  wire [15:0] sensor_data_38,
    input  wire [15:0] sensor_data_39,
    input  wire [15:0] sensor_data_40,
    input  wire [15:0] sensor_data_41,
    input  wire [127:0] sensor_status,
    input  wire [127:0] ai_status,
    input  wire [15:0]  control_register,
    input  wire [7:0]   status_register,
    input  wire [7:0]   fault_register,
    output reg          write_enable,
    output reg  [9:0]   write_addr,
    output reg  [31:0]  write_data,
    input  wire [31:0]  read_data,
    input  wire [2:0]   debug_mode,
    output reg  [31:0]  debug_data,
    output reg          debug_valid,
    // FIX: real AXI4-Lite master engine added below (was a permanent-inactive
    // placeholder - m_axi_awvalid/wvalid/arvalid were forced to 0 every cycle,
    // meaning this design could never actually act as a bus master). These
    // are simple request/done handshake ports for whatever internal logic
    // wants to issue a transaction; nothing in this file decides what address
    // or data to send - that's a scope decision for whoever wires this up.
    input  wire         mst_wr_req,
    input  wire [31:0]  mst_wr_addr,
    input  wire [63:0]  mst_wr_data,
    input  wire [7:0]   mst_wr_strb,
    output reg          mst_wr_done,
    output reg  [1:0]   mst_wr_resp,
    input  wire         mst_rd_req,
    input  wire [31:0]  mst_rd_addr,
    output reg  [63:0]  mst_rd_data,
    output reg  [1:0]   mst_rd_resp,
    output reg          mst_rd_done
);

    // Slave FSM states
    typedef enum logic [2:0] {
        S_IDLE,
        S_WADDR,
        S_WDATA,
        S_WRESP,
        S_RADDR,
        S_RDATA
    } state_t;

    state_t slave_state, slave_next;

    reg [31:0] raddr_reg;
    reg [31:0] waddr_reg;
    reg [63:0] wdata_reg;
    reg [7:0]  wstrb_reg;

    // Combinational next-state logic
    always_comb begin
        slave_next = slave_state;
        case (slave_state)
            S_IDLE: begin
                if (s_axi_awvalid)
                    slave_next = S_WADDR;
                else if (s_axi_arvalid)
                    slave_next = S_RADDR;
            end
            S_WADDR:   slave_next = S_WDATA;
            S_WDATA:   if (s_axi_wvalid) slave_next = S_WRESP;
            S_WRESP:   if (s_axi_bready) slave_next = S_IDLE;
            S_RADDR:   slave_next = S_RDATA;
            S_RDATA:   if (s_axi_rready) slave_next = S_IDLE;
            default:   slave_next = S_IDLE;
        endcase
    end

    // Sequential state update and output generation
    always_ff @(posedge clk_mcu or negedge rst_mcu_n) begin
        if (!rst_mcu_n) begin
            slave_state <= S_IDLE;
            s_axi_awready <= 1'b0;
            s_axi_wready <= 1'b0;
            s_axi_bvalid <= 1'b0;
            s_axi_arready <= 1'b0;
            s_axi_rvalid <= 1'b0;
            s_axi_rdata <= 64'd0;
            s_axi_bresp <= 2'b00;
            s_axi_rresp <= 2'b00;
            write_enable <= 1'b0;
            raddr_reg <= 32'd0;
            waddr_reg <= 32'd0;
            wdata_reg <= 64'd0;
            wstrb_reg <= 8'd0;
        end else begin
            slave_state <= slave_next;
            // Default outputs (overridden in states)
            s_axi_awready <= 1'b0;
            s_axi_wready <= 1'b0;
            s_axi_bvalid <= 1'b0;
            s_axi_arready <= 1'b0;
            s_axi_rvalid <= 1'b0;
            write_enable <= 1'b0;

            case (slave_state)
                S_IDLE: begin
                    s_axi_awready <= 1'b1;
                    s_axi_arready <= 1'b1;
                end
                S_WADDR: begin
                    waddr_reg <= s_axi_awaddr;
                end
                S_WDATA: begin
                    s_axi_wready <= 1'b1;
                    if (s_axi_wvalid) begin
                        wdata_reg <= s_axi_wdata;
                        wstrb_reg <= s_axi_wstrb;
                        if (waddr_reg == 32'h0000_1000) begin
                            write_enable <= 1'b1;
                            write_addr <= waddr_reg[9:0];
                            write_data <= s_axi_wdata[31:0];
                        end
                    end
                end
                S_WRESP: begin
                    s_axi_bvalid <= 1'b1;
                    s_axi_bresp <= 2'b00;
                end
                S_RADDR: begin
                    raddr_reg <= s_axi_araddr;
                end
                S_RDATA: begin
                    s_axi_rvalid <= 1'b1;
                    s_axi_rresp <= 2'b00;
                    case (raddr_reg)
                        32'h0000_0000: s_axi_rdata <= {56'd0, status_register};
                        32'h0000_0008: s_axi_rdata <= {48'd0, control_register};
                        32'h0000_0010: s_axi_rdata <= {56'd0, fault_register};
                        32'h0000_1000: s_axi_rdata <= {32'd0, read_data};
                        default: s_axi_rdata <= 64'd0;
                    endcase
                end
            endcase
        end
    end

    // Master interface - real AXI4-Lite master FSM (see port comment above
    // for why mst_wr_req/mst_rd_req exist and what they're for)
    typedef enum logic [2:0] {
        M_IDLE,
        M_WRITE,
        M_WRITE_RESP,
        M_READ_ADDR,
        M_READ_DATA
    } mst_state_t;

    mst_state_t mst_state;
    reg aw_sent, w_sent;

    always_ff @(posedge clk_mcu or negedge rst_mcu_n) begin
        if (!rst_mcu_n) begin
            mst_state     <= M_IDLE;
            m_axi_awvalid <= 1'b0;
            m_axi_wvalid  <= 1'b0;
            m_axi_arvalid <= 1'b0;
            m_axi_bready  <= 1'b0;
            m_axi_rready  <= 1'b0;
            m_axi_awaddr  <= 32'd0;
            m_axi_wdata   <= 64'd0;
            m_axi_araddr  <= 32'd0;
            m_axi_awprot  <= 3'b000;
            m_axi_arprot  <= 3'b000;
            m_axi_wstrb   <= 8'hFF;
            aw_sent       <= 1'b0;
            w_sent        <= 1'b0;
            mst_wr_done   <= 1'b0;
            mst_wr_resp   <= 2'b00;
            mst_rd_data   <= 64'd0;
            mst_rd_resp   <= 2'b00;
            mst_rd_done   <= 1'b0;
        end else begin
            mst_wr_done <= 1'b0;   // both *_done are single-cycle pulses
            mst_rd_done <= 1'b0;

            case (mst_state)
                M_IDLE: begin
                    aw_sent <= 1'b0;
                    w_sent  <= 1'b0;
                    if (mst_wr_req) begin
                        m_axi_awvalid <= 1'b1;
                        m_axi_awaddr  <= mst_wr_addr;
                        m_axi_awprot  <= 3'b000;
                        m_axi_wvalid  <= 1'b1;
                        m_axi_wdata   <= mst_wr_data;
                        m_axi_wstrb   <= mst_wr_strb;
                        mst_state     <= M_WRITE;
                    end else if (mst_rd_req) begin
                        m_axi_arvalid <= 1'b1;
                        m_axi_araddr  <= mst_rd_addr;
                        m_axi_arprot  <= 3'b000;
                        mst_state     <= M_READ_ADDR;
                    end
                end

                // AW and W are independent AXI channels - each can be
                // accepted on a different cycle, so latch each separately
                // and only move on once both have been accepted.
                M_WRITE: begin
                    if (m_axi_awvalid && m_axi_awready) begin
                        m_axi_awvalid <= 1'b0;
                        aw_sent       <= 1'b1;
                    end
                    if (m_axi_wvalid && m_axi_wready) begin
                        m_axi_wvalid <= 1'b0;
                        w_sent       <= 1'b1;
                    end
                    if ((aw_sent || (m_axi_awvalid && m_axi_awready)) &&
                        (w_sent  || (m_axi_wvalid  && m_axi_wready))) begin
                        m_axi_bready <= 1'b1;
                        mst_state    <= M_WRITE_RESP;
                    end
                end

                M_WRITE_RESP: begin
                    if (m_axi_bvalid && m_axi_bready) begin
                        m_axi_bready <= 1'b0;
                        mst_wr_resp  <= m_axi_bresp;
                        mst_wr_done  <= 1'b1;
                        mst_state    <= M_IDLE;
                    end
                end

                M_READ_ADDR: begin
                    if (m_axi_arvalid && m_axi_arready) begin
                        m_axi_arvalid <= 1'b0;
                        m_axi_rready  <= 1'b1;
                        mst_state     <= M_READ_DATA;
                    end
                end

                M_READ_DATA: begin
                    if (m_axi_rvalid && m_axi_rready) begin
                        m_axi_rready <= 1'b0;
                        mst_rd_data  <= m_axi_rdata;
                        mst_rd_resp  <= m_axi_rresp;
                        mst_rd_done  <= 1'b1;
                        mst_state    <= M_IDLE;
                    end
                end

                default: mst_state <= M_IDLE;
            endcase
        end
    end

    // Debug
    always_ff @(posedge clk_mcu or negedge rst_mcu_n) begin
        if (!rst_mcu_n) begin
            debug_data <= 32'd0;
            debug_valid <= 1'b0;
        end else begin
            debug_data <= {debug_mode, status_register, fault_register};
            debug_valid <= 1'b1;
        end
    end

endmodule
`default_nettype wire
