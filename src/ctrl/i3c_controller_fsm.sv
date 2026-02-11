// SPDX-License-Identifier: Apache-2.0
//
// I3C Controller FSM
// Handles SCL timing generation for both Open Drain (OD) and Push Pull (PP) modes.
// OD mode uses CSR timing parameters (same as I2C).
// PP mode uses simplified 50% duty cycle derived from sys_clk_freq and SDR mode.

module i3c_controller_fsm
  import controller_pkg::*;
  import i3c_pkg::*;
(
    input logic clk_i,
    input logic rst_ni,

    // I3C Bus interface
    input  logic ctrl_scl_i,
    input  logic ctrl_sda_i,
    output logic ctrl_scl_o,
    output logic ctrl_sda_o,

    // Mode selection
    input  logic od_pp_mode_i,           // 0=Open Drain, 1=Push Pull
    output logic sel_od_pp_o,            // Pass-through to PHY

    // PP mode timing inputs
    input  logic [15:0] sys_clk_freq_i,  // System clock frequency in MHz (from CSR)
    input  i3c_trans_mode_e mode_i,      // SDR0-4 speed selection (from command descriptor)

    // OD mode timing inputs (from CSRs, same as I2C FSM)
    input  logic [19:0] thigh_i,         // SCL high period
    input  logic [19:0] tlow_i,          // SCL low period
    input  logic [19:0] t_r_i,           // Rise time
    input  logic [19:0] t_f_i,           // Fall time
    input  logic [19:0] thd_sta_i,       // START hold time
    input  logic [19:0] tsu_sta_i,       // START setup time
    input  logic [19:0] tsu_sto_i,       // STOP setup time
    input  logic [19:0] tsu_dat_i,       // Data setup time
    input  logic [19:0] thd_dat_i,       // Data hold time
    input  logic [19:0] t_buf_i,         // Bus free time

    // Control interface
    input  logic host_enable_i,
    output logic host_idle_o,

    // Data interface (directly connected to upper layer)
    input  logic       tx_valid_i,       // TX data valid from upper layer
    input  logic [7:0] tx_data_i,        // TX data byte
    output logic       tx_ready_o,       // Ready to accept TX data
    output logic       rx_valid_o,       // RX data valid
    output logic [7:0] rx_data_o,        // RX data byte
    input  logic       rx_ready_i,       // Upper layer ready for RX data

    // Command interface
    input  logic start_cmd_i,            // Issue START condition
    input  logic stop_cmd_i,             // Issue STOP condition
    input  logic tx_cmd_i,               // Transmit a byte
    input  logic rx_cmd_i,               // Receive a byte
    output logic cmd_done_o              // Command completed
);

  /*
   * TODO Section:
   * - have control for i3c_enable_fsm so that new data is fed in from flow_active.sv
   * - implement repeated start condition (currently not supported, but can be added by modifying state transitions and timing)
   * - during open drain mode, add behaviour if target takes control of the SDA bus (if sda_o does not match sda_i, that means target is driving the line and FSM should adapt accordingly)
   * - if there are errors, need to handle behavioiur such 
   *   as aborting the transfer and resetting the FSM to idle state.
   *   Error conditions can be detected by monitoring the bus lines for unexpected changes 
   *   (e.g. if SDA goes low when it should be high, or if SCL is not toggling as expected).
   *   In case of an error, the FSM should transition to an error state where it can wait for 
   *   a reset command from the upper layer before returning to idle.
   *   - need to update response descriptor with error status (i3c_response_desc_t from TCRI 7.1.3 Table 11) 
   * 
   */


  // Pass-through mode selection to PHY
  assign sel_od_pp_o = od_pp_mode_i;

  // ============================================================================
  // PP Half-Period Lookup Table
  // ============================================================================
  // Half-period values are pre-calculated for supported system clock frequencies.
  // Values use ceiling division to ensure SCL frequency never exceeds spec max. from I3C TCRI 7.1.1.1
  // Formula: pp_half_period = ceil(sys_clk_freq_mhz * 1e6 / (2 * sdr_freq_hz))

  logic [7:0] pp_half_period;

  always_comb begin
    unique case (sys_clk_freq_i)
      16'd100: begin
        // 100 MHz system clock
        unique case (mode_i)
          sdr0:    pp_half_period = 8'd4;    // 12.5 MHz SCL
          sdr1:    pp_half_period = 8'd7;    // ~7.14 MHz SCL
          sdr2:    pp_half_period = 8'd9;    // ~5.56 MHz SCL
          sdr3:    pp_half_period = 8'd13;   // ~3.85 MHz SCL
          sdr4:    pp_half_period = 8'd25;   // 2 MHz SCL
          default: pp_half_period = 8'd25;   // Default to slowest
        endcase
      end
      16'd200: begin
        // 200 MHz system clock
        unique case (mode_i)
          sdr0:    pp_half_period = 8'd8;    // 12.5 MHz SCL
          sdr1:    pp_half_period = 8'd13;   // ~7.69 MHz SCL
          sdr2:    pp_half_period = 8'd17;   // ~5.88 MHz SCL
          sdr3:    pp_half_period = 8'd25;   // 4 MHz SCL
          sdr4:    pp_half_period = 8'd50;   // 2 MHz SCL
          default: pp_half_period = 8'd50;   // Default to slowest
        endcase
      end
      16'd400: begin
        // 400 MHz system clock
        unique case (mode_i)
          sdr0:    pp_half_period = 8'd16;   // 12.5 MHz SCL
          sdr1:    pp_half_period = 8'd25;   // 8 MHz SCL
          sdr2:    pp_half_period = 8'd34;   // ~5.88 MHz SCL
          sdr3:    pp_half_period = 8'd50;   // 4 MHz SCL
          sdr4:    pp_half_period = 8'd100;  // 2 MHz SCL
          default: pp_half_period = 8'd100;  // Default to slowest
        endcase
      end
      default: begin
        // Unsupported frequency - default to 100 MHz values
        unique case (mode_i)
          sdr0:    pp_half_period = 8'd4;
          sdr1:    pp_half_period = 8'd7;
          sdr2:    pp_half_period = 8'd9;
          sdr3:    pp_half_period = 8'd13;
          sdr4:    pp_half_period = 8'd25;
          default: pp_half_period = 8'd25;
        endcase
      end
    endcase
  end

  // ============================================================================
  // Timing Counter
  // ============================================================================
  typedef enum logic [3:0] {
    tSetupStart,  // START setup: t_r + tsu_sta (OD) or 1 (PP)
    tHoldStart,   // START hold:  t_f + thd_sta (OD) or 1 (PP)
    tSetupData,   // Data setup:  t_r + tsu_dat (OD) or 1 (PP)
    tClockLow,    // SCL low:     tlow - thd_dat (OD) or half_period (PP)
    tClockPulse,  // SCL high:    t_r + thigh (OD) or half_period (PP)
    tHoldBit,     // Bit hold:    t_f + thd_dat (OD) or 1 (PP)
    tSetupStop,   // STOP setup:  t_r + tsu_sto (OD) or 1 (PP)
    tHoldStop,    // Bus free:    t_r + t_buf - tsu_sta (OD) or 1 (PP)
    tNoDelay      // Minimal delay
  } tcount_sel_e;

  logic [19:0] tcount_q;
  logic [19:0] tcount_d;
  logic        load_tcount;
  tcount_sel_e tcount_sel;

  always_comb begin : counter_functions
    tcount_d = tcount_q;
    if (load_tcount) begin
      if (!od_pp_mode_i) begin
        // Open Drain mode: use CSR timing parameters
        unique case (tcount_sel)
          tSetupStart: tcount_d = t_r_i + tsu_sta_i;
          tHoldStart:  tcount_d = t_f_i + thd_sta_i;
          tSetupData:  tcount_d = t_r_i + tsu_dat_i;
          tClockLow:   tcount_d = tlow_i - thd_dat_i;
          tClockPulse: tcount_d = t_r_i + thigh_i;
          tHoldBit:    tcount_d = t_f_i + thd_dat_i;
          tSetupStop:  tcount_d = t_r_i + tsu_sto_i;
          tHoldStop:   tcount_d = t_r_i + t_buf_i - tsu_sta_i;
          tNoDelay:    tcount_d = 20'd1;
          default:     tcount_d = 20'd1;
        endcase
      end else begin
        // Push Pull mode: simplified timing
        unique case (tcount_sel)
          tSetupStart: tcount_d = 20'd1;  // No setup time for PP
          tHoldStart:  tcount_d = 20'd1;  // No hold time for PP
          tSetupData:  tcount_d = 20'd1;
          tClockLow:   tcount_d = 20'(pp_half_period);  // 50% duty
          tClockPulse: tcount_d = 20'(pp_half_period);  // 50% duty
          tHoldBit:    tcount_d = 20'd1;
          tSetupStop:  tcount_d = 20'd1;
          tHoldStop:   tcount_d = 20'd1;
          tNoDelay:    tcount_d = 20'd1;
          default:     tcount_d = 20'd1;
        endcase
      end
    end else if (host_enable_i && (tcount_q > 20'd1)) begin
      tcount_d = tcount_q - 1'b1;
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin : clk_counter
    if (!rst_ni) begin
      tcount_q <= 20'd1;
    end else begin
      tcount_q <= tcount_d;
    end
  end

  // ============================================================================
  // Bit and Byte Counters
  // ============================================================================
  logic [2:0] bit_index;
  logic       bit_decr;
  logic       bit_clr;
  logic [7:0] shift_reg;
  logic       shift_data_en;
  logic       load_tx_data;

  always_ff @(posedge clk_i or negedge rst_ni) begin : bit_counter
    if (!rst_ni) begin
      bit_index <= 3'd7;
    end else if (bit_clr) begin
      bit_index <= 3'd7;
    end else if (bit_decr) begin
      bit_index <= bit_index - 1'b1;
    end
  end

  // Shift register for TX/RX data
  always_ff @(posedge clk_i or negedge rst_ni) begin : shift_register
    if (!rst_ni) begin
      shift_reg <= 8'h00;
    end else if (load_tx_data) begin
      shift_reg <= tx_data_i;
    end else if (shift_data_en) begin
      // Shift in from SDA (MSB first)
      shift_reg <= {shift_reg[6:0], ctrl_sda_i};
    end
  end

  // ============================================================================
  // FSM States
  // ============================================================================
  typedef enum logic [3:0] {
    Idle,         // Bus released, waiting for command
    SetupStart,   // START: Both lines released high
    HoldStart,    // START: SDA low, SCL high
    ClockStart,   // START: SCL pulled low
    ClockLow,     // Data: SCL low, data on SDA
    ClockPulse,   // Data: SCL high, data sampled
    HoldBit,      // Data: SCL low after pulse
    SetupStop,    // STOP: SCL high, SDA low
    HoldStop      // STOP: Both lines released
  } state_e;

  state_e state_q, state_d;

  // Internal control signals
  logic scl_d, sda_d;
  logic is_transmitting;  // Tracks if we're TX or RX

  // ============================================================================
  // State Outputs
  // ============================================================================
  always_comb begin : state_outputs
    host_idle_o = 1'b0;
    scl_d = 1'b1;
    sda_d = 1'b1;
    tx_ready_o = 1'b0;
    rx_valid_o = 1'b0;
    rx_data_o = shift_reg;
    cmd_done_o = 1'b0;

    unique case (state_q)
      Idle: begin
        host_idle_o = 1'b1;
        scl_d = 1'b1;
        sda_d = 1'b1;
        // Ready to accept TX data when idle and TX command pending
        tx_ready_o = tx_cmd_i && tx_valid_i;
      end

      SetupStart: begin
        // Both lines released (high) - setup for START
        scl_d = 1'b1;
        sda_d = 1'b1;
      end

      HoldStart: begin
        // SDA goes low while SCL is still high (START condition)
        scl_d = 1'b1;
        sda_d = 1'b0;
      end

      ClockStart: begin
        // SCL goes low, SDA stays low
        scl_d = 1'b0;
        sda_d = 1'b0;
      end

      ClockLow: begin
        // SCL low, put data on SDA (or release for RX)
        scl_d = 1'b0;
        if (is_transmitting) begin
          sda_d = shift_reg[bit_index];
        end else begin
          sda_d = 1'b1;  // Release SDA for target to drive
        end
      end

      ClockPulse: begin
        // SCL high, sample data
        scl_d = 1'b1;
        if (is_transmitting) begin
          sda_d = shift_reg[bit_index];
        end else begin
          sda_d = 1'b1;  // Keep SDA released for RX
        end
      end

      HoldBit: begin
        // SCL goes low after pulse
        scl_d = 1'b0;
        if (is_transmitting) begin
          sda_d = shift_reg[bit_index];
        end else begin
          sda_d = 1'b1;
        end
        // Output RX data when last bit is done
        if (!is_transmitting && (bit_index == 3'd0)) begin
          rx_valid_o = 1'b1;
        end
      end

      SetupStop: begin
        // SCL high, SDA low (prepare for STOP)
        scl_d = 1'b1;
        sda_d = 1'b0;
      end

      HoldStop: begin
        // Both lines released (STOP condition complete)
        scl_d = 1'b1;
        sda_d = 1'b1;
        cmd_done_o = 1'b1;
      end

      default: begin
        host_idle_o = 1'b1;
        scl_d = 1'b1;
        sda_d = 1'b1;
      end
    endcase
  end

  // ============================================================================
  // State Transitions
  // ============================================================================
  always_comb begin : state_functions
    state_d = state_q;
    load_tcount = 1'b0;
    tcount_sel = tNoDelay;
    bit_decr = 1'b0;
    bit_clr = 1'b0;
    shift_data_en = 1'b0;
    load_tx_data = 1'b0;

    unique case (state_q)
      Idle: begin
        if (host_enable_i) begin
          if (start_cmd_i) begin
            // Issue START condition
            state_d = SetupStart;
            load_tcount = 1'b1;
            tcount_sel = tSetupStart;
          end else if (tx_cmd_i && tx_valid_i) begin
            // Transmit a byte
            state_d = ClockLow;
            load_tcount = 1'b1;
            tcount_sel = tClockLow;
            load_tx_data = 1'b1;
            bit_clr = 1'b1;
          end else if (rx_cmd_i) begin
            // Receive a byte
            state_d = ClockLow;
            load_tcount = 1'b1;
            tcount_sel = tClockLow;
            bit_clr = 1'b1;
          end else if (stop_cmd_i) begin
            // Issue STOP condition
            state_d = SetupStop;
            load_tcount = 1'b1;
            tcount_sel = tSetupStop;
          end
        end
      end

      SetupStart: begin
        if (tcount_q == 20'd1) begin
          state_d = HoldStart;
          load_tcount = 1'b1;
          tcount_sel = tHoldStart;
        end
      end

      HoldStart: begin
        if (tcount_q == 20'd1) begin
          state_d = ClockStart;
          load_tcount = 1'b1;
          tcount_sel = tNoDelay;
        end
      end

      ClockStart: begin
        if (tcount_q == 20'd1) begin
          // START complete, go to Idle to wait for next command
          state_d = Idle;
          load_tcount = 1'b1;
          tcount_sel = tNoDelay;
        end
      end

      ClockLow: begin
        if (tcount_q == 20'd1) begin
          state_d = ClockPulse;
          load_tcount = 1'b1;
          tcount_sel = tClockPulse;
        end
      end

      ClockPulse: begin
        if (tcount_q == 20'd1) begin
          state_d = HoldBit;
          load_tcount = 1'b1;
          tcount_sel = tHoldBit;
          // Sample RX data on falling edge of SCL
          if (!is_transmitting) begin
            shift_data_en = 1'b1;
          end
        end
      end

      HoldBit: begin
        if (tcount_q == 20'd1) begin
          if (bit_index == 3'd0) begin
            // Byte complete, return to Idle
            state_d = Idle;
            load_tcount = 1'b1;
            tcount_sel = tNoDelay;
          end else begin
            // More bits to transfer
            state_d = ClockLow;
            load_tcount = 1'b1;
            tcount_sel = tClockLow;
            bit_decr = 1'b1;
          end
        end
      end

      SetupStop: begin
        if (tcount_q == 20'd1) begin
          state_d = HoldStop;
          load_tcount = 1'b1;
          tcount_sel = tHoldStop;
        end
      end

      HoldStop: begin
        if (tcount_q == 20'd1) begin
          state_d = Idle;
          load_tcount = 1'b1;
          tcount_sel = tNoDelay;
        end
      end

      default: begin
        state_d = Idle;
      end
    endcase
  end

  // ============================================================================
  // State Register
  // ============================================================================
  always_ff @(posedge clk_i or negedge rst_ni) begin : state_transition
    if (!rst_ni) begin
      state_q <= Idle;
    end else begin
      state_q <= state_d;
    end
  end

  // Track TX vs RX mode
  always_ff @(posedge clk_i or negedge rst_ni) begin : tx_rx_mode
    if (!rst_ni) begin
      is_transmitting <= 1'b0;
    end else if (state_q == Idle) begin
      if (tx_cmd_i && tx_valid_i) begin
        is_transmitting <= 1'b1;
      end else if (rx_cmd_i) begin
        is_transmitting <= 1'b0;
      end
    end
  end

  // ============================================================================
  // Output Assignments
  // ============================================================================
  assign ctrl_scl_o = scl_d;
  assign ctrl_sda_o = sda_d;

endmodule
