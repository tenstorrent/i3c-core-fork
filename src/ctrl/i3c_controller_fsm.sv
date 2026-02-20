// SPDX-License-Identifier: Apache-2.0
//
// I3C Controller FSM
// Handles SCL timing generation for both Open Drain (OD) and Push Pull (PP) modes.
// OD mode uses CSR timing parameters (same as I2C).
// PP mode uses simplified 50% duty cycle derived from sys_clk_freq and SDR mode.
//
// Interface designed to connect directly to flow_active.sv

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

    // PP mode timing inputs
    input  sys_clk_freq_e sys_clk_freq_i,  // System clock frequency selection (from CSR)
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

    // TX interface from flow_active.sv
    input  logic       tx_valid_i,       // Byte ready to send
    input  logic [7:0] tx_data_i,        // Data byte to send
    input  start_stop_e tx_start_stop_i, // Start/Stop/Repeated Start indication
    input  logic       tx_is_addr_i,     // Address byte (open-drain, expects ACK)
    input  logic       tx_use_tbit_i,    // Add T-bit after byte (push-pull I3C mode)
    output logic       tx_ready_o,       // Ready for next byte (1 in Idle and after byte completes)
    output logic       rx_ack_o,         // ACK received (valid when tx_ready_o && tx_is_addr_i was set)
    output logic       rx_nack_o,        // NACK received

    // RX interface to flow_active.sv
    input  logic       rx_req_i,         // Read byte request
    output logic       rx_valid_o,       // Received byte valid (pulses 1 cycle)
    output logic [7:0] rx_data_o,        // Received byte data

    output logic       sel_od_pp_o       // Select signal for OD vs PP mode (for timing and output control logic
);

  /*
   * TODO Section:
   * - during open drain mode, add behaviour if target takes control of the SDA bus ex. NACKs
   *   (if sda_o does not match sda_i, that means target is driving the line and FSM should adapt accordingly)
   * - if there are errors, need to handle behaviour such as aborting the transfer and resetting the FSM to idle state.
   * - add logic for OE (output enable) signals for proper tri-state control
   * - add RX byte support (currently only TX is implemented)
   */

  // ============================================================================
  // Latched Command Signals
  // ============================================================================
  // Latch command signals when byte transfer starts to use throughout the transfer
  start_stop_e tx_start_stop_q;
  logic tx_is_addr_q;
  logic tx_use_tbit_q;

  // ============================================================================
  // PP Half-Period Lookup Table
  // ============================================================================
  // Half-period values are pre-calculated for supported system clock frequencies.
  // Values use ceiling division to ensure SCL frequency never exceeds spec max. from I3C TCRI 7.1.1.1
  // Formula: pp_half_period = ceil(sys_clk_freq_mhz * 1e6 / (2 * sdr_freq_hz))

  logic [7:0] pp_half_period;

  //TODO add i2c speeds if i2c_cmd
  always_comb begin
    unique case (sys_clk_freq_i)
      SysClk100MHz: begin
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
      SysClk150MHz: begin
        // 150 MHz system clock
        unique case (mode_i)
          sdr0:    pp_half_period = 8'd6;    // 12.5 MHz SCL
          sdr1:    pp_half_period = 8'd10;   // ~7.5 MHz SCL
          sdr2:    pp_half_period = 8'd13;   // ~5.77 MHz SCL
          sdr3:    pp_half_period = 8'd19;   // ~3.95 MHz SCL
          sdr4:    pp_half_period = 8'd38;   // ~1.97 MHz SCL
          default: pp_half_period = 8'd38;   // Default to slowest
        endcase
      end
      SysClk200MHz: begin
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
      default: begin
        // Invalid enum value - default to 100 MHz values
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

  // TODO: Switching from PP to OD mode mid-transfer (repeated start with different address type) may require special handling to ensure timing requirements are met for both modes.
  // I3C Basic spec 5.1.2.3 

  // ============================================================================
  // Timing Counter
  // ============================================================================
  typedef enum logic [3:0] {
    tSetupStart,  // START setup: t_r + tsu_sta (OD) or pp_half_period/2 (PP)
    tHoldStart,   // START hold:  t_f + thd_sta (OD) or pp_half_period/2 (PP)
    tSetupData,   // Data setup:  t_r + tsu_dat (OD) or 1 (PP)
    tClockLow,    // SCL low:     tlow - thd_dat (OD) or half_period (PP)
    tClockPulse,  // SCL high:    t_r + thigh (OD) or half_period (PP)
    tHoldBit,     // Bit hold:    t_f + thd_dat (OD) or 1 (PP)
    tSetupStop,   // STOP setup:  t_r + tsu_sto (OD) or pp_half_period/2 (PP)
    tHoldStop,    // Bus free:    t_r + t_buf - tsu_sta (OD) or pp_half_period/2 (PP)
    tNoDelay      // Minimal delay
  } tcount_sel_e;

  logic [19:0] tcount_q;
  logic [19:0] tcount_d;
  logic        load_tcount;
  tcount_sel_e tcount_sel;

  always_comb begin : counter_functions
    tcount_d = tcount_q;
    if (load_tcount) begin
      if (sel_od_pp_o == 1'b0) begin
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
        // Push Pull mode: simplified timing with some margin
        unique case (tcount_sel)
          
          tSetupStart: tcount_d = 20'(pp_half_period >> 1);  // Half of half-period for margin
          tHoldStart:  tcount_d = 20'(pp_half_period >> 1);
          tSetupData:  tcount_d = 20'd2;  // Minimum 2 cycles for setup
          tClockLow:   tcount_d = 20'(pp_half_period);       // 50% duty
          tClockPulse: tcount_d = 20'(pp_half_period);       // 50% duty
          tHoldBit:    tcount_d = 20'd2;  // Minimum 2 cycles for hold
          tSetupStop:  tcount_d = 20'(pp_half_period >> 1);
          // TODO: Maybe need to add tAckPulse, due to t_dig_h from I3C Basic Spec 5.1.2.3.1
          tHoldStop:   tcount_d = 20'(pp_half_period >> 1);
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
  // Bit Counter
  // ============================================================================
  logic [3:0] bit_index;  // 4 bits to count 0-8 (8 data bits + 1 ACK/T-bit)
  logic       bit_decr;
  logic       bit_clr;
  logic [7:0] shift_reg;
  logic       shift_data_en;
  logic       load_tx_data;

  always_ff @(posedge clk_i or negedge rst_ni) begin : bit_counter
    if (!rst_ni) begin
      bit_index <= 4'd8;  // Start at 8 (will decrement to 7 for MSB)
    end else if (bit_clr) begin
      bit_index <= 4'd8;
    end else if (bit_decr) begin
      bit_index <= bit_index - 1'b1;
    end
  end

  // Shift register for TX data
  always_ff @(posedge clk_i or negedge rst_ni) begin : shift_register
    if (!rst_ni) begin
      shift_reg <= 8'h00;
    end else if (load_tx_data) begin
      shift_reg <= tx_data_i;
    end else if (shift_data_en) begin
      // Shift in from SDA (MSB first) - for RX
      shift_reg <= {shift_reg[6:0], ctrl_sda_i};
    end
  end

  // ============================================================================
  // ACK/NACK Detection (TX mode)
  // ============================================================================
  logic ack_sampled;
  logic sample_ack;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ack_sampled <= 1'b1;  // Default to NACK (SDA high)
    end else if (sample_ack) begin
      ack_sampled <= ctrl_sda_i;
    end
  end

  // ============================================================================
  // RX Data Path
  // ============================================================================
  logic [7:0] read_byte;       // Shift register for incoming data (MSB first)
  logic       read_byte_clr;   // Clear at start of new RX byte
  // Note: shift_data_en declared above in Bit Counter section, reused for RX

  // RX shift register - MSB first
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      read_byte <= 8'h00;
    end else if (read_byte_clr) begin
      read_byte <= 8'h00;
    end else if (shift_data_en) begin
      read_byte <= {read_byte[6:0], ctrl_sda_i};  // Shift in MSB first
    end
  end

  // ============================================================================
  // RX T-bit Handling
  // ============================================================================
  // T-bit=0: target done sending, T-bit=1: target has more data
  logic rx_tbit_q;      // Latched T-bit value from target
  logic sample_tbit;    // Sample T-bit on last cycle of ReadTbitPulse

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      rx_tbit_q <= 1'b1;  // Default to "more data"
    end else if (sample_tbit) begin
      rx_tbit_q <= ctrl_sda_i;  // Sample T-bit from target
    end
  end

  // ============================================================================
  // FSM States
  // ============================================================================
  typedef enum logic [4:0] {
    Idle,         // Bus released, waiting for command
    SetupStart,   // START: Both lines released high
    HoldStart,    // START: SDA low, SCL high
    ClockStart,   // START: SCL pulled low, prepare for data
    SetupSr,      // Sr: SCL low, release SDA high
    HoldSr,       // Sr: SCL high, SDA high (setup)
    FallSr,       // Sr: SDA falls while SCL high (Sr condition)
    ClockSr,      // Sr: SCL goes low after Sr
    ClockLow,     // TX Data: SCL low, drive data onto SDA
    ClockPulse,   // TX Data: SCL high, data stable
    HoldBit,      // TX Data: SCL low after pulse, prepare next bit
    AckLow,       // TX 9th bit: SCL low, release SDA (for ACK) or drive T-bit
    AckPulse,     // TX 9th bit: SCL high, sample ACK or T-bit stable
    AckHold,      // TX 9th bit: SCL low after ACK/T-bit
    SetupStop,    // STOP: SCL low, SDA low
    RiseStop,     // STOP: SCL high, SDA still low
    HoldStop,     // STOP: Both lines released (STOP complete)
    // RX data bit states (controller releases SDA, target drives)
    ReadClockLow,   // RX Data: SCL low, SDA released
    ReadClockPulse, // RX Data: SCL high, sample SDA at end
    ReadHoldBit,    // RX Data: SCL low after sampling
    // RX 9th bit (T-bit) states - TARGET drives T-bit, controller samples
    // T-bit=0: target done, T-bit=1: target has more data
    ReadTbitLow,    // RX T-bit: SCL low, SDA released (OD mode)
    ReadTbitPulse,  // RX T-bit: SCL high, sample T-bit from target
    ReadTbitHold    // RX T-bit: SCL low, respond based on T-bit value
  } state_e;

  state_e state_q, state_d;

  // Internal control signals
  logic scl_d, sda_d;

  // ============================================================================
  // Latch Command Signals
  // ============================================================================
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      tx_start_stop_q <= None;
      tx_is_addr_q    <= 1'b0;
      tx_use_tbit_q   <= 1'b0;
    end else if (state_q == Idle && tx_valid_i && host_enable_i) begin
      // Latch command signals when starting a new byte transfer
      tx_start_stop_q <= tx_start_stop_i;
      tx_is_addr_q    <= tx_is_addr_i;
      tx_use_tbit_q   <= tx_use_tbit_i;
    end
  end

  // ============================================================================
  // State Outputs
  // ============================================================================
  always_comb begin : state_outputs
    host_idle_o = 1'b0;
    scl_d = 1'b1;
    sda_d = 1'b1;
    tx_ready_o = 1'b0;
    rx_ack_o = 1'b0;
    rx_nack_o = 1'b0;
    rx_valid_o = 1'b0;
    rx_data_o = read_byte;

    // 0 = OD mode, 1 = PP mode

    /*
     * TODO: Look at 5.1.2.3 for handling sel_od_pp_o during transition periods. For now, atleast
    * for simulation this should be okay?
    */
    sel_od_pp_o = ~tx_is_addr_q;  // Default: PP for data, OD for address

    unique case (state_q)
      Idle: begin
        host_idle_o = 1'b1;
        scl_d = 1'b1;
        sda_d = 1'b1;
        // Ready to accept new byte when idle
        tx_ready_o = host_enable_i;
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
        // SCL goes low, SDA stays low, prepare to drive first data bit
        scl_d = 1'b0;
        sda_d = 1'b0;
      end

      SetupSr: begin
        // SCL low, release SDA high for repeated start setup
        scl_d = 1'b0;
        sda_d = 1'b1;
      end

      HoldSr: begin
        // SCL high, SDA high - setup time before Sr
        scl_d = 1'b1;
        sda_d = 1'b1;
      end

      FallSr: begin
        // SDA falls while SCL high - this is the repeated start condition
        scl_d = 1'b1;
        sda_d = 1'b0;
      end

      ClockSr: begin
        // SCL goes low after Sr, SDA stays low
        scl_d = 1'b0;
        sda_d = 1'b0;
      end

      ClockLow: begin
        // SCL low, drive data bit onto SDA
        scl_d = 1'b0;
        // bit_index 8 means we just started, use bit 7 (MSB)
        // bit_index 7-1 are data bits
        // bit_index 0 is handled in AckLow
        if (bit_index >= 4'd1) begin
          sda_d = shift_reg[bit_index[2:0] - 1];
        end else begin
          sda_d = shift_reg[0];
        end
      end

      ClockPulse: begin
        // SCL high, keep data stable
        scl_d = 1'b1;
        if (bit_index >= 4'd1) begin
          sda_d = shift_reg[bit_index[2:0] - 1];
        end else begin
          sda_d = shift_reg[0];
        end
      end

      HoldBit: begin
        // SCL goes low after pulse
        scl_d = 1'b0;
        if (bit_index >= 4'd1) begin
          sda_d = shift_reg[bit_index[2:0] - 1];
        end else begin
          sda_d = shift_reg[0];
        end
      end

      AckLow: begin
        // SCL low for 9th bit
        scl_d = 1'b0;
        if (tx_is_addr_q) begin
          // Address byte: release SDA for target to ACK
          sda_d = 1'b1;
          // Note: sample_ack is set in state_functions to avoid multi-driven signal
        end else if (tx_use_tbit_q) begin
          // Data byte with T-bit: drive T-bit (0 = more data, 1 = end)
          // For now, always drive 0 (more data coming), STOP handles end
          sda_d = 1'b0;
        end else begin
          sda_d = 1'b1;
        end
      end

      AckPulse: begin
        // SCL high for 9th bit - sample ACK or keep T-bit stable
        scl_d = 1'b1;
        if (tx_is_addr_q) begin
          sda_d = 1'b0; // Consult I3C Basic Spec 5.1.2.3.1
        end else if (tx_use_tbit_q) begin
          sda_d = 1'b0;  // Keep T-bit stable
        end else begin
          sda_d = 1'b1;
        end
      end

      AckHold: begin
        // SCL low after 9th bit, signal ready for next byte
        scl_d = 1'b0;
        sda_d = 1'b0;  // Drive low in preparation for next operation
        tx_ready_o = 1'b1;
        // Report ACK/NACK status
        if (tx_is_addr_q) begin
          rx_ack_o = ~ack_sampled;   // ACK = SDA low
          rx_nack_o = ack_sampled;   // NACK = SDA high
        end
      end

      SetupStop: begin
        // SCL low, ensure SDA is low
        scl_d = 1'b0;
        sda_d = 1'b0;
      end

      RiseStop: begin
        // SCL high, SDA still low (setup for STOP)
        scl_d = 1'b1;
        sda_d = 1'b0;
      end

      HoldStop: begin
        // Both lines released (STOP condition complete)
        scl_d = 1'b1;
        sda_d = 1'b1;
      end

      // ========================================================================
      // RX Data Bit States - controller releases SDA, target drives
      // ========================================================================
      ReadClockLow: begin
        scl_d = 1'b0;   // SCL low
        sda_d = 1'b1;   // Release SDA (target drives)
        sel_od_pp_o = 1'b1;  // Push-Pull for data phase
      end

      ReadClockPulse: begin
        scl_d = 1'b1;   // SCL high (sampling window)
        sda_d = 1'b1;   // Release SDA
        sel_od_pp_o = 1'b1;  // Push-Pull
      end

      ReadHoldBit: begin
        scl_d = 1'b0;   // SCL low
        sda_d = 1'b1;   // Release SDA
        sel_od_pp_o = 1'b1;  // Push-Pull
      end

      // ========================================================================
      // RX T-bit States - TARGET drives T-bit, controller samples (Open Drain)
      // T-bit=0: target done sending, T-bit=1: target has more data
      // ========================================================================
      ReadTbitLow: begin
        scl_d = 1'b0;   // SCL low
        sda_d = 1'b1;   // Release SDA (OD mode, target drives T-bit)
        sel_od_pp_o = 1'b0;  // Open Drain for T-bit
      end

      ReadTbitPulse: begin
        scl_d = 1'b1;   // SCL high, sample T-bit from target
        sda_d = 1'b1;   // Release SDA
        sel_od_pp_o = 1'b0;  // Open Drain
      end

      ReadTbitHold: begin
        scl_d = 1'b0;   // SCL low
        // If T-bit=0 (target done): drive SDA low to acknowledge end
        // If T-bit=1 (target has more): keep SDA released
        sda_d = rx_tbit_q ? 1'b1 : 1'b0;  // Drive low if target done (T-bit=0)
        sel_od_pp_o = 1'b0;  // Open Drain
        // Output RX data
        rx_valid_o = 1'b1;
        rx_data_o = read_byte;
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
    sample_ack = 1'b0;
    read_byte_clr = 1'b0;
    sample_tbit = 1'b0;

    unique case (state_q)
      Idle: begin
        if (host_enable_i && rx_req_i) begin
          // Enter RX (read) mode
          read_byte_clr = 1'b1;
          bit_clr = 1'b1;
          state_d = ReadClockLow;
          load_tcount = 1'b1;
          tcount_sel = tClockLow;
        end else if (host_enable_i && tx_valid_i) begin
          load_tx_data = 1'b1;
          bit_clr = 1'b1;
          unique case (tx_start_stop_i)
            Start: begin
              // START requested before byte
              state_d = SetupStart;
              load_tcount = 1'b1;
              tcount_sel = tSetupStart;
            end
            RepeatedStart: begin
              // Repeated Start requested before byte
              state_d = SetupSr;
              load_tcount = 1'b1;
              tcount_sel = tSetupStart;  // Use same timing as START setup
            end
            default: begin
              // No START/Sr (None or Stop), go directly to data transmission
              state_d = ClockLow;
              load_tcount = 1'b1;
              tcount_sel = tClockLow;
            end
          endcase
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
          tcount_sel = tClockLow;  // Use clock low timing
        end
      end

      ClockStart: begin
        if (tcount_q == 20'd1) begin
          // After START, go directly to first data bit
          state_d = ClockLow;
          load_tcount = 1'b1;
          tcount_sel = tClockLow;
          bit_decr = 1'b1;  // Decrement from 8 to 7 (MSB)
        end
      end

      SetupSr: begin
        if (tcount_q == 20'd1) begin
          state_d = HoldSr;
          load_tcount = 1'b1;
          tcount_sel = tSetupStart;  // Setup time before Sr
        end
      end

      HoldSr: begin
        if (tcount_q == 20'd1) begin
          state_d = FallSr;
          load_tcount = 1'b1;
          tcount_sel = tHoldStart;  // Hold time for Sr
        end
      end

      FallSr: begin
        if (tcount_q == 20'd1) begin
          state_d = ClockSr;
          load_tcount = 1'b1;
          tcount_sel = tClockLow;
        end
      end

      ClockSr: begin
        if (tcount_q == 20'd1) begin
          // After Sr, go to first data bit
          state_d = ClockLow;
          load_tcount = 1'b1;
          tcount_sel = tClockLow;
          bit_decr = 1'b1;  // Decrement from 8 to 7 (MSB)
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
        end
      end

      HoldBit: begin
        if (tcount_q == 20'd1) begin
          if (bit_index == 4'd1) begin
            // Last data bit done, go to 9th bit (ACK/T-bit)
            state_d = AckLow;
            load_tcount = 1'b1;
            tcount_sel = tClockLow;
          end else begin
            // More data bits to transfer
            state_d = ClockLow;
            load_tcount = 1'b1;
            tcount_sel = tClockLow;
            bit_decr = 1'b1;
          end
        end
      end

      AckLow: begin
        // Sample ACK on entry to AckLow (target drives SDA after SCL falls)
        if (tx_is_addr_q) begin
          sample_ack = 1'b1;
        end
        if (tcount_q == 20'd1) begin
          state_d = AckPulse;
          load_tcount = 1'b1;
          tcount_sel = tClockPulse;
        end
      end

      AckPulse: begin
        if (tcount_q == 20'd1) begin
          state_d = AckHold;
          load_tcount = 1'b1;
          tcount_sel = tHoldBit;
        end
      end

      AckHold: begin
        if (tcount_q == 20'd1) begin
          if (tx_start_stop_q == Stop) begin
            // STOP requested after this byte
            state_d = SetupStop;
            load_tcount = 1'b1;
            tcount_sel = tClockLow;  // Hold SCL low briefly
          end else begin
            // No STOP, return to Idle to wait for next byte
            state_d = Idle;
            load_tcount = 1'b1;
            tcount_sel = tNoDelay;
          end
        end
      end

      SetupStop: begin
        if (tcount_q == 20'd1) begin
          state_d = RiseStop;
          load_tcount = 1'b1;
          tcount_sel = tSetupStop;
        end
      end

      RiseStop: begin
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

      // ========================================================================
      // RX Data Bit Transitions
      // ========================================================================
      ReadClockLow: begin
        if (tcount_q == 20'd1) begin
          state_d = ReadClockPulse;
          load_tcount = 1'b1;
          tcount_sel = tClockPulse;
        end
      end

      ReadClockPulse: begin
        if (tcount_q == 20'd1) begin
          state_d = ReadHoldBit;
          load_tcount = 1'b1;
          tcount_sel = tHoldBit;
          shift_data_en = 1'b1;  // Sample SDA on last cycle
        end
      end

      ReadHoldBit: begin
        if (tcount_q == 20'd1) begin
          if (bit_index == 4'd1) begin
            // Last data bit done, go to 9th bit (sample T-bit from target)
            state_d = ReadTbitLow;
            load_tcount = 1'b1;
            tcount_sel = tClockLow;
          end else begin
            // More bits to read
            state_d = ReadClockLow;
            load_tcount = 1'b1;
            tcount_sel = tClockLow;
            bit_decr = 1'b1;
          end
        end
      end

      // ========================================================================
      // RX T-bit Transitions - sample T-bit from TARGET
      // ========================================================================
      ReadTbitLow: begin
        if (tcount_q == 20'd1) begin
          state_d = ReadTbitPulse;
          load_tcount = 1'b1;
          tcount_sel = tClockPulse;
        end
      end

      ReadTbitPulse: begin
        if (tcount_q == 20'd1) begin
          state_d = ReadTbitHold;
          load_tcount = 1'b1;
          tcount_sel = tHoldBit;
          sample_tbit = 1'b1;  // Sample T-bit from target on last cycle
        end
      end

      ReadTbitHold: begin
        if (tcount_q == 20'd1) begin
          if (!rx_tbit_q) begin
            // T-bit=0: Target is done, issue STOP
            state_d = SetupStop;
            load_tcount = 1'b1;
            tcount_sel = tClockLow;
          end else begin
            // T-bit=1: Target has more data, return to Idle for next byte
            // TODO: implement controller read abort behaviour
            state_d = Idle;
            load_tcount = 1'b1;
            tcount_sel = tNoDelay;
          end
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

  // ============================================================================
  // Output Assignments
  // ============================================================================
  assign ctrl_scl_o = scl_d;
  assign ctrl_sda_o = sda_d;

endmodule
