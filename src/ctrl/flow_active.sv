// SPDX-License-Identifier: Apache-2.0

// TODO: Add support for data byte ordering modes (HC_CONTROL.DATA_BYTE_ORDER_MODE)

module flow_active
  import controller_pkg::*;
  import i3c_pkg::*;
#(
    parameter int unsigned HciRespDataWidth = 32,
    parameter int unsigned HciCmdDataWidth  = 64,
    parameter int unsigned HciRxDataWidth   = 32,
    parameter int unsigned HciTxDataWidth   = 32,
    parameter int unsigned HciIbiDataWidth  = 32,

    parameter int unsigned HciRespThldWidth = 8,
    parameter int unsigned HciCmdThldWidth  = 8,
    parameter int unsigned HciRxThldWidth   = 3,
    parameter int unsigned HciTxThldWidth   = 3,
    parameter int unsigned HciIbiThldWidth  = 8
) (
    input logic clk_i,
    input logic rst_ni,

    // HCI queues
    // Command FIFO
    input logic cmd_queue_full_i,
    input logic [HciCmdThldWidth-1:0] cmd_queue_ready_thld_i,
    input logic cmd_queue_ready_thld_trig_i,
    input logic cmd_queue_empty_i,
    input logic cmd_queue_rvalid_i,
    output logic cmd_queue_rready_o,
    input logic [HciCmdDataWidth-1:0] cmd_queue_rdata_i,
    // RX FIFO
    input logic rx_queue_full_i,
    input logic [HciRxThldWidth-1:0] rx_queue_start_thld_i,
    input logic rx_queue_start_thld_trig_i,
    input logic [HciRxThldWidth-1:0] rx_queue_ready_thld_i,
    input logic rx_queue_ready_thld_trig_i,
    input logic rx_queue_empty_i,
    output logic rx_queue_wvalid_o,
    input logic rx_queue_wready_i,
    output logic [HciRxDataWidth-1:0] rx_queue_wdata_o,
    // TX FIFO
    input logic tx_queue_full_i,
    input logic [HciTxThldWidth-1:0] tx_queue_start_thld_i,
    input logic tx_queue_start_thld_trig_i,
    input logic [HciTxThldWidth-1:0] tx_queue_ready_thld_i,
    input logic tx_queue_ready_thld_trig_i,
    input logic tx_queue_empty_i,
    input logic tx_queue_rvalid_i,
    output logic tx_queue_rready_o,
    input logic [HciTxDataWidth-1:0] tx_queue_rdata_i,
    // Response FIFO
    input logic resp_queue_full_i,
    input logic [HciRespThldWidth-1:0] resp_queue_ready_thld_i,
    input logic resp_queue_ready_thld_trig_i,
    input logic resp_queue_empty_i,
    output logic resp_queue_wvalid_o,
    input logic resp_queue_wready_i,
    output logic [HciRespDataWidth-1:0] resp_queue_wdata_o,

    // In-band Interrupt queue
    input logic ibi_queue_full_i,
    input logic [HciIbiThldWidth-1:0] ibi_queue_ready_thld_i,
    input logic ibi_queue_ready_thld_trig_i,
    input logic ibi_queue_empty_i,
    output logic ibi_queue_wvalid_o,
    input logic ibi_queue_wready_i,
    output logic [HciIbiDataWidth-1:0] ibi_queue_wdata_o,

    // DAT <-> Controller interface
    output logic                          dat_read_valid_hw_o,
    output logic [$clog2(`DAT_DEPTH)-1:0] dat_index_hw_o,
    input  logic [                  63:0] dat_rdata_hw_i,

    // DCT <-> Controller interface
    output logic                          dct_write_valid_hw_o,
    output logic                          dct_read_valid_hw_o,
    output logic [$clog2(`DCT_DEPTH)-1:0] dct_index_hw_o,
    output logic [                 127:0] dct_wdata_hw_o,
    input  logic [                 127:0] dct_rdata_hw_i,

    // I2C Controller interface
    output logic host_enable_o,  // enable host functionality

    output logic fmt_fifo_rvalid_o,
    output logic [I2CFifoDepthWidth-1:0] fmt_fifo_depth_o,
    input logic fmt_fifo_rready_i,
    output logic [7:0] fmt_byte_o,
    output logic fmt_flag_start_before_o,
    output logic fmt_flag_stop_after_o,
    output logic fmt_flag_read_bytes_o,
    output logic fmt_flag_read_continue_o,
    output logic fmt_flag_nak_ok_o,
    output logic unhandled_unexp_nak_o,
    output logic unhandled_nak_timeout_o,

    // RX FIFO queue from I2C Controller
    input logic                   rx_fifo_wvalid_i,
    input logic [RxFifoWidth-1:0] rx_fifo_wdata_i,

    // I3C Controller interface
    output logic       i3c_tx_valid_o,       // Byte/bit ready to send
    // change this naming from 'done' to ready signal
    input  logic       i3c_tx_ready_i,        // Byte/bit transfer ready
    output logic [7:0] i3c_tx_byte_o,        // Data byte to send
    output start_stop_e i3c_start_stop_o,   // Start/Stop/Repeated Start indication for current byte
    output logic       i3c_tx_is_addr_o,     // This is address byte (open-drain, expects ACK)
    output logic       i3c_tx_use_tbit_o,    // Add T-bit after byte (push-pull I3C mode)
    input  logic       i3c_rx_ack_i,         // ACK received (valid when i3c_tx_ready_i && i3c_tx_is_addr_o)
    input  logic       i3c_rx_nack_i,        // NACK received

    // I3C RX interface - bytes received from i3c_controller_fsm
    input  logic       i3c_rx_valid_i,       // Received byte valid from controller FSM
    input  logic [7:0] i3c_rx_byte_i,        // Received byte data

    // I3C FSM control & status
    input  logic i3c_fsm_en_i,
    output logic i3c_fsm_idle_o,

    // I3C transfer mode (from command descriptor)
    output i3c_trans_mode_e i3c_trans_mode_o,

    // Errors and Interrupts
    output i3c_err_t err,
    output i3c_irq_t irq
);

  assign dct_write_valid_hw_o = '0;
  // rx_queue_wvalid_o is now controlled by the I3CDataRead/PushRxData state logic
  assign ibi_queue_wvalid_o = '0;
  assign err = '0;
  assign irq = '0;

  // TODO: do we need separate i2c fsm states?
  typedef enum logic [5:0] {
    Idle = 6'd0,
    WaitForCmd = 6'd1,
    FetchDAT = 6'd2,
    I2CWriteImmediate = 6'd3,
    // I3CWriteImmediate removed - uses BroadcastAddr → TargetAddr → I3CDataWrite
    FetchTxData = 6'd5,
    PushRxData = 6'd6,
    InitI2CWrite = 6'd7,
    InitI2CRead = 6'd8,
    StallWrite = 6'd9,
    StallRead = 6'd10,
    // IssueCmd removed - merged into unified FSM
    WriteResp = 6'd12,
    I3CAddressAssignment = 6'd13,
    I3CDataWrite = 6'd14,       // DATA PHASE ONLY (also used by ImmediateDataTransfer)
    I3CDataRead = 6'd15,        // DATA PHASE ONLY
    // Shared Address States (used by CCC, private, and immediate transfers)
    BroadcastAddr = 6'd16,      // S/Sr + 0x7E/W
    TargetAddr = 6'd17,         // Sr + Target Addr + RnW
    // CCC-specific States
    CCC_SendCCCCode = 6'd18,    // CCC code + T-bit
    CCC_DefiningByte = 6'd19    // Optional defining byte + T-bit
  } flow_fsm_state_e;


  // CCC FSM merged into main flow_fsm_state_e - no separate enum needed

  // BytesBeforeImmData only used by I2CWriteImmediate (legacy I2C path)
  // TODO: Set from HC_CONTROL.IBA_INCLUDE: 1 if IBA is disabled, otherwise 2
  localparam int unsigned BytesBeforeImmData = 1;

  flow_fsm_state_e state, state_next;

  immediate_data_trans_desc_t immediate_cmd_desc;
  regular_trans_desc_t regular_cmd_desc;
  combo_trans_desc_t combo_cmd_desc;
  addr_assign_desc_t addr_cmd_desc;
  logic [63:0] cmd_desc;

  // Values extracted from the Command Descriptor
  cmd_transfer_dir_e cmd_dir;
  i3c_cmd_attr_e cmd_attr;
  logic [4:0] dev_index;
  logic [3:0] cmd_tid;
  logic [15:0] data_length;
  logic imm_use_def_byte;

  // CCC-specific signals
  logic cmd_present;  // Command Present bit - indicates CCC
  logic [7:0] ccc_code;  // CCC code from descriptor
  logic is_direct_ccc;  // 1 = Direct CCC, 0 = Broadcast CCC

  // Generic incremental counter
  logic [31:0] transfer_cnt;
  logic transfer_cnt_en;
  logic transfer_cnt_rst;

  logic [HciCmdDataWidth-1:0] cmd_queue_rdata;
  logic cmd_queue_rvalid;

  // DAT table
  dat_entry_t dat_rdata;
  logic dat_captured, dat_read_valid_d;

  // DCT table
  // TODO: Use DCT typedef struct
  logic [127:0] dct_rdata;
  logic dct_captured, dct_read_valid_d;

  // Values extracted from the DAT entry
  logic i2c_cmd;

  // TX Queue
  logic [HciTxDataWidth-1:0] tx_dword;
  logic pop_tx_fifo;
  logic [7:0] tx_data_byte;  // Selected byte from tx_dword based on transfer_cnt and byte index within word

  // RX Queue - byte accumulator (stitch 8-bit bytes into 32-bit words)
  logic [1:0] rx_byte_cnt_q, rx_byte_cnt_d;      // Which byte position (0-3) in the 32-bit word
  logic [31:0] rx_word_accum_q, rx_word_accum_d; // Accumulate 4 bytes into 32-bit word
  logic rx_data_phase_active;                     // Set by states that are receiving data

  // Response Queue
  i3c_response_desc_t resp_desc;
  i3c_resp_err_status_e resp_err_status_q, resp_err_status_d;
  logic [15:0] resp_data_length_q, resp_data_length_d;

  // Bus active state - tracks if bus is currently owned (no STOP issued)
  // Used to determine if next transaction starts with Start or RepeatedStart
  logic bus_active_q, bus_active_d;

  // TODO: Set appropriately
  always_comb begin
    resp_err_status_q = Success;
    fmt_flag_read_bytes_o = 1'b0;
    fmt_flag_read_continue_o = 1'b0;
    fmt_flag_nak_ok_o = 1'b0;
    unhandled_unexp_nak_o = 1'b0;
    unhandled_nak_timeout_o = 1'b0;
  end

  // Assign generic Command Descriptor to command specific structures
  assign immediate_cmd_desc = cmd_desc;
  assign regular_cmd_desc = cmd_desc;
  assign combo_cmd_desc = cmd_desc;
  assign addr_cmd_desc = cmd_desc;

  // Initialize descriptor's reserved field
  assign resp_desc.__rsvd23_16 = '0;

  // Assign generic command fields to generic signals
  assign dev_index = cmd_desc[20:16];
  assign cmd_tid = cmd_desc[6:3];
  assign cmd_dir = cmd_desc[29] ? Read : Write;
  assign cmd_attr = i3c_cmd_attr_e'(cmd_desc[2:0]);

  // Assign DAT entry specific signals
  assign i2c_cmd = dat_rdata.device;

  // Assign CCC-specific signals from command descriptor
  // cp and cmd are at the same bit positions in Immediate, Regular, and Combo descriptors
  assign cmd_present = cmd_desc[15];      // CP bit - Command Present
  assign ccc_code = cmd_desc[14:7];       // CCC / HDR command code
  assign is_direct_ccc = ccc_code[7];     // MSB: 1 = Direct CCC, 0 = Broadcast CCC

  // Transfer mode is at bits 28:26 in all descriptor types (Immediate, Regular, Combo)
  assign i3c_trans_mode_o = i3c_trans_mode_e'(cmd_desc[28:26]);

  // Assign constants
  // TODO: Add control logic to constant signals
  assign host_enable_o = 1'b1;
  assign fmt_fifo_depth_o = 8'd1;

  // Capture data from DAT/DCT tables
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (~rst_ni) begin
      dat_read_valid_d <= 1'b0;
      dct_read_valid_d <= 1'b0;
      dat_rdata <= '0;
      dct_rdata <= '0;
      dat_captured <= 1'b0;
      dct_captured <= 1'b0;
    end else begin
      dat_read_valid_d <= dat_read_valid_hw_o;
      dct_read_valid_d <= dct_read_valid_hw_o;
      if (dat_read_valid_d) begin
        dat_rdata <= dat_rdata_hw_i;
        dat_captured <= 1'b1;
      end else begin
        dat_rdata <= dat_rdata;
        dat_captured <= 1'b0;
      end
      if (dct_read_valid_d) begin
        dct_rdata <= dct_rdata_hw_i;
        dct_captured <= 1'b1;
      end else begin
        dct_rdata <= dct_rdata;
        dct_captured <= 1'b0;
      end
    end
  end

  // Capture command FIFO control signals
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (~rst_ni) begin
      cmd_queue_rvalid <= '0;
      cmd_queue_rdata  <= '0;
    end else begin
      cmd_queue_rvalid <= cmd_queue_rvalid_i;
      cmd_queue_rdata  <= cmd_queue_rdata_i;
    end
  end

  always_comb begin
    // Counter now starts at 0 for data phase (address phases in separate states)
    unique case (transfer_cnt & 32'd3)
      32'd0: tx_data_byte = (cmd_attr == ImmediateDataTransfer) ? immediate_cmd_desc.def_or_data_byte1 : tx_dword[7:0];
      32'd1: tx_data_byte = (cmd_attr == ImmediateDataTransfer) ? immediate_cmd_desc.data_byte2        : tx_dword[15:8];
      32'd2: tx_data_byte = (cmd_attr == ImmediateDataTransfer) ? immediate_cmd_desc.data_byte3        : tx_dword[23:16];
      32'd3: tx_data_byte = (cmd_attr == ImmediateDataTransfer) ? immediate_cmd_desc.data_byte4        : tx_dword[31:24];
    endcase
  end
  

  // Assign internals based on the command attribute
  always_comb begin
    unique case (cmd_attr)
      ImmediateDataTransfer: begin
        // If DTT is 5-7, it is a CCC with a defining byte. In such case substract 5 from DTT to
        // get an actual transfer data length.
        // Values:
        // - 0: No payload
        // - 1–4: N bytes are valid
        // - 5: Defining Byte + 0
        // - 6: Defining Byte + 1
        // - 7: Defining Byte + 2
        imm_use_def_byte = immediate_cmd_desc.dtt > 4 ? 1'b1 : 1'b0;
        data_length = imm_use_def_byte ? 16'(immediate_cmd_desc.dtt - 5) : 16'(immediate_cmd_desc.dtt);
      end
      AddressAssignment: begin
        // TODO
        imm_use_def_byte = '0;
        data_length = '0;
      end
      ComboTransfer: begin
        imm_use_def_byte = '0;
        data_length = combo_cmd_desc.data_length;
      end
      InternalControl: begin
        // TODO
        imm_use_def_byte = '0;
        data_length = '0;
      end
      RegularTransfer: begin
        imm_use_def_byte = regular_cmd_desc.dbp;
        data_length = regular_cmd_desc.data_length;
      end
      default: begin
        imm_use_def_byte = '0;
        data_length = '0;
      end
    endcase
  end

  // Control internal transfer counter
  // TODO: Consider using decremental counter with different load values
  // See i2c_controller_fsm.sv for reference
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (~rst_ni) begin
      transfer_cnt <= '0;
    end else begin
      if (transfer_cnt_rst) begin
        transfer_cnt <= '0;  // Always reset to 0 (transfer_cnt_rst_val eliminated)
      end else if (transfer_cnt_en) begin
        transfer_cnt <= transfer_cnt + 1;
      end else begin
        transfer_cnt <= transfer_cnt;
      end
    end
  end

  // Fetch Command Descriptor from the Command Queue
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (~rst_ni) begin
      cmd_desc <= '0;
    end else begin
      if (cmd_queue_rvalid_i & cmd_queue_rready_o) begin
        cmd_desc <= cmd_queue_rdata_i;
      end else begin
        cmd_desc <= cmd_desc;
      end
    end
  end

  // Capture data from TX Queue
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (~rst_ni) begin
      tx_dword <= '0;
    end else begin
      if (pop_tx_fifo) begin
        tx_dword <= tx_queue_rdata_i;
      end else begin
        tx_dword <= tx_dword;
      end
    end
  end

  // Catch every error detected during the Controller operation
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (~rst_ni) begin
      resp_err_status_d <= Success;
    end else begin
      // TODO: Add proper error catching
      if (i3c_fsm_idle_o) begin
        resp_err_status_d <= Success;
      end else begin
        resp_err_status_d <= resp_err_status_d;
      end
    end
  end

  // Catch every data_length update during the Controller operation
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (~rst_ni) begin
      resp_data_length_d <= '0;
    end else begin
      if (i3c_fsm_idle_o) begin
        resp_data_length_d <= '0;
      end else if (|resp_data_length_q) begin
        resp_data_length_d <= resp_data_length_q;
      end else begin
        resp_data_length_d <= resp_data_length_d;
      end
    end
  end

  // Track bus active state (no STOP issued means bus is still owned)
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (~rst_ni) begin
      bus_active_q <= 1'b0;
    end else begin
      bus_active_q <= bus_active_d;
    end
  end

  // RX byte accumulator - stitch 8-bit bytes into 32-bit words
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (~rst_ni) begin
      rx_byte_cnt_q <= 2'd0;
      rx_word_accum_q <= 32'd0;
    end else begin
      rx_byte_cnt_q <= rx_byte_cnt_d;
      rx_word_accum_q <= rx_word_accum_d;
    end
  end

  // Combinational state output update
  always_comb begin
    i3c_fsm_idle_o = 1'b0;
    transfer_cnt_en = 1'b0;
    cmd_queue_rready_o = 1'b0;
    dat_read_valid_hw_o = 1'b0;
    dct_read_valid_hw_o = 1'b0;
    dat_index_hw_o = '0;
    tx_queue_rready_o = 1'b0;
    pop_tx_fifo = 1'b0;
    transfer_cnt_rst = 1'b1;
    fmt_fifo_rvalid_o = 1'b0;
    fmt_flag_start_before_o = 1'b0;
    resp_queue_wvalid_o = 1'b0;
    fmt_flag_stop_after_o = 1'b0;
    fmt_byte_o = '0;
    dct_wdata_hw_o = '0;
    ibi_queue_wdata_o = '0;
    dct_index_hw_o = '0;
    resp_data_length_q = '0;
    resp_queue_wdata_o = '0;
    resp_desc.err_status = i3c_resp_err_status_e'(0);
    resp_desc.tid = '0;
    resp_desc.data_length = '0;
    i3c_tx_valid_o = 1'b0;
    i3c_tx_byte_o = '0;
    i3c_tx_is_addr_o = 1'b0;
    i3c_tx_use_tbit_o = 1'b0;
    i3c_start_stop_o = None;
    bus_active_d = bus_active_q;  // Default: maintain current state
    // RX accumulator defaults
    rx_data_phase_active = 1'b0;
    rx_byte_cnt_d = rx_byte_cnt_q;
    rx_word_accum_d = rx_word_accum_q;
    rx_queue_wvalid_o = 1'b0;
    rx_queue_wdata_o = '0;
    unique case (state)
      // Idle: Wait for command appearance in the Command Queue
      Idle: begin
        i3c_fsm_idle_o = 1'b1;
      end
      // WaitForCmd: Fetch Command Descriptor
      WaitForCmd: begin
        cmd_queue_rready_o = 1'b1;
      end
      // FetchDAT: Fetch DAT entry
      FetchDAT: begin
        // TODO: Optimize DAT read so it takes just 1 cycle
        dat_read_valid_hw_o = 1'b1;
        dat_index_hw_o = $clog2(`DAT_DEPTH)'(dev_index);
      end
      // I2CWriteImmediate: Execute Immediate Transfer to Legacy I2C Device via I2C Controller
      I2CWriteImmediate: begin
        // TODO: Figure out if the transfer should proceed if its DTT is set to `Defining Byte + 0`
        // since in such scenario it sends only a target device address. It might be better to just
        // report an error.
        transfer_cnt_rst = 1'b0;
        transfer_cnt_en = fmt_fifo_rready_i;
        fmt_fifo_rvalid_o = 1'b1;
        fmt_flag_start_before_o = 1'b0;
        fmt_flag_stop_after_o = 1'b0;
        resp_data_length_q = '0;
        unique case (transfer_cnt)
          // TODO: Add support for broadcast address control before private transfers. This can
          // be realized via HC_CONTROL.I2C_DEV_PRESENT and HC_CONTROL.IBA_INCLUDE register fields.
          // 32'd0: fmt_byte_o = {7'h7e, 1'b0};
          // Target address
          32'd0: fmt_byte_o = {dat_rdata.static_address, 1'b0};
          // Byte 1
          32'd1:
          fmt_byte_o = imm_use_def_byte ? immediate_cmd_desc.data_byte2
                                               : immediate_cmd_desc.def_or_data_byte1;
          // Byte 2
          32'd2:
          fmt_byte_o = imm_use_def_byte ? immediate_cmd_desc.data_byte3
                                               : immediate_cmd_desc.data_byte2;
          // Byte 3
          32'd3: fmt_byte_o = immediate_cmd_desc.data_byte3;
          // Byte 4
          32'd4: fmt_byte_o = immediate_cmd_desc.data_byte4;
          default: fmt_byte_o = '0;
        endcase

        // Send start condition before first byte
        if (transfer_cnt == 0) begin
          fmt_flag_start_before_o = 1'b1;
        end
        // Send stop condition after last byte if TOC is set to STOP
        if (transfer_cnt == data_length + (BytesBeforeImmData - 1)) begin
          fmt_flag_stop_after_o = immediate_cmd_desc.toc;
        end
        // Disable FIFO valid whenever I2C Controller is not ready or an immediate transfer is finished
        if (fmt_fifo_rready_i | (transfer_cnt == data_length + BytesBeforeImmData)) begin
          fmt_fifo_rvalid_o = 1'b0;
        end
      end
      // I3CWriteImmediate removed - now uses BroadcastAddr → TargetAddr → I3CDataWrite
      FetchTxData: begin
        transfer_cnt_rst = 1'b0;
        tx_queue_rready_o = !tx_queue_empty_i;
        pop_tx_fifo = tx_queue_rvalid_i && tx_queue_rready_o;
      end
      PushRxData: begin
        transfer_cnt_rst = 1'b0;
        // Push accumulated word when queue has space
        if (~rx_queue_full_i) begin
          rx_queue_wvalid_o = 1'b1;
          // Handle partial word (flush) vs full word
          if (rx_byte_cnt_q == 2'd0) begin
            // Full 4-byte word (counter wrapped to 0 after accumulating 4th byte)
            rx_queue_wdata_o = rx_word_accum_q;
          end else begin
            // Partial word - pad with zeros
            unique case (rx_byte_cnt_q)
              2'd1: rx_queue_wdata_o = {24'd0, rx_word_accum_q[7:0]};
              2'd2: rx_queue_wdata_o = {16'd0, rx_word_accum_q[15:0]};
              2'd3: rx_queue_wdata_o = {8'd0, rx_word_accum_q[23:0]};
              default: rx_queue_wdata_o = rx_word_accum_q;
            endcase
          end
          // Reset accumulator for next word
          rx_byte_cnt_d = 2'd0;
          rx_word_accum_d = 32'd0;
        end
      end
      InitI2CWrite: begin
        // TODO
      end
      InitI2CRead: begin
        // TODO
      end
      StallWrite: begin
        // TODO
      end
      StallRead: begin
        // TODO
      end
      // BroadcastAddr: Send broadcast address 0x7E/W with START or Sr
      // Shared by both CCC and private transfers
      BroadcastAddr: begin
        i3c_tx_byte_o = {`I3C_RSVD_ADDR, 1'b0};
        i3c_tx_valid_o = 1'b1;
        i3c_start_stop_o = bus_active_q ? RepeatedStart : Start;
        bus_active_d = 1'b1;  // Bus is now active
        i3c_tx_is_addr_o = 1'b1;  // Open-drain, expects ACK
      end

      // TargetAddr: Send target address + RnW with Repeated Start
      // Shared by both CCC direct and private transfers
      TargetAddr: begin
        i3c_tx_byte_o = {dat_rdata.dynamic_address[7:1], (cmd_dir == Read) ? 1'b1 : 1'b0};
        i3c_tx_valid_o = 1'b1;
        i3c_start_stop_o = RepeatedStart;
        i3c_tx_is_addr_o = 1'b1;  // Open-drain, expects ACK
      end

      // CCC_SendCCCCode: Send CCC code byte with T-bit
      CCC_SendCCCCode: begin
        i3c_tx_byte_o = ccc_code;
        i3c_tx_valid_o = 1'b1;
        i3c_tx_use_tbit_o = 1'b1;
      end

      // CCC_DefiningByte: Send optional defining byte with T-bit
      CCC_DefiningByte: begin
        i3c_tx_byte_o = (cmd_attr == ImmediateDataTransfer) ?
                        immediate_cmd_desc.def_or_data_byte1 :
                        regular_cmd_desc.dbp;
        i3c_tx_valid_o = 1'b1;
        i3c_tx_use_tbit_o = 1'b1;
      end

      // I3CAddressAssignment: Address Assignment CCC (SETDASA, ENTDAA)
      // Broadcast addr and CCC code handled by BroadcastAddr/CCC_SendCCCCode
      // This state handles: Sr → StaticAddr/W → DynAddr → P
      I3CAddressAssignment: begin
        transfer_cnt_rst = 1'b0;
        transfer_cnt_en = i3c_tx_ready_i;
        i3c_tx_valid_o = 1'b1;

        unique case (transfer_cnt)
          32'd0: begin
            // Send Repeated Start + Static Address + W
            i3c_tx_byte_o = {dat_rdata.static_address, 1'b0};
            i3c_start_stop_o = RepeatedStart;
            i3c_tx_is_addr_o = 1'b1;  // Open-drain, expects ACK
          end
          32'd1: begin
            // Send Dynamic Address + T-bit, then STOP
            // TODO: Should send actual dynamic address from DAT, not static
            i3c_tx_byte_o = dat_rdata.dynamic_address;
            i3c_tx_use_tbit_o = 1'b1;
            i3c_start_stop_o = Stop;
            bus_active_d = 1'b0;  // Bus released
          end
          default: begin
            // Should not reach here
            i3c_tx_byte_o = '0;
          end
        endcase
      end
      // I3CDataWrite: Data phase only - send bytes from TX queue or descriptor
      // Address phases handled by BroadcastAddr and TargetAddr states
      I3CDataWrite: begin
        transfer_cnt_rst = 1'b0;
        transfer_cnt_en = i3c_tx_ready_i;
        i3c_tx_valid_o = 1'b1;
        i3c_tx_byte_o = tx_data_byte;  // Counter starts at 0 for data phase
        i3c_tx_use_tbit_o = 1'b1;  // Push-pull mode with T-bit

        // STOP after last data byte if TOC is set
        // Use appropriate toc field based on command attribute
        if (transfer_cnt == data_length - 32'd1) begin
          if ((cmd_attr == ImmediateDataTransfer) ? immediate_cmd_desc.toc : regular_cmd_desc.toc) begin
            i3c_start_stop_o = Stop;
            bus_active_d = 1'b0;  // Bus released
          end
        end
      end
      // I3CDataRead: Data phase only - receive bytes from target
      // Address phases handled by BroadcastAddr and TargetAddr states
      I3CDataRead: begin
        transfer_cnt_rst = 1'b0;
        transfer_cnt_en = i3c_rx_valid_i;
        rx_data_phase_active = 1'b1;  // Enable shared RX accumulation

        // Track received data length for response (counter starts at 0)
        resp_data_length_q = 16'(transfer_cnt + 1);

        // Issue STOP on last data byte if TOC bit is set
        // Use appropriate toc field based on command attribute
        if (transfer_cnt == data_length - 32'd1) begin
          if ((cmd_attr == ImmediateDataTransfer) ? immediate_cmd_desc.toc : regular_cmd_desc.toc) begin
            i3c_start_stop_o = Stop;
            bus_active_d = 1'b0;  // Bus released
          end
        end
      end
      // WriteResp: Generate Response Descriptor and load it to Response Queue
      WriteResp: begin
        resp_queue_wvalid_o = 1'b0;
        resp_desc.err_status = resp_err_status_d;
        resp_desc.tid = cmd_tid;
        resp_desc.data_length = resp_data_length_d;

        if (resp_queue_wready_i) begin
          resp_queue_wvalid_o = 1'b1;
          resp_queue_wdata_o  = resp_desc;
        end
      end
      default: begin
        resp_desc.err_status = i3c_resp_err_status_e'(0);
        resp_desc.tid = '0;
        resp_desc.data_length = '0;
      end
    endcase

    // Shared RX byte accumulation logic - ONLY accumulates, PushRxData handles pushing
    if (rx_data_phase_active && i3c_rx_valid_i) begin
      // Shift byte into accumulator based on byte position
      unique case (rx_byte_cnt_q)
        2'd0: rx_word_accum_d[7:0]   = i3c_rx_byte_i;
        2'd1: rx_word_accum_d[15:8]  = i3c_rx_byte_i;
        2'd2: rx_word_accum_d[23:16] = i3c_rx_byte_i;
        2'd3: rx_word_accum_d[31:24] = i3c_rx_byte_i;
      endcase
      // Increment byte counter (wraps 0->1->2->3->0)
      rx_byte_cnt_d = rx_byte_cnt_q + 2'd1;
    end
  end

  // Combinational state transition
  always_comb begin
    state_next = state;
    unique case (state)
      // Idle: Wait for command appearance in the Command Queue
      Idle: begin
        if (i3c_fsm_en_i) begin
          state_next = WaitForCmd;
        end
      end
      // WaitForCmd: Fetch Command Descriptor
      WaitForCmd: begin
        if (~cmd_queue_empty_i & cmd_queue_rvalid_i) begin
          state_next = FetchDAT;
        end
      end
      // FetchDAT: Fetch DAT entry
      // All I3C commands go through BroadcastAddr (CCC, private, immediate, address assignment)
      FetchDAT: begin
        if (dat_captured) begin
          state_next = BroadcastAddr;
        end
      end
      // I2CWriteImmediate: Execute Immediate Transfer to Legacy I2C Device via I2C Controller
      I2CWriteImmediate: begin
        if (transfer_cnt == data_length + BytesBeforeImmData) begin
          // TODO: Do not generate Response Descriptor if WROC field of the Command Descriptor is set to 0
          state_next = WriteResp;
        end
      end
      // I3CWriteImmediate removed - now uses BroadcastAddr → TargetAddr → I3CDataWrite
      FetchTxData: begin
        if (pop_tx_fifo) begin
          if (~i2c_cmd) begin
            // Same for both CCC and private transfers - address phases already handled
            state_next = I3CDataWrite;
          end else begin
            state_next = InitI2CWrite;
          end
        end
      end
      PushRxData: begin
        // Wait for RX queue to have space, then decide next state
        if (~rx_queue_full_i) begin
          // Push succeeded, decide next state based on transfer completion
          // Counter now starts at 0 for data phase
          if (transfer_cnt >= data_length) begin
            // Transfer complete
            state_next = WriteResp;
          end else begin
            // More data to receive
            if (~i2c_cmd) begin
              // Same for both CCC and private - address phases already handled
              state_next = I3CDataRead;
            end else begin
              state_next = InitI2CRead;
            end
          end
        end
        // else: stay in PushRxData until queue has space (backpressure)
      end
      InitI2CWrite: begin
        // TODO
      end
      InitI2CRead: begin
        // TODO
      end
      StallWrite: begin
        // TODO
      end
      StallRead: begin
        // TODO
      end

      // BroadcastAddr: Send 0x7E/W, then branch based on cmd_present
      BroadcastAddr: begin
        if (i3c_tx_ready_i) begin
          if (cmd_present) begin
            state_next = CCC_SendCCCCode;  // CCC path
          end else begin
            state_next = TargetAddr;        // Private transfer path
          end
        end
      end

      // TargetAddr: Send target address, then go to data phase
      TargetAddr: begin
        if (i3c_tx_ready_i) begin
          if (cmd_dir == Write) begin
            // ImmediateDataTransfer has data in descriptor, skip FetchTxData
            if (cmd_attr == ImmediateDataTransfer) begin
              // Handle no-data case (data_length == 0)
              state_next = (data_length == 0) ? WriteResp : I3CDataWrite;
            end else begin
              state_next = FetchTxData;
            end
          end else begin
            state_next = I3CDataRead;  // Read starts immediately after address
          end
        end
      end

      // CCC_SendCCCCode: After CCC code, determine next step
      CCC_SendCCCCode: begin
        if (i3c_tx_ready_i) begin
          if (cmd_attr == AddressAssignment) begin
            // Address assignment handles static/dynamic addr internally
            state_next = I3CAddressAssignment;
          end else if (imm_use_def_byte) begin
            state_next = CCC_DefiningByte;
          end else if (is_direct_ccc) begin
            state_next = TargetAddr;  // Direct CCC needs target address
          end else if (data_length == 0) begin
            state_next = WriteResp;   // No data, done
          end else begin
            // Broadcast CCC with data
            if (cmd_dir == Read) begin
              state_next = I3CDataRead;
            end else begin
              state_next = FetchTxData;
            end
          end
        end
      end

      // CCC_DefiningByte: After defining byte, determine next step
      CCC_DefiningByte: begin
        if (i3c_tx_ready_i) begin
          if (is_direct_ccc) begin
            state_next = TargetAddr;
          end else if (data_length == 0) begin
            state_next = WriteResp;
          end else begin
            // Broadcast CCC with data
            if (cmd_dir == Read) begin
              state_next = I3CDataRead;
            end else begin
              state_next = FetchTxData;
            end
          end
        end
      end

      // I3CAddressAssignment: Address Assignment CCC (static addr + dynamic addr)
      I3CAddressAssignment: begin
        // Complete after sending both static addr and dynamic addr
        // TODO: Support multi-target using addr_cmd_desc.dev_count
        if (i3c_tx_ready_i && transfer_cnt == 32'd1) begin
          state_next = WriteResp;
        end
      end
      // I3CDataWrite: Data phase only (counter starts at 0)
      I3CDataWrite: begin
        // Done when all data_length bytes sent
        if (i3c_tx_ready_i && transfer_cnt == data_length - 32'd1) begin
          // For ImmediateDataTransfer, check wroc to decide if we write response
          if (cmd_attr == ImmediateDataTransfer) begin
            state_next = immediate_cmd_desc.wroc ? WriteResp : Idle;
          end else begin
            state_next = WriteResp;
          end
        end else if ((transfer_cnt & 32'd3) == 32'd3 && i3c_tx_ready_i) begin
          // Refill TX FIFO every 4 bytes (not needed for ImmediateDataTransfer since max 4 bytes)
          if (cmd_attr != ImmediateDataTransfer) begin
            state_next = FetchTxData;
          end
        end
      end

      // I3CDataRead: Data phase only (counter starts at 0)
      I3CDataRead: begin
        // Transition to PushRxData when ready to push
        if (i3c_rx_valid_i) begin
          // 4 bytes accumulated OR last byte of transfer
          if (rx_byte_cnt_q == 2'd3 || transfer_cnt == data_length - 32'd1) begin
            state_next = PushRxData;
          end
        end
      end
      // WriteResp: Generate Response Descriptor and load it to Response Queue
      WriteResp: begin
        if (resp_queue_wready_i) begin
          state_next = Idle;
        end
      end
      default: begin
        state_next = Idle;
      end
    endcase
  end
  

  // Sequential state update
  always_ff @(posedge clk_i or negedge rst_ni) begin : proc_test
    if (~rst_ni) begin
      state <= Idle;
    end else begin
      state <= state_next;
    end
  end

endmodule
