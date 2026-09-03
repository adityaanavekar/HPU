`timescale 1ns/1ps

// ================================================================
// HPU SoC - Nexys A7-100T / XC7A100T-1CSG324C
// Phase 1: adaptive INT8 / INT16 / INT32 compute engine
//
// MMIO:
//   5000_0000 CTRL       bit0 START
//   5000_0004 STATUS     bit0 BUSY, bit1 DONE, bit2 ERR, bit3 OVF
//   5000_0008 CONFIG     [1:0] precision, [4:2] opcode
//   5000_000C LENGTH     element count for DOT
//   5000_0010 OPERAND_A  byte address
//   5000_0014 OPERAND_B  byte address
//   5000_0018 RESULT_LO
//   5000_001C RESULT_HI
//   5000_0020 ACC_LO
//   5000_0024 ACC_HI
//   5000_0028 CYCLE_CNT
//   5000_002C ELEM_CNT
//   5000_1000-5000_1FFF operand window
//
// HPU datapath:
//   S1 operand register
//   S2 multiplier / scalar product register
//   S3 balanced reduction / scalar operation register
//   S4 final word-result register
//   S5 accumulator register
// ================================================================

module hpu_top (
    input  wire        CLK100MHZ,
    input  wire        CPU_RESETN,
    input  wire [15:0] SW,
    output wire [15:0] LED,
    output wire        UART_TXD
);

    (* ASYNC_REG = "TRUE" *) reg [1:0] reset_sync;
    (* ASYNC_REG = "TRUE" *) reg [15:0] sw_sync;
    wire reset;

    initial begin
        reset_sync = 2'b11;
        sw_sync = 16'h0000;
    end

    always @(posedge CLK100MHZ) begin
        reset_sync[0] <= ~CPU_RESETN;
        reset_sync[1] <= reset_sync[0];
        sw_sync <= SW;
    end

    assign reset = reset_sync[1];

    wire [31:0] instr_addr;
    wire [31:0] instr_data;
    wire [31:0] data_addr;
    wire [31:0] data_wdata;
    wire [3:0]  data_we;
    wire        data_req;
    wire [31:0] data_rdata;

    wire rom_instr_select = (instr_addr < 32'h0001_0000);
    wire ram_instr_select = (instr_addr >= 32'h0001_0000) &&
                             (instr_addr < 32'h0002_0000);
    wire rom_data_select = (data_addr < 32'h0001_0000);
    wire ram_data_select = (data_addr >= 32'h0001_0000) &&
                           (data_addr < 32'h0002_0000);
    wire uart_data_select = (data_addr == 32'h4000_0000);
    wire uart_status_select = (data_addr == 32'h4000_0004);
    wire gpio_output_select = (data_addr == 32'h4000_1000);
    wire gpio_input_select = (data_addr == 32'h4000_1004);
    wire timer_select = (data_addr == 32'h4000_2000);
    wire hpu_mmio_select = (data_addr >= 32'h5000_0000) &&
                           (data_addr <= 32'h5000_002C);
    wire hpu_operand_window_select = (data_addr >= 32'h5000_1000) &&
                                     (data_addr <= 32'h5000_1FFF);

    wire [31:0] rom_instr_data;
    wire [31:0] rom_data_rdata;
    wire [31:0] ram_instr_data;
    wire [31:0] ram_data_rdata;
    wire [3:0] ram_we = (data_req && ram_data_select) ? data_we : 4'b0000;

    wire uart_busy;
    wire uart_start = data_req && uart_data_select &&
                       (data_we != 4'b0000) && !uart_busy;
    wire gpio_write_enable = data_req && gpio_output_select &&
                             (data_we != 4'b0000);

    wire hpu_req = data_req &&
                   (hpu_mmio_select || hpu_operand_window_select);
    wire [31:0] hpu_rdata;

    hpu_cpu CPU (
        .clk(CLK100MHZ),
        .reset(reset),
        .instr_addr(instr_addr),
        .instr_data(instr_data),
        .data_addr(data_addr),
        .data_wdata(data_wdata),
        .data_we(data_we),
        .data_req(data_req),
        .data_rdata(data_rdata)
    );

    boot_rom BOOT_ROM (
        .instr_word_addr(instr_addr[15:2]),
        .instr_rdata(rom_instr_data),
        .data_word_addr(data_addr[15:2]),
        .data_rdata(rom_data_rdata)
    );

    system_ram SYSTEM_RAM (
        .clk(CLK100MHZ),
        .instr_word_addr(instr_addr[15:2]),
        .instr_rdata(ram_instr_data),
        .data_word_addr(data_addr[15:2]),
        .data_wdata(data_wdata),
        .we(ram_we),
        .data_rdata(ram_data_rdata)
    );

    gpio_unit GPIO (
        .clk(CLK100MHZ),
        .reset(reset),
        .write_enable(gpio_write_enable),
        .wdata(data_wdata[15:0]),
        .leds(LED)
    );

    wire [31:0] timer_value;
    timer_unit TIMER (
        .clk(CLK100MHZ),
        .reset(reset),
        .timer_value(timer_value)
    );

    uart_tx UART (
        .clk(CLK100MHZ),
        .reset(reset),
        .start(uart_start),
        .data(data_wdata[7:0]),
        .tx(UART_TXD),
        .busy(uart_busy)
    );

    hpu_compute_engine HPU (
        .clk(CLK100MHZ),
        .reset(reset),
        .bus_req(hpu_req),
        .bus_addr(data_addr),
        .bus_wdata(data_wdata),
        .bus_we(data_we),
        .bus_rdata(hpu_rdata)
    );

    assign instr_data = rom_instr_select ? rom_instr_data :
                        ram_instr_select ? ram_instr_data :
                        32'h0000_0013;

    assign data_rdata = rom_data_select ? rom_data_rdata :
                        ram_data_select ? ram_data_rdata :
                        uart_status_select ? {31'd0, uart_busy} :
                        gpio_input_select ? {16'd0, sw_sync} :
                        timer_select ? timer_value :
                        (hpu_mmio_select || hpu_operand_window_select) ?
                            hpu_rdata : 32'd0;

endmodule


module hpu_compute_engine (
    input  wire        clk,
    input  wire        reset,
    input  wire        bus_req,
    input  wire [31:0] bus_addr,
    input  wire [31:0] bus_wdata,
    input  wire [3:0]  bus_we,
    output reg  [31:0] bus_rdata
);

    localparam [1:0] PREC_INT8  = 2'd0;
    localparam [1:0] PREC_INT16 = 2'd1;
    localparam [1:0] PREC_INT32 = 2'd2;

    localparam [2:0] OP_NOP = 3'd0;
    localparam [2:0] OP_DOT = 3'd1;
    localparam [2:0] OP_ADD = 3'd2;
    localparam [2:0] OP_SUB = 3'd3;
    localparam [2:0] OP_MUL = 3'd4;
    localparam [2:0] OP_MAC = 3'd5;

    reg busy, done, err, ovf;
    reg [31:0] config_reg;
    reg [31:0] length_reg;
    reg [31:0] op_a_base_reg;
    reg [31:0] op_b_base_reg;
    reg signed [63:0] result_reg;
    reg signed [63:0] acc_reg;
    reg [31:0] cycle_cnt_reg;
    reg [31:0] elem_cnt_reg;

    reg [1:0] precision_latched;
    reg [2:0] op_latched;
    reg [9:0] word_count_latched;
    reg [9:0] issue_count;

    reg [9:0] opa_addr;
    reg [9:0] opb_addr;
    wire [31:0] opa_rdata;
    wire [31:0] opb_rdata;
    wire [31:0] operand_cpu_rdata;
    wire operand_window_select;
    wire cpu_operand_we = bus_req && operand_window_select &&
                           (bus_we != 4'b0000);
    wire [9:0] cpu_operand_addr = bus_addr[11:2];

    operand_ram OPERANDS (
        .clk(clk),
        .cpu_we(cpu_operand_we),
        .cpu_be(bus_we),
        .cpu_addr(cpu_operand_addr),
        .cpu_wdata(bus_wdata),
        .cpu_rdata(operand_cpu_rdata),
        .opa_addr(opa_addr),
        .opa_rdata(opa_rdata),
        .opb_addr(opb_addr),
        .opb_rdata(opb_rdata)
    );

    assign operand_window_select = (bus_addr >= 32'h5000_1000) &&
                                   (bus_addr <= 32'h5000_1FFF);

    wire start_write = bus_req && (bus_addr == 32'h5000_0000) &&
                       (bus_we != 4'b0000) && bus_wdata[0];

    wire cfg_precision_ok = config_reg[1:0] != 2'd3;
    wire cfg_op_ok = config_reg[4:2] <= OP_MAC;
    wire cfg_is_dot = config_reg[4:2] == OP_DOT;
    wire cfg_is_nop = config_reg[4:2] == OP_NOP;

    wire cfg_length_ok =
        cfg_is_nop ||
        ((length_reg != 32'd0) &&
         (length_reg <= ((config_reg[1:0] == PREC_INT8) ? 32'd2048 :
                         (config_reg[1:0] == PREC_INT16) ? 32'd1024 :
                         32'd512)));

    function [9:0] words_for_length;
        input [31:0] length;
        input [1:0] precision;
        begin
            case (precision)
                PREC_INT8:  words_for_length = (length + 32'd3) >> 2;
                PREC_INT16: words_for_length = (length + 32'd1) >> 1;
                default:    words_for_length = length[9:0];
            endcase
        end
    endfunction

    function signed [63:0] scalar_extend;
        input [31:0] x;
        input [1:0] precision;
        begin
            case (precision)
                PREC_INT8:  scalar_extend = {{56{x[7]}},x[7:0]};
                PREC_INT16: scalar_extend = {{48{x[15]}},x[15:0]};
                default:    scalar_extend = {{32{x[31]}},x};
            endcase
        end
    endfunction

    function signed [63:0] sat_add64;
        input signed [63:0] a;
        input signed [63:0] b;
        reg signed [64:0] wide_sum;
        begin
            wide_sum = {a[63],a} + {b[63],b};
            if ((a[63] == b[63]) && (wide_sum[64] != wide_sum[63]))
                sat_add64 = a[63] ?
                    64'h8000_0000_0000_0000 :
                    64'h7FFF_FFFF_FFFF_FFFF;
            else
                sat_add64 = wide_sum[63:0];
        end
    endfunction

    function overflow64;
        input signed [63:0] a;
        input signed [63:0] b;
        input signed [63:0] s;
        begin
            overflow64 = (a[63] == b[63]) && (s[63] != a[63]);
        end
    endfunction

    // ------------------------------------------------------------
    // Pipeline registers.
    // ------------------------------------------------------------
    reg [31:0] a_word_s1;
    reg [31:0] b_word_s1;

    // Raw multiplier outputs are registered directly after each multiply.
    // Keeping the precision-selection mux OUTSIDE these registers lets
    // Vivado absorb the registers into DSP48E1 MREG/PREG stages.
    reg signed [15:0] mul8_0_s2;
    reg signed [15:0] mul8_1_s2;
    reg signed [15:0] mul8_2_s2;
    reg signed [15:0] mul8_3_s2;
    reg signed [31:0] mul16_0_s2;
    reg signed [31:0] mul16_1_s2;
    reg signed [31:0] i32_p11_s2;
    reg signed [31:0] i32_p10_s2;
    reg signed [31:0] i32_p01_s2;
    reg        [31:0] i32_p00_s2;

    reg signed [63:0] scalar_a_s1;
    reg signed [63:0] scalar_b_s1;
    reg signed [63:0] scalar_mul8_s2;
    reg signed [63:0] scalar_mul16_s2;
    reg signed [63:0] scalar_mul32_s2;
    reg signed [63:0] scalar_result_s3;

    reg signed [63:0] red01_s3;
    reg signed [63:0] red23_s3;
    reg signed [63:0] result_s4;

    reg [4:0] valid_pipe;
    reg [4:0] last_pipe;

    wire issue_now = busy && (issue_count < word_count_latched);
    wire last_issue = issue_now &&
                      (issue_count == (word_count_latched - 10'd1));

    wire signed [7:0] a8_0 = a_word_s1[7:0];
    wire signed [7:0] a8_1 = a_word_s1[15:8];
    wire signed [7:0] a8_2 = a_word_s1[23:16];
    wire signed [7:0] a8_3 = a_word_s1[31:24];

    wire signed [7:0] b8_0 = b_word_s1[7:0];
    wire signed [7:0] b8_1 = b_word_s1[15:8];
    wire signed [7:0] b8_2 = b_word_s1[23:16];
    wire signed [7:0] b8_3 = b_word_s1[31:24];

    (* use_dsp = "yes" *) wire signed [15:0] mul8_0 = a8_0 * b8_0;
    (* use_dsp = "yes" *) wire signed [15:0] mul8_1 = a8_1 * b8_1;
    (* use_dsp = "yes" *) wire signed [15:0] mul8_2 = a8_2 * b8_2;
    (* use_dsp = "yes" *) wire signed [15:0] mul8_3 = a8_3 * b8_3;

    wire signed [15:0] a16_0 = a_word_s1[15:0];
    wire signed [15:0] a16_1 = a_word_s1[31:16];
    wire signed [15:0] b16_0 = b_word_s1[15:0];
    wire signed [15:0] b16_1 = b_word_s1[31:16];

    (* use_dsp = "yes" *) wire signed [31:0] mul16_0 = a16_0 * b16_0;
    (* use_dsp = "yes" *) wire signed [31:0] mul16_1 = a16_1 * b16_1;

    // Exact signed 32x32 decomposition into four 16x16 products.
    (* use_dsp = "yes" *) wire signed [31:0] i32_p11 =
        $signed(a_word_s1[31:16]) * $signed(b_word_s1[31:16]);

    (* use_dsp = "yes" *) wire signed [31:0] i32_p10 =
        $signed(a_word_s1[31:16]) * $signed({1'b0,b_word_s1[15:0]});

    (* use_dsp = "yes" *) wire signed [31:0] i32_p01 =
        $signed({1'b0,a_word_s1[15:0]}) * $signed(b_word_s1[31:16]);

    // Low x low is an unsigned 16x16 product.  It must be zero-extended
    // during INT32 reconstruction; sign-extending it corrupts products
    // whose bit 31 is 1.
    (* use_dsp = "yes" *) wire [31:0] i32_p00 =
        a_word_s1[15:0] * b_word_s1[15:0];

    wire signed [31:0] prod0_s2 =
        (precision_latched == PREC_INT8) ?
            {{16{mul8_0_s2[15]}},mul8_0_s2} :
        (precision_latched == PREC_INT16) ?
            mul16_0_s2 : i32_p11_s2;

    wire signed [31:0] prod1_s2 =
        (precision_latched == PREC_INT8) ?
            {{16{mul8_1_s2[15]}},mul8_1_s2} :
        (precision_latched == PREC_INT16) ?
            mul16_1_s2 : i32_p10_s2;

    wire signed [31:0] prod2_s2 =
        (precision_latched == PREC_INT8) ?
            {{16{mul8_2_s2[15]}},mul8_2_s2} :
        (precision_latched == PREC_INT16) ?
            32'sd0 : i32_p01_s2;

    wire signed [31:0] prod3_s2 =
        (precision_latched == PREC_INT8) ?
            {{16{mul8_3_s2[15]}},mul8_3_s2} :
        (precision_latched == PREC_INT16) ?
            32'sd0 : $signed(i32_p00_s2);

    wire signed [63:0] scalar_mul_s2 =
        (precision_latched == PREC_INT8)  ? scalar_mul8_s2 :
        (precision_latched == PREC_INT16) ? scalar_mul16_s2 :
                                             scalar_mul32_s2;

    (* use_dsp = "yes" *)
    wire signed [63:0] scalar_mul8_wire =
        $signed(scalar_a_s1[7:0]) * $signed(scalar_b_s1[7:0]);

    (* use_dsp = "yes" *)
    wire signed [63:0] scalar_mul16_wire =
        $signed(scalar_a_s1[15:0]) * $signed(scalar_b_s1[15:0]);

    (* use_dsp = "yes" *)
    wire signed [63:0] scalar_mul32_wire =
        $signed(scalar_a_s1[31:0]) * $signed(scalar_b_s1[31:0]);

    wire signed [63:0] scalar_add_sub =
        (op_latched == OP_ADD) ?
            (scalar_a_s1 + scalar_b_s1) :
            (scalar_a_s1 - scalar_b_s1);

    // ------------------------------------------------------------
    // Sequential pipeline and control.
    // ------------------------------------------------------------
    always @(posedge clk) begin
        if (reset) begin
            busy <= 1'b0;
            done <= 1'b0;
            err <= 1'b0;
            ovf <= 1'b0;

            config_reg <= 32'd0;
            length_reg <= 32'd0;
            op_a_base_reg <= 32'd0;
            op_b_base_reg <= 32'd2048;

            result_reg <= 64'sd0;
            acc_reg <= 64'sd0;
            cycle_cnt_reg <= 32'd0;
            elem_cnt_reg <= 32'd0;

            precision_latched <= PREC_INT8;
            op_latched <= OP_NOP;
            word_count_latched <= 10'd0;
            issue_count <= 10'd0;

            opa_addr <= 10'd0;
            opb_addr <= 10'd512;

            a_word_s1 <= 32'd0;
            b_word_s1 <= 32'd0;
            mul8_0_s2 <= 16'sd0;
            mul8_1_s2 <= 16'sd0;
            mul8_2_s2 <= 16'sd0;
            mul8_3_s2 <= 16'sd0;
            mul16_0_s2 <= 32'sd0;
            mul16_1_s2 <= 32'sd0;
            i32_p11_s2 <= 32'sd0;
            i32_p10_s2 <= 32'sd0;
            i32_p01_s2 <= 32'sd0;
            i32_p00_s2 <= 32'd0;
            scalar_a_s1 <= 64'sd0;
            scalar_b_s1 <= 64'sd0;
            scalar_mul8_s2 <= 64'sd0;
            scalar_mul16_s2 <= 64'sd0;
            scalar_mul32_s2 <= 64'sd0;
            scalar_result_s3 <= 64'sd0;
            red01_s3 <= 64'sd0;
            red23_s3 <= 64'sd0;
            result_s4 <= 64'sd0;
            valid_pipe <= 5'b00000;
            last_pipe <= 5'b00000;
        end else begin
            done <= 1'b0;

            // Configuration registers are writable only while idle.
            if (bus_req && (bus_we != 4'b0000) && !busy) begin
                case (bus_addr)
                    32'h5000_0008: config_reg <= bus_wdata;
                    32'h5000_000C: length_reg <= bus_wdata;
                    32'h5000_0010: op_a_base_reg <= bus_wdata;
                    32'h5000_0014: op_b_base_reg <= bus_wdata;
                    default: begin end
                endcase
            end

            if (start_write) begin
                if (busy) begin
                    err <= 1'b1;
                end else begin
                    err <= 1'b0;
                    ovf <= 1'b0;
                    result_reg <= 64'sd0;
                    acc_reg <= 64'sd0;
                    cycle_cnt_reg <= 32'd1;
                    elem_cnt_reg <= 32'd0;
                    issue_count <= 10'd0;
                    valid_pipe <= 5'b00000;
                    last_pipe <= 5'b00000;

                    if (cfg_precision_ok && cfg_op_ok && cfg_length_ok) begin
                        precision_latched <= config_reg[1:0];
                        op_latched <= config_reg[4:2];

                        if (cfg_is_nop) begin
                            word_count_latched <= 10'd0;
                            busy <= 1'b0;
                            done <= 1'b1;
                            elem_cnt_reg <= 32'd0;
                        end else begin
                            word_count_latched <= cfg_is_dot ?
                                words_for_length(length_reg, config_reg[1:0]) :
                                10'd1;
                            busy <= 1'b1;
                            elem_cnt_reg <= cfg_is_dot ?
                                length_reg : 32'd1;
                        end
                    end else begin
                        busy <= 1'b0;
                        done <= 1'b1;
                        err <= 1'b1;
                        cycle_cnt_reg <= 32'd1;
                        elem_cnt_reg <= 32'd0;
                    end
                end
            end

            if (busy) begin
                cycle_cnt_reg <= cycle_cnt_reg + 32'd1;

                // Issue one operand word per cycle. RAM outputs arrive on
                // the following clock edge.
                if (issue_now) begin
                    opa_addr <= op_a_base_reg[11:2] + issue_count;
                    opb_addr <= op_b_base_reg[11:2] + issue_count;
                    issue_count <= issue_count + 10'd1;
                end

                valid_pipe <= {valid_pipe[3:0], issue_now};
                last_pipe <= {last_pipe[3:0], last_issue};

                // S1: synchronous operand RAM output.
                if (valid_pipe[0]) begin
                    a_word_s1 <= opa_rdata;
                    b_word_s1 <= opb_rdata;

                    scalar_a_s1 <= scalar_extend(opa_rdata,
                                                  precision_latched);
                    scalar_b_s1 <= scalar_extend(opb_rdata,
                                                  precision_latched);
                end

                // S2: register every multiplier output directly.
                // These assignments are deliberately one-to-one with the
                // multiplier operators above; the precision mux is applied
                // only after the registered DSP result.
                if (valid_pipe[1]) begin
                    mul8_0_s2 <= mul8_0;
                    mul8_1_s2 <= mul8_1;
                    mul8_2_s2 <= mul8_2;
                    mul8_3_s2 <= mul8_3;
                    mul16_0_s2 <= mul16_0;
                    mul16_1_s2 <= mul16_1;
                    i32_p11_s2 <= i32_p11;
                    i32_p10_s2 <= i32_p10;
                    i32_p01_s2 <= i32_p01;
                    i32_p00_s2 <= i32_p00;

                    scalar_mul8_s2 <= scalar_mul8_wire;
                    scalar_mul16_s2 <= scalar_mul16_wire;
                    scalar_mul32_s2 <= scalar_mul32_wire;
                end

                // S3: balanced reduction and scalar operation.
                if (valid_pipe[2]) begin
                    if (precision_latched == PREC_INT8) begin
                        red01_s3 <= {{32{prod0_s2[31]}},prod0_s2} +
                                    {{32{prod1_s2[31]}},prod1_s2};
                        red23_s3 <= {{32{prod2_s2[31]}},prod2_s2} +
                                    {{32{prod3_s2[31]}},prod3_s2};
                    end else if (precision_latched == PREC_INT16) begin
                        red01_s3 <= {{32{prod0_s2[31]}},prod0_s2} +
                                    {{32{prod1_s2[31]}},prod1_s2};
                        red23_s3 <= 64'sd0;
                    end else begin
                        red01_s3 <= ({{32{prod0_s2[31]}},prod0_s2} <<< 32) +
                                    ({{32{prod1_s2[31]}},prod1_s2} <<< 16);
                        red23_s3 <= ({{32{prod2_s2[31]}},prod2_s2} <<< 16) +
                                    {32'd0,i32_p00_s2};
                    end

                    if (op_latched == OP_ADD || op_latched == OP_SUB)
                        scalar_result_s3 <= scalar_add_sub;
                    else if (op_latched == OP_MUL ||
                             op_latched == OP_MAC)
                        scalar_result_s3 <= scalar_mul_s2;
                    else
                        scalar_result_s3 <= 64'sd0;
                end

                // S4: one final 64-bit word result.
                if (valid_pipe[3]) begin
                    if (op_latched == OP_DOT)
                        result_s4 <= red01_s3 + red23_s3;
                    else
                        result_s4 <= scalar_result_s3;
                end

                // S5: accumulator and completion.
                if (valid_pipe[4]) begin
                    result_reg <= result_s4;

                    if (op_latched == OP_DOT) begin
                        acc_reg <= sat_add64(acc_reg, result_s4);
                        if (overflow64(acc_reg, result_s4,
                                       sat_add64(acc_reg, result_s4)))
                            ovf <= 1'b1;
                    end else if (op_latched == OP_MAC) begin
                        acc_reg <= sat_add64(acc_reg, result_s4);
                        if (overflow64(acc_reg, result_s4,
                                       sat_add64(acc_reg, result_s4)))
                            ovf <= 1'b1;
                    end else begin
                        acc_reg <= result_s4;
                    end

                    if (last_pipe[4]) begin
                        busy <= 1'b0;
                        done <= 1'b1;
                        cycle_cnt_reg <= cycle_cnt_reg + 32'd1;
                    end
                end
            end else begin
                valid_pipe <= 5'b00000;
                last_pipe <= 5'b00000;
            end
        end
    end

    always @(*) begin
        case (bus_addr)
            32'h5000_0000: bus_rdata = 32'd0;
            32'h5000_0004: bus_rdata = {28'd0,ovf,err,done,busy};
            32'h5000_0008: bus_rdata = config_reg;
            32'h5000_000C: bus_rdata = length_reg;
            32'h5000_0010: bus_rdata = op_a_base_reg;
            32'h5000_0014: bus_rdata = op_b_base_reg;
            32'h5000_0018: bus_rdata = result_reg[31:0];
            32'h5000_001C: bus_rdata = result_reg[63:32];
            32'h5000_0020: bus_rdata = acc_reg[31:0];
            32'h5000_0024: bus_rdata = acc_reg[63:32];
            32'h5000_0028: bus_rdata = cycle_cnt_reg;
            32'h5000_002C: bus_rdata = elem_cnt_reg;
            default: bus_rdata = operand_window_select ?
                                 operand_cpu_rdata : 32'd0;
        endcase
    end

endmodule


module operand_ram (
    input  wire        clk,
    input  wire        cpu_we,
    input  wire [3:0]  cpu_be,
    input  wire [9:0]  cpu_addr,
    input  wire [31:0] cpu_wdata,
    output reg  [31:0] cpu_rdata,
    input  wire [9:0]  opa_addr,
    output reg  [31:0] opa_rdata,
    input  wire [9:0]  opb_addr,
    output reg  [31:0] opb_rdata
);

    // Register-backed operand store is intentional here. The 4-KiB store
    // has one CPU access plus two independent engine read ports, so a
    // conventional single/dual-port BRAM cannot implement the interface
    // without replication. Register storage removes the large LUT-RAM
    // methodology warning class while preserving all three access ports.
    (* ram_style = "registers" *) reg [7:0] a0 [0:511];
    (* ram_style = "registers" *) reg [7:0] a1 [0:511];
    (* ram_style = "registers" *) reg [7:0] a2 [0:511];
    (* ram_style = "registers" *) reg [7:0] a3 [0:511];

    (* ram_style = "registers" *) reg [7:0] b0 [0:511];
    (* ram_style = "registers" *) reg [7:0] b1 [0:511];
    (* ram_style = "registers" *) reg [7:0] b2 [0:511];
    (* ram_style = "registers" *) reg [7:0] b3 [0:511];

    integer k;
    initial begin
        for (k = 0; k < 512; k = k + 1) begin
            a0[k] = 8'd0; a1[k] = 8'd0;
            a2[k] = 8'd0; a3[k] = 8'd0;
            b0[k] = 8'd0; b1[k] = 8'd0;
            b2[k] = 8'd0; b3[k] = 8'd0;
        end
    end

    always @(posedge clk) begin
        if (cpu_we) begin
            if (!cpu_addr[9]) begin
                if (cpu_be[0]) a0[cpu_addr[8:0]] <= cpu_wdata[7:0];
                if (cpu_be[1]) a1[cpu_addr[8:0]] <= cpu_wdata[15:8];
                if (cpu_be[2]) a2[cpu_addr[8:0]] <= cpu_wdata[23:16];
                if (cpu_be[3]) a3[cpu_addr[8:0]] <= cpu_wdata[31:24];
            end else begin
                if (cpu_be[0]) b0[cpu_addr[8:0]] <= cpu_wdata[7:0];
                if (cpu_be[1]) b1[cpu_addr[8:0]] <= cpu_wdata[15:8];
                if (cpu_be[2]) b2[cpu_addr[8:0]] <= cpu_wdata[23:16];
                if (cpu_be[3]) b3[cpu_addr[8:0]] <= cpu_wdata[31:24];
            end
        end

        cpu_rdata <= cpu_addr[9] ?
                     {b3[cpu_addr[8:0]],b2[cpu_addr[8:0]],
                      b1[cpu_addr[8:0]],b0[cpu_addr[8:0]]} :
                     {a3[cpu_addr[8:0]],a2[cpu_addr[8:0]],
                      a1[cpu_addr[8:0]],a0[cpu_addr[8:0]]};

        if (!opa_addr[9])
            opa_rdata <= {a3[opa_addr[8:0]],a2[opa_addr[8:0]],
                          a1[opa_addr[8:0]],a0[opa_addr[8:0]]};
        else
            opa_rdata <= {b3[opa_addr[8:0]],b2[opa_addr[8:0]],
                          b1[opa_addr[8:0]],b0[opa_addr[8:0]]};

        if (!opb_addr[9])
            opb_rdata <= {a3[opb_addr[8:0]],a2[opb_addr[8:0]],
                          a1[opb_addr[8:0]],a0[opb_addr[8:0]]};
        else
            opb_rdata <= {b3[opb_addr[8:0]],b2[opb_addr[8:0]],
                          b1[opb_addr[8:0]],b0[opb_addr[8:0]]};
    end

endmodule


module hpu_cpu
(
    input  wire        clk,
    input  wire        reset,
    output wire [31:0] instr_addr,
    input  wire [31:0] instr_data,
    output reg  [31:0] data_addr,
    output reg  [31:0] data_wdata,
    output reg  [3:0]  data_we,
    output reg         data_req,
    input  wire [31:0] data_rdata
);

    localparam STATE_FETCH       = 4'd0;
    localparam STATE_DECODE      = 4'd1;
    localparam STATE_ALU         = 4'd2;
    localparam STATE_MEM_READ    = 4'd3;
    localparam STATE_MEM_CAPTURE = 4'd4;
    localparam STATE_MEM_WRITE   = 4'd5;
    localparam STATE_MEM_DONE    = 4'd6;
    localparam STATE_WRITEBACK   = 4'd7;

    reg [3:0] state;
    reg [31:0] pc;
    reg [31:0] instruction;
    reg [31:0] registers [0:31];
    reg [31:0] alu_result;
    reg [31:0] memory_result;
    reg [4:0] destination_register;
    reg [2:0] memory_funct3;
    reg [1:0] memory_offset;
    reg load_operation;
    integer i;

    assign instr_addr = pc;

    wire [6:0] opcode = instruction[6:0];
    wire [4:0] rd = instruction[11:7];
    wire [2:0] funct3 = instruction[14:12];
    wire [4:0] rs1 = instruction[19:15];
    wire [4:0] rs2 = instruction[24:20];

    wire [31:0] rs1_value = (rs1 == 5'd0) ? 32'd0 : registers[rs1];
    wire [31:0] rs2_value = (rs2 == 5'd0) ? 32'd0 : registers[rs2];

    wire [31:0] immediate_i = {{20{instruction[31]}}, instruction[31:20]};
    wire [31:0] immediate_s = {{20{instruction[31]}}, instruction[31:25], instruction[11:7]};
    wire [31:0] immediate_b = {{19{instruction[31]}}, instruction[31], instruction[7], instruction[30:25], instruction[11:8], 1'b0};
    wire [31:0] immediate_u = {instruction[31:12], 12'b0};
    wire [31:0] immediate_j = {{11{instruction[31]}}, instruction[31], instruction[19:12], instruction[20], instruction[30:21], 1'b0};

    wire [31:0] load_effective_address = rs1_value + immediate_i;
    wire [31:0] store_effective_address = rs1_value + immediate_s;

    function [31:0] sign_extend_byte;
        input [7:0] value;
        begin
            sign_extend_byte = {{24{value[7]}}, value};
        end
    endfunction

    function [31:0] sign_extend_half;
        input [15:0] value;
        begin
            sign_extend_half = {{16{value[15]}}, value};
        end
    endfunction

    always @(posedge clk) begin
        if (reset) begin
            pc <= 32'd0;
            instruction <= 32'h0000_0013;
            data_addr <= 32'd0;
            data_wdata <= 32'd0;
            data_we <= 4'b0000;
            data_req <= 1'b0;
            alu_result <= 32'd0;
            memory_result <= 32'd0;
            destination_register <= 5'd0;
            memory_funct3 <= 3'd0;
            memory_offset <= 2'd0;
            load_operation <= 1'b0;
            state <= STATE_FETCH;
            // x0 is architecturally hard-wired to zero by rs1_value/rs2_value.
            // Do not synthesize a physical resettable x0 register.
            for (i = 1; i < 32; i = i + 1) begin
                registers[i] <= 32'd0;
            end
        end else begin
            case (state)
                STATE_FETCH: begin
                    data_req <= 1'b0;
                    data_we <= 4'b0000;
                    state <= STATE_DECODE;
                end

                STATE_DECODE: begin
                    instruction <= instr_data;
                    pc <= pc + 32'd4;
                    state <= STATE_ALU;
                end

                STATE_ALU: begin
                    case (opcode)
                        7'b0110111: begin if (rd != 5'd0) registers[rd] <= immediate_u; state <= STATE_FETCH; end
                        7'b0010111: begin if (rd != 5'd0) registers[rd] <= (pc - 32'd4) + immediate_u; state <= STATE_FETCH; end

                        7'b0010011: begin
                            destination_register <= rd;
                            load_operation <= 1'b0;
                            case (funct3)
                                3'b000: alu_result <= rs1_value + immediate_i;
                                3'b010: alu_result <= ($signed(rs1_value) < $signed(immediate_i)) ? 32'd1 : 32'd0;
                                3'b011: alu_result <= (rs1_value < immediate_i) ? 32'd1 : 32'd0;
                                3'b100: alu_result <= rs1_value ^ immediate_i;
                                3'b110: alu_result <= rs1_value | immediate_i;
                                3'b111: alu_result <= rs1_value & immediate_i;
                                3'b001: alu_result <= rs1_value << instruction[24:20];
                                3'b101: alu_result <= instruction[30] ? ($signed(rs1_value) >>> instruction[24:20]) : (rs1_value >> instruction[24:20]);
                                default: alu_result <= 32'd0;
                            endcase
                            state <= STATE_WRITEBACK;
                        end

                        7'b0110011: begin
                            destination_register <= rd;
                            load_operation <= 1'b0;
                            case (funct3)
                                3'b000: alu_result <= instruction[30] ? (rs1_value - rs2_value) : (rs1_value + rs2_value);
                                3'b001: alu_result <= rs1_value << rs2_value[4:0];
                                3'b010: alu_result <= ($signed(rs1_value) < $signed(rs2_value)) ? 32'd1 : 32'd0;
                                3'b011: alu_result <= (rs1_value < rs2_value) ? 32'd1 : 32'd0;
                                3'b100: alu_result <= rs1_value ^ rs2_value;
                                3'b101: alu_result <= instruction[30] ? ($signed(rs1_value) >>> rs2_value[4:0]) : (rs1_value >> rs2_value[4:0]);
                                3'b110: alu_result <= rs1_value | rs2_value;
                                3'b111: alu_result <= rs1_value & rs2_value;
                                default: alu_result <= 32'd0;
                            endcase
                            state <= STATE_WRITEBACK;
                        end

                        7'b0000011: begin
                            data_addr <= {load_effective_address[31:2], 2'b00};
                            memory_offset <= load_effective_address[1:0];
                            destination_register <= rd;
                            memory_funct3 <= funct3;
                            load_operation <= 1'b1;
                            data_we <= 4'b0000;
                            data_req <= 1'b1;
                            state <= STATE_MEM_READ;
                        end

                        7'b0100011: begin
                            data_addr <= {store_effective_address[31:2], 2'b00};
                            memory_offset <= store_effective_address[1:0];
                            memory_funct3 <= funct3;
                            load_operation <= 1'b0;
                            data_req <= 1'b1;
                            case (funct3)
                                3'b000: begin
                                    case (store_effective_address[1:0])
                                        2'b00: data_wdata <= {24'd0, rs2_value[7:0]};
                                        2'b01: data_wdata <= {16'd0, rs2_value[7:0], 8'd0};
                                        2'b10: data_wdata <= {8'd0, rs2_value[7:0], 16'd0};
                                        default: data_wdata <= {rs2_value[7:0], 24'd0};
                                    endcase
                                end
                                3'b001: data_wdata <= store_effective_address[1] ? {rs2_value[15:0], 16'd0} : {16'd0, rs2_value[15:0]};
                                3'b010: data_wdata <= rs2_value;
                                default: data_wdata <= 32'd0;
                            endcase
                            state <= STATE_MEM_WRITE;
                        end

                        7'b1100011: begin
                            case (funct3)
                                3'b000: if (rs1_value == rs2_value) pc <= (pc - 32'd4) + immediate_b;
                                3'b001: if (rs1_value != rs2_value) pc <= (pc - 32'd4) + immediate_b;
                                3'b100: if ($signed(rs1_value) < $signed(rs2_value)) pc <= (pc - 32'd4) + immediate_b;
                                3'b101: if ($signed(rs1_value) >= $signed(rs2_value)) pc <= (pc - 32'd4) + immediate_b;
                                3'b110: if (rs1_value < rs2_value) pc <= (pc - 32'd4) + immediate_b;
                                3'b111: if (rs1_value >= rs2_value) pc <= (pc - 32'd4) + immediate_b;
                                default: begin end
                            endcase
                            state <= STATE_FETCH;
                        end

                        7'b1101111: begin
                            if (rd != 5'd0) registers[rd] <= pc;
                            pc <= (pc - 32'd4) + immediate_j;
                            state <= STATE_FETCH;
                        end

                        7'b1100111: begin
                            if (rd != 5'd0) registers[rd] <= pc;
                            pc <= (rs1_value + immediate_i) & 32'hFFFF_FFFE;
                            state <= STATE_FETCH;
                        end

                        default: state <= STATE_FETCH;
                    endcase
                end

                STATE_MEM_READ: begin
                    data_req <= 1'b1;
                    data_we <= 4'b0000;
                    state <= STATE_MEM_CAPTURE;
                end

                STATE_MEM_CAPTURE: begin
                    data_req <= 1'b0;
                    data_we <= 4'b0000;
                    case (memory_funct3)
                        3'b000: begin
                            case (memory_offset)
                                2'b00: memory_result <= sign_extend_byte(data_rdata[7:0]);
                                2'b01: memory_result <= sign_extend_byte(data_rdata[15:8]);
                                2'b10: memory_result <= sign_extend_byte(data_rdata[23:16]);
                                default: memory_result <= sign_extend_byte(data_rdata[31:24]);
                            endcase
                        end
                        3'b001: memory_result <= memory_offset[1] ? sign_extend_half(data_rdata[31:16]) : sign_extend_half(data_rdata[15:0]);
                        3'b010: memory_result <= data_rdata;
                        3'b100: begin
                            case (memory_offset)
                                2'b00: memory_result <= {24'd0, data_rdata[7:0]};
                                2'b01: memory_result <= {24'd0, data_rdata[15:8]};
                                2'b10: memory_result <= {24'd0, data_rdata[23:16]};
                                default: memory_result <= {24'd0, data_rdata[31:24]};
                            endcase
                        end
                        3'b101: memory_result <= memory_offset[1] ? {16'd0, data_rdata[31:16]} : {16'd0, data_rdata[15:0]};
                        default: memory_result <= 32'd0;
                    endcase
                    state <= STATE_WRITEBACK;
                end

                STATE_MEM_WRITE: begin
                    data_req <= 1'b1;
                    case (memory_funct3)
                        3'b000: begin
                            case (memory_offset)
                                2'b00: data_we <= 4'b0001;
                                2'b01: data_we <= 4'b0010;
                                2'b10: data_we <= 4'b0100;
                                default: data_we <= 4'b1000;
                            endcase
                        end
                        3'b001: data_we <= memory_offset[1] ? 4'b1100 : 4'b0011;
                        3'b010: data_we <= 4'b1111;
                        default: data_we <= 4'b0000;
                    endcase
                    state <= STATE_MEM_DONE;
                end

                STATE_MEM_DONE: begin
                    data_req <= 1'b0;
                    data_we <= 4'b0000;
                    state <= STATE_FETCH;
                end

                STATE_WRITEBACK: begin
                    if (destination_register != 5'd0) begin
                        registers[destination_register] <= load_operation ? memory_result : alu_result;
                    end
                    load_operation <= 1'b0;
                    state <= STATE_FETCH;
                end

                default: state <= STATE_FETCH;
            endcase
        end
    end

endmodule

module boot_rom
(
    input  wire [13:0] instr_word_addr,
    output reg  [31:0] instr_rdata,
    input  wire [13:0] data_word_addr,
    output reg  [31:0] data_rdata
);

    reg [31:0] rom [0:16383];
    integer j;

    always @(*) begin
        instr_rdata = rom[instr_word_addr];
        data_rdata = rom[data_word_addr];
    end

    initial begin
        for (j = 0; j < 16384; j = j + 1) begin
            rom[j] = 32'h0000_0013;
        end

        rom[0]  = 32'h4000_10B7;
        rom[1]  = 32'h0001_0137;
        rom[2]  = 32'h1000_0193;
        rom[3]  = 32'h0550_0213;
        rom[4]  = 32'h0041_82B3;
        rom[5]  = 32'h0051_2023;
        rom[6]  = 32'h0001_2303;
        rom[7]  = 32'h1550_0393;
        rom[8]  = 32'h0072_8463;
        rom[9]  = 32'h02C0_006F;
        rom[10] = 32'h0073_0463;
        rom[11] = 32'h0240_006F;
        rom[12] = 32'h4000_2437;
        rom[13] = 32'h0004_2483;
        rom[14] = 32'h0004_9463;
        rom[15] = 32'h0140_006F;
        rom[16] = 32'h0000_C937;
        rom[17] = 32'h0DE9_0913;
        rom[18] = 32'h0120_A023;
        rom[19] = 32'h0000_006F;
        rom[20] = 32'h0000_E937;
        rom[21] = 32'hEAD9_0913;
        rom[22] = 32'h0120_A023;
        rom[23] = 32'h0000_006F;
    end

endmodule




module system_ram (
    input  wire        clk,
    input  wire [13:0] instr_word_addr,
    output reg  [31:0] instr_rdata,
    input  wire [13:0] data_word_addr,
    input  wire [31:0] data_wdata,
    input  wire [3:0]  we,
    output reg  [31:0] data_rdata
);

    // Four byte-wide true-dual-port memories. This maps naturally to
    // block RAM and avoids the inefficient "32-bit RAM with byte write
    // enable" inference warning on this depth.
    (* ram_style = "block" *) reg [7:0] mem0 [0:16383];
    (* ram_style = "block" *) reg [7:0] mem1 [0:16383];
    (* ram_style = "block" *) reg [7:0] mem2 [0:16383];
    (* ram_style = "block" *) reg [7:0] mem3 [0:16383];

    always @(posedge clk) begin
        instr_rdata <= {mem3[instr_word_addr],mem2[instr_word_addr],
                        mem1[instr_word_addr],mem0[instr_word_addr]};
    end

    always @(posedge clk) begin
        if (we[0]) mem0[data_word_addr] <= data_wdata[7:0];
        if (we[1]) mem1[data_word_addr] <= data_wdata[15:8];
        if (we[2]) mem2[data_word_addr] <= data_wdata[23:16];
        if (we[3]) mem3[data_word_addr] <= data_wdata[31:24];

        data_rdata <= {mem3[data_word_addr],mem2[data_word_addr],
                       mem1[data_word_addr],mem0[data_word_addr]};
    end

endmodule


module gpio_unit
(
    input  wire        clk,
    input  wire        reset,
    input  wire        write_enable,
    input  wire [15:0] wdata,
    output reg  [15:0] leds
);
    always @(posedge clk) begin
        if (reset) leds <= 16'd0;
        else if (write_enable) leds <= wdata;
    end
endmodule


module timer_unit
(
    input  wire       clk,
    input  wire       reset,
    output reg [31:0] timer_value
);
    always @(posedge clk) begin
        if (reset) timer_value <= 32'd0;
        else timer_value <= timer_value + 32'd1;
    end
endmodule


module uart_tx
(
    input  wire       clk,
    input  wire       reset,
    input  wire       start,
    input  wire [7:0] data,
    output reg        tx,
    output reg        busy
);
    localparam CLKS_PER_BIT = 868;

    reg [9:0] shift_register;
    reg [9:0] clock_counter;
    reg [3:0] bit_counter;

    always @(posedge clk) begin
        if (reset) begin
            tx <= 1'b1;
            busy <= 1'b0;
            shift_register <= 10'b11_1111_1111;
            clock_counter <= 10'd0;
            bit_counter <= 4'd0;
        end else if (!busy) begin
            tx <= 1'b1;
            if (start) begin
                busy <= 1'b1;
                shift_register <= {1'b1, data, 1'b0};
                clock_counter <= 10'd0;
                bit_counter <= 4'd0;
                tx <= 1'b0;
            end
        end else begin
            if (clock_counter == CLKS_PER_BIT - 1) begin
                clock_counter <= 10'd0;
                shift_register <= {1'b1, shift_register[9:1]};
                tx <= shift_register[1];
                if (bit_counter == 4'd9) begin
                    busy <= 1'b0;
                    tx <= 1'b1;
                end
                bit_counter <= bit_counter + 4'd1;
            end else begin
                clock_counter <= clock_counter + 10'd1;
            end
        end
    end

endmodule
