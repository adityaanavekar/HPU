`timescale 1ns / 1ps
// ============================================================================
// HPU PROJECT - PHASE 1
// CPU / MEMORY / PERIPHERAL FOUNDATION
//
// Target Board:
// Digilent Nexys A7-100T
// FPGA:
// Xilinx Artix-7 XC7A100T-1CSG324C
//
// Language:
// Verilog-2001
//
// Architecture:
//
//   CPU              : Control processor
//   Boot ROM         : Boot/program memory
//   System RAM       : Data + executable memory
//   GPIO             : LEDs + Switch input
//   Timer            : Free-running counter
//   UART             : Transmitter + busy status
//
// Memory Map:
//
//   0x0000_0000 - 0x0000_FFFF : Boot ROM
//   0x0001_0000 - 0x0001_FFFF : System RAM
//
//   0x4000_0000              : UART TX DATA
//   0x4000_0004              : UART STATUS
//
//   0x4000_1000              : GPIO OUTPUT
//   0x4000_1004              : GPIO INPUT
//
//   0x4000_2000              : TIMER COUNT
//
// Reserved:
//
//   0x5000_0000              : Intelligent Scheduler
//   0x6000_0000              : GPU
//   0x7000_0000              : NPU
//   0x8000_0000              : ISP
//   0x9000_0000              : SD / DMA
//
// ============================================================================



// ============================================================================
// TOP LEVEL
// ============================================================================

module hpu_top
(
    input  wire        CLK100MHZ,
    input  wire        CPU_RESETN,

    input  wire [15:0] SW,

    output wire [15:0] LED,
    output wire        UART_TXD
);


    // ------------------------------------------------------------------------
    // RESET
    // ------------------------------------------------------------------------

    wire reset;

    assign reset = ~CPU_RESETN;


    // ------------------------------------------------------------------------
    // CPU INSTRUCTION BUS
    // ------------------------------------------------------------------------

    wire [31:0] instr_addr;
    wire [31:0] instr_data;


    // ------------------------------------------------------------------------
    // CPU DATA BUS
    // ------------------------------------------------------------------------

    wire [31:0] data_addr;
    wire [31:0] data_wdata;
    wire [3:0]  data_we;
    wire        data_req;
    wire [31:0] data_rdata;


    // ------------------------------------------------------------------------
    // ADDRESS SELECT SIGNALS
    // ------------------------------------------------------------------------

    wire rom_instr_select;
    wire ram_instr_select;

    wire rom_data_select;
    wire ram_data_select;

    wire uart_data_select;
    wire uart_status_select;

    wire gpio_output_select;
    wire gpio_input_select;

    wire timer_select;


    // ------------------------------------------------------------------------
    // INSTRUCTION ADDRESS DECODE
    // ------------------------------------------------------------------------

    assign rom_instr_select =
        (instr_addr >= 32'h0000_0000) &&
        (instr_addr <  32'h0001_0000);

    assign ram_instr_select =
        (instr_addr >= 32'h0001_0000) &&
        (instr_addr <  32'h0002_0000);


    // ------------------------------------------------------------------------
    // DATA ADDRESS DECODE
    // ------------------------------------------------------------------------

    assign rom_data_select =
        (data_addr >= 32'h0000_0000) &&
        (data_addr <  32'h0001_0000);

    assign ram_data_select =
        (data_addr >= 32'h0001_0000) &&
        (data_addr <  32'h0002_0000);


    assign uart_data_select =
        (data_addr == 32'h4000_0000);

    assign uart_status_select =
        (data_addr == 32'h4000_0004);


    assign gpio_output_select =
        (data_addr == 32'h4000_1000);

    assign gpio_input_select =
        (data_addr == 32'h4000_1004);


    assign timer_select =
        (data_addr == 32'h4000_2000);


    // ------------------------------------------------------------------------
    // ROM SIGNALS
    // ------------------------------------------------------------------------

    wire [31:0] rom_instr_data;
    wire [31:0] rom_data_rdata;


    // ------------------------------------------------------------------------
    // RAM SIGNALS
    // ------------------------------------------------------------------------

    wire [31:0] ram_instr_data;
    wire [31:0] ram_data_rdata;

    wire [3:0] ram_we;


    // ------------------------------------------------------------------------
    // CRITICAL RAM WRITE GATING
    //
    // RAM receives write enables ONLY when:
    //
    // 1. CPU has an active request
    // 2. Address belongs to RAM
    // 3. CPU requested a write
    //
    // ------------------------------------------------------------------------

    assign ram_we =
        (data_req && ram_data_select) ?
        data_we :
        4'b0000;


    // ------------------------------------------------------------------------
    // UART SIGNALS
    // ------------------------------------------------------------------------

    wire uart_start;
    wire uart_busy;


    assign uart_start =
        data_req &&
        uart_data_select &&
        (data_we != 4'b0000) &&
        (~uart_busy);


    // ------------------------------------------------------------------------
    // GPIO WRITE SIGNAL
    // ------------------------------------------------------------------------

    wire gpio_write_enable;


    assign gpio_write_enable =
        data_req &&
        gpio_output_select &&
        (data_we != 4'b0000);


    // ------------------------------------------------------------------------
    // CPU
    // ------------------------------------------------------------------------

    hpu_cpu CPU
    (
        .clk         (CLK100MHZ),
        .reset       (reset),

        .instr_addr  (instr_addr),
        .instr_data  (instr_data),

        .data_addr   (data_addr),
        .data_wdata  (data_wdata),
        .data_we     (data_we),
        .data_req    (data_req),

        .data_rdata  (data_rdata)
    );


    // ------------------------------------------------------------------------
    // BOOT ROM
    //
    // Only word addresses [15:2] are passed.
    //
    // This removes unused address-bit warnings.
    // ------------------------------------------------------------------------

    boot_rom BOOT_ROM
    (
        .instr_word_addr (instr_addr[15:2]),
        .instr_rdata     (rom_instr_data),

        .data_word_addr  (data_addr[15:2]),
        .data_rdata      (rom_data_rdata)
    );


    // ------------------------------------------------------------------------
    // SYSTEM RAM
    // ------------------------------------------------------------------------

    system_ram SYSTEM_RAM
    (
        .clk             (CLK100MHZ),

        .instr_word_addr (instr_addr[15:2]),
        .instr_rdata     (ram_instr_data),

        .data_word_addr  (data_addr[15:2]),
        .data_wdata      (data_wdata),
        .we              (ram_we),

        .data_rdata      (ram_data_rdata)
    );


    // ------------------------------------------------------------------------
    // GPIO
    //
    // Only lower 16 bits are physically connected to LEDs.
    //
    // Passing only 16 bits removes the unused upper-bit warning.
    // ------------------------------------------------------------------------

    gpio_unit GPIO
    (
        .clk          (CLK100MHZ),
        .reset        (reset),

        .write_enable (gpio_write_enable),
        .wdata        (data_wdata[15:0]),

        .leds         (LED)
    );


    // ------------------------------------------------------------------------
    // TIMER
    // ------------------------------------------------------------------------

    wire [31:0] timer_value;


    timer_unit TIMER
    (
        .clk         (CLK100MHZ),
        .reset       (reset),

        .timer_value (timer_value)
    );


    // ------------------------------------------------------------------------
    // UART
    // ------------------------------------------------------------------------

    uart_tx UART
    (
        .clk   (CLK100MHZ),
        .reset (reset),

        .start (uart_start),
        .data  (data_wdata[7:0]),

        .tx    (UART_TXD),
        .busy  (uart_busy)
    );


    // ------------------------------------------------------------------------
    // INSTRUCTION READ MUX
    //
    // CPU can execute instructions from:
    //
    // Boot ROM
    // System RAM
    //
    // ------------------------------------------------------------------------

    assign instr_data =

        rom_instr_select ?

            rom_instr_data :

        ram_instr_select ?

            ram_instr_data :

            32'h0000_0013;


    // ------------------------------------------------------------------------
    // DATA READ MUX
    // ------------------------------------------------------------------------

    assign data_rdata =

        rom_data_select ?

            rom_data_rdata :

        ram_data_select ?

            ram_data_rdata :

        uart_status_select ?

            {31'd0, uart_busy} :

        gpio_input_select ?

            {16'd0, SW} :

        timer_select ?

            timer_value :

            32'd0;


endmodule



// ============================================================================
// HPU CPU
//
// Small multi-cycle RV32I-style processor.
//
// Intended role:
//
//   Control
//   Scheduling interface
//   Memory management
//   Peripheral communication
//
// GPU/NPU/ISP will later perform the heavy computation.
// ============================================================================
// HPU CPU
//
// Multi-cycle RV32I-style processor
// ============================================================================

module hpu_cpu
(
    input  wire        clk,
    input  wire        reset,

    // Instruction interface
    output reg  [31:0] instr_addr,
    input  wire [31:0] instr_data,

    // Data interface
    output reg  [31:0] data_addr,
    output reg  [31:0] data_wdata,
    output reg  [3:0]  data_we,
    output reg         data_req,

    input  wire [31:0] data_rdata
);


    // =========================================================================
    // STATES
    // =========================================================================

    localparam STATE_FETCH       = 4'd0;
    localparam STATE_DECODE      = 4'd1;
    localparam STATE_ALU         = 4'd2;
    localparam STATE_MEM_READ    = 4'd3;
    localparam STATE_MEM_CAPTURE = 4'd4;
    localparam STATE_MEM_WRITE   = 4'd5;
    localparam STATE_MEM_DONE    = 4'd6;
    localparam STATE_WRITEBACK   = 4'd7;


    reg [3:0] state;


    // =========================================================================
    // INTERNAL REGISTERS
    // =========================================================================

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


    // =========================================================================
    // INSTRUCTION FIELDS
    // =========================================================================

    wire [6:0] opcode;
    wire [4:0] rd;
    wire [2:0] funct3;
    wire [4:0] rs1;
    wire [4:0] rs2;
    wire [6:0] funct7;

    assign opcode = instruction[6:0];
    assign rd     = instruction[11:7];
    assign funct3 = instruction[14:12];
    assign rs1    = instruction[19:15];
    assign rs2    = instruction[24:20];
    assign funct7 = instruction[31:25];


    // =========================================================================
    // REGISTER READ
    // =========================================================================

    wire [31:0] rs1_value;
    wire [31:0] rs2_value;

    assign rs1_value =
        (rs1 == 5'd0) ? 32'd0 : registers[rs1];

    assign rs2_value =
        (rs2 == 5'd0) ? 32'd0 : registers[rs2];


    // =========================================================================
    // IMMEDIATES
    // =========================================================================

    wire [31:0] immediate_i;
    wire [31:0] immediate_s;
    wire [31:0] immediate_b;
    wire [31:0] immediate_u;
    wire [31:0] immediate_j;


    assign immediate_i =
    {
        {20{instruction[31]}},
        instruction[31:20]
    };


    assign immediate_s =
    {
        {20{instruction[31]}},
        instruction[31:25],
        instruction[11:7]
    };


    assign immediate_b =
    {
        {19{instruction[31]}},
        instruction[31],
        instruction[7],
        instruction[30:25],
        instruction[11:8],
        1'b0
    };


    assign immediate_u =
    {
        instruction[31:12],
        12'b0
    };


    assign immediate_j =
    {
        {11{instruction[31]}},
        instruction[31],
        instruction[19:12],
        instruction[20],
        instruction[30:21],
        1'b0
    };


    // =========================================================================
    // EFFECTIVE MEMORY ADDRESSES
    // =========================================================================

    wire [31:0] load_effective_address;
    wire [31:0] store_effective_address;

    assign load_effective_address =
        rs1_value + immediate_i;

    assign store_effective_address =
        rs1_value + immediate_s;


    // =========================================================================
    // SIGN EXTENSION FUNCTIONS
    // =========================================================================

    function [31:0] sign_extend_byte;

        input [7:0] value;

        begin

            sign_extend_byte =
            {
                {24{value[7]}},
                value
            };

        end

    endfunction


    function [31:0] sign_extend_half;

        input [15:0] value;

        begin

            sign_extend_half =
            {
                {16{value[15]}},
                value
            };

        end

    endfunction


    // =========================================================================
    // CPU STATE MACHINE
    // =========================================================================

    always @(posedge clk)
    begin

        if (reset)
        begin

            pc <= 32'd0;

            instruction <= 32'h00000013;

            instr_addr <= 32'd0;

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


            for (i = 0; i < 32; i = i + 1)
            begin

                registers[i] <= 32'd0;

            end

        end

        else
        begin

            // x0 is permanently zero
            registers[0] <= 32'd0;


            case (state)


                // =============================================================
                // FETCH
                // =============================================================

                STATE_FETCH:
                begin

                    instr_addr <= pc;

                    data_req <= 1'b0;
                    data_we <= 4'b0000;

                    state <= STATE_DECODE;

                end


                // =============================================================
                // DECODE
                // =============================================================

                STATE_DECODE:
                begin

                    instruction <= instr_data;

                    pc <= pc + 32'd4;

                    state <= STATE_ALU;

                end


                // =============================================================
                // EXECUTE
                // =============================================================

                STATE_ALU:
                begin

                    case (opcode)


                        // -----------------------------------------------------
                        // LUI
                        // -----------------------------------------------------

                        7'b0110111:
                        begin

                            if (rd != 5'd0)
                                registers[rd] <= immediate_u;

                            state <= STATE_FETCH;

                        end


                        // -----------------------------------------------------
                        // AUIPC
                        // -----------------------------------------------------

                        7'b0010111:
                        begin

                            if (rd != 5'd0)
                                registers[rd] <=
                                    (pc - 32'd4) + immediate_u;

                            state <= STATE_FETCH;

                        end


                        // -----------------------------------------------------
                        // OP-IMM
                        // -----------------------------------------------------

                        7'b0010011:
                        begin

                            destination_register <= rd;
                            load_operation <= 1'b0;


                            case (funct3)

                                // ADDI
                                3'b000:
                                    alu_result <=
                                        rs1_value + immediate_i;


                                // SLTI
                                3'b010:
                                begin

                                    if ($signed(rs1_value) <
                                        $signed(immediate_i))

                                        alu_result <= 32'd1;

                                    else

                                        alu_result <= 32'd0;

                                end


                                // SLTIU
                                3'b011:
                                begin

                                    if (rs1_value < immediate_i)

                                        alu_result <= 32'd1;

                                    else

                                        alu_result <= 32'd0;

                                end


                                // XORI
                                3'b100:
                                    alu_result <=
                                        rs1_value ^ immediate_i;


                                // ORI
                                3'b110:
                                    alu_result <=
                                        rs1_value | immediate_i;


                                // ANDI
                                3'b111:
                                    alu_result <=
                                        rs1_value & immediate_i;


                                // SLLI
                                3'b001:
                                    alu_result <=
                                        rs1_value <<
                                        instruction[24:20];


                                // SRLI / SRAI
                                3'b101:
                                begin

                                    if (instruction[30])

                                        alu_result <=
                                            $signed(rs1_value) >>>
                                            instruction[24:20];

                                    else

                                        alu_result <=
                                            rs1_value >>
                                            instruction[24:20];

                                end


                                default:
                                    alu_result <= 32'd0;

                            endcase


                            state <= STATE_WRITEBACK;

                        end


                        // -----------------------------------------------------
                        // REGISTER ALU
                        // -----------------------------------------------------

                        7'b0110011:
                        begin

                            destination_register <= rd;
                            load_operation <= 1'b0;


                            case (funct3)

                                // ADD / SUB
                                3'b000:
                                begin

                                    if (instruction[30])

                                        alu_result <=
                                            rs1_value - rs2_value;

                                    else

                                        alu_result <=
                                            rs1_value + rs2_value;

                                end


                                // SLL
                                3'b001:
                                    alu_result <=
                                        rs1_value <<
                                        rs2_value[4:0];


                                // SLT
                                3'b010:
                                begin

                                    if ($signed(rs1_value) <
                                        $signed(rs2_value))

                                        alu_result <= 32'd1;

                                    else

                                        alu_result <= 32'd0;

                                end


                                // SLTU
                                3'b011:
                                begin

                                    if (rs1_value < rs2_value)

                                        alu_result <= 32'd1;

                                    else

                                        alu_result <= 32'd0;

                                end


                                // XOR
                                3'b100:
                                    alu_result <=
                                        rs1_value ^ rs2_value;


                                // SRL / SRA
                                3'b101:
                                begin

                                    if (instruction[30])

                                        alu_result <=
                                            $signed(rs1_value) >>>
                                            rs2_value[4:0];

                                    else

                                        alu_result <=
                                            rs1_value >>
                                            rs2_value[4:0];

                                end


                                // OR
                                3'b110:
                                    alu_result <=
                                        rs1_value | rs2_value;


                                // AND
                                3'b111:
                                    alu_result <=
                                        rs1_value & rs2_value;


                                default:
                                    alu_result <= 32'd0;

                            endcase


                            state <= STATE_WRITEBACK;

                        end


                        // -----------------------------------------------------
                        // LOAD
                        // -----------------------------------------------------

                        7'b0000011:
                        begin

                            // Align address to 32-bit word boundary
                            data_addr <=
                            {
                                load_effective_address[31:2],
                                2'b00
                            };

                            // Preserve original byte position
                            memory_offset <=
                                load_effective_address[1:0];

                            destination_register <= rd;

                            memory_funct3 <= funct3;

                            load_operation <= 1'b1;

                            data_we <= 4'b0000;

                            data_req <= 1'b1;

                            state <= STATE_MEM_READ;

                        end


                        // -----------------------------------------------------
                        // STORE
                        // -----------------------------------------------------

                        7'b0100011:
                        begin

                            // Align address to word boundary
                            data_addr <=
                            {
                                store_effective_address[31:2],
                                2'b00
                            };

                            // Save byte offset
                            memory_offset <=
                                store_effective_address[1:0];

                            memory_funct3 <= funct3;

                            load_operation <= 1'b0;

                            data_req <= 1'b1;


                            case (funct3)


                                // SB
                                3'b000:
                                begin

                                    case (store_effective_address[1:0])

                                        2'b00:
                                            data_wdata <=
                                            {
                                                24'd0,
                                                rs2_value[7:0]
                                            };


                                        2'b01:
                                            data_wdata <=
                                            {
                                                16'd0,
                                                rs2_value[7:0],
                                                8'd0
                                            };


                                        2'b10:
                                            data_wdata <=
                                            {
                                                8'd0,
                                                rs2_value[7:0],
                                                16'd0
                                            };


                                        2'b11:
                                            data_wdata <=
                                            {
                                                rs2_value[7:0],
                                                24'd0
                                            };

                                    endcase

                                end


                                // SH
                                3'b001:
                                begin

                                    if (store_effective_address[1])

                                        data_wdata <=
                                        {
                                            rs2_value[15:0],
                                            16'd0
                                        };

                                    else

                                        data_wdata <=
                                        {
                                            16'd0,
                                            rs2_value[15:0]
                                        };

                                end


                                // SW
                                3'b010:
                                begin

                                    data_wdata <=
                                        rs2_value;

                                end


                                default:
                                    data_wdata <= 32'd0;

                            endcase


                            state <= STATE_MEM_WRITE;

                        end


                        // -----------------------------------------------------
                        // BRANCH
                        // -----------------------------------------------------

                        7'b1100011:
                        begin

                            case (funct3)

                                // BEQ
                                3'b000:
                                begin

                                    if (rs1_value == rs2_value)

                                        pc <=
                                            (pc - 32'd4) +
                                            immediate_b;

                                end


                                // BNE
                                3'b001:
                                begin

                                    if (rs1_value != rs2_value)

                                        pc <=
                                            (pc - 32'd4) +
                                            immediate_b;

                                end


                                // BLT
                                3'b100:
                                begin

                                    if ($signed(rs1_value) <
                                        $signed(rs2_value))

                                        pc <=
                                            (pc - 32'd4) +
                                            immediate_b;

                                end


                                // BGE
                                3'b101:
                                begin

                                    if ($signed(rs1_value) >=
                                        $signed(rs2_value))

                                        pc <=
                                            (pc - 32'd4) +
                                            immediate_b;

                                end


                                // BLTU
                                3'b110:
                                begin

                                    if (rs1_value < rs2_value)

                                        pc <=
                                            (pc - 32'd4) +
                                            immediate_b;

                                end


                                // BGEU
                                3'b111:
                                begin

                                    if (rs1_value >= rs2_value)

                                        pc <=
                                            (pc - 32'd4) +
                                            immediate_b;

                                end


                                default:
                                begin
                                end

                            endcase


                            state <= STATE_FETCH;

                        end


                        // -----------------------------------------------------
                        // JAL
                        // -----------------------------------------------------

                        7'b1101111:
                        begin

                            if (rd != 5'd0)
                                registers[rd] <= pc;

                            pc <=
                                (pc - 32'd4) +
                                immediate_j;

                            state <= STATE_FETCH;

                        end


                        // -----------------------------------------------------
                        // JALR
                        // -----------------------------------------------------

                        7'b1100111:
                        begin

                            if (rd != 5'd0)
                                registers[rd] <= pc;

                            pc <=
                            (
                                rs1_value + immediate_i
                            )
                            &
                            32'hFFFF_FFFE;

                            state <= STATE_FETCH;

                        end


                        // -----------------------------------------------------
                        // UNKNOWN / NOP
                        // -----------------------------------------------------

                        default:
                        begin

                            state <= STATE_FETCH;

                        end

                    endcase

                end


                // =============================================================
                // MEMORY READ
                // =============================================================

                STATE_MEM_READ:
                begin

                    data_req <= 1'b1;

                    data_we <= 4'b0000;

                    state <= STATE_MEM_CAPTURE;

                end


                // =============================================================
                // MEMORY CAPTURE
                // =============================================================

                STATE_MEM_CAPTURE:
                begin

                    data_req <= 1'b0;

                    data_we <= 4'b0000;


                    case (memory_funct3)


                        // LB
                        3'b000:
                        begin

                            case (memory_offset)

                                2'b00:
                                    memory_result <=
                                        sign_extend_byte(
                                            data_rdata[7:0]
                                        );

                                2'b01:
                                    memory_result <=
                                        sign_extend_byte(
                                            data_rdata[15:8]
                                        );

                                2'b10:
                                    memory_result <=
                                        sign_extend_byte(
                                            data_rdata[23:16]
                                        );

                                2'b11:
                                    memory_result <=
                                        sign_extend_byte(
                                            data_rdata[31:24]
                                        );

                            endcase

                        end


                        // LH
                        3'b001:
                        begin

                            if (memory_offset[1])

                                memory_result <=
                                    sign_extend_half(
                                        data_rdata[31:16]
                                    );

                            else

                                memory_result <=
                                    sign_extend_half(
                                        data_rdata[15:0]
                                    );

                        end


                        // LW
                        3'b010:
                        begin

                            memory_result <=
                                data_rdata;

                        end


                        // LBU
                        3'b100:
                        begin

                            case (memory_offset)

                                2'b00:
                                    memory_result <=
                                    {
                                        24'd0,
                                        data_rdata[7:0]
                                    };

                                2'b01:
                                    memory_result <=
                                    {
                                        24'd0,
                                        data_rdata[15:8]
                                    };

                                2'b10:
                                    memory_result <=
                                    {
                                        24'd0,
                                        data_rdata[23:16]
                                    };

                                2'b11:
                                    memory_result <=
                                    {
                                        24'd0,
                                        data_rdata[31:24]
                                    };

                            endcase

                        end


                        // LHU
                        3'b101:
                        begin

                            if (memory_offset[1])

                                memory_result <=
                                {
                                    16'd0,
                                    data_rdata[31:16]
                                };

                            else

                                memory_result <=
                                {
                                    16'd0,
                                    data_rdata[15:0]
                                };

                        end


                        default:
                        begin

                            memory_result <= 32'd0;

                        end

                    endcase


                    state <= STATE_WRITEBACK;

                end


                // =============================================================
                // MEMORY WRITE
                // =============================================================

                STATE_MEM_WRITE:
                begin

                    data_req <= 1'b1;


                    case (memory_funct3)


                        // SB
                        3'b000:
                        begin

                            case (memory_offset)

                                2'b00:
                                    data_we <= 4'b0001;

                                2'b01:
                                    data_we <= 4'b0010;

                                2'b10:
                                    data_we <= 4'b0100;

                                2'b11:
                                    data_we <= 4'b1000;

                            endcase

                        end


                        // SH
                        3'b001:
                        begin

                            if (memory_offset[1])

                                data_we <= 4'b1100;

                            else

                                data_we <= 4'b0011;

                        end


                        // SW
                        3'b010:
                        begin

                            data_we <= 4'b1111;

                        end


                        default:
                        begin

                            data_we <= 4'b0000;

                        end

                    endcase


                    state <= STATE_MEM_DONE;

                end


                // =============================================================
                // MEMORY DONE
                // =============================================================

                STATE_MEM_DONE:
                begin

                    data_req <= 1'b0;

                    data_we <= 4'b0000;

                    state <= STATE_FETCH;

                end


                // =============================================================
                // WRITEBACK
                // =============================================================

                STATE_WRITEBACK:
                begin

                    if (destination_register != 5'd0)
                    begin

                        if (load_operation)

                            registers[destination_register] <=
                                memory_result;

                        else

                            registers[destination_register] <=
                                alu_result;

                    end


                    load_operation <= 1'b0;

                    state <= STATE_FETCH;

                end


                // =============================================================
                // DEFAULT
                // =============================================================

                default:
                begin

                    state <= STATE_FETCH;

                end


            endcase

        end

    end


endmodule


// ============================================================================
// ============================================================================
// BOOT ROM
//
// 16K x 32-bit
//
// Two independent read ports:
//
//   Instruction read port
//   CPU data read port
//
// Address input is a word address.
// ============================================================================

// ============================================================================
// BOOT ROM
//
// 16K x 32-bit
//
// Dual read ports:
//   1. Instruction fetch
//   2. CPU data read
//
// HPU PHASE 1C - STABLE CPU / SOC DIAGNOSTIC
// ============================================================================

module boot_rom
(
    input  wire [13:0] instr_word_addr,
    output reg  [31:0] instr_rdata,

    input  wire [13:0] data_word_addr,
    output reg  [31:0] data_rdata
);

    reg [31:0] rom [0:16383];

    integer j;


    // ------------------------------------------------------------------------
    // INSTRUCTION READ PORT
    // ------------------------------------------------------------------------

    always @(*)
    begin
        instr_rdata = rom[instr_word_addr];
    end


    // ------------------------------------------------------------------------
    // DATA READ PORT
    // ------------------------------------------------------------------------

    always @(*)
    begin
        data_rdata = rom[data_word_addr];
    end


    // ------------------------------------------------------------------------
    // BOOT / DIAGNOSTIC PROGRAM
    //
    // Register allocation:
    //
    // x1  GPIO base
    // x2  RAM base
    // x3  value A
    // x4  value B
    // x5  ADD result
    // x6  loaded RAM value
    // x7  expected value
    // x8  timer base
    // x9  timer value
    // x18 diagnostic output
    //
    // Final PASS:
    //
    // LED = C0DE
    //
    // Failure:
    //
    // LED = DEAD
    // ------------------------------------------------------------------------

    initial
    begin

        // ------------------------------------------------------------
        // Default all ROM locations to NOP
        // ------------------------------------------------------------

        for (j = 0; j < 16384; j = j + 1)
        begin
            rom[j] = 32'h0000_0013;
        end


        // ============================================================
        // INITIALIZATION
        // ============================================================


        // rom[0]
        //
        // LUI x1, 0x40001
        //
        // x1 = 0x40001000
        // GPIO output base

        rom[0] = 32'h4000_10B7;


        // rom[1]
        //
        // LUI x2, 0x00010
        //
        // x2 = 0x00010000
        // System RAM base

        rom[1] = 32'h0001_0137;


        // rom[2]
        //
        // ADDI x3, x0, 0x100

        rom[2] = 32'h1000_0193;


        // rom[3]
        //
        // ADDI x4, x0, 0x055

        rom[3] = 32'h0550_0213;


        // ============================================================
        // ALU TEST
        //
        // x5 = x3 + x4
        //
        // 0x100 + 0x055 = 0x155
        // ============================================================


        // rom[4]
        //
        // ADD x5, x3, x4

        rom[4] = 32'h0041_82B3;


        // ============================================================
        // RAM STORE
        //
        // Store x5 at RAM base
        // ============================================================


        // rom[5]
        //
        // SW x5, 0(x2)

        rom[5] = 32'h0051_2023;


        // ============================================================
        // RAM LOAD
        //
        // Load RAM base into x6
        // ============================================================


        // rom[6]
        //
        // LW x6, 0(x2)

        rom[6] = 32'h0001_2303;


        // ============================================================
        // EXPECTED VALUE
        // ============================================================


        // rom[7]
        //
        // ADDI x7, x0, 0x155

        rom[7] = 32'h1550_0393;


        // ============================================================
        // CHECK ALU + RAM RESULT
        //
        // BEQ x5, x7, +8
        //
        // If equal -> skip FAIL jump
        // ============================================================


        // rom[8]

        rom[8] = 32'h0072_8463;


        // rom[9]
        //
        // JAL x0, FAIL
        //
        // FAIL is rom[20]
        //
        // Offset:
        // (20 - 9) * 4 = 44 = 0x2C

        rom[9] = 32'h02C0_006F;


        // ============================================================
        // CHECK RAM LOAD
        //
        // BEQ x6, x7, +8
        // ============================================================


        // rom[10]

        rom[10] = 32'h0073_0463;


        // rom[11]
        //
        // JAL x0, FAIL
        //
        // (20 - 11) * 4 = 36 = 0x24

        rom[11] = 32'h0240_006F;


        // ============================================================
        // TIMER TEST
        //
        // Read timer and ensure it is non-zero
        // ============================================================


        // rom[12]
        //
        // LUI x8, 0x40002
        //
        // x8 = 0x40002000

        rom[12] = 32'h4000_2437;


        // rom[13]
        //
        // LW x9, 0(x8)

        rom[13] = 32'h0004_2483;


        // rom[14]
        //
        // BNE x9, x0, +8
        //
        // If timer != 0 -> PASS

        rom[14] = 32'h0004_9463;


        // rom[15]
        //
        // JAL x0, FAIL
        //
        // (20 - 15) * 4 = 20 = 0x14

        rom[15] = 32'h0140_006F;


        // ============================================================
        // PASS
        // ============================================================


        // rom[16]
        //
        // LUI x18, 0x0000C
        //
        // x18 = 0x0000C000

        rom[16] = 32'h0000_C937;


        // rom[17]
        //
        // ADDI x18, x18, 0x0DE
        //
        // x18 = 0x0000C0DE

        rom[17] = 32'h0DE9_0913;


        // rom[18]
        //
        // SW x18, 0(x1)
        //
        // LED = C0DE

        rom[18] = 32'h0120_A023;


        // rom[19]
        //
        // PASS LOOP

        rom[19] = 32'h0000_006F;


        // ============================================================
        // FAIL
        // ============================================================


        // rom[20]
        //
        // LUI x18, 0x0000E
        //
        // x18 = 0x0000E000

        rom[20] = 32'h0000_E937;


        // rom[21]
        //
        // ADDI x18, x18, -339
        //
        // 0xE000 - 0x153 = 0xDEAD

        rom[21] = 32'hEAD9_0913;


        // rom[22]
        //
        // SW x18, 0(x1)
        //
        // LED = DEAD

        rom[22] = 32'h0120_A023;


        // rom[23]
        //
        // FAIL LOOP

        rom[23] = 32'h0000_006F;

    end

endmodule



// ============================================================================
// HPU SYSTEM RAM
// ============================================================================
//
// Artix-7 BRAM-friendly dual-port system memory.
//
// Capacity:
//     16,384 words × 32 bits
//     = 65,536 bytes
//     = 64 KiB
//
// Port A:
//     Instruction fetch
//
// Port B:
//     CPU data read/write
//
// Vivado is directed to infer Block RAM instead of LUT RAM.
//
// ============================================================================

module system_ram
(
    input  wire        clk,

    // ------------------------------------------------------------------------
    // INSTRUCTION PORT
    // ------------------------------------------------------------------------

    input  wire [13:0] instr_word_addr,
    output reg  [31:0] instr_rdata,


    // ------------------------------------------------------------------------
    // DATA PORT
    // ------------------------------------------------------------------------

    input  wire [13:0] data_word_addr,
    input  wire [31:0] data_wdata,
    input  wire [3:0]  we,

    output reg  [31:0] data_rdata
);


    // ------------------------------------------------------------------------
    // MEMORY ARRAY
    //
    // 16,384 × 32 bits
    //
    // The ram_style attribute tells Vivado to implement this memory using
    // dedicated Artix-7 Block RAM resources.
    // ------------------------------------------------------------------------

    (* ram_style = "block" *)
    reg [31:0] memory [0:16383];


    // ------------------------------------------------------------------------
    // PORT A : INSTRUCTION FETCH
    // ------------------------------------------------------------------------

    always @(posedge clk)
    begin
        instr_rdata <= memory[instr_word_addr];
    end


    // ------------------------------------------------------------------------
    // PORT B : DATA READ / WRITE
    // ------------------------------------------------------------------------

    always @(posedge clk)
    begin

        // Byte-enable write support.

        if (we[0])
            memory[data_word_addr][7:0] <=
                data_wdata[7:0];

        if (we[1])
            memory[data_word_addr][15:8] <=
                data_wdata[15:8];

        if (we[2])
            memory[data_word_addr][23:16] <=
                data_wdata[23:16];

        if (we[3])
            memory[data_word_addr][31:24] <=
                data_wdata[31:24];


        // Synchronous BRAM read.

        data_rdata <= memory[data_word_addr];

    end


endmodule

// ============================================================================
// GPIO UNIT
//
// Output:
//
//   LED[15:0]
//
// Input:
//
//   Switches are read directly by the memory read multiplexer in hpu_top.
//
// GPIO therefore only needs the LED write register.
// ============================================================================

module gpio_unit
(
    input  wire        clk,
    input  wire        reset,

    input  wire        write_enable,
    input  wire [15:0] wdata,

    output reg  [15:0] leds
);


    always @(posedge clk)
    begin

        if (reset)

            leds <= 16'd0;

        else
        begin

            if (write_enable)

                leds <= wdata;

        end

    end


endmodule



// ============================================================================
// TIMER UNIT
//
// Free-running 32-bit cycle counter.
// ============================================================================

module timer_unit
(
    input  wire        clk,
    input  wire        reset,

    output reg  [31:0] timer_value
);


    always @(posedge clk)
    begin

        if (reset)

            timer_value <= 32'd0;

        else

            timer_value <=
                timer_value +
                32'd1;

    end


endmodule



// ============================================================================
// UART TRANSMITTER
//
// Configuration:
//
//   Clock      : 100 MHz
//   Baud Rate  : approximately 115200
//   Data Bits  : 8
//   Stop Bits  : 1
//   Parity     : None
//
// ============================================================================

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


    always @(posedge clk)
    begin


        if (reset)
        begin

            tx <= 1'b1;

            busy <= 1'b0;

            shift_register <=
                10'b1111111111;

            clock_counter <=
                10'd0;

            bit_counter <=
                4'd0;

        end


        else
        begin


            // ------------------------------------------------------------
            // IDLE / START
            // ------------------------------------------------------------

            if (!busy)
            begin

                tx <= 1'b1;


                if (start)
                begin

                    busy <= 1'b1;

                    shift_register <=
                    {
                        1'b1,
                        data,
                        1'b0
                    };

                    clock_counter <=
                        10'd0;

                    bit_counter <=
                        4'd0;

                    // Start bit begins immediately

                    tx <=
                        1'b0;

                end

            end


            // ------------------------------------------------------------
            // TRANSMISSION
            // ------------------------------------------------------------

            else
            begin


                if (clock_counter ==
                    CLKS_PER_BIT - 1)
                begin

                    clock_counter <=
                        10'd0;


                    shift_register <=
                    {
                        1'b1,
                        shift_register[9:1]
                    };


                    tx <=
                        shift_register[1];


                    if (bit_counter == 4'd9)
                    begin

                        busy <=
                            1'b0;

                        tx <=
                            1'b1;

                    end


                    bit_counter <=
                        bit_counter +
                        4'd1;

                end


                else
                begin

                    clock_counter <=
                        clock_counter +
                        10'd1;

                end


            end


        end


    end


endmodule
