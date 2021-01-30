`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2020/12/28 15:16:00
// Design Name: 
// Module Name: cpu
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module pc(clk, d, q, reset);   //pc register
    input clk, reset;
    input [31:0] d;
    output [31:0] q;
    reg [31:0] this_pc;
    assign q = this_pc;
    initial this_pc = 32'b0;
    always @(posedge clk or posedge reset)
    begin
        this_pc = d;
        if(reset)
            this_pc = 32'b0;
    end
endmodule

module inst_mem(addr, instr);   //instruction memory
    input [31:0] addr;
    output [31:0] instr;
    reg [7:0] inst[0:7];
    reg [31:0] temp;
    assign instr = temp;
    initial begin
        inst[0] = 8'b00010011;    //addi a4, x0, 1
        inst[1] = 8'b00000111;
        inst[2] = 8'b00010000;
        inst[3] = 8'b00000000;
        inst[4] = 8'b00100011;    //sw a4, x0(0)
        inst[5] = 8'b00100000;
        inst[6] = 8'b11100000;
        inst[7] = 8'h00000000;
    end
    always @(*)
    begin
        temp[7:0] = inst[addr];
        temp[15:8] = inst[addr + 1];
        temp[23:16] = inst[addr + 2];
        temp[31:24] = inst[addr + 3];
    end
endmodule

module add1(b, c);  //the adder for pc + 4
    input [31:0] b;
    output [31:0] c;
    assign c = b + 4;
endmodule

module fi(clk, reset, pc_out, npc_out, npc_in, ir_out, j);
    input clk, reset, j;
    input [31:0] npc_in;
    output [31:0] pc_out;
    output [31:0] npc_out;
    output [31:0] ir_out;
    wire [31:0] add_out;
    wire [31:0] pc_in;
    reg j_reg;
    pc pc_reg(clk, pc_in, pc_out, reset);
    add1 add1_adder(pc_out, add_out);
    inst_mem inst_memory(pc_out, ir_out);
    assign pc_in = j_reg? npc_in : add_out;
    assign npc_out = add_out;
    always @(j)
    begin
        if(j == 1)
            j_reg = 1;
        else
            j_reg = 0;
    end
endmodule

module fi_id(clk, pc_in, npc1_in, ir_in, dead_in,        //reg betw fetch_inst & inst_decode
                  pc_out, npc1_out, ir_out, dead_out);
    input clk, dead_in;
    input [31:0] pc_in;
    input [31:0] npc1_in;
    input [31:0] ir_in;
    output reg dead_out;
    output reg [31:0] pc_out;
    output reg [31:0] npc1_out;
    output reg [31:0] ir_out;
    always @(posedge clk)
    begin
        pc_out = pc_in;
        npc1_out = npc1_in;
        ir_out = ir_in;
        dead_out = dead_in;
    end
endmodule

module rf(clk, regwr, r_reg1, r_reg2, w_reg, w_data, r_data1, r_data2); //register file
    input clk, regwr;
    input [4:0] r_reg1;
    input [4:0] r_reg2;
    input [4:0] w_reg;
    input [31:0] w_data;
    output [31:0] r_data1;
    reg [31:0] temp1;
    output [31:0] r_data2;
    reg [31:0] temp2;
    reg [31:0] reg_data[1:31];
    assign r_data1 = temp1;
    assign r_data2 = temp2;
    always @(posedge clk)
    begin
        if(regwr)
        begin
            if(w_reg != 5'b0)
                reg_data[w_reg] = w_data;
        end
    end
    always @(negedge clk)
    begin
        if(r_reg1 != 5'b0)
            temp1 = reg_data[r_reg1];
        else
            temp1 = 32'b0;
        if(r_reg2 != 5'b0)
            temp2 = reg_data[r_reg2];
        else
            temp2 = 32'b0;
    end
endmodule

module sigext(inst, imm32);     //sigext
    input [31:0] inst;
    output reg signed [31:0] imm32;
    reg signed [4:0] shamt;
    reg signed [11:0] imm12;
    reg signed [12:0] imm13;
    reg signed [20:0] imm21;
    always @(*)
    begin
        case(inst[6:0])
        (7'b0110111 || 7'b0010111):     //lui & auipc
        begin
            imm32[31:12] = inst[31:12];
            imm32[11:0] = 12'b0;
        end
        7'b1101111:     //jal
        begin
            imm21[20] = inst[31];
            imm21[10:1] = inst[30:21];
            imm21[11] = inst[20];
            imm21[19:12] = inst[19:12];
            imm21[0] = 1'b0;
            imm32 = imm21;
        end
        (7'b1100111 || 7'b0000011):     //jalr & lw
        begin
            imm12[11:0] = inst[31:20];
            imm32 = imm12;
        end
        7'b1100011:     //b
        begin
            imm13[12] = inst[31];
            imm13[10:5] = inst[30:24];
            imm13[4:1] = inst[11:8];
            imm13[11] = inst[7];
            imm13[0] = 1'b0;
            imm32 = imm13;
        end
        7'b0100011:     //sw
        begin
            imm12[11:5] = inst[31:25];
            imm12[4:0] = inst[11:7];
            imm32 = imm12;
        end
        7'b0010011:     //arthmetic & logic with imm
        begin
            if(inst[13:12] == 2'b01)    //sl & sr
            begin
                shamt[4:0] = inst[24:20];
                imm32 = shamt;
            end
            else    //other, same as lw
            begin
                imm12[11:0] = inst[31:20];
                imm32 = imm12;
            end
        end
        default: imm32 = 32'bz;
        endcase
    end
endmodule

module id(clk, pc_in, npc1_in, ir_in, regwr, w_reg, w_data,
               pc_out, npc1_out, ir_out, rs1_out, rs2_out, imm32_out);
    input clk, regwr;
    input [31:0] pc_in;
    input [31:0] npc1_in;
    input [31:0] ir_in;
    input [4:0] w_reg;      //write back reg
    input [31:0] w_data;    //write back data
    output [31:0] pc_out;
    output [31:0] npc1_out;
    output [31:0] ir_out;
    output [31:0] rs1_out;
    output [31:0] rs2_out;
    output [31:0] imm32_out;
    rf rf_reg(clk, regwr, ir_in[19:15], ir_in[24:20], w_reg, w_data, rs1_out, rs2_out);
    sigext sigext_imm(ir_in, imm32_out);
    assign ir_out = ir_in;
    assign pc_out = pc_in;
    assign npc1_out = npc1_in;
endmodule

module id_ex(clk, pc_in, npc1_in, ir_in, rs1_in, rs2_in, imm32_in, dead_in,      //reg betw inst_decode & exec
                  pc_out, npc1_out, ir_out, rs1_out, rs2_out, imm32_out, dead_out); 
    input clk, dead_in;
    input [31:0] pc_in;
    input [31:0] npc1_in;
    input [31:0] ir_in;
    input [31:0] rs1_in;
    input [31:0] rs2_in;
    input [31:0] imm32_in;
    output reg dead_out;
    output reg [31:0] pc_out;
    output reg [31:0] npc1_out;
    output reg [31:0] ir_out;
    output reg [31:0] rs1_out;
    output reg [31:0] rs2_out;
    output reg [31:0] imm32_out;
    always @(posedge clk)
    begin
        pc_out = pc_in;
        npc1_out = npc1_in;
        ir_out = ir_in;
        rs1_out = rs1_in;
        rs2_out = rs2_in;
        imm32_out = imm32_in;
        dead_out = dead_in;
    end
endmodule

module alu(a, b, c, flag, op);   //alu
    input signed [31:0] a;
    input signed [31:0] b;
    input [3:0] op;
    output reg signed [31:0] c;
    output reg [1:0] flag;
    always @(*)
    begin
        flag = 2'b11;   //lt & g
        case(op)
            4'b0010: c = a + b;
            4'b0011: 
            begin
                c = a - b;
                if(c > 0)
                    flag = 2'b01;
                else if(c < 0)
                    flag = 2'b10;
                else
                    flag = 2'b00;
            end
            4'b0100: c = a & b;
            4'b0101: c = a | b;
            4'b1000: c = a ^ b;
            4'b1010: c = a << b;
            4'b1011: c = a >> b;
            4'b1100: c = a >>> b;
        endcase
    end
endmodule

module alucu(inst, aluop);
    input [31:0] inst;
    output reg [3:0] aluop;
    always @(*)
    begin
        if(inst[6:0] == 7'b1100011 ||   //b
           (inst[6:0] == 7'b0110011 && inst[14:12] == 3'b0 && inst[31:25] == 7'b0100000))   //sub
            aluop = 4'b0011;
        else if((inst[6:0] == 7'b0110011 || inst[6:0] == 7'b0010011) && inst[14:12] == 3'b111)  //and
            aluop = 4'b0100;
        else if((inst[6:0] == 7'b0110011 || inst[6:0] == 7'b0010011) && inst[14:12] == 3'b110)  //or
            aluop = 4'b0101;
        else if((inst[6:0] == 7'b0110011 || inst[6:0] == 7'b0010011) && inst[14:12] == 3'b100)  //xor
            aluop = 4'b1000;
        else if((inst[6:0] == 7'b0110011 || inst[6:0] == 7'b0010011) && inst[14:12] == 3'b001)  //sll
            aluop = 4'b1010;
        else if((inst[6:0] == 7'b0110011 || inst[6:0] == 7'b0010011) && inst[14:12] == 3'b101   //srl
                 && inst[31:25] == 7'b0000000)
            aluop = 4'b1011;
        else if((inst[6:0] == 7'b0110011 || inst[6:0] == 7'b0010011) && inst[14:12] == 3'b101   //srl
                 && inst[31:25] == 7'b0100000)
            aluop = 4'b1100;
        else
            aluop = 4'b0010;
    end
endmodule

module add2(a, b, c);
    input [31:0] a;
    input [31:0] b;
    output [31:0] c;
    assign c = a + b;
endmodule

module ex(clk, pc_in, npc1_in, ir_in, rs1_in, rs2_in, imm32_in,
               npc2_out, ir_out, data_out, addr_out, flag_out);
    input clk;
    input [31:0] pc_in;
    input [31:0] npc1_in;
    input [31:0] ir_in;
    input [31:0] rs1_in;
    input [31:0] rs2_in;
    input [31:0] imm32_in;
    output wire [31:0] npc2_out;
    output reg [31:0] ir_out;
    output reg [31:0] data_out;
    output reg [31:0] addr_out;
    output wire [1:0] flag_out;
    wire [3:0] aluop;
    reg [31:0] aluin1;
    reg [31:0] aluin2;
    reg [31:0] pcoffset;
    wire [31:0] aluout;
    alu alu_al(aluin1, aluin2, aluout, flag_out, aluop);
    alucu alucu_ctrl(ir_in, aluop);
    add2 add2_adder(pc_in, pcoffset, npc2_out);
    always @(*)
    begin
        ir_out = ir_in;
        case(ir_in[6:0])
        (7'b0110111 || 7'b010111):     //lui & auipc
        begin
            aluin1 = pc_in;
            aluin2 = imm32_in;
            data_out = aluout;
        end
        7'b1101111:     //jal
        begin
            data_out = npc1_in;
            pcoffset = imm32_in;
        end
        7'b1100111:     //jalr
        begin
            data_out = npc1_in;
            pcoffset = rs1_in;
        end
        7'b1100011:     //b
        begin
            pcoffset = imm32_in;
            aluin1 = rs1_in;
            aluin2 = rs2_in;
            data_out = aluout;
        end
        7'b0000011:     //lw
        begin
            aluin1 = rs1_in;
            aluin2 = imm32_in;
            addr_out = aluout;
        end
        7'b0100011:     //sw
        begin
            aluin1 = rs1_in;
            aluin2 = imm32_in;
            data_out = rs2_in;
            addr_out = aluout;
        end
        7'b0010011:     //arth & logic imm
        begin
            aluin1 = rs1_in;
            aluin2 = imm32_in;
            data_out = aluout;
        end
        7'b0110011:     //arth & logic reg
        begin
            aluin1 = rs1_in;
            aluin2 = rs2_in;
            data_out = aluout;
        end
        endcase
    end
endmodule

module ex_ma(clk, npc2_in, ir_in, data_in, addr_in, flag_in, dead_in,  //reg betw exec & memory_access
                  npc2_out, ir_out, data_out, addr_out, flag_out, dead_out);
    input clk, dead_in;
    input [31:0] npc2_in;
    input [31:0] ir_in;
    input [31:0] data_in;
    input [31:0] addr_in;
    input [1:0] flag_in;
    output reg dead_out;
    output reg [31:0] npc2_out;
    output reg [31:0] ir_out;
    output reg [31:0] data_out;
    output reg [31:0] addr_out;
    output reg [1:0] flag_out;
    always @(posedge clk)
    begin
        npc2_out = npc2_in;
        ir_out = ir_in;
        data_out = data_in;
        addr_out = addr_in;
        dead_out = dead_in;
        flag_out = flag_in;
    end
endmodule

module data_mem(clk, addr, w_data, r_data, r, w);   //data memory
    input [31:0] addr;
    input [31:0] w_data;
    input clk, r, w;
    output [31:0] r_data;
    reg [7:0] data[0:7];
    reg [31:0] temp;
    assign r_data = r?temp:32'bz;
    always @(*)
    begin
        if(r)
        begin
            temp[7:0] = data[addr];
            temp[15:8] = data[addr + 1];
            temp[23:16] = data[addr + 2];
            temp[31:24] = data[addr + 3];
        end
    end
    always @(negedge clk)
    begin
        if(w)
        begin
            data[addr] = w_data[7:0];
            data[addr + 1] = w_data[15:8];
            data[addr + 2] = w_data[23:16];
            data[addr + 3] = w_data[31:24];
        end
    end
endmodule

module ma(clk, npc2_in, ir_in, data_in, addr_in, flag_in, dead_in,
               ir_out, data_out, mem_out, j, new_pc); 
    input clk, dead_in;
    input [31:0] npc2_in;
    input [31:0] ir_in;
    input [31:0] data_in;
    input [31:0] addr_in;
    input [1:0] flag_in;
    output reg j;
    output reg [31:0] ir_out;
    output reg [31:0] data_out;
    output wire [31:0] mem_out;
    output reg [31:0] new_pc;
    reg rm, wm;
    wire r, w;
    assign r = rm;
    assign w = wm;
    data_mem data_memory(clk, addr_in, data_in, mem_out, r, w);
    always @(*)
    begin
        new_pc = npc2_in;
        ir_out = ir_in;
        data_out = data_in;
        rm = 0;
        wm = 0;
        if(ir_in[6:0] == 7'b1100011)     //b
        begin
            case(ir_in[14:12])
            3'b000: j = (flag_in == 2'b00)? 1:0;    //eq
            3'b001: j = (flag_in != 2'b00)? 1:0;    //ne
            3'b100: j = (flag_in == 2'b10)? 1:0;    //lt
            3'b101: j = (flag_in[1] == 0)? 1:0;     //ge
            endcase
        end
        else if(ir_in[6:0] == 7'b1100111 || ir_in[6:0] == 7'b1101111)   //j
            j = 1;
        else
            j = 0;
        if(ir_in[6:0] == 7'b0000011)    //lw
            rm = 1;
        else if(ir_in[6:0] == 7'b0100011 && dead_in != 1)   //sw
            wm = 1;
    end
endmodule

module ma_wb(clk, ir_in, data_in, mem_in, dead_in,   //reg betw memory access & write back
                  ir_out, data_out, mem_out, dead_out);
    input clk, dead_in;
    input [31:0] ir_in;
    input [31:0] data_in;
    input [31:0] mem_in;
    output reg dead_out;
    output reg [31:0] ir_out;
    output reg [31:0] data_out;
    output reg [31:0] mem_out;
    always @(posedge clk)
    begin
        ir_out = ir_in;
        data_out = data_in;
        mem_out = mem_in;
        dead_out = dead_in;
    end
endmodule

module wb(clk, ir_in, data_in, mem_in, dead_in,
               regwr, rd_data, rd_reg);
    input clk, dead_in;
    input [31:0] ir_in;
    input [31:0] data_in;
    input [31:0] mem_in;
    output reg regwr;
    output reg [31:0] rd_data;
    output reg [4:0] rd_reg;
    always @(*)
    begin
        rd_reg = ir_in[11:7];
        if(!dead_in && ir_in[6:0] == 7'b0000011)    //lw
        begin
            rd_data = mem_in;
            regwr = 1;
        end
        else if(dead_in != 1 && (
                    ir_in[6:0] == 7'b0110111 || //lui
                    ir_in[6:0] == 7'b0010111 || //auipc
                    ir_in[6:0] == 7'b1101111 || //jal
                    ir_in[6:0] == 7'b1100111 || //jalr
                    ir_in[6:0] == 7'b0010011 || //arth & logic imm
                    ir_in[6:0] == 7'b0110011))  //arth & logic reg
        begin
            rd_data = data_in;
            regwr = 1;
        end
    end
endmodule

module cpu();
    reg clk, reset;
    wire dead_1, dead_2, dead_3, dead_4, dead_5, dead_1o, dead_2o;
    wire [31:0] pc_1;
    wire [31:0] npc1_1;
    wire [31:0] ir_1;
    wire [31:0] pc_2i;
    wire [31:0] npc1_2i;
    wire [31:0] ir_2i;
    wire [31:0] pc_2o;
    wire [31:0] npc1_2o;
    wire [31:0] ir_2o;
    wire [31:0] rs1_2o;
    wire [31:0] rs2_2o;
    wire [31:0] imm32_2o;
    wire [31:0] pc_3i;
    wire [31:0] npc1_3i;
    wire [31:0] ir_3i;
    wire [31:0] rs1_3i;
    wire [31:0] rs2_3i;
    wire [31:0] imm32_3i;
    wire [31:0] npc2_3o;
    wire [31:0] ir_3o;
    wire [31:0] data_3o;
    wire [31:0] addr_3o;
    wire [1:0] flag_3o;
    wire [31:0] npc2_4i;
    wire [31:0] ir_4i;
    wire [31:0] data_4i;
    wire [31:0] addr_4i;
    wire [1:0] flag_4i;
    wire [31:0] ir_4o;
    wire [31:0] data_4o;
    wire [31:0] mem_4o;
    wire [31:0] ir_5;
    wire [31:0] data_5;
    wire [31:0] mem_5;
    wire j, regwr;
    wire [31:0] jump_pc;
    wire [31:0] w_data;
    wire [4:0] w_reg;
    reg [31:0] reg_src1;
    reg [31:0] reg_src2;
    reg [31:0] data_dm;
    reg [31:0] addr_dm;
    fi fi_block(clk, reset, pc_1, npc1_1, jump_pc, ir_1, j);
    fi_id fi_id_reg(clk, pc_1, npc1_1, ir_1, dead_1, pc_2i, npc1_2i, ir_2i, dead_1o);
    id id_block(clk, pc_2i, npc1_2i, ir_2i, regwr, w_reg, w_data, pc_2o, npc1_2o, ir_2o, rs1_2o, rs2_2o, imm32_2o);
    id_ex id_ex_reg(clk, pc_2o, npc1_2o, ir_2o, rs1_2o, rs2_2o, imm32_2o, dead_2, pc_3i, npc1_3i, ir_3i, rs1_3i, rs2_3i, imm32_3i, dead_2o);
    ex ex_block(clk, pc_3i, npc1_3i, ir_3i, reg_src1, reg_src2, imm32_3i, npc2_3o, ir_3o, data_3o, addr_3o, flag_3o);
    ex_ma ex_ma_reg(clk, npc2_pc, ir_3o, data_dm, addr_dm, flag_3o, dead_3, npc2_4i, ir_4i, data_4i, addr_4i, flag_4i, dead_4);
    ma ma_block(clk, npc2_4i, ir_4i, data_4i, addr_4i, flag_4i, dead_4, ir_4o, data_4o, mem_4o, j, jump_pc);
    ma_wb ma_wb_reg(clk, ir_4o, data_4o, mem_4o, dead_4, ir_5, data_5, mem_5, dead_5);
    wb wb_block(clk, ir_5, data_5, mem_5, dead_5, regwr, w_data, w_reg);
    assign dead_1 = j;
    assign dead_2 = j?j:dead_1o;
    assign dead_3 = j?j:dead_2o;
    initial begin
        clk = 0;
        reset = 1;
        #100
        reset = 0;
    end
    always #50 clk = ~clk;
    always @(posedge clk)
    begin
        reg_src1 = rs1_3i;
        reg_src2 = rs2_3i;
        data_dm = data_3o;
        addr_dm = addr_3o;
    end
    always @(negedge clk)
    begin
        //4 to 3
        if(ir_4i[6:0] == 7'b0110111 || ir_4i[6:0] == 7'b0010111 || ir_4i[6:0] == 7'b1101111
        || ir_4i[6:0] == 7'b1100111 || ir_4i[6:0] == 7'b0010011 || ir_4i[6:0] == 7'b0110011)   //rd exist, ex lw
        begin
            if(ir_3i[19:15] == ir_4i[11:7] && 
              (ir_3i[6:0] == 7'b1100111 || ir_3i[6:0] == 7'b1100011 || ir_3i[6:0] == 7'b0000011
            || ir_3i[6:0] == 7'b0100011 || ir_3i[6:0] == 7'b0010011 || ir_3i[6:0] == 7'b0110011))
            begin
                reg_src1 = data_4i;
            end
            if(ir_3i[24:20] == ir_4i[11:7] &&
            (ir_3i[6:0] == 7'b1100011 || ir_3i[6:0] == 7'b0100011 || ir_3i[6:0] == 7'b0110011))
            begin
                reg_src2 = data_4i;
            end
        end
        //4o to 3
        if(ir_4o[6:0] == 7'b0000011)    //lw
        begin
            if(ir_3i[19:15] == ir_4o[11:7] && ir_3i[6:0] == 7'b0100011) //sw
            begin
                reg_src2 = data_4o;
            end
        end
        //5 to 3
        if(ir_5[6:0] == 7'b0000011)     //lw
        begin
            if(ir_3i[19:15] == ir_5[11:7] && 
              (ir_3i[6:0] == 7'b1100111 || ir_3i[6:0] == 7'b1100011 || ir_3i[6:0] == 7'b0000011
            || ir_3i[6:0] == 7'b0100011 || ir_3i[6:0] == 7'b0010011 || ir_3i[6:0] == 7'b0110011))
            begin
                reg_src1 = mem_5;
            end
            if(ir_3i[24:20] == ir_4i[11:7] &&
            (ir_3i[6:0] == 7'b1100011 || ir_3i[6:0] == 7'b0100011 || ir_3i[6:0] == 7'b0110011))
            begin
                reg_src2 = mem_5;
            end
        end
        //5 to 4
        if(ir_5[6:0] == 7'b0000011)     //lw
        begin
            if(ir_4i[24:20] == ir_5[11:7] && ir_4i == 7'b0100011)
            begin
                data_dm = mem_5;
            end
        end
    end
endmodule