`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: The Pennsylvania State University
// Engineer: Corey Ortiz
// 
// Create Date: 08/14/2024 05:07:58 PM
// Design Name: Developing a Computer
// Module Name: datapath
// Project Name: Baby Computer
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
module datapath(
    input clk
    );
    
  // With pipeline registers.
  
  
  // Fetching instruction phase:
  wire [31:0] pc, nextPc;
  wire stall;
 
  program_counter pcmodule(.clk(clk), .nextPc(nextPc), .pc(pc), .stall(stall));
  pc_adder pa(.pc(pc), .offset(32'd4), .nextPc(nextPc));    
    
  wire [31:0] inst;
  im im(.pc(pc), .inst(inst));

  //IF/ID stage
  wire [31:0] inst_d;
  IF_ID if_id( .clk(clk), .inst(inst), .inst_d(inst_d), .stall(stall));
  
  //Now the decoding stage.
  wire [3:0] aluControl;  // The 4 bit ALU 
  wire regWrite, memToReg, memWrite, memRead, aluSrc, regDst;
  wire [31:0] regOut1;
  wire [31:0] regOut2;
  wire [4:0] rt, rd, rs;
  wire [31:0] imm32;
  wire [31:0] writeData;
  
  assign rd = inst_d[15:11];
  assign rt = inst_d[20:16];
  assign rs = inst_d[25:21];

    // Register File
  register_file regisFile(
         .readAddr1(inst_d[25:21]), // RS  
         .readAddr2(inst_d[20:16]), // RT
         .writeAddr(writeAddr_b), 
         .writeData(writeData), 
         .regWrite(regWrite_b),
         .clk(clk),
         .regOut1(regOut1),
         .regOut2(regOut2)
    );
      //Control Unit
    control_unit controlUnit(
        .op(inst_d[31:26]),  // Opcode is always the 6 MSB of the instruction
        .func(inst_d[5:0]), // Func is always the 6 bits of the instruction
        .regWrite(regWrite), 
        .memToReg(memToReg),
        .memWrite(memWrite),
        .aluSrc(aluSrc),
        .regDst(regDst),
        .memRead(memRead),
        .aluControl(aluControl),
        .stall(stall)
    );
    // Following the diagram to update the immedate input to be inst_d[15:0]
    imm_extend  immExtend ( .imm(inst_d[15:0]), .imm32(imm32) );
    
    // ID/EX stage
    wire regWrite_x, memToReg_x, memWrite_x, memRead_x, aluSrc_x, regDst_x;
    wire [31:0] regOut1_x;
    wire [31:0] regOut2_x;
    wire [4:0] rt_x, rd_x, rs_x;
    wire [31:0] imm32_x;
    wire [3:0] aluControl_x;
    
    
    ID_EX id_Ex( .clk(clk),
                .regWrite(regWrite),
                .memToReg(memToReg),
                .memWrite(memWrite),
                .aluSrc(aluSrc),
                .regDst(regDst),
                .memRead(memRead),
                .regOut2(regOut2),
                .regOut1(regOut1),
                .imm32(imm32),
                .rt(rt),
                .rd(rd),
                .rs(rs),
                .aluControl(aluControl),
                .regWrite_x(regWrite_x),  // Outputs of the pipline register itself.
                .memToReg_x(memToReg_x),
                .memWrite_x(memWrite_x),
                .aluSrc_x(aluSrc_x),
                .regDst_x(regDst_x),
                .memRead_x(memRead_x),
                .regOut2_x(regOut2_x),
                .regOut1_x(regOut1_x),
                .imm32_x(imm32_x),
                .rs_x(rs_x),
                .rt_x(rt_x),
                .rd_x(rd_x),
                .aluControl_x(aluControl_x) );
     
     
    hazard_unit HAZARD_UNIT ( 
                .rt_x(rt_x),
                .rt_d(rt),
                .rs_d(rs),
                .memRead_x(memRead_x),
                .stall(stall)
                );         
    // Following the diagram this is where the forwarding unit will be existing
    wire [1:0] forwardA;
    wire [1:0] forwardB;
    wire [31:0] aluIn1, aluIn2 ;
    wire [31:0] aluOut_m;
    wire [31:0] memOut_b;
    wire [31:0] aluMuxOut;
    
    Forwarding_Unit  forwarding_unit ( .writeAddr_m(writeAddr_m),
                                      .writeAddr_b(writeAddr_b),
                                      .rs_x(rs_x),
                                      .rt_x(rt_x),
                                      .regWrite_m(regWrite_m),
                                      .regWrite_b(regWrite_b),
                                      .forwardA(forwardA),
                                      .forwardB(forwardB) );  
                                      
    mux_3x1 mux_for_forwardA (  .in0(regOut1_x),
                           .in1(memOut_b), //this is for forward from MEM/WB
                           .in2(aluOut_m),  // this is for forward from EX/WB
                           .sel(forwardA),
                           .out(aluIn1) );
    
    mux_3x1 mux_for_forwardB (  
                            .in0(regOut2_x),
                           .in1(memOut_b), //this is for forward from EX/MEM
                           .in2(aluOut_m),  // this is for forward from MEM/WB
                           .sel(forwardB),
                           .out(aluMuxOut) );
           
                
                
                
    // Now the execution stage
    
    wire [31:0] aluOut;
    wire [4:0] writeAddr;
    //ALU Mux  
    mux_2x1_32b ALU_mux (
        .in0(aluMuxOut), // from the register file "read data 2"
        .in1(imm32_x), // If mux selects 1 then we want to use the sign-extend 32 immediate
        .sel(aluSrc_x), // the selector of the mux
        .out(aluIn2)  //The mux output
    
    );
    
    alu ALU(
        .aluIn1(aluIn1),  //following the hardware diagram from class
        .aluIn2(aluIn2),
        .aluControl(aluControl_x),
        .aluOut(aluOut) //
    );  
    
        //Mux for Register Destination, doing this here because if we are not grabbing anything from data memory will go straight into the input of the EX/MEM Pipeline register
    mux_2x1_5b MuxRegDst(
        .in0(rt_x),
        .in1(rd_x),
        .sel(regDst_x),
        .out(writeAddr)
    );
    
    
    wire regWrite_m, memToReg_m, memWrite_m, memRead_m;
    wire [4:0] writeAddr_m;
    wire [31:0] regOut2_m;
    // EX/MEM stage
     EX_MEM Ex_Mem( .clk(clk),
                    .regWrite_x(regWrite_x),
                    .memWrite_x(memWrite_x),
                    .memToReg_x(memToReg_x),
                    .memRead_x(memRead_x),
                    .aluOut(aluOut),
                    .regOut2_x(regOut2_x),
                    .writeAddr(writeAddr),
                    .regWrite_m(regWrite_m),
                    .memToReg_m(memToReg_m),
                    .memWrite_m(memWrite_m),
                    .memRead_m(memRead_m),
                    .aluOut_m(aluOut_m),
                    .regOut2_m(regOut2_m),
                    .writeAddr_m(writeAddr_m) );
                    
     // Now were are in data memory stage
    wire [31:0] memOut;
    data_mem DATA_mem (
        .clk(clk),
        .addr(aluOut_m),
        .memIn(regOut2_m),
        .memRead(memRead_m),
        .memWrite(memWrite_m),
        .memOut(memOut) 
    );
    
    // MEM/WB stage 
    wire [4:0] writeAddr_b;
    wire regWrite_b;
    wire [31:0] aluOut_b;
    wire memToReg_b;
    MEM_WB Mem_WB ( .clk(clk),
                    .regWrite_m(regWrite_m),
                    .memToReg_m(memToReg_m),
                    .aluOut_m(aluOut_m),
                    .memOut(memOut),
                    .writeAddr_m(writeAddr_m),
                    .regWrite_b(regWrite_b),
                    .memToReg_b(memToReg_b),
                    .memOut_b(memOut_b),
                    .aluOut_b(aluOut_b),
                    .writeAddr_b(writeAddr_b) );
                    
    // Write back stage
    mux_2x1_32b MemToReg (
        .in1(memOut_b),   /// previously: mem 
        .in0(aluOut_b),   ///  previously: alu
        .sel(memToReg_b),
        .out(writeData)
    );
    
         
// Without Pipeline registerz
 
//    wire [31:0] pc, nextPc;
//    wire[4:0] writeAddr;
//    wire [31:0] writeData;
//    program_counter pcmodule(.clk(clk), .nextPc(nextPc), .pc(pc));
//    pc_adder pa(.pc(pc), .offset(32'd4), .nextPc(nextPc));
    
//    wire [31:0] inst;  // Wiring the instruction memeory output in order for the other modules can use.
//    inst_mem instructMem(.pc(pc), .inst(inst));   // Called the instantion instructMem
    
//    // We need the wires to be able to transport the outputs so that each part of the hardware can utilize the outputs of each module
    
//    //Outputs of Control Unit
//    wire regWrite, memToReg, memWrite, memRead, aluSrc, regDst;
//    wire [3:0] aluControl;  // The 4 bit ALU 
    
//    wire [31:0] regOut1;
//    wire [31:0] regOut2;
//    // Register File
//    register_file regisFile(
//         .readAddr1(inst[25:21]), // RS  // Following the QUIZ of Single-Cycle Control Unit
//         .readAddr2(inst[20:16]), // RT
//         .writeAddr(writeAddr), // For the next HW
//         .writeData(writeData),  // For the next HW
//         .regWrite(regWrite),
//         .clk(clk),
//         .regOut1(regOut1),
//         .regOut2(regOut2)
//    );
    
//    //Control Unit
//    control_unit controlUnit(
//        .op(inst[31:26]),  // Opcode is always the 6 MSB of the instruction
//        .func(inst[5:0]), // Func is always the 6 bits of the instruction
//        .regWrite(regWrite),  // Utilizing the wires that we created to connect it with the Control Unit Module
//        .memToReg(memToReg),
//        .memWrite(memWrite),
//        .aluSrc(aluSrc),
//        .regDst(regDst),
//        .memRead(memRead),
//        .aluControl(aluControl) 
//    );
    
    
//    //Sign-Extend
//    wire [15:0] imm = inst[15:0];
//    wire [31:0] imm32; // We will be input for the ALU
//    imm_extend  immExtend ( .imm(imm), .imm32(imm32) );
    
//    //Mux 2:1 
//    wire [31:0] aluIn2; // Output of the mux to the ALU
//    mux_2x1_32b ALU_mux (
//        .in0(regOut2), // from the register file "read data 2"
//        .in1(imm32), // If mux selects 1 then we want to use the sign-extend 32 immediate
//        .sel(aluSrc), // the selector of the mux
//        .out(aluIn2)  //The mux output
    
//    );
    
    
//    // ALU
//    wire [31:0] aluOut; //output wire once the ALU has done its function
//    alu ALU(
//        .aluIn1(regOut1),  //following the hardware diagram from class
//        .aluIn2(aluIn2),
//        .aluControl(aluControl),
//        .aluOut(aluOut) //
//    );
    
//    // Data memory
//    wire [31:0] memOut;
    
//    data_mem DATA_mem (
//        .clk(clk),
//        .addr(aluOut),
//        .memIn(regOut2),
//        .memRead(memRead),
//        .memWrite(memWrite),
//        .memOut(memOut) 
//    );
    
    
    
//    // WB Stage
//    mux_2x1_32b MemToReg (
//        .in1(memOut),
//        .in0(aluOut),
//        .sel(memToReg),
//        .out(writeData)
//    );
    
//    //Mux for Register Destination
//    mux_2x1_5b MuxRegDst(
//        .in0(inst[20:16]),
//        .in1(inst[15:11]),
//        .sel(regDst),
//        .out(writeAddr)
//    );
   
    
endmodule

/* ================= Modules to implement for HW1 =====================*/
module program_counter(
    input clk,
    input [31:0] nextPc,
    input stall,
    output reg [31:0] pc
    );
    initial begin
        pc = 32'd96; // PC initialized to start from 100.
    end
    // ==================== Students fill here BEGIN ====================
    always @(posedge clk)  // This is a sequential circuit therefore we need non-blocking assignment 
    begin
        if(stall == 0)
           pc <= nextPc;  //update pc only a pos-edge
    end
    // ==================== Students fill here END ======================
endmodule

module pc_adder(
    input [31:0] pc, offset,
    output reg [31:0] nextPc
    );
    // ==================== Students fill here BEGIN ====================
    always @(*)  // This is a sequential circuit therefore we blocking assignment
    begin
       nextPc = pc + offset;  // don't need to usee "assign" since we are using "always" 
    end
    // ==================== Students fill here END ======================
endmodule

/* ================= Modules to implement for HW3 =====================*/
module im(
    input [31:0] pc,
    output reg [31:0] inst
    );
    
    // This is an instruction memory that holds 64 instructions, 32b each.
    reg [31:0] memory [0:63];
    
    // Initializing instruction memory.
    initial begin   
    
   //  first set    
        memory[25] = {6'b100011, 5'd0, 5'd1, 16'd0};  // lw $1, 0($zero) : Load Instruction
        memory[26] = {6'b100011, 5'd0, 5'd2, 16'd4};  //  lw $2, 4($zero) : Load Instruction with an offset of 4
        memory[27] = {6'b000000, 5'd1, 5'd2, 5'd3, 11'b00000100010};  // R-Type: sub $3, $1, $2
        memory[28] = {6'b100011, 5'd3, 5'd4, 16'hFFFC};  //  lw $4, -4($3)

// Second set to test if pipeline register work
//          memory[25] = {6'b100011, 5'd0, 5'd1, 16'd0};
//          memory[26] = {6'b100011, 5'd0, 5'd2, 16'd4};
//          memory[27] = {6'b100011, 5'd0, 5'd3, 16'd8};
//          memory[28] = {6'b100011, 5'd0, 5'd4, 16'd16};
//          memory[29] = {6'b000000, 5'd1, 5'd2, 5'd5, 11'b00000100000};
//          memory[30] = {6'b100011, 5'd3, 5'd6, 16'hFFFC};
//          memory[31] = {6'b000000, 5'd4, 5'd3, 5'd7, 11'b00000100010};

// Third set to test forwarding, but no load use hazard
//            memory[25] = {6'b100011, 5'd0, 5'd1, 16'd0};  // lw $1, 0($0)
//            memory[26] = {6'b100011, 5'd0, 5'd2, 16'd4}; // lw $2, 4($0)
//            memory[27] = {6'b100011, 5'd0, 5'd4, 16'd16};  // lw $4, 16($0)
//            memory[28] = {6'b000000, 5'd1, 5'd2, 5'd3, 11'b00000100010};  // sub $3, $1, $2
//            memory[29] = {6'b100011, 5'd3, 5'd4, 16'hFFFC};  // lw $4, -4($3)
    end
    // ==================== Students fill here BEGIN ====================
       always @(*)
       begin 
        inst = memory[ pc[31:2] ]; // This is the same as doing "inst = memeory[pc/4]". We are cutting off the least 2 bits of the 32 bit-pc because the 2 bits = 4 in decimal. Therefore we are dividing by 4.
       end
        
    // ==================== Students fill here END ======================
endmodule

module register_file(
    input [4:0] readAddr1, readAddr2, writeAddr,
    input [31:0] writeData,
    input regWrite, clk,
    output reg [31:0] regOut1, regOut2
    );
    
    // Initializing registers. Do not touch here.
    reg [31:0] register [0:31]; // 32 registers, 32b each.
    integer i;
    initial begin
        for (i=0; i<32; i=i+1) begin
            register[i] = 32'd0; // Initialize to zero
        end
    end
    // ==================== Students fill here BEGIN ====================
    always @(*) 
    begin
        regOut1 = register[readAddr1];   // Rs  Instruction bits of [25-21]
        regOut2 = register[readAddr2];   // Rt   Istruncton bits of [20-16]
    end
    
    // Write Back Operation
    always @(negedge clk)
    begin
        if (regWrite == 1'b1)
        begin
            register[writeAddr] <= writeData;
        end
    end
    // ==================== Students fill here END ======================
endmodule

module control_unit(
    input [5:0] op, func,
    input stall,
    output reg regWrite, memToReg, memWrite, aluSrc, regDst, memRead,
    output reg [3:0] aluControl
    );
    // ==================== Students fill here BEGIN ====================
    always @(*) begin
     // At Every PC + 4 we will need to reset the control signals
     // At every pc + 4 could be loading in a different instruction
       regWrite = 0;
       memToReg = 0;
       memWrite = 0;
       aluSrc = 0;
       regDst = 0;
       memRead = 0;
       aluControl = 4'b0000;
       
       if(stall == 0)
            case(op) // This will help as at each PC + 4 what is the OP code for the instruction
                6'b100011: begin // load word
                    regWrite = 1;   // Following the Control Signal Table
                    memToReg = 1;    // Since reseting sets all of them to 0 so I only need to change it to 1 for ones that need to change
                    aluSrc = 1;
                    memRead = 1;
                    aluControl = 4'b0010; // ALU Control for add since load word uses addition to deal with offset
                end
        
                6'b101011: begin // store word
                    aluSrc = 1;
                    memWrite = 1;
                    aluControl = 4'b0010; //ALU Control set the same as Load word for their offsets
                end
        
                6'b000000: begin  // r-type instructions specifically just add and sub
                    regWrite = 1;
                    regDst = 1;
                    case(func)
                        6'b100010: aluControl = 4'b0110; // For Sub
                        6'b100000: aluControl = 4'b0010; // For Add
                    endcase // For the case of what type of function is it
                end
            
            endcase // for the Main Case Scenario
     
     end   // For the always  
    
    // ==================== Students fill here END ======================
endmodule

/* ================= Modules to implement for HW4 =====================*/
module imm_extend(
    input [15:0] imm,
    output reg [31:0] imm32
    );
    // ==================== Students fill here BEGIN ====================
    always @(*)
    begin         // Since we are in 2's complement I need to copy the 15th bit number to be copied for the upper 16 bits of the 32 and then concatenate that with the input, imm. 
        imm32 = { {16 {imm[15]} }, imm};
    end
    // ==================== Students fill here END ======================
endmodule

module mux_2x1_32b(
    input [31:0] in0, in1,
    input sel,
    output reg [31:0] out
    );
    // ==================== Students fill here BEGIN ====================
    always @(*) 
    begin
        out = ( sel == 0 ) ? in0 : in1;  // C-Code implementation same as Verilog
    end 
    // ==================== Students fill here END ======================
endmodule

module alu(
    input [31:0] aluIn1, aluIn2,
    input [3:0] aluControl,
    output reg [31:0] aluOut
    );

    // ==================== Students fill here BEGIN ====================
    always @(*)
    begin
        
        case(aluControl) // Similar in HW3's dealing with add and sub
            4'b0110: begin
                aluOut = aluIn1 - aluIn2; // subtracting
            end
            4'b0010: begin
                aluOut = aluIn1 + aluIn2; // adding
            end
        endcase
    end
    // ==================== Students fill here END ======================
endmodule

module data_mem(
    input clk, memWrite, memRead,
    input [31:0] addr, memIn,
    output reg [31:0] memOut
    );
    
    reg [31:0] memory [0:63]; // 64x32 memory
    
    // Initialize data memory. Do not touch this part.
    initial begin
        memory[0] = 32'd16817;
        memory[1] = 32'd16801;
        memory[2] = 32'd16;
        memory[3] = 32'hDEAD_BEEF;
        memory[4] = 32'h4242_4242;
    end
    
    // ==================== Students fill here BEGIN ====================
    
    always @(*) 
    begin
        if (memRead == 1)
            memOut = memory[addr[31:2]]; // Divide the address by 4 like how we did in HW3
        else // memRead == 0
            memOut = 32'dx;
    end 
    
    always @(negedge clk)
    begin
        if (memWrite == 1)
            memory[ addr[31:2] ] <= memIn;  // memory[addr / 4 ] = memIn
    end 

    // ==================== Students fill here END ======================
endmodule

/* ================= Modules to implement for HW5 =====================*/
module mux_2x1_5b(
    input [4:0] in0, in1,
    input sel,
    output reg [4:0] out
    );
    // ==================== Students fill here BEGIN ====================
    always @(*) 
    begin
         out = ( sel == 0 ) ? in0 : in1;
    end
       
    // ==================== Students fill here END ======================
endmodule


/* Final Project: Creating the Pipline registers */

module IF_ID( 
     input clk,
     input [31:0] inst,
     input stall,
     output reg [31:0] inst_d
     );
     always @(posedge clk) 
     begin
        if(stall == 0)
             inst_d <= inst;  
     
     end
     
endmodule

module ID_EX(
    input clk,
    input regWrite, memToReg, memWrite, aluSrc, regDst, memRead,
    input [31:0] regOut1, regOut2, imm32,
    input [4:0] rt, rd, rs,
    input [3:0] aluControl,
    output reg regWrite_x, memToReg_x, memWrite_x, aluSrc_x, regDst_x, memRead_x,
    output reg [31:0] regOut1_x, regOut2_x, imm32_x,
    output reg [4:0] rt_x, rd_x,rs_x,
    output reg [3:0] aluControl_x
    );
    
    always @(posedge clk) 
    begin
        regWrite_x <= regWrite;
        memToReg_x <= memToReg;
        memWrite_x <= memWrite;
        aluSrc_x <= aluSrc;
        regDst_x <= regDst;
        memRead_x <= memRead;
        regOut1_x <= regOut1;
        regOut2_x <= regOut2;
        imm32_x <= imm32;
        rt_x <= rt; 
        rd_x <= rd;
        rs_x <= rs;
        aluControl_x <= aluControl;
    end
 
endmodule


module EX_MEM( 
       input clk,
       input regWrite_x, memToReg_x, memWrite_x, memRead_x,
       input [31:0] aluOut, regOut2_x,
       input [4:0] writeAddr,
       output reg regWrite_m, memToReg_m, memWrite_m, memRead_m,
       output reg [31:0] aluOut_m, regOut2_m,
       output reg [4:0] writeAddr_m
       );
       always @(posedge clk) 
       begin
            regWrite_m <= regWrite_x;
            memToReg_m <= memToReg_x;
            memWrite_m <= memWrite_x;
            memRead_m <= memRead_x;
            aluOut_m <= aluOut;
            writeAddr_m <= writeAddr;
            regOut2_m <= regOut2_x;
       end
    
endmodule
    

module MEM_WB(
        input clk, 
        input regWrite_m, memToReg_m,
        input [31:0] aluOut_m, memOut,
        input [4:0] writeAddr_m,
        output reg regWrite_b, memToReg_b,
        output reg [31:0] aluOut_b, memOut_b,
        output reg [4:0] writeAddr_b
        );
        
        always @(posedge clk)
        begin
            regWrite_b <= regWrite_m;
            memToReg_b <= memToReg_m;
            aluOut_b <= aluOut_m;
            memOut_b <= memOut;
            writeAddr_b <= writeAddr_m;
        end
endmodule

module Forwarding_Unit (   input [4:0] writeAddr_m,
                           input [4:0] writeAddr_b,
                           input [4:0] rs_x, rt_x,
                           input regWrite_m, regWrite_b,
                           output reg [1:0] forwardA, forwardB
                        );
         always @(*)
         begin
                 // if forwarding is not needed.
                forwardA = 2'b00;
                forwardB = 2'b00;
                // Following the lecture slide c-code equivalent
                // Forward A 
                if ( regWrite_m  && ( writeAddr_m != 0) && (writeAddr_m == rs_x)) // Rs  
                       forwardA = 2'b10;  // EX/MEM Forward
                else if ( regWrite_b && (writeAddr_b != 0) && (writeAddr_b == rs_x)) // 
                       forwardA = 2'b01;   // Mem/WB forward
                
               // Forward B 
                if ( regWrite_m  && ( writeAddr_m != 0) && (writeAddr_m == rt_x)) // Rt
                       forwardB = 2'b10;
                else if ( regWrite_b && (writeAddr_b != 0) && (writeAddr_b == rt_x)) //
                       forwardB = 2'b01;
             
         end
  
endmodule



module mux_3x1 ( 
            input [31:0] in0, in1, in2,
            input [1:0] sel,
            output reg [31:0] out);
        always @(*)
        begin
            case (sel)
                2'b00: out = in0;
                2'b01: out = in1;
                2'b10: out = in2;
            endcase
        end
endmodule


module hazard_unit ( 
            input [4:0] rt_x, rt_d, rs_d,
            input memRead_x,
            output reg stall
        );
        
        initial begin
           stall = 0;
        end
       
       always @(*)
       begin
            
            if( memRead_x && ( (rt_x == rs_d) || (rt_x == rt_d) ) )
                stall = 1;  // Load use hazard we will need stall the pipline.
            else
                stall = 0;  // if there is no need to stall
            
       end
endmodule
        
     