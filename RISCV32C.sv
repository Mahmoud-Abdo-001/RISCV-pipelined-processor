///////////////////////////////////////////////////////////////////////////
////////////////////////////////Piplined RISCV TOP/////////////////////////
///////////////////////////////////////////////////////////////////////////
//top module of piplined risc v processor
//PiplinedRISCV.sv
//
// by Mahmoud Abdo
//
//
/* RISCV pipeline without any external memories */
module RISCV32C(
  /* Clock */
  input logic clk, 
  input logic reset,

  /* Instruction Memory unit */
  output logic [31:0] PCF,   /* Instruction address */
  input logic [31:0] InstrF,  /* Instruction to be executed */
  
  /* Data Memory unit */
	output logic MemWriteM,   /* Data Memory Write Operation Enable */
	output logic [31:0] ALUResultM,   /* Data Memory R/W address */
  output logic [31:0] WriteDataM ,    /* Data to be written in the data memory unit */
  input logic [31:0] ReadDataM      /* Data Memory output data R */
);

logic ALUSrcAE, RegWriteM, RegWriteW, ZeroE, SignE, PCJalSrcE, PCSrcE;
logic [1:0] ALUSrcBE;
logic StallD, StallF, FlushD, FlushE, ResultSrcE0;
logic [1:0] ResultSrcW; 
logic [2:0] ImmSrcD;
logic [3:0] ALUControlE;
logic [31:0] InstrD;
logic [4:0] Rs1D, Rs2D, Rs1E, Rs2E;
logic [4:0] RdE, RdM, RdW;
logic [1:0] ForwardAE, ForwardBE;


DataPath Datapath(
.clk(clk),
.reset(reset),
.ResultSrcW(ResultSrcW),
.PCJalSrcE(PCJalSrcE), 
.PCSrcE(PCSrcE), 
.ALUSrcAE(ALUSrcAE), 
.ALUSrcBE(ALUSrcBE),
.RegWriteW(RegWriteW),
.ImmSrcD(ImmSrcD),
.ALUControlE(ALUControlE),
.ZeroE(ZeroE),
.SignE(SignE),
.PCF(PCF),
.InstrF(InstrF),
.InstrD(InstrD),
.ALUResultM(ALUResultM), 
.WriteDataM(WriteDataM),
.ReadDataM(ReadDataM),
.ForwardAE(ForwardAE), 
.ForwardBE(ForwardBE),
.Rs1D(Rs1D), 
.Rs2D(Rs2D), 
.Rs1E(Rs1E), 
.Rs2E(Rs2E),
.RdE(RdE), 
.RdM(RdM), 
.RdW(RdW),
.StallD(StallD), 
.StallF(StallF), 
.FlushD(FlushD), 
.FlushE(FlushE)
);

controller controllerC(
.clk(clk),
.reset(reset),
.op(InstrD[6:0]),
.funct3D(InstrD[14:12]),
.funct7b5(InstrD[30]),
.ZeroE(ZeroE),
.SignE(SignE),
.FlushE(FlushE),
.ResultSrcE0(ResultSrcE0),
.ResultSrcW(ResultSrcW),
.MemWriteM(MemWriteM),
.PCJalSrcE(PCJalSrcE), 
.PCSrcE(PCSrcE), 
.ALUSrcAE(ALUSrcAE), 
.ALUSrcBE(ALUSrcBE),
.RegWriteM(RegWriteM),
.RegWriteW(RegWriteW),
.ImmSrcD(ImmSrcD),
.ALUControlE(ALUControlE)
);

HazardUnit HazardUnitH (
.Rs1D(Rs1D),
.Rs2D(Rs2D),
.Rs1E(Rs1E),
.Rs2E(Rs2E),
.RdE(RdE),
.RdM(RdM),
.RdW(RdW),
.ResultSrcE0(ResultSrcE0),
.PCSrcE(PCSrcE),
.RegWriteW(RegWriteW),
.RegWriteM(RegWriteM),
.StallF(StallF),
.StallD(StallD),
.FlushD(FlushD),
.FlushE(FlushE),
.ForwardAE(ForwardAE),
.ForwardBE(ForwardBE)
);



endmodule


///////////////////////////////////////////////////////////////////////////
//////////////////////////* DataPath *//////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
module DataPath(
  input logic clk, reset,
  input logic StallD, StallF, FlushD, FlushE,
	input logic [1:0] ResultSrcW,
	input logic PCJalSrcE, PCSrcE, ALUSrcAE, 
	input logic [1:0] ALUSrcBE,
	input logic RegWriteW,
	input logic [2:0] ImmSrcD,
	input logic [3:0] ALUControlE,
	output logic ZeroE,
	output logic SignE,
	output logic [31:0] PCF,
	input logic [31:0] InstrF,
	output logic [31:0] InstrD,
	output logic [31:0] ALUResultM, WriteDataM,
	input logic [31:0] ReadDataM,
	input logic [1:0] ForwardAE, ForwardBE,
	output logic [4:0] Rs1D, Rs2D, Rs1E, Rs2E,
  output logic [4:0] RdE, RdM, RdW
  );

	logic [31:0] PCD, PCE, ALUResultE, ALUResultW, ReadDataW;
	logic [31:0] PCNextF, PCPlus4F, PCPlus4D, PCPlus4E, PCPlus4M, PCPlus4W, PCTargetE, BranJumpTargetE;
	logic [31:0] WriteDataE;
	logic [31:0] ImmExtD, ImmExtE;
	logic [31:0] SrcAEfor, SrcAE, SrcBE, RD1D, RD2D, RD1E, RD2E;
	logic [31:0] ResultW;
  logic        PCJalSrcM ;
	logic [4:0]  RdD; // destination register address

	
  mux2 Emux_jal(
  .d0(PCTargetE),
  .d1(ALUResultE),
  .s(PCJalSrcM),
  .y(BranJumpTargetE)
  );

	mux2 Fmux(
  .d0(PCPlus4F),
  .d1(BranJumpTargetE),
  .s(PCSrcE),
  .y(PCNextF)
  );

	flopenr IF(
  .clk(clk),
  .reset(reset),
  .en(~StallF), /* active low */
  .d(PCNextF),
  .q(PCF)
  );

	adder NextI(
  .a(PCF),
  .b(32'd4),
  .y(PCPlus4F)
  );
	
	DP_F_D_REG DPFDREG(
  .clk(clk),
  .reset(reset),
  .clear(FlushD),
  .enable(~StallD),
  .InstrF(InstrF),
  .PCF(PCF),
  .PCPlus4F(PCPlus4F),
  .InstrD(InstrD),
  .PCD(PCD),
  .PCPlus4D(PCPlus4D)
  );	

	regfile RegFile(
  .clk(clk), 
  .we3(RegWriteW), 
  .a1(Rs1D),
  .a2(Rs2D),
  .a3(RdW), 
  .wd3(ResultW), 
  .rd1(RD1D),
  .rd2(RD2D)
  );

extend ext(
  .instr(InstrD[31:7]),
  .ImmSrc(ImmSrcD),
  .ImmExt(ImmExtD)
  );

	
  DP_D_E_REG DPDEREG(
  .clk(clk),
  .reset(reset),
  .clear(FlushE),
  .RD1D(RD1D),
  .RD2D(RD2D),
  .PCD(PCD), 
  .Rs1D(Rs1D),
  .Rs2D(Rs2D),
  .RdD(RdD), 
  .ImmExtD(ImmExtD),
  .PCPlus4D(PCPlus4D),
  .RD1E(RD1E),
  .RD2E(RD2E),
  .PCE(PCE), 
  .Rs1E(Rs1E),
  .Rs2E(Rs2E),
  .RdE(RdE), 
  .ImmExtE(ImmExtE),
  .PCPlus4E(PCPlus4E)
  );

	mux3 FWSrcA(
  .d0(RD1E),
  .d1(ResultW),
  .d2(ALUResultM),
  .s(ForwardAE),
  .y(SrcAEfor)
  );
  

  mux3 FWSrcB(
  .d0(RD2E),
  .d1(ResultW),
  .d2(ALUResultM),
  .s(ForwardBE),
  .y(WriteDataE)
  );

	mux2 SrcA(
    .d0(SrcAEfor),
    .d1(32'b0),
    .s(ALUSrcAE),
    .y(SrcAE)
  ); // for lui

	mux3 SrcB(
  .d0(WriteDataE),
  .d1(ImmExtE),
  .d2(PCTargetE),
  .s(ALUSrcBE),
  .y(SrcBE)  
  );

  /* Next PC for jump and branch instructions */
  adder Jump_Target(
    .a(PCE),
    .b(ImmExtE),
    .y(PCTargetE)
  );

  alu ALU(
  .a(SrcAE), 
  .b(SrcBE),
  .alucontrol(ALUControlE),
  .result(ALUResultE),
  .zero(ZeroE),
  .Sign(SignE)
  );
	
		
/* Execute - Memory Access Pipeline Register */
DP_E_M_REG DPEMREG(
  .clk(clk),
  .reset(reset),
  .PCJalSrcE(PCJalSrcE),
  .ALUResultE(ALUResultE),
  .WriteDataE(WriteDataE), 
  .RdE(RdE), 
  .PCPlus4E(PCPlus4E),
  .ALUResultM(ALUResultM),
  .WriteDataM(WriteDataM),
  .RdM(RdM), 
  .PCPlus4M(PCPlus4M),
  .PCJalSrcM(PCJalSrcM)
);

/* Memory - Register Write Back Stage */ 
DP_M_W_REG DPMWREG(
  .clk(clk),
  .reset(reset),
  .ALUResultM(ALUResultM),
  .ReadDataM(ReadDataM),  
  .RdM(RdM), 
  .PCPlus4M(PCPlus4M),
  .ALUResultW(ALUResultW),
  .ReadDataW(ReadDataW),
  .RdW(RdW), 
  .PCPlus4W(PCPlus4W)
);

mux3 Result(
  .d0(ALUResultW),
  .d1(ReadDataW),
  .d2(PCPlus4W),
  .s(ResultSrcW),
  .y(ResultW)  
);

  assign Rs1D = InstrD[19:15];
  assign Rs2D = InstrD[24:20];	
  assign RdD = InstrD[11:7];

endmodule

///////////////////////////////////////////////////////////////////////////
///////////////////////// Controller UNIT /////////////////////////////////
///////////////////////////////////////////////////////////////////////////
module controller(
  input logic clk, reset,
	input logic [6:0] op,
  input logic [2:0] funct3D,
	input logic funct7b5,
	input logic ZeroE,
	input logic SignE,
	input logic FlushE,
	output logic ResultSrcE0,
	output logic [1:0] ResultSrcW,
	output logic MemWriteM,
	output logic PCJalSrcE, PCSrcE, ALUSrcAE, 
	output logic [1:0] ALUSrcBE,
	output logic RegWriteM, RegWriteW,
	output logic [2:0] ImmSrcD,
	output logic [3:0] ALUControlE
  );

logic [1:0] ALUOpD;
logic [1:0] ResultSrcD, ResultSrcE, ResultSrcM;
logic [3:0] ALUControlD;
logic BranchD, BranchE, MemWriteD, MemWriteE, JumpD, JumpE;
logic ZeroOp, ALUSrcAD, RegWriteD, RegWriteE;
logic [1:0] ALUSrcBD;
logic SignOp;
logic BranchOp;
logic [2:0]funct3E ;  

/* Main Decoder */
maindec MD(
.op(op),
.ResultSrc(ResultSrcD),
.MemWrite(MemWriteD),
.Branch(BranchD), 
.ALUSrcA(ALUSrcAD),
.RegWrite(RegWriteD), 
.Jump(JumpD),
.ImmSrc(ImmSrcD),
.ALUOp(ALUOpD),
.ALUSrcB(ALUSrcBD)
);

/* alu decoder */
aludec AluDec(
.opb5(op[5]),
.funct3(funct3D),
.funct7b5(funct7b5), 
.ALUOp(ALUOpD),
.ALUControl(ALUControlD)
);

C_D_E_REG CDEREG(
.clk(clk),
.reset(reset),
.clear(FlushE),
.RegWriteD(RegWriteD),
.MemWriteD(MemWriteD),
.JumpD(JumpD),
.BranchD(BranchD),
.ALUSrcAD(ALUSrcAD),
.ALUSrcBD(ALUSrcBD),
.ResultSrcD(ResultSrcD),
.ALUControlD(ALUControlD),
.funct3D(funct3D),
.RegWriteE(RegWriteE),
.MemWriteE(MemWriteE),
.JumpE(JumpE),
.BranchE(BranchE),
.ALUSrcAE(ALUSrcAE),
.ALUSrcBE(ALUSrcBE),
.ResultSrcE(ResultSrcE),
.ALUControlE(ALUControlE),
.funct3E(funct3E)
);



C_E_M_REG CEMREG(
.clk(clk), 
.reset(reset),
.RegWriteE(RegWriteE),
.MemWriteE(MemWriteE),
.ResultSrcE(ResultSrcE),  
.RegWriteM(RegWriteM),
.MemWriteM(MemWriteM),
.ResultSrcM(ResultSrcM)
);

C_M_W_REG CMWREG(
.clk(clk),
.reset(reset), 
.RegWriteM(RegWriteM), 
.ResultSrcM(ResultSrcM), 
.RegWriteW(RegWriteW), 
.ResultSrcW(ResultSrcW)
);

assign ZeroOp = ZeroE ^ funct3E[0];        // Complements Zero flag for BNE Instruction
assign SignOp = (SignE ^ funct3E[0]) ;     //Complements Sign for BGE
assign BranchOp = funct3E[2] ? (SignOp) : (ZeroOp); 
assign PCSrcE = (BranchE & BranchOp) | JumpE;
assign ResultSrcE0 = ResultSrcE[0];
assign PCJalSrcE = (op == 7'b1100111) ? 1 : 0; // jalr

endmodule

///////////////////////////////////////////////////////////////////////////
///////////////////////// HAZARD UNIT /////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
module HazardUnit (
  input logic [4:0] Rs1D,Rs2D,
  input logic [4:0] Rs1E,Rs2E,
  input logic [4:0] RdE,RdM,RdW,
  input logic       ResultSrcE0,PCSrcE,
  input logic       RegWriteW,RegWriteM,

  output logic StallF,StallD,FlushD,FlushE,
  output logic [1:0] ForwardAE,ForwardBE
  );

logic lwStall;

always_comb begin 
  ForwardAE = 2'b00;
  
  if ((Rs1E == RdM) & (RegWriteM) & (Rs1E != 0))begin
    ForwardAE = 2'b10;   // for forwarding ALU Result in Memory Stage
  end

  else if ((Rs1E == RdW) & (RegWriteW) & (Rs1E != 0)) begin
  ForwardAE = 2'b01;  // for forwarding WriteBack Stage Result
  end

end

always_comb begin
  ForwardBE = 2'b00;

  if ((Rs2E == RdM) & (RegWriteM) & (Rs2E != 0))begin
    ForwardBE = 2'b10; // for forwarding ALU Result in Memory Stage
  end  

  else if (((Rs2E == RdW) & RegWriteW) & (Rs2E != 0))begin
    ForwardBE = 2'b01; // for forwarding WriteBack Stage Result
  end 

end

assign lwStall = (ResultSrcE0 == 1) & ((RdE == Rs1D) | (RdE == Rs2D));
assign StallF = lwStall ;
assign StallD = lwStall ;
assign FlushD = PCSrcE;
assign FlushE = lwStall | PCSrcE;

endmodule

///////////////////////////////////////////////////////////////////////////
///////////////////// DadtaPath PIPELINING REGISTERS //////////////////////
///////////////////////////////////////////////////////////////////////////
/*fetch decode reigister*/
module DP_F_D_REG (
  input logic clk,reset,clear,enable,
  input logic [31:0] InstrF,PCF,PCPlus4F,
  output logic [31:0] InstrD, PCD, PCPlus4D
);
always_ff @( posedge clk, posedge reset ) begin
    if (reset) begin // Asynchronous Clear
        InstrD <= 0;
        PCD <= 0;
        PCPlus4D <= 0;
    end

    else if (enable) begin 
		 if (clear) begin // Synchrnous Clear
			  InstrD <= 0;
			  PCD <= 0;
			  PCPlus4D <= 0;	 
		 end
		 
		 else begin	 
			  InstrD <= InstrF;
			  PCD <= PCF;
			  PCPlus4D <= PCPlus4F;
		 end
	 end
end
endmodule

/*decode executeregister*/
module DP_D_E_REG(
  input logic clk, reset, clear,
  input logic [31:0] RD1D, RD2D, PCD, 
  input logic [4:0] Rs1D, Rs2D, RdD, 
  input logic [31:0] ImmExtD, PCPlus4D,
  output logic [31:0] RD1E, RD2E, PCE, 
  output logic [4:0] Rs1E, Rs2E, RdE, 
  output logic [31:0] ImmExtE, PCPlus4E
  );

  always_ff @( posedge clk, posedge reset ) begin
        if (reset) begin
            RD1E <= 0;
            RD2E <= 0;
            PCE <= 0;
            Rs1E <= 0;
            Rs2E <= 0;
            RdE <= 0;
            ImmExtE <= 0;
            PCPlus4E <= 0;
        end
        else if (clear) begin
            RD1E <= 0;
            RD2E <= 0;
            PCE <= 0;
            Rs1E <= 0;
            Rs2E <= 0;
            RdE <= 0;
            ImmExtE <= 0;
            PCPlus4E <= 0;
        end
        else begin
            RD1E <= RD1D;
            RD2E <= RD2D;
            PCE <= PCD;
            Rs1E <= Rs1D;
            Rs2E <= Rs2D;
            RdE <= RdD;
            ImmExtE <= ImmExtD;
            PCPlus4E <= PCPlus4D;
        end
    end
endmodule

/* execute mem register*/
module DP_E_M_REG(
  input logic clk, reset,
  input logic PCJalSrcE,
  input logic [31:0] ALUResultE, WriteDataE, 
  input logic [4:0] RdE, 
  input logic [31:0] PCPlus4E,

  output logic [31:0] ALUResultM, WriteDataM,
  output logic [4:0]  RdM, 
  output logic [31:0] PCPlus4M,
  output logic        PCJalSrcM
);

always_ff @( posedge clk, posedge reset ) begin 
    if (reset) begin
        ALUResultM <= 0;
        WriteDataM <= 0;
        RdM <= 0; 
        PCPlus4M <= 0;
        PCJalSrcM <= 0 ;
    end

    else begin
        ALUResultM <= ALUResultE;
        WriteDataM <= WriteDataE;
        RdM <= RdE; 
        PCPlus4M <= PCPlus4E; 
        PCJalSrcM <= PCJalSrcE;
    end
end
endmodule


/* mem write back register*/
module DP_M_W_REG(
  input logic clk, reset,
  input logic [31:0] ALUResultM, ReadDataM,  
  input logic [4:0] RdM, 
  input logic [31:0] PCPlus4M,
  output logic [31:0] ALUResultW, ReadDataW,
  output logic [4:0] RdW, 
  output logic [31:0] PCPlus4W
  );

always_ff @( posedge clk, posedge reset ) begin 
    if (reset) begin
        ALUResultW <= 0;
        ReadDataW <= 0;
        
        RdW <= 0; 
        PCPlus4W <= 0;
    end
    else begin
        ALUResultW <= ALUResultM;
        ReadDataW <= ReadDataM;
        
        RdW <= RdM; 
        PCPlus4W <= PCPlus4M;        
    end
end
endmodule
///////////////////////////////////////////////////////////////////////////
//////////////////// Controller PIPELINING REGISTERS //////////////////////
///////////////////////////////////////////////////////////////////////////
/* decode Execute register */
module C_D_E_REG(
  input logic clk, reset, clear,
  input logic RegWriteD, MemWriteD, JumpD, BranchD, ALUSrcAD,
	input logic [1:0] ALUSrcBD,
  input logic [1:0] ResultSrcD, 
  input logic [3:0] ALUControlD, 
  input logic [2:0] funct3D,          
  output logic RegWriteE, MemWriteE, JumpE, BranchE,  ALUSrcAE,
	output logic [1:0] ALUSrcBE,
  output logic [1:0] ResultSrcE,
  output logic [3:0] ALUControlE, 
  output logic [2:0] funct3E
  );

always_ff @( posedge clk , posedge reset ) begin

		if (reset) begin
      funct3E <= 0;
			RegWriteE <= 0;
			MemWriteE <= 0;
			JumpE <= 0;
			BranchE <= 0; 
			ALUSrcAE <= 0;
			ALUSrcBE <= 0;
			ResultSrcE <= 0;
			ALUControlE <= 0;          
		end
		else if (clear) begin
      funct3E <= 0;
			RegWriteE <= 0;
			MemWriteE <= 0;
			JumpE <= 0;
			BranchE <= 0; 
			ALUSrcAE <= 0;
			ALUSrcBE <= 0;
			ResultSrcE <= 0;
			ALUControlE <= 0;    			
		end
		else begin
      funct3E <= funct3D;
			RegWriteE <= RegWriteD;
			MemWriteE <= MemWriteD;
			JumpE <= JumpD;
			BranchE <= BranchD; 
			ALUSrcAE <= ALUSrcAD;
			ALUSrcBE <= ALUSrcBD;
			ResultSrcE <= ResultSrcD;
			ALUControlE <= ALUControlD;   
		end
	 end
endmodule

/* Execute mem register */
module C_E_M_REG(
  input logic clk, reset,
  input logic RegWriteE, MemWriteE,
  input logic [1:0] ResultSrcE,  
  output logic RegWriteM, MemWriteM,
  output logic [1:0] ResultSrcM
  );

  always_ff @( posedge clk, posedge reset ) begin
    if (reset) begin
        RegWriteM <= 0;
        MemWriteM <= 0;
        ResultSrcM <= 0;
    end
    else begin
        RegWriteM <= RegWriteE;
        MemWriteM <= MemWriteE;
        ResultSrcM <= ResultSrcE; 
    end
  end
endmodule

/* mem write back register */
module C_M_W_REG(
  input logic clk, reset, 
  input logic RegWriteM, 
  input logic [1:0] ResultSrcM, 
  output logic RegWriteW, 
  output logic [1:0] ResultSrcW
  );
  always_ff @( posedge clk, posedge reset ) begin
    if (reset) begin
            RegWriteW <= 0;
            ResultSrcW <= 0;           
      end
    else begin
            RegWriteW <= RegWriteM;
            ResultSrcW <= ResultSrcM; // lol this wasted 1 hour
      end
  end
endmodule

///////////////////////////////////////////////////////////////////////////
//////////////////////// Fetch stage submodules////////////////////////////
///////////////////////////////////////////////////////////////////////////
module flopenr (
  input logic clk, reset, en,
	input logic [31:0] d,
	output logic [31:0] q);
					
	always_ff @(posedge clk, posedge reset)
		
		if (reset) q <= 0;
		else if (en) q <= d;
endmodule


//adder
module adder(input  [31:0] a, b,
             output [31:0] y);
  assign y = $signed(a) + $signed(b);
endmodule

//mux 2x1
module mux2 (input  logic [31:0] d0, d1, 
             input  logic            s, 
             output logic [31:0] y);
  assign y = s ? d1 : d0; 
endmodule

//mux3 --- in the execution stage 
module mux3  (input  logic [31:0] d0, d1, d2,
              input  logic [1:0]       s, 
              output logic [31:0] y);

  assign y = s[1] ? d2 : (s[0] ? d1 : d0); 
endmodule


///////////////////////////////////////////////////////////////////////////
//////////////////////// Decode stage submodules///////////////////////////
//////////////////////////////////////////////////////////////////////////


/*
This unit generates the control signals from the 7 bit opcode.
Determines the type of instruction
*/
module maindec(input  logic [6:0] op,
               output logic [1:0] ResultSrc,
               output logic       MemWrite,
               output logic       Branch, ALUSrcA,
               output logic       RegWrite, Jump,
               output logic [2:0] ImmSrc,
               output logic [1:0] ALUOp,ALUSrcB);

  logic [13:0] controls;
  assign {RegWrite, ImmSrc, ALUSrcA, ALUSrcB, MemWrite,ResultSrc, Branch, ALUOp, Jump} = controls;

  always_comb
    case(op)
        // RegWrite_ImmSrc_ALUSrc_MemWrite_ResultSrc_Branch_ALUOp_Jump
        7'b0000011: controls = 14'b1_000_0_01_0_01_0_00_0; // lw
		    7'b0100011: controls = 14'b0_001_0_01_1_00_0_00_0; // sw
		    7'b0110011: controls = 14'b1_xxx_0_00_0_00_0_10_0; // R–type
		    7'b1100011: controls = 14'b0_010_0_00_0_00_1_01_0; // B-type
		    7'b0010011: controls = 14'b1_000_0_01_0_00_0_10_0; // I–type ALU
		    7'b1101111: controls = 14'b1_011_0_00_0_10_0_00_1; // jal
		    7'b0010111: controls = 14'b1_100_1_10_0_00_0_00_0; // auipc // PC Target for SrcB
		    7'b0110111: controls = 14'b1_100_1_01_0_00_0_00_0; // lui
		    7'b1100111: controls = 14'b1_000_0_01_0_10_0_00_1; // jalr
		    7'b0000000: controls = 14'b0_000_0_00_0_00_0_00_0; // for default values on reset
		    default: 	controls = 14'bx_xxx_x_xx_x_xx_x_xx_x; // instruction not implemented
    endcase
endmodule

/*
ALU Decoder
Receives control signal from the Main Decoder Unit and 
determines the type of operation that has to be performed by the ALU
*/
module aludec(input  logic       opb5,
              input  logic [2:0] funct3,
              input  logic       funct7b5, 
              input  logic [1:0] ALUOp,
              output logic [3:0] ALUControl);

  logic  RtypeSub;
  assign RtypeSub = funct7b5 & opb5;  // TRUE for R-type subtract instruction

  always_comb
    case(ALUOp)
      2'b00:                ALUControl = 4'b0000; // addition
      2'b01:                ALUControl = 4'b0001; // subtraction
      default: case(funct3) // R-type or I-type ALU
                 3'b000:  if (RtypeSub) 
                            ALUControl = 4'b0001; // sub
                          else          
                            ALUControl = 4'b0000; // add, addi
                 3'b001: ALUControl = 4'b0100; // sll, slli
			           3'b010: ALUControl = 4'b0101; // slt, slti
			           3'b011: ALUControl = 4'b1000; // sltu, sltiu
			           3'b100: ALUControl = 4'b0110; // xor, xori
			           3'b101: if (~funct7b5)
			          	          ALUControl = 4'b0111;	// srl
			                  else
			           	          ALUControl = 4'b1111;  // sra
			           3'b110: ALUControl = 4'b0011; // or, ori
			           3'b111: ALUControl = 4'b0010; // and, andi
			          default: ALUControl = 4'bxxxx; // ???
			    endcase
    endcase
endmodule


/* Register File */
module regfile(input  logic        clk, 
               input  logic        we3, 
               input  logic [ 4:0] a1, a2, a3, 
               input  logic [31:0] wd3, 
               output logic [31:0] rd1, rd2
               );

  logic [31:0] registers [31:0];


  /*
  three ported register file
  read two ports combinationally on rising edge (A1/RD1, A2/RD2)
  write third port on falling edge of clock (A3/WD3/WE3)
  register 0 hardwired to 0
  */

  always_ff @(negedge clk)
    if (we3)begin
      registers[a3] <= wd3;
    end 	

  assign rd1 = (a1 != 0) ? registers[a1] : 0;
  assign rd2 = (a2 != 0) ? registers[a2] : 0;
  
endmodule

/* Extend unit */
module extend(
  input  logic [31:7] instr,
   input  logic [2:0]  ImmSrc,
  output logic [31:0] ImmExt
  );

  always_comb
    case(ImmSrc)             
      3'b000:   ImmExt = {{20{instr[31]}}, instr[31:20]};   // I-type         
      3'b001:   ImmExt = {{20{instr[31]}}, instr[31:25], instr[11:7]};  // S-type (stores)            
      3'b010:   ImmExt = {{20{instr[31]}}, instr[7], instr[30:25], instr[11:8], 1'b0};  // B-type (branches)           
      3'b011:   ImmExt = {{12{instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0};  // J-type (jal)
      3'b100:   ImmExt = {instr[31:12],12'b0};   // U-type (jal)
      default:  ImmExt = 32'bx; // undefined
    endcase             
endmodule

///////////////////////////////////////////////////////////////////////////
//////////////////////// Execute stage submodules//////////////////////////
///////////////////////////////////////////////////////////////////////////
//ALU UNIT 
module alu(input  logic [31:0] a, b,
           input  logic [3:0]  alucontrol,
           output logic [31:0] result,
           output logic        zero,Sign);

  logic [31:0] condinvb, sum;
  logic        overflow;             

  assign condinvb = alucontrol[0] ? ~b : b;
  assign sum = a + condinvb + {{31{1'b0}},alucontrol[0]}; //sub using 1's comp
  assign overflow = ~(alucontrol[0] ^ b[31] ^ a[31]) & (a[31] ^ sum[31]) & (~alucontrol[1]);

  always_comb
   casex (alucontrol)
				4'b000x: result = sum;				// sum or diff
				4'b0010: result = a & b;	// and
				4'b0011: result = a | b;	// or
				4'b0100: result = a << b[4:0];// sll, slli
				4'b0101: result = {{30{1'b0}}, overflow ^ sum[31]}; //slt, slti
				4'b0110: result = a ^ b;   // Xor
				4'b0111: result = a >> b[4:0];  // shift logic
				4'b1000: result = ($unsigned(a) < $unsigned(b)); //sltu, stlui
				4'b1111: result = a >>> b[4:0]; //shift arithmetic
				default: result = 32'bx;
		endcase

  assign zero = ~(|result);
  assign Sign = result[31];
endmodule


