module TOP_DUT (
    /* Clock */
    input logic clk, reset
    );
				
	logic [31:0] PCF, InstrF;
	logic [31:0] DataAdrM , WriteDataM , ReadDataM;
    logic MemWriteM ; 

	
	RISCV32C RV32( 
    .clk(clk), 
    .reset(reset), 
    .PCF(PCF), 
    .InstrF(InstrF), 
    .MemWriteM(MemWriteM), 
    .ALUResultM(DataAdrM), 
    .WriteDataM(WriteDataM), 
    .ReadDataM(ReadDataM)
    );

	imem #(
        .IMEM_FILE("D:\\CPU\\RVPIPELINED32\\RISCVpipeline\\firmware2\\firmware.hex"),
        .depth(1024),
        .width(32)
    )IM(
    .a(PCF[9:0]),
    .reset(reset),
    .rd(InstrF)
    );

	dmem #(
        .DMEM_FILE("D:\\CPU\\RVPIPELINED32\\RISCVpipeline\\firmware\\DMEM.hex"),
        .depth(1024),
        .width(32)
    )DM(
        .clk(clk),
		.reset(reset),
        .we(MemWriteM),
        .a(DataAdrM[9:0]),
        .wd(WriteDataM),
        .rd(ReadDataM)
        );

endmodule