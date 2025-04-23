module dmem #(
    parameter string DMEM_FILE = "",
    parameter int depth = 256,
    parameter int width = 32
)(
    input  logic clk, reset, we,   // Active-low reset
    input  logic [$clog2(depth)-1:0] a, 
    input  logic [width-1:0] wd,
    output logic [width-1:0] rd
);

logic [width-1:0] RAM [depth-1:0];  // 256 x 32-bit memory

// **Memory Initialization (Only Runs Once)**
initial begin
    if (DMEM_FILE != "") begin
        $readmemh(DMEM_FILE, RAM);
    end
end

// **Synchronous Reset and Write Operation**
always_ff @(posedge clk) begin
    if (!reset) begin  // Active-low reset
        for (int i = 0; i < depth; i++) 
            RAM[i] <= '0;
    end 
    else if (we) begin
        RAM[a[$clog2(depth)-1:2]] <= wd;
    end
end

// **Word-Aligned Read Operation**
assign rd = RAM[a[$clog2(depth)-1:2]];

endmodule
