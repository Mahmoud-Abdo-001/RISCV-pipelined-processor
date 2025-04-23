module imem #(
    parameter string IMEM_FILE = "",
    parameter int depth = 1024,
    parameter int width = 32
)(
    input  logic reset,  // Active-low reset
    input  logic [$clog2(depth)-1:0] a,
    output logic [width-1:0] rd
);

logic [width-1:0] RAM [depth-1:0];

// **Initial Block for Memory Initialization**
initial begin
    // for (int i = 0; i < depth; i++) begin
        // RAM[i] = 'h0;
    // end
    if (IMEM_FILE != "") begin
        $readmemh(IMEM_FILE, RAM);
    end
end

// **Word-Aligned Read Operation**
assign rd = (reset) ? {32{1'b0}} : RAM[a[$clog2(depth)-1:2]];

endmodule

