package ic_pkg;
    localparam type data_t = logic [63:0];
endpackage;

module top (
    input logic clk_i,
    input logic rst_i,

    input ic_pkg::data_t [31:0] in_i,
    input logic [31:0]          valid_i,
    input logic [31:0]          ready_o,

    output ic_pkg::data_t [31:0] out_i,
    output logic [31:0]         valid_o,
    input  logic [31:0]         ready_i,
);

    for (genvar i = 0; i < $clog2())

endmodule;
