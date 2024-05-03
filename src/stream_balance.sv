module stream_balance #(
    parameter int unsigned NumOut = 2,
    parameter int unsigned BLen = 2,
    parameter type addr_t = logic [$clog2(NumOut)-1:0],
    parameter addr_t [BLen-1:0] BTable  = '0
) (
    input  logic clk_i,
    input  logic rst_ni,

    input  logic valid_i,
    output logic ready_o,

    output logic [NumOut-1:0] valid_o,
    input  logic [NumOut-1:0] ready_i
);

    logic [BLen-1:0] shift_q, shift_n;

    always_comb begin
        valid_o = '0;
        ready_o = '0;
        shift_n = shift_q;
        for (int i = 0; i < BLen; i++) begin
            if (shift_q[i]) begin
                valid_o[BTable[i]] = valid_i;
                ready_o = ready_i[BTable[i]];
            end
        end
        if (valid_i & ready_o) begin
            shift_n = shift_q << 1;
        end
    end

    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            shift_q <= BLen'('b1);
        end else begin
            shift_q <= shift_n;
        end
    end

endmodule;
