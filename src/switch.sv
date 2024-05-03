module switch #(
    parameter int unsigned NumIn = 0,
    parameter int unsigned NumOut = 0,
    parameter int unsigned NumDst = 0,
    parameter type data_t = logic,
    parameter int unsigned RouteTable [NumDst-1:0][NumOut-1:0] = '0,
    // derived
    parameter type addr_t = logic [$clog2(NumDst)-1:0]
) (
    input  logic clk_i,
    input  logic rst_ni,

    input  data_t [NumIn-1:0] data_in,
    input  addr_t [NumIn-1:0] addr_in,

    input  logic  [NumIn-1:0] valid_i,
    output logic  [NumIn-1:0] ready_o,

    output data_t [NumOut-1:0] data_out,
    output addr_t [NumOut-1:0] addr_out,
    output logic  [NumOut-1:0] valid_o,
    input  logic  [NumOut-1:0] ready_i
);

    typedef struct packed {
        data_t data;
        addr_t addr;
    } packet_t;

    logic req, gnt;
    packet_t sel_packet;

    logic [$clog2(NumOut)-1:0] sel;

    // input arbitration

    packet_t [NumIn-1:0] packet_in;

    rr_arb_tree #(
        .NumIn,
        .DataType(packet_t),
        .AxiVldRdy(1'b1),
        .LockIn(1'b1)
    ) i_q_mux (
        .clk_i,
        .rst_ni,
        .flush_i(1'b0),
        .rr_i('0),
        .req_i(valid_i),
        .gnt_o(ready_o),
        .data_i(packet_in),
        .req_o(req),
        .gnt_i(gnt),
        .data_o(sel_data),
        .idx_o()
    );

    for (genvar i = 0; i < NumIn; i++) begin
        assign packet_in[i].data = data_in[i];
        assign packet_in[i].addr = addr_in[i];
    end

    // output demux

    stream_demux #(
        .N_OUP(NumOut)
    ) i_out_mux (
        .inp_valid_i(req),
        .inp_ready_o(gnt),
        .oup_sel_i(sel),
        .oup_valid_o(valid_o),
        .oup_ready_i(ready_i)
    );

    for (genvar i = 0; i < NumOut; i++) begin
        assign data_out = sel_packet.data;
        assign addr_out = sel_packet.addr;
    end

    // implement route table:
    // - add an arbiter for each input that has multiple dsts

    logic [NumDst-1:0][$clog2(NumOut)-1:0] lookup;

    for (genvar i = 0; i < NumDst; i++) begin
        if ($countones(RouteTable[i]) > 1) begin
            // load balance between destinations
            
        end else begin
            for (genvar j = 0; j < NumOut; j++) begin
                if (RouteTable[i][j] == 1'b1) begin
                    // packets with dst i are always routed to output j
                    assign lookup[i] = j;
                end
            end
        end
    end

    assign sel = lookup[sel_packet.addr];
endmodule;
