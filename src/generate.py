import os
import random
import numpy as np

TEMPLATE = """
module switch_{name} #(
    parameter type data_t = logic,
    // automatically generated
    parameter type addr_t = logic [$clog2(num_out)-1:0]
) (
    input  logic clk_i,
    input  logic rst_ni,

    input  data_t [{num_in}-1:0] data_in,
    input  addr_t [{num_in}-1:0] addr_in,

    input  logic  [{num_in}-1:0] valid_i,
    output logic  [{num_in}-1:0] ready_o,

    output data_t [{num_out}-1:0] data_out,
    output addr_t [{num_out}-1:0] addr_out,
    output logic  [{num_out}-1:0] valid_o,
    input  logic  [{num_out}-1:0] ready_i
);

    typedef struct packed {
        data_t data;
        addr_t addr;
    } packet_t;

    packet_t [NumIn-1:0] packets;

    for (genvar i = 0; i < {num_in}; i++) begin : gen_packets
        packets[i].data = data_in[i];
        packets[i].addr = addr_in[i];
    end

    // automatically generated:
    {}

endmodule;
"""


def gen_switch(route_table: np.ndarray) -> str:
    """
    Inputs:
        route_table: (H_dst, H_out, H_in)
    """

    src = []

    # route_table[i,j,k] = Pr that a packet at input k with destination i is routed to j
    # load balance: at a particular input the same dst goes to multiple outs

    for in_p in range(route_table.shape[2]):
        num_dst = route_table.shape[0]
        cases = [f'{j} : split_{in_p}_sel = {j};' for j in range(num_dst)]
        splitter = f"""
            logic [$clog2({num_dst})-1:0] split_{in_p}_sel;
            logic [{num_dst}-1:0] split_{in_p}_valid;
            logic [{num_dst}-1:0] split_{in_p}_ready;
            stream_demux #(
                .N_OUP(NumOut)
            ) i_split{in_p} (
                .inp_valid_i(valid_i[{in_p}]),
                .inp_ready_o(ready_o[{in_p}]),
                .oup_sel_i(split_{in_p}_sel),
                .oup_valid_o(split_{in_p}_valid),
                .oup_ready_i(split_{in_p}_ready)
            );

            always_comb begin
                case(addr_in[{in_p}])
                    {'\n\t\t'.join(cases)}
                    default :;
                endcase
            end
        """
        src.append(splitter)

        bal = f"""
            logic [{route_table.shape[0]}-1:0][{route_table.shape[1]}-1:0] bal_valid;
            logic [{route_table.shape[0]}-1:0][{route_table.shape[1]}-1:0] bal_ready;
        """

        for dst in range(route_table.shape[1]):
            out_count = np.count_nonzero(route_table[:,dst,in_p])
            bal_len = np.sum(route_table[:, dst, in_p])
            if out_count > 1:
                btable = [[i]*route_table[i, dst, in_p] for i in range(route_table.shape[0]) if route_table[i, dst, in_p] > 0]
                btable = [i for j in btable for i in j]
                btable = "{" + ", ".join(random.shuffle(btable)) + "}"
                balance = f"""
                    logic [{out_count}-1:0] bal_{in_p}_{dst}_valid;
                    logic [{out_count}-1:0] bal_{in_p}_{dst}_ready;
                    stream_balance #(
                        .NumOut({out_count}),
                        .BLen({bal_len}),
                        .BTable({btable})
                    ) i_bal_{in_p}_{dst} (
                        .clk_i,
                        .rst_ni,
                        .valid_i(split_{in_p}_valid[{dst}]),
                        .ready_o(split_{in_p}_ready[{dst}]),
                        .valid_o(bal_{in_p}_{dst}_valid),
                        .ready_i(bal_{in_p}_{dst}_ready),
                    );
                """
                src.append(balance)
            elif out_count == 1:
                direct = f"""
                    logic [0:0] bal_{in_p}_{dst}_valid;
                    logic [0:0] bal_{in_p}_{dst}_ready;
                    assign bal_{in_p}_{dst}_valid[0] = split_{in_p}_valid[{dst}];
                    assign split_{in_p}_ready[{dst}] = bal_{in_p}_{dst}_ready[0];
                """
                src.append(direct)

        for out in range(route_table.shape[1]):
            # for a particular output, determine all inputs
            for i in range(route_table.shape[2]):
                route_table[:, out, i]
            num_ins = np.sum(route_table[:, out, :])
            arb = f"""
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
                .req_i({{ foo }}),
                .gnt_o({{ foo }}),
                .data_i({{ foo }}),
                .req_o(valid_o[{out}]),
                .gnt_i(ready_i[{out}]),
                .data_o({{data_out[{out}], addr_out[{out}]}}),
                .idx_o()
            );
            """
    return "\n".join(src)

# num_dst x num_out
test = np.random.rand(5, 3)
test = np.exp(test)
test /= np.sum(test, axis=1)[:, None]

gen_switch(test, 8, 0.05)