import numpy as np
import math
import random
import networkx as nx

TEMPLATE = """
module switch_{name} #(
    parameter type data_t = logic,
    // automatically generated
    parameter type addr_t = logic [$clog2({num_out})-1:0]
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

    typedef struct packed {{
        data_t data;
        addr_t addr;
    }} packet_t;

    packet_t [{num_in}-1:0] packets;

    for (genvar i = 0; i < {num_in}; i++) begin : gen_packets
        assign packets[i].data = data_in[i];
        assign packets[i].addr = addr_in[i];
    end

    // automatically generated:
    {src}

endmodule;
"""

FILTER = """
    logic [0:0] filter_{in_p}_valid;
    logic [0:0] filter_{in_p}_ready;
    stream_filter i_filt{in_p} (
        .valid_i(valid_i[{in_p}]),
        .ready_o(ready_o[{in_p}]),
        .drop_i({sel}),
        .valid_o(filter_{in_p}_valid),
        .ready_i(filter_{in_p}_ready)
    );
"""

DEMUX = """
    logic [$clog2({num_out})-1:0] demux_{in_p}_sel;
    logic [{num_out}-1:0] demux_{in_p}_valid;
    logic [{num_out}-1:0] demux_{in_p}_ready;
    stream_demux #(
        .N_OUP({num_out})
    ) i_demux{in_p} (
        .inp_valid_i(filter_{in_p}_valid),
        .inp_ready_o(filter_{in_p}_ready),
        .oup_sel_i(demux_{in_p}_sel),
        .oup_valid_o(demux_{in_p}_valid),
        .oup_ready_i(demux_{in_p}_ready)
    );

    always_comb begin
        case(addr_in[{in_p}])
            {cases}
            default : demux_{in_p}_sel = 0;
        endcase
    end
"""

DEMUX_BYPASS = """
    logic [0:0] demux_{in_p}_valid;
    logic [0:0] demux_{in_p}_ready;
    assign demux_{in_p}_valid = filter_{in_p}_valid;
    assign filter_{in_p}_ready = demux_{in_p}_ready;
"""

LOAD_BALANCE = """
    // balance packets from input {in_p} with destination {dst}
    // to {num_out} outputs
    logic [{num_out}-1:0] bal_{in_p}_{dst}_valid;
    logic [{num_out}-1:0] bal_{in_p}_{dst}_ready;
    stream_balance #(
        .NumOut({num_out}),
        .BLen({bal_len}),
        .BTable({{{btable}}})
    ) i_bal_{in_p}_{dst} (
        .clk_i,
        .rst_ni,
        .valid_i(demux_{in_p}_valid[{key}]),
        .ready_o(demux_{in_p}_ready[{key}]),
        .valid_o(bal_{in_p}_{dst}_valid),
        .ready_i(bal_{in_p}_{dst}_ready)
    );
"""

ARBITER = """
    rr_arb_tree #(
        .NumIn({num_in}),
        .DataType(packet_t),
        .AxiVldRdy(1'b1),
        .LockIn(1'b1)
    ) i_q_mux (
        .clk_i,
        .rst_ni,
        .flush_i(1'b0),
        .rr_i('0),
        .req_i({valid_i}),
        .gnt_o({ready_o}),
        .data_i({data_i}),
        .req_o(valid_o[{idx_out}]),
        .gnt_i(ready_i[{idx_out}]),
        .data_o({{data_out[{idx_out}], addr_out[{idx_out}]}}),
        .idx_o()
    );
"""

def to_graph(route_table: np.ndarray) -> nx.MultiDiGraph:
    """
    Inputs:
        route_table: (H_dst, H_out, H_in)
    """
    G = nx.MultiDiGraph()

    for in_p in range(route_table.shape[2]):
        G.add_node(f'in{in_p}', index=in_p)

    for out_p in range(route_table.shape[1]):
        G.add_node(f'out{out_p}', index=out_p)

    print(route_table)

    for in_p in range(route_table.shape[2]):
        for dst in range(route_table.shape[0]):
            # check if input has a routing rule for this dst
            if np.count_nonzero(route_table[dst, :, in_p]) > 1:
                G.add_node(f'lb{in_p}[{dst}]', dst=dst, index=in_p)
                G.add_edge(f'in{in_p}', f'lb{in_p}[{dst}]', dst=dst)
            for out_p in range(route_table.shape[1]):
                w = route_table[dst, out_p, in_p]
                if w > 0 and np.count_nonzero(route_table[dst, :, in_p]) > 1:
                    G.add_edge(f'lb{in_p}[{dst}]', f'out{out_p}', weight=w, dst=dst)
                elif w > 0:
                    G.add_edge(f'in{in_p}', f'out{out_p}', dst=dst)
    return G

def build_sv(G: nx.MultiDiGraph, num_in, num_dst, num_out):
    src = []
    in_nodes = [n for n in G.nodes() if n.startswith('in')]
    for in_p in in_nodes:
        idx = G.nodes[in_p]['index']
        num_node_out = len(G.out_edges(in_p))
        if num_node_out == 0:
            continue
        filt = []
        cases = []
        for i, e in enumerate(G.out_edges(in_p, keys=True)):
            dst = G.edges[e[0], e[1], e[2]]['dst']
            filt.append(f"addr_in[{idx}] != {i}")
            G.edges[e[0], e[1], e[2]]['index'] = i;
            cases.append(f"{dst}: demux_{idx}_sel = {i};")
        filt = ' && '.join(filt)
        cases = '\n\t\t\t'.join(cases)
        src.append(FILTER.format(in_p=idx, sel=filt))
        if num_node_out == 1:
            src.append(DEMUX_BYPASS.format(in_p=idx))
        else:
            src.append(DEMUX.format(num_out=num_node_out, in_p=idx, cases=cases))

    lb_nodes = [n for n in G.nodes() if n.startswith('lb')]
    for lb in lb_nodes:
        idx = G.nodes[lb]['index']
        dst = G.nodes[lb]['dst']
        weights = [e[2]['weight'] for e in G.out_edges(lb, data=True)]
        num_node_out = len(G.out_edges(in_p))
        blen = int(sum(weights))
        btable = [[i]*w.astype(int) for i, w in enumerate(weights)]
        bwidth = math.ceil(math.log2(blen))
        btable = [f"{bwidth}'d{i}" for b in btable for i in b]
        random.shuffle(btable)
        btable = ', '.join(btable)
        e = list(G.in_edges(lb, data=True))
        assert len(e) == 1
        key = e[0][2]['index']
        src.append(LOAD_BALANCE.format(num_out=num_node_out, in_p=idx, dst=dst, bal_len=blen, btable=btable, key=key))

    return TEMPLATE.format(name='test', num_out=num_out, num_in=num_in, src='\n'.join(src))

def quantize(table: np.ndarray, zero_tol: float) -> np.ndarray:
    """
    Inputs:
        route_table: (H_dst, H_out, H_in)
    """
    table[table < zero_tol] = 0
    mini = np.min(table, axis=1, initial=1, where=(table>0))
    mini[mini == 0] = 1
    qtable = table / mini.reshape(mini.shape[0], 1, mini.shape[1])
    qtable = np.round(qtable)
    return qtable


test = np.random.rand(2, 2, 2)
test = quantize(test, 0.3)
G = to_graph(test)

txt = build_sv(G, test.shape[2], test.shape[0], test.shape[1])

with open('out.sv', 'w') as f:
    f.write(txt)

# nx.drawing.nx_pydot.write_dot(G, 'test.dot')