import numpy as np
import math
import random
import networkx as nx
import matplotlib.pyplot as plt

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
    // remove packets from input {in_p} if their destination 
    // is not present in the routing table
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
    // demux packets from input {in_p} for {num_out} destinations
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
    // packets from {in_p} are always routed the same way
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
    // arbitrate access for {num_in} connections to output
    // port {idx_out}
    rr_arb_tree #(
        .NumIn({num_in}),
        .DataType(packet_t),
        .AxiVldRdy(1'b1),
        .LockIn(1'b1)
    ) i_arb_{idx_out} (
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
    G = nx.DiGraph()

    for in_p in range(route_table.shape[2]):
        G.add_node(f'in{in_p}', in_p=in_p)

    for out_p in range(route_table.shape[1]):
        G.add_node(f'out{out_p}', out_p=out_p)

    print(route_table)

    for in_p in range(route_table.shape[2]):
        for dst in range(route_table.shape[0]):
            # check if input has a routing rule for this dst
            if np.count_nonzero(route_table[dst, :, in_p]) > 1:
                G.add_node(f'lb{in_p}[{dst}]', dst=dst, in_p=in_p)
                G.add_edge(f'in{in_p}', f'lb{in_p}[{dst}]', dst=[dst], in_p=in_p)
            for out_p in range(route_table.shape[1]):
                w = route_table[dst, out_p, in_p]
                if w > 0 and np.count_nonzero(route_table[dst, :, in_p]) > 1:                 
                    G.add_edge(f'lb{in_p}[{dst}]', f'out{out_p}', weight=w, in_p=in_p, out_p=out_p, dst=[dst])
                elif w > 0 and G.has_edge(f'in{in_p}', f'out{out_p}'):
                    G.edges[f'in{in_p}', f'out{out_p}']['dst'].append(dst)
                elif w > 0:
                    G.add_edge(f'in{in_p}', f'out{out_p}', in_p=in_p, out_p=out_p, dst=[dst])
    return G

def build_sv(G: nx.DiGraph, num_in, num_dst, num_out):
    src = []
    in_nodes = [n for n in G.nodes() if n.startswith('in')]
    for in_n in in_nodes:
        in_p = G.nodes[in_n]['in_p']
        num_node_out = len(G.out_edges(in_n))
        if num_node_out == 0:
            continue
        filt = []
        cases = []
        for i, e in enumerate(G.out_edges(in_n)):
            dsts = G.edges[e[0], e[1]]['dst']
            G.edges[e[0], e[1]]['dx_index'] = i;
            for d in dsts:
                filt.append(f"addr_in[{in_p}] != {d}")
                cases.append(f"{d}: demux_{in_p}_sel = {i};")
        cases = '\n\t\t\t'.join(cases)
        filt = ' && '.join(filt)
        src.append(FILTER.format(in_p=in_p, sel=filt))
        if num_node_out == 1:
            src.append(DEMUX_BYPASS.format(in_p=in_p))
        else:
            src.append(DEMUX.format(num_out=num_node_out, in_p=in_p, cases=cases))

    lb_nodes = [n for n in G.nodes() if n.startswith('lb')]
    for lb in lb_nodes:
        in_p = G.nodes[lb]['in_p']
        dst = G.nodes[lb]['dst']
        weights = [e[2]['weight'] for e in G.out_edges(lb, data=True)]
        for i, e in enumerate(G.out_edges(lb)):
            G.edges[e[0], e[1]]['lb_index'] = i
        num_node_out = len(G.out_edges(lb))
        blen = int(sum(weights))
        btable = [[i]*w.astype(int) for i, w in enumerate(weights)]
        bwidth = math.ceil(math.log2(blen))
        btable = [f"{bwidth}'d{i}" for b in btable for i in b]
        random.shuffle(btable)
        btable = ', '.join(btable)
        e = list(G.in_edges(lb, data=True))
        assert len(e) == 1
        key = e[0][2]['in_p']
        src.append(LOAD_BALANCE.format(num_out=num_node_out, in_p=in_p, dst=dst, bal_len=blen, btable=btable, key=key))

    out_nodes = [n for n in G.nodes() if n.startswith('out')]
    for out_n in out_nodes:
        out_p = G.nodes[out_n]['out_p']
        valid = []
        ready = []
        data = []
        num_in = len(G.in_edges(out_n))
        for i, e in enumerate(G.in_edges(out_n, data=True)):
            in_p = e[2]['in_p']
            if e[0].startswith('lb'):
                dst = G.nodes[e[0]]['dst']
                lb_index = e[2]['lb_index']
                valid.append(f'bal_{in_p}_{dst}_valid[{lb_index}]')
                ready.append(f'bal_{in_p}_{dst}_ready[{lb_index}]')
            else:
                dx_index = e[2]['dx_index']
                valid.append(f'demux_{in_p}_valid[{dx_index}]')
                ready.append(f'demux_{in_p}_ready[{dx_index}]')
            data.append(f'packets[{in_p}]')
        valid = ', '.join(valid)
        ready = ', '.join(ready)
        data = ', '.join(data)
        valid = f"{{{valid}}}"
        ready = f"{{{ready}}}"
        data = f"{{{data}}}"
        src.append(ARBITER.format(idx_out=out_p, num_in=num_in, valid_i=valid, ready_o=ready, data_i=data))
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

pos=nx.circular_layout(G)
nx.draw(G, pos, with_labels = True)
nx.draw_networkx_edge_labels(G, pos)
plt.savefig('foo.png')

txt = build_sv(G, test.shape[2], test.shape[0], test.shape[1])

with open('out.sv', 'w') as f:
    f.write(txt)

# nx.drawing.nx_pydot.write_dot(G, 'test.dot')