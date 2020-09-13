use petgraph::{graph, EdgeDirection, algo};
use petgraph::algo::kosaraju_scc;
use std::collections::HashMap;

type DirectedGraph = graph::DiGraph::<u32, i32>;


fn ford_fulkerson(g: DirectedGraph, source: usize, target: usize) -> Option<DirectedGraph>{
    let mut current_graph: DirectedGraph = g.clone();
    let mut inverse_graph: DirectedGraph = g.filter_map(|_, _| Some(0), |_, _| Some(0));
    let mut forward_graph: DirectedGraph;
    let mut reverse_graph: DirectedGraph;
    let mut full_graph: DirectedGraph;
    loop {
        forward_graph = current_graph.filter_map(|_, _| Some(0), |e, _| match current_graph.edge_weight(e) {
            Some(w) => if w>&0 {Some(*w)} else {None}
            _ => None
        });
    
        reverse_graph = inverse_graph.filter_map(|_, _| Some(0), |e, _| match inverse_graph.edge_weight(e) {
            Some(w) => if w>&0 {Some(*w)} else {None}
            _ => None
        });
        let mut endpoints: (graph::NodeIndex, graph::NodeIndex);
        let mut edges: Vec<(u32, u32, i32)> = vec![]; 
        for i in 0..forward_graph.edge_indices().len(){
            endpoints =  forward_graph.edge_endpoints(graph::edge_index(i)).unwrap();
            edges.push((endpoints.0.index() as u32, endpoints.1.index() as u32, *forward_graph.edge_weight(graph::edge_index(i)).unwrap()));
        }

        for i in 0..reverse_graph.edge_indices().len(){
            endpoints =  reverse_graph.edge_endpoints(graph::edge_index(i)).unwrap();
            edges.push((endpoints.1.index() as u32, endpoints.0.index() as u32, -*reverse_graph.edge_weight(graph::edge_index(i)).unwrap()));
        }
        
        full_graph = graph::Graph::from_edges(edges);
        let shortest_path = algo::astar(
            &full_graph,
            graph::node_index(source),               // start
            |n| n == graph::node_index(target),      // is_goal
            |_| 1, // edge_cost
            |_| 0,           // estimate_cost
        );
        let path = match shortest_path {
            Some(tuple) => tuple.1,
            None => vec![]
        };
        if path.len()==0{
            return None;
        }
        let mut weights: Vec<i32> = vec![];
        for i in 1..(path.len()) {
            let graph_index = full_graph.find_edge(path[i-1], path[i]).unwrap();
            weights.push(full_graph[graph_index]);
        }
        let min_weight = weights.iter().map(|x| x.abs()).min().unwrap();
        weights = weights.iter().map(|w| min_weight * w/w.abs()).collect();
        let mut last_node = graph::node_index(source);
        for (i, node) in path.iter().enumerate() {
            if node.index()==source {
                continue
            }
            if weights[i-1]>0 {
                let graph_index = current_graph.find_edge(last_node, node.clone()).unwrap();
                let w = current_graph.edge_weight(graph_index).unwrap();
                current_graph.update_edge(last_node, node.clone(), w-weights[i-1]);
            }
            else {
                let graph_index = current_graph.find_edge(node.clone(), last_node).unwrap();
                let w = current_graph.edge_weight(graph_index).unwrap();
                current_graph.update_edge(node.clone(), last_node, w-weights[i-1]);


            }
            last_node = *node;
            
        }
        
        for i in 0..g.edge_indices().len(){
            let (node_a, node_b) = g.edge_endpoints(graph::edge_index(i)).unwrap();
            inverse_graph.update_edge(node_a, node_b, g.edge_weight(graph::edge_index(i)).unwrap() - current_graph.edge_weight(graph::edge_index(i)).unwrap());
        }
        let edges = current_graph.edges_directed(graph::node_index(target), EdgeDirection::Incoming);
        let mut finished = true;
        for edge in edges{
            if edge.weight() != &0 {
                finished = false;
                break;
            }
        }
        if finished {
            println!("Subgraph completed!");
            return Some(inverse_graph);
        }
        
        
    }

}


fn main() {
    let supplies: Vec<(u32, i32)> = vec![(1, 6), (2, 5), (3, 4), (4, 5), (5, 4)];
    let capacities: Vec<(u32, i32)> = vec![(6, 6), (7, 7), (8, 6), (9, 5)];
    let bipartite_edges: Vec<(u32, u32)> = vec![(1, 6), (1, 8), (2, 7), (3, 6), (3, 8), (3, 9), (4,7), (5,8), (5,9)];
    let bipartite_graph = graph::UnGraph::<u32, i32>::from_edges(bipartite_edges.iter());
    let max_amount: i32 = *supplies.iter().map(|(_, b)| b).max().max(capacities.iter().map(|(_, b)| b).max()).unwrap();
    let target: u32 = *supplies.iter().map(|(a, _)| a).max().max(capacities.iter().map(|(a, _)| a).max()).unwrap()+1;
    let components = kosaraju_scc(&bipartite_graph);
    let mut edges: Vec<(u32, u32, i32)> = supplies.iter().map(|(a,b)| (0 as u32, *a, *b)).collect();
    edges.extend(capacities.iter().map(|(a,b)| (*a, target, *b)));
    edges.extend(bipartite_edges.iter().map(|(a,b)| (*a, *b, max_amount)));
    let mut solution: Vec<DirectedGraph> = vec![];
    for component in &components[..components.len()-1] {
        let mut nodes = HashMap::new();
        for node in component { 
            nodes.insert(node.index(), true);
        }
        let mut subgraph_edges: Vec<(u32, u32, i32)> = vec![];
        for edge in &edges {
            if *(nodes.get(&(edge.0 as usize)).unwrap_or_else(|| &false)) || *(nodes.get(&(edge.1 as usize)).unwrap_or_else(|| &false)) {
                subgraph_edges.push(edge.clone());
            }
        }
        let g = DirectedGraph::from_edges(subgraph_edges.iter());
        match ford_fulkerson(g, 0, target as usize) {
            Some(graph) => solution.push(graph),
            None => {
                println!("No solution!");
                break;
            }

        }
        
    }
    if components.len()==solution.len()+1 {
        println!("Solution: {:?}", solution)
    }
    
}



