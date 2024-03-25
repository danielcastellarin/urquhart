#include <hype_tree.hpp>

namespace hype_urq {

Tree::Tree(const std::vector<EpicPolygon>& polygons){
    // create the root node
    root = add_vertex(EpicPolygon(), graph);
    // add all elements and point them to the root
    for(auto p : polygons){
        BoostVertexT v = add_vertex(p, graph);
        add_edge(root, v, graph);
    }
}

void Tree::view_tree(){
    write_graphviz(std::cout, graph);
}

void Tree::view_h2_polygons() {
    size_t i = 0;
    for (const auto& p : get_children(root)) {
        std::cout << i << ": ";
        for (int i = 0; i < graph[p].n; ++i) {
            std::cout << "[" << graph[p].landmarkRefs(i) << "," << graph[p].landmarkRefs((i+1)%graph[p].n) << "] ";
        }
        std::cout << std::endl;
        i++;
    }
}

EpicPolygon Tree::get_vertex(const BoostVertexT v){
    return graph[v];
}

BoostVertexT Tree::merge_op(BoostVertexT i, BoostVertexT j, const EpicPolygon& data){
    BoostVertexT v = add_vertex(data, graph);
    add_edge(root, v, graph);

    // add v as the new parent of i and j
    // if i or j are already a polygon, we remove it from the tree
    // since we don't use intermediary polygons
    if (graph[i].n > 3){
        BoostEdgeIter ei, ei_end;
        for (boost::tie(ei, ei_end) = out_edges(i, graph); ei != ei_end; ++ei) {
            BoostVertexT edgeSource = source(*ei, graph);
            BoostVertexT edgeTarget = target(*ei, graph);
            add_edge(v, edgeTarget, graph);
        } 
        // clear vertex will maintain an isolated vertex.
        // This is required so that the ids are consistent.
        // TODO: Study the boost graph lib to find a more memory-efficient way of doing this
        clear_vertex(i, graph);
    } else {
        clear_in_edges(i, graph);
        add_edge(v, i, graph);
    }

    if (graph[j].n > 3){
        BoostEdgeIter ej, ej_end;
        for (boost::tie(ej, ej_end) = out_edges(j, graph); ej != ej_end; ++ej) {
            BoostVertexT edgeSource = source(*ej, graph);
            BoostVertexT edgeTarget = target(*ej, graph);
            add_edge(v, edgeTarget, graph);
        } 
        clear_vertex(j, graph);
    } else {
        clear_in_edges(j, graph);
        add_edge(v, j, graph);
    }
    return v;
}

BoostVertexT Tree::get_ancestor(const BoostVertexT v){
    BoostRGraph reversedGraph(graph);
    std::vector<BoostVertexT> polygons;
    BFSVisitor vis(polygons);
    breadth_first_search(reversedGraph, v, visitor(vis));
    BoostVertexT ancestor = 0;
    if (polygons.size() > 0) ancestor = polygons[polygons.size()-1];
    // BoostVertexT ancestor = polygons.size() > 0 ? polygons[-1] : 0;
    return ancestor;
}

std::vector<BoostVertexT> Tree::get_children(const BoostVertexT v){
    std::vector<BoostVertexT> polygons;
    BoostEdgeIter ei, ei_end;
    for (boost::tie(ei, ei_end) = out_edges(v, graph); ei != ei_end; ++ei) {
        auto child = target(*ei, graph);
        polygons.push_back(child);
    }
    return polygons;
}

// std::vector<BoostVertexT> Tree::new_get_children(const BoostVertexT v){
//     std::vector<BoostVertexT> polygons;
//     for (auto vd : boost::make_iterator_range(adjacent_vertices(v, graph))) {
//         // std::cout << "vertex " << vd << " is an out-edge of vertex " << v << "\n";
//         polygons.push_back(vd);
//     }
//     return polygons;
// }

std::vector<BoostVertexT> Tree::traverse(){
    std::vector<BoostVertexT> polygons;
    BFSVisitor vis(polygons);
    breadth_first_search(graph, root, visitor(vis));
    return polygons;
}

std::vector<BoostVertexT> Tree::traverse(const BoostVertexT v){
    std::vector<BoostVertexT> polygons;
    BFSVisitor vis(polygons);
    breadth_first_search(graph, v, visitor(vis));
    return polygons;
}
}