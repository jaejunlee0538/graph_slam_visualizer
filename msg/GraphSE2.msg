std_msgs/Header 	header
EdgeSE2[] 			edges # set of edges
float32[]           edge_weights # leave empty if the graph is not robustified(each edges are weighted)
VertexSE2[] 		vertices # set of vertices. The index of vertices must be same with the indices defined in edges