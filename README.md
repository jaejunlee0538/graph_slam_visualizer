
## Fileds
``` txt
std_msgs/Header 	header
EdgeSE2[] 			edges 
float32[]           edge_weights 
VertexSE2[] 		vertices 
```

## 2D Pose Graph
A 2d pose graph can be visualized by publishing `GraphSE2` message. 
`header, edges, vertices` are always required for visualization.
But `edge_weights` is optional and should be left empty. 
It is required only when the graph is robustified.

![][2d pose graph]

## Robustified Pose Graph
A robustified pose graph can be visualized by setting `edge_weights` field.
Each i-th `edge_weights` must map into i-th `edges` in `GraphSE2` message.
Each `edge_weights` field must have a value between **0.0f~1.0f**.
- **0.0f** means the edge has no influence in graph optimization.
- **1.0f** means the edge influnces the graph as much as possible.

Fully unweighted edges are greyed out.

![robustified][robustified pose graph]


[2d pose graph]: __resources/fr101_local_loop_enabled.png
[robustified pose graph]: __resources/1stEngineering_With_Camera_OUtliers3.png