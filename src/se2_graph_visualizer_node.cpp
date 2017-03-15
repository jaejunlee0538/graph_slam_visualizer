//
// Created by ub1404 on 16. 5. 7.
//
#include <ros/ros.h>
#include <ros/publisher.h>
#include <visualization_msgs/MarkerArray.h>
#include <graph_slam_visualizer/GraphSE2.h>
#include <ros/subscriber.h>
#include <ColorMap.h>
ros::Publisher graph_vis_pub;

struct Triangle2D{
    Triangle2D(const double& width, const double& height){
        x[0] = 0;
        y[0] = 0.5 * width;

        x[1] = 0;
        y[1] = -0.5 * width;

        x[2] = height;
        y[2] = 0;
    }

    Triangle2D(const double& x1, const double& x2, const double& x3, const double& y1, const double& y2, const double& y3){
        x[0] = x1; y[0] = y1;
        x[1] = x2; y[1] = y2;
        x[2] = x3; y[2] = y3;
    }

    Triangle2D transformed(const double& dx, const double& dy, const double& dtheta){
        double c=cos(dtheta),s=sin(dtheta);
        return Triangle2D(c*x[0]-s*y[0]+dx, c*x[1]-s*y[1]+dx, c*x[2]-s*y[2]+dx,
                          s*x[0]+c*y[0]+dy, s*x[1]+c*y[1]+dy, s*x[2]+c*y[2]+dy);
    }

    double x[3];
    double y[3];
};

ros::NodeHandle* p_nh;
double vertex_scale;
double odom_edge_width;
double loop_edge_width;
double loop_edge_alpha;
double odom_edge_alpha;
bool pose_nose_coloring;
double graph_z_position;
double graph_heights;

constexpr float normalizedRGB(const int& color){
    return color/255.f;
}
constexpr float normalizedDistance(const float& when0, const float& when1){
    return 1.0f / (when1 - when0);
}

ColorMap nodeColors({MyRGB(1.0f,0.0f,0.0f), MyRGB(0.0f,1.0f,0.0f), MyRGB(0.69f,0.4f,0.8f), MyRGB(0.2f,0.2f,1.0f)});


void graphse2_callback(const graph_slam_visualizer::GraphSE2ConstPtr& graph){
    p_nh->param("vertex_scale",vertex_scale, 10.0);
    p_nh->param("odom_edge_width", odom_edge_width, 0.1);
    p_nh->param("loop_edge_width", loop_edge_width, 0.1);
    p_nh->param("loop_edge_alpha", loop_edge_alpha, 0.8);
    p_nh->param("odom_edge_alpha", odom_edge_alpha, 0.8);
    p_nh->param("pose_nose_coloring", pose_nose_coloring, false);
    p_nh->param("graph_heights", graph_heights, 0.0);
    p_nh->param("graph_z_position", graph_z_position, 0.0);
    Triangle2D tri(vertex_scale*0.1,vertex_scale*0.15);

    visualization_msgs::MarkerArray::Ptr array(new visualization_msgs::MarkerArray());
    geometry_msgs::Point p1, p2, p3;
    visualization_msgs::Marker marker_vertex;

    int cnt = 0;
    ////////////////////////////////////////////////////////////////////
    //Converting vertices into MarkerArray
    int v_size = graph->vertices.size();
    std::vector<double> vertexZvalue(v_size);
    double elevation = 0.0;
    if(v_size) elevation = graph_heights / v_size;
    if(graph->vertices.size()) {
        marker_vertex.action = visualization_msgs::Marker::MODIFY;
        marker_vertex.header = graph->header;
        marker_vertex.ns = "vertex_se2";
        marker_vertex.type = visualization_msgs::Marker::TRIANGLE_LIST;
        marker_vertex.scale.x = 1;
        marker_vertex.scale.y = 1;
        marker_vertex.scale.z = 1;
        if(!pose_nose_coloring) {
            marker_vertex.color.r = 1.0;
            marker_vertex.color.g = 0.0;
            marker_vertex.color.b = 0.0;
            marker_vertex.color.a = 1.0;
        }
        marker_vertex.id = 1;

        marker_vertex.points.resize(3 * graph->vertices.size());
        if(pose_nose_coloring)
            marker_vertex.colors.resize(3*v_size);
        for(int i=0;i<v_size;i++){
            Triangle2D tmp = tri.transformed(graph->vertices[i].x, graph->vertices[i].y, graph->vertices[i].theta);
            vertexZvalue[i] = graph_z_position + elevation * i;
            p1.x = tmp.x[0]; p1.y = tmp.y[0]; p1.z = vertexZvalue[i];
            p2.x = tmp.x[1]; p2.y = tmp.y[1]; p2.z = vertexZvalue[i];
            p3.x = tmp.x[2]; p3.y = tmp.y[2]; p3.z = vertexZvalue[i];
            marker_vertex.points[i*3]= p1;
            marker_vertex.points[i*3+1]= p2;
            marker_vertex.points[i*3+2]= p3;
            if(pose_nose_coloring) {
                std_msgs::ColorRGBA rgba;
                auto rgb = nodeColors.getRGB((double) i / v_size);
                if (!rgb.validate())
                    ROS_ERROR("Invalid color. %.3f %.3f %.3f(%d - %d)", rgb.r, rgb.g, rgb.b, i, v_size);
                rgba.r = rgb.r;
                rgba.g = rgb.g;
                rgba.b = rgb.b;
                rgba.a = 1.0f;
                marker_vertex.colors[i * 3] = rgba;
                marker_vertex.colors[i * 3 + 1] = rgba;
                marker_vertex.colors[i * 3 + 2] = rgba;
            }
        }
        array->markers.push_back(marker_vertex);
    }

    ////////////////////////////////////////////////////////////////////
    //Converting edges into MarkerArray
    if(graph->edges.size()) {
        visualization_msgs::Marker marker_edge;
        visualization_msgs::Marker marker_edge_loop;
        marker_edge.action = visualization_msgs::Marker::MODIFY;
        marker_edge.header = graph->header;
        marker_edge.ns = "edges_se2_odom";
        marker_edge.type = visualization_msgs::Marker::LINE_LIST;
        marker_edge.scale.x = odom_edge_width;
        marker_edge.id = 0;
        marker_edge.colors.clear();

        marker_edge_loop.action = visualization_msgs::Marker::MODIFY;
        marker_edge_loop.header = graph->header;
        marker_edge_loop.ns = "edges_se2_loop";
        marker_edge_loop.type = visualization_msgs::Marker::LINE_LIST;
        marker_edge_loop.scale.x = loop_edge_width;
        marker_edge_loop.id = 0;
        marker_edge_loop.colors.clear();
//        marker_edge.points.reserve(2 * graph->edges.size());
        std_msgs::ColorRGBA rgba;
        std::vector<float> weights;
        if(graph->edge_weights.empty() || graph->edge_weights.size()!=graph->edges.size()){
            std::fill_n(std::back_inserter(weights), graph->edges.size(), 1.0f);
        }else{
            std::copy(graph->edge_weights.begin(),graph->edge_weights.end(), std::back_inserter(weights));
        }

        for (size_t i=0;i<graph->edges.size();i++) {
            p1.x = graph->vertices[graph->edges[i].vi_idx].x;
            p1.y = graph->vertices[graph->edges[i].vi_idx].y;
            p1.z = vertexZvalue[graph->edges[i].vi_idx];
            p2.x = graph->vertices[graph->edges[i].vj_idx].x;
            p2.y = graph->vertices[graph->edges[i].vj_idx].y;
            p2.z = vertexZvalue[graph->edges[i].vj_idx];

            if(abs(graph->edges[i].vj_idx-graph->edges[i].vi_idx) > 1){
                //loop closing edges
                marker_edge_loop.points.push_back(p1);
                marker_edge_loop.points.push_back(p2);
                rgba.a = loop_edge_alpha;
                rgba.r = normalizedRGB(120) + weights[i]*(normalizedRGB(127)- normalizedRGB(120));
                rgba.g = normalizedRGB(120) + weights[i]*(normalizedRGB(0) - normalizedRGB(120));
                rgba.b = normalizedRGB(120) + weights[i]*(normalizedRGB(255) - normalizedRGB(120));

                marker_edge_loop.colors.push_back(rgba);
                marker_edge_loop.colors.push_back(rgba);
            }else{
                //odometry edges
                marker_edge.points.push_back(p1);
                marker_edge.points.push_back(p2);
                rgba.a = odom_edge_alpha;
                rgba.r = normalizedRGB(100) - weights[i]*normalizedRGB(100);
                rgba.g = normalizedRGB(155) + weights[i]*normalizedRGB(50);
                rgba.b = normalizedRGB(100) - weights[i]*normalizedRGB(100);

                marker_edge.colors.push_back(rgba);
                marker_edge.colors.push_back(rgba);
            }
        }
        array->markers.push_back(marker_edge);
        array->markers.push_back(marker_edge_loop);
    }
    ////////////////////////////////////////////////////////////////////
    graph_vis_pub.publish(array);
}

int main(int argc, char** argv){

    ros::init(argc, argv, "graph_visualization_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    p_nh = &pnh;

    graph_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("graph_se2_vis", 10);

    ros::Subscriber sub_robust = nh.subscribe("g2o_graph_se2",10, graphse2_callback);
    ros::spin();
}