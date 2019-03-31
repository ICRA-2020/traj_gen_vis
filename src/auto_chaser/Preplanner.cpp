#include "auto_chaser/Preplanner.h"

Preplanner::Preplanner(){};

void Preplanner::init(ros::NodeHandle nh){

    // preplanner params parsing 
    nh.param("w_v",params.w_v,0.3);            
    nh.param("r_safe",params.r_safe,2.0);
    nh.param("min_z",params.min_z,0.4);
    nh.param("w_v",params.w_v,0.3);
    nh.param("vs_min",params.vs_min,0.3);
    nh.param("vsf_resolution",params.vsf_resolution,0.5);
    nh.param("d_connect_max",params.d_connect_max,3.0);

    nh.param("d_trakcing_max",params.d_trakcing_max,5.0);
    nh.param("d_trakcing_min",params.d_trakcing_min,0.6);
    nh.param("max_azim",params.max_azim,(3.141592/4));
    nh.param("min_azim",params.min_azim,(3.141592/7));


    // world_frame_id 
    nh.param<string>("world_frame_id",markers_visibility_field_base.header.frame_id,"/world");
    nh.param<string>("world_frame_id",preplanned_path.header.frame_id,"/world");


    // marker initialize 

    // waypoints 
    marker_wpnts.header.frame_id = markers_visibility_field_base.header.frame_id;
    marker_wpnts.ns = "waypoints";
    marker_wpnts.id = 0;
    marker_wpnts.type = visualization_msgs::Marker::SPHERE_LIST;
    marker_wpnts.color.r = 14.0/255.0;
    marker_wpnts.color.g = 50.0/255.0;
    marker_wpnts.color.b = 1.0;
    marker_wpnts.color.a = 0.8;
    marker_wpnts.pose.orientation.w = 1.0;
    double scale = 0.3; 
    marker_wpnts.scale.x = scale;
    marker_wpnts.scale.y = scale;
    marker_wpnts.scale.z = scale;    


};

FieldParams Preplanner::get_local_vsf_param_around_target(Point target_pnt){

    FieldParams vsf_param;    
    double lx,ly,lz;
    lx = ly = 2*params.d_trakcing_max * cos(params.max_azim);
    lz = params.d_trakcing_max * sin(params.max_azim) - params.d_trakcing_min * sin(params.min_azim) ;

    vsf_param.x0 = target_pnt.x - lx/2;
    vsf_param.y0 = target_pnt.y - ly/2;
    vsf_param.z0 = target_pnt.z;
    vsf_param.lx = lx;
    vsf_param.ly = ly;
    vsf_param.lz = lz;
    
    vsf_param.resolution = params.vsf_resolution;
    vsf_param.ray_stride_res =  params.vsf_resolution; // not used for vsf grid 

};


void Preplanner::compute_visibility_field_seq(GridField* global_edf,vector<Point> target_pnts){

    vsf_field_ptr_seq.resize(target_pnts.size());

    int t = 1;
    float max_score = -1;  // for visualization 
    // for each target pnt
    for (auto it = target_pnts.begin();it<target_pnts.end();it++,t++){
        
        // get local conservative grid map around the current target point
        int VSF_MODE = 1;
        vsf_field_ptr_seq[t-1].reset(new GridField(get_local_vsf_param_around_target(*it))); 
        
        // field value update with edf grid 
        for(int ix = 0 ; ix<vsf_field_ptr_seq[t-1].get()->Nx ; ix++)
            for(int iy = 0 ; iy<vsf_field_ptr_seq[t-1].get()->Ny ; iy++)
                for(int iz = 0 ; iz<vsf_field_ptr_seq[t-1].get()->Nz ; iz++){
                    
                    // assign visibilty value with minimum clamping to evaluated node 
                    Point eval_pnt = vsf_field_ptr_seq[t-1].get()->getCellPnt(Vector3i(ix,iy,iz));      
                    float vs = global_edf->getRayMin(*it,eval_pnt,params.vs_min); // visibility score from distance field                    
                    vsf_field_ptr_seq[t-1].get()->field_vals[ix][iy][iz] = vs;

                    // let's save the point if certain condition is satified (for graph construction)                
                    
                    Vector3i pnt_idx_in_edf = global_edf->getCellIdx(eval_pnt);
                    float edf_val = global_edf->field_vals[pnt_idx_in_edf(0)][pnt_idx_in_edf(1)][pnt_idx_in_edf(2)];  
                    Vector3f bearing_vec =(geo2eigen(eval_pnt) - geo2eigen(*it)); 
                    float relative_dist = bearing_vec.norm();                      
                    float azim = atan2(bearing_vec(2),Vector2f(bearing_vec(0),bearing_vec(1)).norm());
                    
                    if(edf_val > params.r_safe && // safe 
                        relative_dist > params.d_trakcing_min && // tracking spec
                        relative_dist < params.d_trakcing_max && // tracking spec
                        vs > params.vs_min && // non-occlusion
                        azim < params.max_azim)  // tracking spec 
                        // save
                        vsf_field_ptr_seq[t-1].get()->saved_points.push_back(eval_pnt);
                    
                    if (vs >= max_score)
                        max_score = vs;

                }
        std::cout<<"[Preplanner] nodes at time "<<t<<" are "<<vsf_field_ptr_seq[t-1].get()->saved_points.size()<<std::endl;
    }

    // save the markers

    // marker initialization     
    markers_visibility_field_seq.markers.resize(target_pnts.size());    

    markers_visibility_field_base.header.stamp = ros::Time::now();
    markers_visibility_field_base.points.clear();
    markers_visibility_field_base.colors.clear();
    t = 1;

    for (auto it = target_pnts.begin();it<target_pnts.end();it++,t++){ // for time
        int idx = 0;
        markers_visibility_field_base.ns = "time_"+to_string(t);

        // we draw only saved points from above 
        for (auto it_node = vsf_field_ptr_seq[t-1].get()->saved_points.begin() ; it_node < vsf_field_ptr_seq[t-1].get()->saved_points.end() ; it_node++,idx++){
            Vector3i key = vsf_field_ptr_seq[t-1].get()->getCellIdx(*it_node);
            float vs = vsf_field_ptr_seq[t-1].get()->field_vals[key(0)][key(1)][key(2)];
            // std::cout<<vs<<std::endl;
            // marker update
            std_msgs::ColorRGBA color;
            get_color((vs-params.vs_min)/(max_score-params.vs_min),color.r,color.g,color.b);            
            color.a = 0.1;

            markers_visibility_field_base.colors.push_back(color);
            markers_visibility_field_base.points.push_back(*it_node);
            idx ++;
        }

        markers_visibility_field_seq.markers.push_back(markers_visibility_field_base);
        markers_visibility_field_base.points.clear();
        markers_visibility_field_base.colors.clear();
        
    }
}


void Preplanner::graph_construct(GridField* global_edf,Point x0){
    
    // init graph with the initial position of chaser 
    di_graph = Graph();
    descriptor_map.clear();
    
    vector<Node<Point>> prev_layer;
    Node<Point> initial_node; initial_node.value = x0; initial_node.name = "x0";
    prev_layer.push_back(initial_node);
    
    Vertex_d v0 = boost::add_vertex(x0,di_graph);
    descriptor_map.insert(make_pair(VertexName("x0"),v0));
 
    int H = vsf_field_ptr_seq.size(); // total prediction horizon 
    int N_edge = 0; 
    int N_edge_sub = 0;

    // in case of t = 0, we don't need (just current step). 
    for(int t = 1; t<H;t++){
        N_edge_sub = 0;
        GridField* cur_vsf_ptr = vsf_field_ptr_seq[t].get();        
        vector<Node<Point>> cur_layer = cur_vsf_ptr->generate_node(t); // current layer   

        for (auto it_cur = cur_layer.begin() ; it_cur<cur_layer.end(); it_cur++){
            
            // step1 :  let's register the node(pnt,name) in the current layer into graph 
            Point cur_pnt = it_cur->value; Vector3f cur_vec = geo2eigen(cur_pnt); 
            Vertex_d cur_vert = boost::add_vertex(cur_pnt,di_graph);
            descriptor_map.insert(make_pair(it_cur->name,cur_vert));
            
            // call the previous layer  
            GridField* prev_vsf_ptr = vsf_field_ptr_seq[t-1].get();                        
            
            // step2 : let's connect with previous layer and add edges 
            for(auto it_prev = prev_layer.begin(); it_prev < prev_layer.end();it_prev++){ // prev_layer 
                Vertex_d prev_vert = descriptor_map[it_prev->name];
                Point prev_pnt = it_prev->value; Vector3f prev_vec = geo2eigen(prev_pnt);

                // this condition should be satisfied to be connected 
                if(((cur_vec-prev_vec).norm() < params.d_connect_max) && (global_edf->getRayMin(cur_pnt,prev_pnt,0) > params.r_safe) ){
                    float weight = (cur_vec-prev_vec).norm() + 
                            params.w_v*1/sqrt(cur_vsf_ptr->getRayMean(cur_pnt,prev_pnt) * prev_vsf_ptr->getRayMean(prev_pnt,cur_pnt)) 
                            + params.w_d*abs((geo2eigen(cur_vsf_ptr->getCentre()) - cur_vec).norm() - params.d_trakcing_des);                     
                    boost::add_edge(prev_vert,cur_vert,weight,di_graph);
                    N_edge ++;
                    N_edge_sub++;
                }
            }            
        }
        prev_layer = cur_layer;
        cout<<"[Preplanner] connected edge to this layer: "<<N_edge_sub<<std::endl;
    }
    
    cout<<"[Preplanner] total number of edges "<<N_edge<<std::endl;


    // graph finishing 

    GridField* prev_vsf_ptr = vsf_field_ptr_seq[H-1].get();
    Vertex_d vf = boost::add_vertex(Point(),di_graph);
    descriptor_map.insert(make_pair(VertexName("xf"),vf));

    // step2 : let's connect with previous layer 
    for(auto it_prev = prev_layer.begin(); it_prev < prev_layer.end();it_prev++){ // prev_layer 
        Vertex_d prev_vert = descriptor_map[it_prev->name];
        // this condition should be satisfied to be connected 
            boost::add_edge(prev_vert,vf,0,di_graph);
    }
}

VertexPath Preplanner::dijkstra(Vertex_d v0,Vertex_d vf){
    


    // Create things for Dijkstra
    std::vector<Vertex_d> predecessors(boost::num_vertices(di_graph)); // To store parents
    std::vector<Weight> distances(boost::num_vertices(di_graph)); // To store distances

    IndexMap indexMap = boost::get(boost::vertex_index, di_graph);
    NameMap nameMap = boost::get(boost::vertex_name, di_graph);

    PredecessorMap predecessorMap(&predecessors[0], indexMap);
    DistanceMap distanceMap(&distances[0], indexMap);    

    boost::dijkstra_shortest_paths(di_graph, v0, boost::distance_map(distanceMap).predecessor_map(predecessorMap));

    typedef std::vector<Graph::edge_descriptor> PathType;

    PathType path;
    Vertex_d v = vf; // We want to start at the destination and work our way back to the source
    for(Vertex_d u = predecessorMap[v]; // Start by setting 'u' to the destintaion node's predecessor
        u != v; // Keep tracking the path until we get to the source
        v = u, u = predecessorMap[v]) // Set the current vertex to the current predecessor, and the predecessor to one level up
    {
        std::pair<Graph::edge_descriptor, bool> edgePair = boost::edge(u, v, di_graph);
        Graph::edge_descriptor edge = edgePair.first;

        path.push_back( edge );
    }

    if (path.size())
    {
       ROS_INFO("path exist");
        // Write shortest path
        float totalDistance = 0;

        VertexPath vertex_path1;
        VertexPath vertex_path2;
        for(PathType::reverse_iterator pathIterator = path.rbegin(); pathIterator != path.rend(); ++pathIterator)
        {

//            ROS_INFO("path insertion");
            vertex_path1.push_back(nameMap[boost::source(*pathIterator, di_graph)]);
            vertex_path2.push_back(nameMap[boost::target(*pathIterator, di_graph)]);
        }

        vertex_path1.push_back(vertex_path2.back());
        return vertex_path1;
    }
    else{
        ROS_WARN("[Preplanner] path does not exist. returning zero length path. ");
        return VertexPath();
    }    
}


void Preplanner::get_shortest_path(){

    ROS_INFO("shorted path requested.");
    VertexPath solution_seq = dijkstra(descriptor_map["x0"],descriptor_map["xf"]);
    // if path exist 
    if(solution_seq.size()){

        solution_seq.pop_back();

        // from graph path to real path 
        preplanned_path.poses.clear();
        // marker update  
        marker_wpnts.points.resize(solution_seq.size());
        marker_wpnts.colors.resize(solution_seq.size());  


        for(auto it = solution_seq.begin();it<solution_seq.end();it++){
            geometry_msgs::PoseStamped pose_stamped;

            pose_stamped.pose.position = *it;
            preplanned_path.poses.push_back(pose_stamped);

            marker_wpnts.colors.push_back(marker_wpnts.color);
            marker_wpnts.points.push_back(*it);
        }
    
    }

    
}