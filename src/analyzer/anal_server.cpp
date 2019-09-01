#include "analyzer/anal_server.h"



AnalServer::AnalServer(std::string octomap_file,std::string chaser_file,std::string target_file,float arrow_width,float arrow_alpha,int arrow_draw_step):arrow_width(arrow_width),arrow_alpha(arrow_alpha),arrow_draw_step(arrow_draw_step){

    // data logging
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.header.frame_id = "/world";
    chaser_path.header.frame_id = "/world";
    target_path.header.frame_id = "/world";
    double map_z_min=0.1;


    std::ifstream infile;
    infile.open(chaser_file);
    if(infile.is_open()){

        while (! infile.eof()){
            std::string line;
            getline(infile, line); // if no delimiter given, new line is that
            // std::cout<<line<<std::endl;
            std::stringstream stream(line);
            std::string val;
            int xyz_idx = 0;
            geometry_msgs::Point wpnt;

            while(! stream.eof()) {
                getline(stream, val, ',');
                if(xyz_idx == 0)
                    wpnt.x = atof(val.c_str());
                else if(xyz_idx == 1)
                    wpnt.y = atof(val.c_str());
                else
                    wpnt.z = atof(val.c_str());
                xyz_idx ++;
            }
            chaser_history.push_back(wpnt);
        }

        chaser_history.pop_back();
        std::cout<<"chaser loaded"<<std::endl;

    }
    else
    {
        std::cout<<"no chaser loaded"<<std::endl;
        return;
    }
	infile.close();
    // data logging

    infile.open(target_file);
    if(infile.is_open()){
        while (! infile.eof()){
            std::string line;
            getline(infile, line); // if no delimiter given, new line is that
            // std::cout<<line<<std::endl;
            std::stringstream stream(line);
            std::string val;
            int xyz_idx = 0;
            geometry_msgs::Point wpnt;

            while(! stream.eof()) {
                getline(stream, val, ',');
                if(xyz_idx == 0)
                    wpnt.x = atof(val.c_str());
                else if(xyz_idx == 1)
                    wpnt.y = atof(val.c_str());
                else
                    // wpnt.z = atof(val.c_str());
                    wpnt.z = map_z_min+0.1;

                xyz_idx ++;
            }
            target_history.push_back(wpnt);
        }
		
        target_history.pop_back();
        std::cout<<"target loaded"<<std::endl;
        
    }
    else
    {
        std::cout<<"no target loaded"<<std::endl;
        return;
    }


    for (auto it = chaser_history.begin();it<chaser_history.end();it++){
        geometry_msgs::PoseStamped pose ;
        pose.pose.position = *it; 
        chaser_path.poses.push_back(pose);        
    }


    for (auto it = target_history.begin();it<target_history.end();it++){
        geometry_msgs::PoseStamped pose ;
        pose.pose.position = *it; 
        target_path.poses.push_back(pose);        
    }


    // true map building 

    //octomap::AbstractOcTree* tree = octomap::AbstractOcTree::readBinary(octomap_file);
    //octomap::OcTree* octo_ptr =static_cast<octomap::OcTree*>(tree);
	
    octomap::OcTree* octo_ptr =new octomap::OcTree(octomap_file);
    
	std::cout<<"[ANAL] reading octree size: " << octo_ptr->size()<<std::endl;
	double x,y,z;
    octo_ptr->getMetricMin(x,y,z);
    z = map_z_min;
    octomap::point3d boundary_min(x,y,z); 

    octo_ptr->getMetricMax(x,y,z);
    octomap::point3d boundary_max(x,y,z); 
    bool unknownAsOccupied = false;
    DynamicEDTOctomap edf_logger(5.0f,octo_ptr,boundary_min,boundary_max,unknownAsOccupied);
    edf_logger.update();
    std::cout<<"[ANAL] edf computation: " << octo_ptr->size()<<std::endl;

    // log1 target clutter 
    for(auto it = target_history.begin();it<target_history.end();it++){
        Point eval_pnt = *it;
        clutter_target.push_back(edf_logger.getDistance(octomap::point3d(eval_pnt.x,eval_pnt.y,eval_pnt.z)));     

    }

    // log1-1 chaser clutter 
    for(auto it = chaser_history.begin();it<chaser_history.end();it++){
        Point eval_pnt = *it;
        clutter_chaser.push_back(edf_logger.getDistance(octomap::point3d(eval_pnt.x,eval_pnt.y,eval_pnt.z)));     
    }
    

    // log2 chaser total distance 
    chaser_travel = 0;

    for(int i= 0; i<chaser_history.size()-1;i++){
        
        chaser_travel += (geo2eigen(chaser_history[i]) - geo2eigen(chaser_history[i+1])).norm();
        
    }

	std::cout<<"total travel distance: "<<chaser_travel<<std::endl;


    // let's consider the boundary of TEDF
    vector<double> xs;
    vector<double> ys;
    vector<double> zs;

    for (auto it = target_history.begin();it<target_history.end();it++){
        xs.push_back(it->x);
        ys.push_back(it->y);
        zs.push_back(it->z);        
    }

    for (auto it = chaser_history.begin();it<chaser_history.end();it++){
        xs.push_back(it->x);
        ys.push_back(it->y);
        zs.push_back(it->z);        
    }


    double x_min = *min_element(xs.begin(),xs.end());
    double y_min = *min_element(ys.begin(),ys.end());
    double z_min = *min_element(zs.begin(),zs.end());

    double x_max = *max_element(xs.begin(),xs.end());
    double y_max = *max_element(ys.begin(),ys.end());
    double z_max = *max_element(zs.begin(),zs.end());

    double d_scale = 5;
    // boundary_min = octomap::point3d(x_min-d_scale,
    //                                 y_min-d_scale,
    //                                 map_z_min);
    
    // boundary_max = octomap::point3d(x_max+d_scale,
    //                                 y_max+d_scale,
    //                                 z_max+d_scale);
    

    FieldParams global_field_param;
    global_field_param.x0 = boundary_min.x();
    global_field_param.y0 = boundary_min.y();
    global_field_param.z0 = boundary_min.z();
    global_field_param.lx = boundary_max.x() - boundary_min.x();
    global_field_param.ly = boundary_max.y() - boundary_min.y();
    global_field_param.lz = boundary_max.z() - boundary_min.z();
    global_field_param.resolution = 0.2;
    global_field_param.ray_stride_res = 0.08;

    GridField edf_field = GridField(global_field_param);

    edf_marker.header.frame_id = "/world";
    edf_marker.type = visualization_msgs::Marker::CUBE_LIST;
    edf_marker.scale.x = global_field_param.resolution;
    edf_marker.scale.y = global_field_param.resolution;
    edf_marker.scale.z = global_field_param.resolution;
    edf_marker.pose.orientation.w = 1.0;

    ROS_INFO("field traverse start");
    for(int ix = 0 ; ix<edf_field.Nx ; ix++)
        for(int iy = 0 ; iy<edf_field.Ny ; iy++)
            for(int iz = 0 ; iz<edf_field.Nz ; iz++){
                Point eval_pnt = edf_field.getCellPnt(Vector3i(ix,iy,iz));  
                // query edf value from edf mapper                       
                float dist_val = edf_logger.getDistance(octomap::point3d(eval_pnt.x,eval_pnt.y,eval_pnt.z));
                
                std_msgs::ColorRGBA color;                    
                get_color_dist(dist_val,color,0.8);
                if(dist_val<0.8){
                    edf_marker.points.push_back(eval_pnt);
                    edf_marker.colors.push_back(color);
                }

                // edf value assign 
                edf_field.field_vals[ix][iy][iz] = dist_val;
            }

    ROS_INFO("field traverse end");
    

    // log3 vs along the path 
    int total_N = target_history.size();
    std::cout<<"vs: "<<std::endl;
    for(int i =0;i<total_N;i++){
        

        float vs = (edf_field.getRayMin(target_history[i],chaser_history[i],0.0));                
        clutter_bearing.push_back(vs);
        std::cout<<vs<<std::endl;
      
    }



    float vs_min = *std::min_element(clutter_bearing.begin(),clutter_bearing.end());
    float vs_max = *std::max_element(clutter_bearing.begin(),clutter_bearing.end());
    
    VectorXi set_index = VectorXi::LinSpaced(10,0,total_N-1);

    float FOV = 3.141592/2;

    for(int i =0;i<10;i++){

        // pose for axis drawing
        geometry_msgs::PoseStamped pose;
        tf::Quaternion quat;
        int cur_idx = set_index(i);
        float yaw=atan2(target_history[cur_idx].y-chaser_history[cur_idx].y,
            target_history[cur_idx].x-chaser_history[cur_idx].x);
        quat.setRPY(0,0,yaw);
        pose.header.frame_id = "/world";
        pose.pose.position = chaser_history[cur_idx];
        pose.pose.orientation.x = quat.x();
        pose.pose.orientation.y = quat.y();
        pose.pose.orientation.z = quat.z();
        pose.pose.orientation.w = quat.w();
        chaser_pos_vec.push_back(pose);
        
    };


    std::cout<<"arrow draw step: "<<arrow_draw_step<<std::endl;
    for(int i = 0; i<total_N;i+=arrow_draw_step){

        float vs = clutter_bearing[i];
        arrow.points.clear();
        arrow.points.push_back(chaser_history[i]);
        arrow.points.push_back(target_history[i]);

        arrow.scale.x = 0.08;
        arrow.scale.y = arrow_width;
        arrow.scale.z = 0.15;
        arrow.ns ="t" + to_string(i);
        get_color((vs-vs_min)/(vs_max-vs_min),arrow.color.r,arrow.color.g,arrow.color.b);
        if (vs>0.1)
			arrow.color.a = arrow_alpha;            
		else
			arrow.color.a = 0.0;
		arrow_array.markers.push_back(arrow);  

    }

    ROS_INFO("[ANAL] history size: %d ",total_N);



};


void AnalServer::write(){

    // log1 
    std::ofstream wnpt_file;
    wnpt_file.open((write_path + "/target_SEDT.txt").c_str());
	
    cout<<"clutter target writing start. data size: "<<clutter_target.size()<<endl;
    if(wnpt_file.is_open()){
        for (auto it = clutter_target.begin();it < clutter_target.end();it++)
        wnpt_file<<*it<<"\n";
		wnpt_file.close();    
    
	}else
        cout<<"logging file for clutter target is not opend"<<endl;


    // log1 
    std::ofstream wnpt_file11;
    wnpt_file11.open((write_path + "/chaser_SEDT.txt").c_str());
	
    cout<<"clutter chaser writing start. data size: "<<clutter_chaser.size()<<endl;
    if(wnpt_file11.is_open()){
        for (auto it = clutter_chaser.begin();it < clutter_chaser.end();it++)
        wnpt_file11<<*it<<"\n";
		wnpt_file11.close();    
    
	}else
        cout<<"logging file for clutter chaser is not opend"<<endl;

    // log2
    std::ofstream wnpt_file2;
    wnpt_file2.open((write_path +"/bearing_clutter.txt").c_str());

    cout<<"clutter bearing writing start. data size: "<<clutter_bearing.size()<<endl;
    if(wnpt_file2.is_open()){
        for (auto it = clutter_bearing.begin();it < clutter_bearing.end();it++)
        wnpt_file2<<*it<<"\n";
        wnpt_file2.close();    
    }else
        cout<<"logging file for bearing clutter is not opend"<<endl;

    // log 4 

    std::cout<<"total travel of chaser is: "<<chaser_travel<<std::endl;

}

void AnalServer::publish(){
    pub_chaser_path.publish(chaser_path);
    pub_target_path.publish(target_path);
    pub_result.publish(arrow_array);
    pub_edf.publish(edf_marker);
    // pose publish 
    for(int i =0;i<N_point_plot;i++)
        pub_pose[i].publish(chaser_pos_vec[i]);
    
}
