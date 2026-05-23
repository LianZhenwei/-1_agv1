#include "dijkstra_path_planner/dijkstra_path_planner.h"
int GPS = -1;
int ODOM = -1;
int SLAM = -1;

PathPlannerNode::PathPlannerNode(ros::NodeHandle &nh) : nh_(nh), rviz_vis_(nh)
{
    int odom_ = -1;
    int slam_ = -1;
    int gps_ = -1;
    nh.param<int>("ODOM_", odom_, -1);
    nh.param<int>("SLAM_", slam_, -1);
    nh.param<int>("GPS_", gps_, -1);
    ODOM = odom_;
    SLAM = slam_;
    GPS = gps_;
    ros::param::get("l_default", l_default_);
    if (SLAM != 1)
    {
        for (size_t i = 0; i < 20; i++)
        {
            ROS_WARN("Detected SLAM = %d, GPS = %d, ODOM = %d", SLAM, GPS, ODOM);
            ROS_WARN("SLAM != 1 or GPS != 1, please check the parameters in user.yaml!");
        }
        ros::shutdown();
        exit(EXIT_FAILURE);
    }

    sub_localization_ = nh_.subscribe("/localization", 1, &PathPlannerNode::odometryGetCallBack_slam, this);
    sub_odom_ = nh_.subscribe("/odom", 1, &PathPlannerNode::odometryGetCallBack, this);
    sub_goal_ = nh_.subscribe("/move_base_simple/goal", 10, &PathPlannerNode::goalCb, this);
    timer1 = nh.createTimer(ros::Duration(1), &PathPlannerNode::timer_cb1, this);
    pub_global_path_ = nh_.advertise<by_global_path_planning::Path>("/to_control", 1);
    pub_nearest_start_node_ = nh_.advertise<geometry_msgs::PointStamped>("/nearest_start_node", 1);
    pub_nearest_goal_node_ = nh_.advertise<geometry_msgs::PointStamped>("/nearest_goal_node", 1);
}

void PathPlannerNode::initGraph()
{
    node_coords_.clear();
    all_road_points_.clear();
    node_list_.clear();
    graph_.~Graphlnk();
    new (&graph_) Graphlnk<int, double>();

    initGraphFromTxt();
    ROS_INFO("--------------------------------------------step1");
    ROS_INFO("Graph initialization completed. Total nodes: %d", graph_.numberOfVertices());
    map<string, vector<geometry_msgs::Point>> rviz_road_points;
    for (const auto &road : all_road_points_)
    {
        vector<geometry_msgs::Point> rviz_points;
        for (const auto &np : road.second)
        {
            geometry_msgs::Point gp;
            gp.x = np.x;
            gp.y = np.y;
            gp.z = 0.0;
            rviz_points.push_back(gp);
        }
        rviz_road_points[road.first] = rviz_points;
    }
    map<int, geometry_msgs::Point> rviz_node_coords;
    for (const auto &node : node_coords_)
    {
        geometry_msgs::Point gp;
        gp.x = node.second.x;
        gp.y = node.second.y;
        gp.z = 0.0;
        rviz_node_coords[node.first] = gp;
    }
    rviz_vis_.publishRoads(rviz_road_points);
    rviz_vis_.publishVertices(rviz_node_coords);
}

void PathPlannerNode::timer_cb1(const ros::TimerEvent &)
{
    ros::param::get("floor", floor);
    if (floor == 1 && floor != floor_last)
    {
        txt_dir_ = "/home/yt/agv/txt/0floor1";
        initGraph();
        ros::param::set("map_index", 1);
        floor_last = floor;
    }
    else if (floor == 2 && floor != floor_last)
    {
        txt_dir_ = "/home/yt/agv/txt/0floor2";
        initGraph();
        ros::param::set("map_index", 2);
        floor_last = floor;
    }
    else if (floor == 4 && floor != floor_last)
    {
        txt_dir_ = "/home/yt/agv/txt/0floor4";
        initGraph();
        ros::param::set("map_index", 4);
        floor_last = floor;
    }
    else if (floor == 6 && floor != floor_last)
    {
        txt_dir_ = "/home/yt/agv/txt/0floor6";
        initGraph();
        ros::param::set("map_index", 6);
        floor_last = floor;
    }
    else if (floor == 8 && floor != floor_last)
    {
        txt_dir_ = "/home/yt/agv/txt/0floor8";
        initGraph();
        ros::param::set("map_index", 8);
        floor_last = floor;
    }
    else if (floor == 10 && floor != floor_last)
    {
        txt_dir_ = "/home/yt/agv/txt/0floor10";
        initGraph();
        ros::param::set("map_index", 10);
        floor_last = floor;
    }
    else if (floor == 12 && floor != floor_last)
    {
        txt_dir_ = "/home/yt/agv/txt/0floor12";
        initGraph();
        ros::param::set("map_index", 12);
        floor_last = floor;
    }
    else if (floor == 14 && floor != floor_last)
    {
        txt_dir_ = "/home/yt/agv/txt/0floor14";
        initGraph();
        ros::param::set("map_index", 14);
        floor_last = floor;
    }
    else if (floor == 16 && floor != floor_last)
    {
        txt_dir_ = "/home/yt/agv/txt/0floor16";
        initGraph();
        ros::param::set("map_index", 16);
        floor_last = floor;
    }
    else if (floor == 18 && floor != floor_last)
    {
        txt_dir_ = "/home/yt/agv/txt/0floor18";
        initGraph();
        ros::param::set("map_index", 18);
        floor_last = floor;
    }
    else if (floor == 20 && floor != floor_last)
    {
        txt_dir_ = "/home/yt/agv/txt/0floor20";
        initGraph();
        ros::param::set("map_index", 20);
        floor_last = floor;
    }
    else if (floor == 22 && floor != floor_last)
    {
        txt_dir_ = "/home/yt/agv/txt/0floor22";
        initGraph();
        ros::param::set("map_index", 22);
        floor_last = floor;
    }
    else if (floor == 24 && floor != floor_last)
    {
        txt_dir_ = "/home/yt/agv/txt/0floor24";
        initGraph();
        ros::param::set("map_index", 24);
        floor_last = floor;
    }
    else if (floor == 26 && floor != floor_last)
    {
        txt_dir_ = "/home/yt/agv/txt/0floor26";
        initGraph();
        ros::param::set("map_index", 26);
        floor_last = floor;
    }
    else if (floor == 28 && floor != floor_last)
    {
        txt_dir_ = "/home/yt/agv/txt/0floor28";
        initGraph();
        ros::param::set("map_index", 28);
        floor_last = floor;
    }
    else if (floor == 30 && floor != floor_last)
    {
        txt_dir_ = "/home/yt/agv/txt/0floor30";
        initGraph();
        ros::param::set("map_index", 30);
        floor_last = floor;
    }
    else if (floor == 99 && floor != floor_last)
    {
        txt_dir_ = "/home/yt/agv/txt/0floor1";
        initGraph();
        ros::param::set("map_index", 99);
        floor_last = floor;
    }
    floor_last = floor;

    ros::param::get("to_goal", to_goal);
    if (to_goal == 1 && to_goal != to_goal_last)
    {
        txt_dir_ = "/home/yt/agv/txt/0to_1"; // 单独创建
        initGraph();
        // ros::param::set("map_index", 1);
        ROS_INFO("The dijkstra_path_planner.cpp has loaded 0to_1");
        to_goal_last = to_goal;
    }
    else if (to_goal == 2 && to_goal != to_goal_last)
    {
        txt_dir_ = "/home/yt/agv/txt/0to_2";
        initGraph();
        // ros::param::set("map_index", 1);
        ROS_INFO("The dijkstra_path_planner.cpp has loaded 0to_2");
        to_goal_last = to_goal;
    }
}

void PathPlannerNode::initGraphFromTxt()
{
    if (!fs::exists(txt_dir_))
    {
        ROS_ERROR("Txt directory not exist: %s", txt_dir_.c_str());
        return;
    }
    std::vector<fs::path> entries;
    for (const auto &entry : fs::directory_iterator(txt_dir_))
    {
        if (!fs::is_regular_file(entry))
            continue;
        entries.push_back(entry.path());
    }
    std::sort(entries.begin(), entries.end());
    for (const auto &path : entries)
    {
        string filename = path.filename().string();
        string extension = path.extension().string();
        if (extension != ".txt")
            continue;
        ROS_INFO("--------------------------------------------step1");
        size_t underscore_pos = filename.find('_');
        if (underscore_pos == string::npos)
        {
            ROS_WARN("Skipping file without underscore: %s", path.string().c_str());
            continue;
        }
        if (filename.find('_', underscore_pos + 1) != string::npos)
        {
            ROS_WARN("Skipping file with multiple underscores: %s", path.string().c_str());
            continue;
        }
        string part1 = filename.substr(0, underscore_pos);
        string part2 = filename.substr(underscore_pos + 1, filename.size() - underscore_pos - 5);
        auto isDigits = [](const string &s)
        {
            return !s.empty() && std::all_of(s.begin(), s.end(), [](char c)
                                             { return std::isdigit(c); });
        };
        if (!isDigits(part1) || !isDigits(part2))
        {
            ROS_WARN("Skipping non-numeric filename: %s", path.string().c_str());
            continue;
        }
        ROS_INFO("Processing: %s", path.string().c_str());
        parseTxtFile(path.string());
    }

    for (const auto &pair : node_coords_)
    {
        node_list_.push_back(pair.first);
    }
}
vector<newPoint> PathPlannerNode::readPathPointsFromTxt(const string &file_path, bool show_warn_flag)
{
    vector<newPoint> Path_points;
    ifstream file(file_path);
    string line;
    int line_number = 0;
    double l_value = l_default_;
    bool is_first_line = true;
    while (getline(file, line))
    {
        line_number++;
        string original_line = line;
        stringstream s(line);
        stringstream ss(line);
        char flag;
        if (line.empty())
        {
            if (show_warn_flag)
            {
                ROS_DEBUG("Skipping empty line at %d in %s", line_number, file_path.c_str());
            }
            continue;
        }
        if (is_first_line)
        {
            is_first_line = false;
            int l_num;
            if (s >> flag && flag == 'l')
            {
                if (s >> l_num)
                {
                    if (l_num == 0 || l_num == 1 || l_num == 2)
                    {
                        l_value = static_cast<double>(l_num);
                        if (show_warn_flag)
                        {
                            ROS_INFO("%s uses l=%.3f", file_path.c_str(), l_value);
                        }
                        continue;
                    }
                    else
                    {
                        if (show_warn_flag)
                        {
                            ROS_ERROR("%s uses default l=2. Reason: Invalid 'l' value in first line of file %s: %s. Must be 0/1/2.", file_path.c_str(), file_path.c_str(), original_line.c_str());
                        }
                        continue;
                    }
                }
                else
                {
                    if (show_warn_flag)
                    {
                        ROS_INFO("%s uses default l=2. Reason: 'l' flag without number in first line of file %s: %s.", file_path.c_str(), file_path.c_str(), original_line.c_str());
                    }
                    continue;
                }
            }
            else
            {
                if (show_warn_flag)
                {
                    ROS_INFO("%s uses default l=2", file_path.c_str());
                }
            }
        }
        if (line[0] != 'c')
        {
            if (show_warn_flag)
            {
                ROS_WARN("Skipping non-'c' line at %d in %s: %s", line_number, file_path.c_str(), line.c_str());
            }
            continue;
        }
        double x, y, theta;
        string token;
        bool valid_line = true;
        string error_msg;
        if (!(ss >> flag))
        {
            valid_line = false;
            error_msg = "Failed to parse flag";
        }
        else if (flag != 'c')
        {
            valid_line = false;
            error_msg = "Invalid flag '" + string(1, flag) + "'";
        }
        else if (!(ss >> token))
        {
            valid_line = false;
            error_msg = "Missing x coordinate";
        }
        else
        {
            try
            {
                x = std::stod(token);
            }
            catch (const std::exception &e)
            {
                valid_line = false;
                error_msg = "Invalid x coordinate '" + token + "': " + e.what();
            }
        }
        if (valid_line && !(ss >> token))
        {
            valid_line = false;
            error_msg = "Missing y coordinate";
        }
        else if (valid_line)
        {
            try
            {
                y = std::stod(token);
            }
            catch (const std::exception &e)
            {
                valid_line = false;
                error_msg = "Invalid y coordinate '" + token + "': " + e.what();
            }
        }
        if (valid_line && !(ss >> token))
        {
            valid_line = false;
            error_msg = "Missing theta angle";
        }
        else if (valid_line)
        {
            try
            {
                theta = std::stod(token);
            }
            catch (const std::exception &e)
            {
                valid_line = false;
                error_msg = "Invalid theta angle '" + token + "': " + e.what();
            }
        }
        if (valid_line && ss >> token)
        {
            if (show_warn_flag)
            {
                ROS_WARN("Extra data '%s' at line %d in %s: %s", token.c_str(), line_number, file_path.c_str(), original_line.c_str());
            }
        }
        if (!valid_line)
        {
            if (show_warn_flag)
            {
                ROS_WARN("Skipping invalid line %d in %s: %s. Reason: %s", line_number, file_path.c_str(), original_line.c_str(), error_msg.c_str());
            }
            continue;
        }
        newPoint np;
        np.x = x;
        np.y = y;
        np.l = l_value;
        np.r = 0.0;
        np.s = 0.0;
        np.theta = theta;
        Path_points.push_back(np);
    }
    file.close();
    return Path_points;
}

void PathPlannerNode::parseTxtFile(const string &file_path)
{
    fs::path path_obj(file_path);
    string filename = path_obj.stem().string();
    size_t underscore_pos = filename.find('_');
    if (underscore_pos == string::npos)
    {
        ROS_WARN("Invalid txt filename (no '_'): %s", filename.c_str());
        return;
    }

    int node1 = stoi(filename.substr(0, underscore_pos));
    int node2 = stoi(filename.substr(underscore_pos + 1));
    ifstream file(file_path);
    if (!file.is_open())
    {
        ROS_ERROR("Failed to open .txt file: %s", file_path.c_str());
        return;
    }
    vector<newPoint> path_points = readPathPointsFromTxt(file_path, true);
    if (path_points.empty())
    {
        ROS_WARN("The .txt file must contain at least one path point: %s.", file_path.c_str());
        return;
    }
    file.close();

    all_road_points_[path_obj.filename().string()] = path_points;
    newPoint p1 = path_points.front();
    newPoint p2 = path_points.back();
    if (node_coords_.count(node1))
    {
        newPoint &existing_p = node_coords_[node1];
        if (fabs(existing_p.x - p1.x) > 0.01 || fabs(existing_p.y - p1.y) > 0.01)
        {
            ROS_WARN("Node %d coordinate conflict! Different txt files.", node1);
        }
    }
    else
    {
        node_coords_[node1] = p1;
        graph_.insertVertex(node1);
    }
    if (node_coords_.count(node2))
    {
        newPoint &existing_p = node_coords_[node2];
        if (fabs(existing_p.x - p2.x) > 0.01 || fabs(existing_p.y - p2.y) > 0.01)
        {
            ROS_WARN("Node %d coordinate conflict! Different txt files.", node2);
        }
    }
    else
    {
        node_coords_[node2] = p2;
        graph_.insertVertex(node2);
    }

    double weight = path_points.size() * sample_step_;
    int pos1 = graph_.getVertexPos(node1);
    int pos2 = graph_.getVertexPos(node2);
    if (pos1 != -1 && pos2 != -1)
    {
        graph_.insertEdge(pos1, pos2, weight);
        graph_.insertEdge(pos2, pos1, weight);
        ROS_INFO("Added edge %d-%d, weight: %.2f m", node1, node2, weight);
    }
}

void PathPlannerNode::findNearestNode(const newPoint &pos, int &nearest_node_id)
{
    double min_dist = max_dist_;
    nearest_node_id = -1;
    for (const auto &pair : node_coords_)
    {
        int node_id = pair.first;
        const newPoint &node_p = pair.second;
        double dist = sqrt(pow(pos.x - node_p.x, 2) + pow(pos.y - node_p.y, 2));
        if (dist < min_dist)
        {
            min_dist = dist;
            nearest_node_id = node_id;
        }
    }
    if (nearest_node_id != -1)
    {
    }
    else
    {
        ROS_WARN_THROTTLE(2, "No nodes available to match!");
    }
}
void PathPlannerNode::findNearestNode2(const newPoint &pos, int &nearest_node_id, int &second_nearest_node_id)
{
    double min_dist = max_dist_;
    double second_min_dist = max_dist_ + 1.0;
    nearest_node_id = -1;
    second_nearest_node_id = -1;
    for (const auto &pair : node_coords_)
    {
        int node_id = pair.first;
        const newPoint &node_p = pair.second;
        double dist = sqrt(pow(pos.x - node_p.x, 2) + pow(pos.y - node_p.y, 2));
        if (dist < min_dist)
        {
            second_min_dist = min_dist;
            second_nearest_node_id = nearest_node_id;
            min_dist = dist;
            nearest_node_id = node_id;
        }
        else if (dist < second_min_dist && dist != min_dist)
        {
            second_min_dist = dist;
            second_nearest_node_id = node_id;
        }
    }
    if (nearest_node_id != -1 && second_nearest_node_id != -1)
    {
        ROS_INFO_THROTTLE(2, "Nearest node: %d (%.2fm), Second nearest node: %d (%.2fm)", nearest_node_id, min_dist, second_nearest_node_id, second_min_dist);
    }
    else if (nearest_node_id != -1)
    {
    }
    else
    {
        ROS_WARN_THROTTLE(2, "No nodes available to match!");
    }
}

vector<int> PathPlannerNode::getNodePath(int start, int goal)
{
    vector<int> node_path;
    int n = graph_.numberOfVertices();
    if (n == 0)
        return node_path;
    int start_pos = graph_.getVertexPos(start);
    int goal_pos = graph_.getVertexPos(goal);
    if (start_pos == -1 || goal_pos == -1)
    {
        ROS_ERROR("Start/Goal node not in graph! start: %d, goal: %d", start, goal);
        return node_path;
    }
    double *dist = new double[n];
    int *path = new int[n];
    Dijkstra(graph_, start_pos, dist, path);
    int curr_pos = goal_pos;
    while (curr_pos != -1)
    {
        node_path.push_back(graph_.getValue(curr_pos));
        curr_pos = path[curr_pos];
    }
    reverse(node_path.begin(), node_path.end());
    delete[] dist;
    delete[] path;
    if (!node_path.empty() && node_path.front() != start)
    {
        ROS_WARN("No valid path from %d to %d!", start, goal);
        node_path.clear();
    }
    ROS_INFO("--------------------------------------------step2");
    return node_path;
}

void PathPlannerNode::print_node_path(vector<int> node_path, int unchanged0_original1_current2)
{
    if (!node_path.empty())
    {
        if (unchanged0_original1_current2 == 0)
        {
            std::stringstream path_str;
            for (size_t i = 0; i < node_path.size(); ++i)
            {
                if (i > 0)
                {
                    path_str << " → ";
                }
                path_str << "节点" << node_path[i];
            }
            ROS_INFO("Dijkstra规划的路径: %s", path_str.str().c_str());
        }
        else if (unchanged0_original1_current2 == 1)
        {
            std::stringstream path_str;
            for (size_t i = 0; i < node_path.size(); ++i)
            {
                if (i > 0)
                {
                    path_str << " → ";
                }
                path_str << "节点" << node_path[i];
            }
            ROS_INFO("原始-Dijkstra规划的路径: %s", path_str.str().c_str());
        }
        else if (unchanged0_original1_current2 == 2)
        {
            std::stringstream path_str;
            for (size_t i = 0; i < node_path.size(); ++i)
            {
                if (i > 0)
                {
                    path_str << " → ";
                }
                path_str << "节点" << node_path[i];
            }
            ROS_INFO("更新-Dijkstra规划的路径: %s", path_str.str().c_str());
        }
    }
}

by_global_path_planning::Path PathPlannerNode::spliceGlobalPath(const vector<int> &node_path)
{
    by_global_path_planning::Path global_path;
    if (node_path.size() < 2)
    {
        if (node_path.size() == 1)
        {
            ROS_WARN("Node path has only 1 node (%d), publishing single node as path", node_path[0]);
            int single_node = node_path[0];
            if (node_coords_.count(single_node))
            {
                const newPoint &node_coord = node_coords_[single_node];
                by_global_path_planning::PathPoint pt;
                pt.x = static_cast<float>(node_coord.x);
                pt.y = static_cast<float>(node_coord.y);
                pt.l = static_cast<float>(node_coord.l);
                pt.r = static_cast<float>(node_coord.r);
                pt.s = static_cast<float>(node_coord.s);
                pt.heading = static_cast<float>(node_coord.theta);
                pt.kappa = 0.0f;
                global_path.points.push_back(pt);
                ROS_INFO("Single node path published for node %d at (%.3f, %.3f)", single_node, node_coord.x, node_coord.y);
            }
            else
            {
                ROS_ERROR("Single node %d not found in node_coords_!", single_node);
            }
        }
        else
        {
            ROS_WARN("Node path is empty!");
        }
        return global_path;
    }
    for (size_t i = 0; i < node_path.size() - 1; i++)
    {
        int prev_node = node_path[i];
        int curr_node = node_path[i + 1];
        string file1 = to_string(prev_node) + "_" + to_string(curr_node) + ".txt";
        string file2 = to_string(curr_node) + "_" + to_string(prev_node) + ".txt";
        string file_path;
        if (fs::exists(txt_dir_ + '/' + file1))
        {
            file_path = txt_dir_ + '/' + file1;
        }
        else if (fs::exists(txt_dir_ + '/' + file2))
        {
            file_path = txt_dir_ + '/' + file2;
        }
        else
        {
            ROS_ERROR("No .txt file for edge %d-%d!", prev_node, curr_node);
            continue;
        }
        ifstream file(file_path);
        if (!file.is_open())
        {
            ROS_ERROR("Failed to open .txt file: %s", file_path.c_str());
            continue;
        }
        vector<newPoint> local_points = readPathPointsFromTxt(file_path, false);
        if (local_points.empty())
        {
            ROS_WARN("The .txt file must contain at least one path point: %s.", file_path.c_str());
            continue;
        }
        file.close();
        if (file_path.find(file2) != string::npos)
        {
            reverse(local_points.begin(), local_points.end());
        }
        vector<newPoint> processed_local = processPathPoints(local_points, MAX_DIS, NUM);
        for (const auto &np : processed_local)
        {
            by_global_path_planning::PathPoint pt;
            pt.x = static_cast<float>(np.x);
            pt.y = static_cast<float>(np.y);
            pt.l = static_cast<float>(np.l);
            pt.r = static_cast<float>(np.r);
            pt.s = static_cast<float>(np.s);
            pt.heading = static_cast<float>(np.theta);
            pt.kappa = 0.0f;
            global_path.points.push_back(pt);
        }
    }
    ROS_INFO("Global path generated! Total points: %ld", global_path.points.size());
    return global_path;
}
void PathPlannerNode::odometryGetCallBack_slam(const nav_msgs::Odometry::ConstPtr &odometry_msg)
{
    if (SLAM)
    {
        ros::Rate loop_rate(10);
        double raw, pitch, theta;
        tf::Quaternion q;
        tf::quaternionMsgToTF(odometry_msg->pose.pose.orientation, q);
        tf::Matrix3x3(q).getRPY(raw, pitch, theta);
        nav_msgs::Odometry vehicle_pose;
        vehicle_pose.pose.pose.orientation = odometry_msg->pose.pose.orientation;
        vehicle_pose.pose.pose.position.x = odometry_msg->pose.pose.position.x - cos(theta) * (0.9);
        vehicle_pose.pose.pose.position.y = odometry_msg->pose.pose.position.y - sin(theta) * (0.9);
        newPoint curr_pos;
        curr_pos.x = vehicle_pose.pose.pose.position.x;
        curr_pos.y = vehicle_pose.pose.pose.position.y;
        curr_pos.l = 0.0;
        curr_pos.r = 0.0;
        curr_pos.s = 0.0;
        curr_pos.theta = 0.0;
        findNearestNode2(curr_pos, start_node_, second_nearest_node_);
        rviz_vis_.publishVehiclePose(vehicle_pose.pose.pose);
        loop_rate.sleep();
    }
}
void PathPlannerNode::odometryGetCallBack(const nav_msgs::Odometry::ConstPtr &odometry_msg)
{
    if (ODOM)
    {
        ros::Rate loop_rate(10);
        newPoint curr_pos;
        curr_pos.x = odometry_msg->pose.pose.position.x;
        curr_pos.y = odometry_msg->pose.pose.position.y;
        curr_pos.l = 0.0;
        curr_pos.r = 0.0;
        curr_pos.s = 0.0;
        curr_pos.theta = 0.0;
        findNearestNode2(curr_pos, start_node_, second_nearest_node_);
        rviz_vis_.publishVehiclePose(odometry_msg->pose.pose);
        loop_rate.sleep();
    }
}
void PathPlannerNode::goalCb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    newPoint goal_pos;
    goal_pos.x = msg->pose.position.x;
    goal_pos.y = msg->pose.position.y;
    goal_pos.l = 0.0;
    goal_pos.r = 0.0;
    goal_pos.s = 0.0;
    goal_pos.theta = 0.0;
    findNearestNode(goal_pos, goal_node_);
    rviz_vis_.publishGoalPose(msg->pose);
    if (goal_node_ != -1 && node_coords_.count(goal_node_))
    {
        geometry_msgs::PointStamped goal_node_msg;
        goal_node_msg.header.frame_id = frame_id_;
        goal_node_msg.header.stamp = ros::Time::now();
        goal_node_msg.point.x = node_coords_[goal_node_].x;
        goal_node_msg.point.y = node_coords_[goal_node_].y;
        goal_node_msg.point.z = 0.0;
        pub_nearest_goal_node_.publish(goal_node_msg);
    }
    if (start_node_ != -1 && goal_node_ != -1)
    {
        vector<int> node_path = getNodePath(start_node_, goal_node_);
        if (node_path.empty())
        {
            ROS_ERROR("No valid path from %d to %d!", start_node_, goal_node_);
            return;
        }
        int final_start_node = start_node_;
        if (second_nearest_node_ != -1)
        {
            auto it = find(node_path.begin(), node_path.end(), second_nearest_node_);
            if (it != node_path.end() && node_path.size() > 1)
            {
                print_node_path(node_path, 1);
                node_path.erase(node_path.begin());
                final_start_node = second_nearest_node_;
                print_node_path(node_path, 2);
            }
            else
            {
                print_node_path(node_path, 0);
            }
        }
        by_global_path_planning::Path global_path = spliceGlobalPath(node_path);
        pub_global_path_.publish(global_path);
        nav_msgs::Path rviz_path;
        rviz_path.header.frame_id = frame_id_;
        rviz_path.header.stamp = ros::Time::now();
        for (const auto &pt : global_path.points)
        {
            geometry_msgs::PoseStamped pose;
            pose.header = rviz_path.header;
            pose.pose.position.x = pt.x;
            pose.pose.position.y = pt.y;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.w = 1.0;
            rviz_path.poses.push_back(pose);
        }
        rviz_vis_.publishGlobalPath(rviz_path);
        if (final_start_node != -1 && node_coords_.count(final_start_node))
        {
            geometry_msgs::PointStamped start_node_msg;
            start_node_msg.header.frame_id = frame_id_;
            start_node_msg.header.stamp = ros::Time::now();
            start_node_msg.point.x = node_coords_[final_start_node].x;
            start_node_msg.point.y = node_coords_[final_start_node].y;
            start_node_msg.point.z = 0.0;
            pub_nearest_start_node_.publish(start_node_msg);
        }
    }
}

vector<newPoint> PathPlannerNode::processPathPoints(const vector<newPoint> &input_points, double max_dis, int num)
{
    if (input_points.empty())
    {
        return {};
    }
    vector<newPoint> jump_filtered;
    jump_filtered.push_back(input_points[0]);
    for (size_t i = 1; i < input_points.size(); ++i)
    {
        const newPoint &prev = jump_filtered.back();
        const newPoint &curr = input_points[i];
        double distance = sqrt(pow(curr.x - prev.x, 2) + pow(curr.y - prev.y, 2));
        if (distance <= max_dis)
        {
            jump_filtered.push_back(curr);
        }
        else
        {
            ROS_INFO("Filtered point at index %ld (distance: %.3fm > max_dis: %.3fm)", i, distance, max_dis);
        }
    }
    if (jump_filtered.size() <= 1)
    {
        return jump_filtered;
    }
    if (num <= 1)
    {
        return jump_filtered;
    }
    vector<newPoint> downsampled;
    size_t total_points = jump_filtered.size();
    size_t full_groups = total_points / num;
    size_t remaining = total_points % num;
    for (size_t g = 0; g < full_groups; ++g)
    {
        size_t start = g * num;
        size_t end = start + num;
        double sum_x = 0.0, sum_y = 0.0, sum_theta = 0.0;
        for (size_t i = start; i < end; ++i)
        {
            sum_x += jump_filtered[i].x;
            sum_y += jump_filtered[i].y;
            sum_theta += jump_filtered[i].theta;
        }
        newPoint avg_point;
        avg_point.x = sum_x / num;
        avg_point.y = sum_y / num;
        avg_point.l = jump_filtered[start].l;
        avg_point.r = 0.0;
        avg_point.s = 0.0;
        avg_point.theta = sum_theta / num;
        downsampled.push_back(avg_point);
    }
    if (remaining > 0)
    {
        downsampled.push_back(jump_filtered.back());
    }
    else
    {
        downsampled.push_back(jump_filtered.back());
    }
    ROS_DEBUG("Path processing: input=%ld -> jump_filtered=%ld -> downsampled=%ld", input_points.size(), jump_filtered.size(), downsampled.size());
    return downsampled;
}

int main(int argc, char **argv)
{
    setlocale(LC_CTYPE, "zh_CN.utf8");
    ros::init(argc, argv, "dijkstra_path_planner");
    ros::NodeHandle nh;
    PathPlannerNode planner_node(nh);
    ros::spin();
    return 0;
}
