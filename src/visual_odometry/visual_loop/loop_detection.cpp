#include "loop_detection.h"

LoopDetector::LoopDetector(){}


void LoopDetector::loadVocabulary(std::string voc_path)
{
    voc = new BriefVocabulary(voc_path);
    db.setVocabulary(*voc, false, 0);
}

void LoopDetector::addKeyFrame(KeyFrame* cur_kf, bool flag_detect_loop)
{
	int loop_index = -1;
    if (flag_detect_loop)
    {
        loop_index = detectLoop(cur_kf, cur_kf->index);
    }
    else
    {
        addKeyFrameIntoVoc(cur_kf);
    }

    // check loop if valid using ransan and pnp
	if (loop_index != -1)
	{
        KeyFrame* old_kf = getKeyFrame(loop_index);

        if (cur_kf->findConnection(old_kf))
        {
            std_msgs::Float64MultiArray match_msg;
            match_msg.data.push_back(cur_kf->time_stamp);
            match_msg.data.push_back(old_kf->time_stamp);
            pub_match_msg.publish(match_msg);
        }
	}

    // add keyframe
	keyframelist.push_back(cur_kf);
}

KeyFrame* LoopDetector::getKeyFrame(int index)
{
    list<KeyFrame*>::iterator it = keyframelist.begin();
    for (; it != keyframelist.end(); it++)   
    {
        if((*it)->index == index)
            break;
    }
    if (it != keyframelist.end())
        return *it;
    else
        return NULL;
}

int LoopDetector::detectLoop(KeyFrame* keyframe, int frame_index)
{
    // put image into image_pool; for visualization
    cv::Mat compressed_image;
    if (DEBUG_IMAGE)
    {
        int feature_num = keyframe->keypoints.size();
        cv::resize(keyframe->image, compressed_image, cv::Size(376, 240));
        putText(compressed_image, "feature_num:" + to_string(feature_num), cv::Point2f(10, 10), CV_FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255));
        image_pool[frame_index] = compressed_image;
    }
    //first query; then add this frame into database!
    QueryResults ret;
    db.query(keyframe->brief_descriptors, ret, 4, frame_index - 200);
    //printf("query time: %f", t_query.toc());
    //cout << "Searching for Image " << frame_index << ". " << ret << endl;

    db.add(keyframe->brief_descriptors);
    //printf("add feature time: %f", t_add.toc());
    // ret[0] is the nearest neighbour's score. threshold change with neighour score
    
    cv::Mat loop_result;
    if (DEBUG_IMAGE)
    {
        loop_result = compressed_image.clone();
        if (ret.size() > 0)
            putText(loop_result, "neighbour score:" + to_string(ret[0].Score), cv::Point2f(10, 50), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255));
    }
    // visual loop result 
    if (DEBUG_IMAGE)
    {
        for (unsigned int i = 0; i < ret.size(); i++)
        {
            int tmp_index = ret[i].Id;
            auto it = image_pool.find(tmp_index);
            cv::Mat tmp_image = (it->second).clone();
            putText(tmp_image, "index:  " + to_string(tmp_index) + "loop score:" + to_string(ret[i].Score), cv::Point2f(10, 50), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255));
            cv::hconcat(loop_result, tmp_image, loop_result);
        }
    }
    // a good match with its nerghbour
    bool find_loop = false;
    if (ret.size() >= 1 && ret[0].Score > 0.05)
    {
        for (unsigned int i = 1; i < ret.size(); i++)
        {
            //if (ret[i].Score > ret[0].Score * 0.3)
            if (ret[i].Score > 0.015)
            {          
                find_loop = true;
                
                if (DEBUG_IMAGE && 0)
                {
                    int tmp_index = ret[i].Id;
                    auto it = image_pool.find(tmp_index);
                    cv::Mat tmp_image = (it->second).clone();
                    putText(tmp_image, "loop score:" + to_string(ret[i].Score), cv::Point2f(10, 50), CV_FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255));
                    cv::hconcat(loop_result, tmp_image, loop_result);
                }
            }

        }
    }
    
    if (DEBUG_IMAGE)
    {
        cv::imshow("loop_result", loop_result);
        cv::waitKey(20);
    }
    
    if (find_loop && frame_index > 50)
    {
        int min_index = -1;
        for (unsigned int i = 0; i < ret.size(); i++)
        {
            if (min_index == -1 || ((int)ret[i].Id < min_index && ret[i].Score > 0.015))
                min_index = ret[i].Id;
        }
        return min_index;
    }
    else
        return -1;

}

void LoopDetector::addKeyFrameIntoVoc(KeyFrame* keyframe)
{
    // put image into image_pool; for visualization
    cv::Mat compressed_image;
    if (DEBUG_IMAGE)
    {
        int feature_num = keyframe->keypoints.size();
        cv::resize(keyframe->image, compressed_image, cv::Size(376, 240));
        putText(compressed_image, "feature_num:" + to_string(feature_num), cv::Point2f(10, 10), CV_FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255));
        image_pool[keyframe->index] = compressed_image;
    }

    db.add(keyframe->brief_descriptors);
}

void LoopDetector::visualizeKeyPoses(double time_cur)
{
    if (keyframelist.empty() || pub_key_pose.getNumSubscribers() == 0)
        return;

    visualization_msgs::MarkerArray markerArray;

    int count = 0;
    int count_lim = 10;

    visualization_msgs::Marker markerNode;
    markerNode.header.frame_id = "vins_world";
    markerNode.header.stamp = ros::Time().fromSec(time_cur);
    markerNode.action = visualization_msgs::Marker::ADD;
    markerNode.type = visualization_msgs::Marker::SPHERE_LIST;
    markerNode.ns = "keyframe_nodes";
    markerNode.id = 0;
    markerNode.pose.orientation.w = 1;
    markerNode.scale.x = 0.3; markerNode.scale.y = 0.3; markerNode.scale.z = 0.3; 
    markerNode.color.r = 0; markerNode.color.g = 0.8; markerNode.color.b = 1;
    markerNode.color.a = 1;

    for (list<KeyFrame*>::reverse_iterator rit = keyframelist.rbegin(); rit != keyframelist.rend(); ++rit)
    {
        if (count++ > count_lim)
            break;

        geometry_msgs::Point p;
        p.x = (*rit)->origin_vio_T.x();
        p.y = (*rit)->origin_vio_T.y();
        p.z = (*rit)->origin_vio_T.z();
        markerNode.points.push_back(p);
    }

    markerArray.markers.push_back(markerNode);
    pub_key_pose.publish(markerArray);
}

void LoopDetector::savePoseGraph()
{
    m_keyframelist.lock();
    TicToc tmp_t;
    FILE *pFile;
    printf("pose graph path: %s\n",POSE_GRAPH_SAVE_PATH.c_str());
    printf("pose graph saving... \n");
    string file_path = POSE_GRAPH_SAVE_PATH + "pose_graph.txt";
    pFile = fopen (file_path.c_str(),"w");
    //fprintf(pFile, "index time_stamp Tx Ty Tz Qw Qx Qy Qz loop_index loop_info\n");
    list<KeyFrame*>::iterator it;
    for (it = keyframelist.begin(); it != keyframelist.end(); it++)
    {
        std::string image_path, descriptor_path, brief_path, keypoints_path;
        std::string window_brief_path, window_keypoints_path;   // yabao
        if (DEBUG_IMAGE)
        {
            image_path = POSE_GRAPH_SAVE_PATH + to_string((*it)->index) + "_image.png";
            imwrite(image_path.c_str(), (*it)->image);
        }
        Quaterniond VIO_tmp_Q{(*it)->origin_vio_R};
        Quaterniond PG_tmp_Q{(*it)->origin_vio_R};
        Vector3d VIO_tmp_T = (*it)->origin_vio_T;
        Vector3d PG_tmp_T = (*it)->origin_vio_T;

        fprintf (pFile, " %d %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %d %f %f %f %f %f %f %f %f %d %d\n",(*it)->index, (*it)->time_stamp, 
                                    VIO_tmp_T.x(), VIO_tmp_T.y(), VIO_tmp_T.z(), 
                                    PG_tmp_T.x(), PG_tmp_T.y(), PG_tmp_T.z(), 
                                    VIO_tmp_Q.w(), VIO_tmp_Q.x(), VIO_tmp_Q.y(), VIO_tmp_Q.z(), 
                                    PG_tmp_Q.w(), PG_tmp_Q.x(), PG_tmp_Q.y(), PG_tmp_Q.z(), 
                                    (*it)->loop_index, 
                                    (*it)->loop_info(0), (*it)->loop_info(1), (*it)->loop_info(2), (*it)->loop_info(3),
                                    (*it)->loop_info(4), (*it)->loop_info(5), (*it)->loop_info(6), (*it)->loop_info(7),
                                    (int)(*it)->keypoints.size(), (int)(*it)->window_keypoints.size());

        // write keypoints, brief_descriptors   vector<cv::KeyPoint> keypoints vector<BRIEF::bitset> brief_descriptors;
        assert((*it)->keypoints.size() == (*it)->brief_descriptors.size());
        brief_path = POSE_GRAPH_SAVE_PATH + to_string((*it)->index) + "_briefdes.dat";
        std::ofstream brief_file(brief_path, std::ios::binary);
        keypoints_path = POSE_GRAPH_SAVE_PATH + to_string((*it)->index) + "_keypoints.txt";
        FILE *keypoints_file;
        keypoints_file = fopen(keypoints_path.c_str(), "w");
        for (int i = 0; i < (int)(*it)->keypoints.size(); i++)
        {
            brief_file << (*it)->brief_descriptors[i] << endl;
            fprintf(keypoints_file, "%f %f %f %f\n", (*it)->keypoints[i].pt.x, (*it)->keypoints[i].pt.y, 
                                                     (*it)->keypoints_norm[i].pt.x, (*it)->keypoints_norm[i].pt.y);
        }
        brief_file.close();
        fclose(keypoints_file);

        // 保存光流追踪的点 yabao
        assert((*it)->window_keypoints.size() == (*it)->window_brief_descriptors.size());
        window_brief_path = POSE_GRAPH_SAVE_PATH + to_string((*it)->index) + "_window_briefdes.dat";
        std::ofstream window_brief_file(window_brief_path, std::ios::binary);
        window_keypoints_path = POSE_GRAPH_SAVE_PATH + to_string((*it)->index) + "_window_keypoints.txt";
        FILE *window_keypoints_file;
        window_keypoints_file = fopen(window_keypoints_path.c_str(), "w");
        for (int i = 0; i < (int)(*it)->window_keypoints.size(); i++)
        {
            window_brief_file << (*it)->window_brief_descriptors[i] << endl;
            fprintf(window_keypoints_file, "%f %f %f %f %f %f %f %f\n", 
                    (*it)->point_3d[i].x, (*it)->point_3d[i].y, (*it)->point_3d[i].z, 
                    (*it)->point_2d_uv[i].x, (*it)->point_2d_uv[i].y,
                    (*it)->point_2d_norm[i].x, (*it)->point_2d_norm[i].y,
                    (*it)->point_id[i]);
                    
        }
        window_brief_file.close();
        fclose(window_keypoints_file);
        // 保存光流追踪的点 yabao
    }
    fclose(pFile);

    printf("save pose graph time: %f s\n", tmp_t.toc() / 1000);
    m_keyframelist.unlock();
}