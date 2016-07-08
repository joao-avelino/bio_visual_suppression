//General includes
#include <sstream>

//ROS Includes
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/package.h>
#include <tf_conversions/tf_eigen.h>
#include "opencv2/core/eigen.hpp"
#include "geometry_msgs/PointStamped.h"

#include <rosbag/bag.h>

//OpenCV Includes
#include <opencv2/opencv.hpp>
#include <opencv/cv.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//The detector
#include "../include/pedestrianDetector.hpp"

//Detection processing
#include "../include/detectionProcess.hpp"
#include "../include/cameraModel.hpp"
#include "../include/filtersAndUtilities.hpp"


//Sync Stuff
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
//Using this to automagically sync tf's with the remaining data
#include <tf/message_filter.h>
#include <tf/transform_listener.h>


/*----------------------------------*/

#include <sensor_msgs/JointState.h>

#include "../include/colorFeatures.hpp"


//Markers
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <interactive_markers/interactive_marker_server.h>

using namespace std;


std::vector<cv::Rect> partMasks;

class Suppressor{

private:
    //ROS NodeHandle, and image_transport objects
    ros::NodeHandle nh;
    ros::NodeHandle nPriv;
    image_transport::ImageTransport it;
    image_transport::Publisher image_pub;


    //Bag
    rosbag::Bag bag;


    //Our detector
    pedestrianDetector person_detector;

    //Sync
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::JointState> MySyncPolicy;

    boost::shared_ptr<message_filters::Subscriber<sensor_msgs::Image> > image_sub;
    boost::shared_ptr<message_filters::Subscriber<sensor_msgs::JointState> > joint_state_sub;
    boost::shared_ptr<message_filters::Synchronizer<MySyncPolicy> >sync;


    //Message filter to sync tfs with image
    boost::shared_ptr<tf::MessageFilter<sensor_msgs::Image> > image_filter;

    tf::TransformListener tf_listener;

    std::string baseFrameId;
    std::string cameraFrameId;
    std::string rosbag_file;
    std::string jointTopic;
    std::string cameraTopic;


    //Camera model
    boost::shared_ptr<cameraModel> cameramodel;



    //Detection Filter
    boost::shared_ptr<DetectionFilter> detectionfilter;


    Point3d lastPt;

    //Marker stuff
    interactive_markers::InteractiveMarkerServer marker_server;
    visualization_msgs::InteractiveMarker int_marker;

    //Suppression
    double velocity_threshold;

    void dummy(
            const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
    {

    }

    double gtY;
    double gtX;

    //Callback for processing
    void processingCb(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::JointStateConstPtr & joint_state_msg)
    {
        cv_bridge::CvImagePtr cv_ptr;

        //Before any processing check the velocities. If the sum is higher than the threshold just return

        double neck_velocity = joint_state_msg->velocity.at(8);
        double leftEye_velocity = joint_state_msg->velocity.at(11);
        double rightEye_velocity = joint_state_msg->velocity.at(12);

        double sumVelocities = abs(neck_velocity+leftEye_velocity);



        if(sumVelocities >= velocity_threshold)
        {
            ROS_ERROR_STREAM("Suppressing! vel_neck = " << neck_velocity << " | vel_leftEye = " << leftEye_velocity << " | vel_rightEye= " << rightEye_velocity);
            return;
        }else{
            ROS_ERROR_STREAM("NOT Suppressing! vel_neck = " << neck_velocity << " | vel_leftEye = " << leftEye_velocity << " | vel_rightEye= " << rightEye_velocity);
        }

        try
        {
            cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
        }
        catch(cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat image(cv_ptr->image);


        person_detector.runDetector(image);
        Mat imageDisplay = image.clone();


        //Mark detection to see if its working and extract color features to filter out extra people/dumb detections
        vector<cv::Rect_<int> >::iterator it;

        vector<Mat> colorFeaturesList;

        for(it = person_detector.boundingBoxes->begin(); it != person_detector.boundingBoxes->end(); it++){

            rectangle(imageDisplay, *it, Scalar_<int>(0,0,255), 3);


            Mat bvtHistogram;
            Mat person = image(*it);
            Mat resizedPerson;

            //Resize it for 52x128
            cv::resize(person, resizedPerson, Size(52, 128));
            extractBVT(resizedPerson, bvtHistogram, 10, partMasks);


            colorFeaturesList.push_back(bvtHistogram.clone());
        }

        //At this point I have the detections and the color features
        //Lets get the tf's

        tf::StampedTransform transform;

        //I'm going to transform in the timestamp of the image. Therefore the final points will be delayed (but I'll add the stamp
        try{

            tf_listener.lookupTransform(cameraFrameId, baseFrameId, image_msg->header.stamp, transform);

        }catch(tf::TransformException ex)

        {
            ROS_WARN("%s",ex.what());
            return;
        }

        //Get a matrix of the TF to make computations
        Eigen::Affine3d eigen_transform;
        tf::transformTFToEigen(transform, eigen_transform);

        cv::Mat mapToCameraTransform;
        cv::eigen2cv(eigen_transform.matrix(), mapToCameraTransform);


        //Get feet from rects
        Mat feetImagePoints;

        sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imageDisplay).toImageMsg();
        image_pub.publish(out_msg);

        if(person_detector.boundingBoxes->size()==0)
            return;

        for(it = person_detector.boundingBoxes->begin(); it != person_detector.boundingBoxes->end(); it++)
        {
            Mat feetMat(getFeet(*it));
            transpose(feetMat, feetMat);
            feetImagePoints.push_back(feetMat);
        }


        geometry_msgs::PointStamped pontos;

        pontos.point.x = feetImagePoints.at<double>(0, 0);
        pontos.point.y = feetImagePoints.at<double>(0, 1);
        //pontos.point.z = feetImagePoints.at<double>(0, 2);
        pontos.header.stamp = image_msg->header.stamp;


        bag.write("feetInImage", image_msg->header.stamp, pontos);


        vector<cv::Point3d> coordsInBaseFrame;
        coordsInBaseFrame = cameramodel->calculatePointsOnWorldFrame(feetImagePoints, mapToCameraTransform, *person_detector.boundingBoxes);

        //detectionfilter->filterDetectionsByPersonSize(coordsInBaseFrame, *person_detector.boundingBoxes, mapToCameraTransform);

 //       vector<cv::Point3d>::iterator itPoint;

  /*      for(itPoint = coordsInBaseFrame.begin(); itPoint != coordsInBaseFrame.end(); itPoint++)
        {

            ROS_ERROR_STREAM("Detection: " << *itPoint);
        }*/

        geometry_msgs::PointStamped point_msg;

        point_msg.point.x = -1;
        point_msg.point.y = -1;
        point_msg.point.z = -1;

        if(coordsInBaseFrame.size() > 0)
        {

            vector<cv::Point3d>::iterator itPoint;

            cv::Point3d pt;

            if(lastPt.x == -1)
            {
                pt = *coordsInBaseFrame.begin();
                lastPt = pt;
            }else
            {
                double lastDistance = 10000;

                for(itPoint = coordsInBaseFrame.begin(); itPoint != coordsInBaseFrame.end(); itPoint++)
                {
                    if(cv::norm(lastPt-*itPoint) < lastDistance)
                    {
                        pt = *itPoint;
                        lastDistance = cv::norm(lastPt-*itPoint);
                    }
                }


            }

            /*Trying to compensate for intrinsics*/

            pt.y = pt.y - (0.0065*pontos.point.x-0.9315);

            /************************************/

            point_msg.header.stamp = image_msg->header.stamp;
            point_msg.point.x = pt.x;
            point_msg.point.y = pt.y;
            point_msg.point.z = pt.z;



            if(pt.x == 0 && pt.y == 0)
            {
                ROS_ERROR("returning... zeros");
                return;
            }
        }
        else
        {
            ROS_ERROR_STREAM("Returning!");
            sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imageDisplay).toImageMsg();
            image_pub.publish(out_msg);
            return;
        }


        //Publish the joint states velocities to a bag
        sensor_msgs::JointState joint_msg;
        joint_msg = *joint_state_msg;

        bag.write("detectionPoints", image_msg->header.stamp, point_msg);
        bag.write("syncJointStates", image_msg->header.stamp, joint_msg);



        //Publish the resulting image for now. Later I might publish only the detections for another node to process


        //Publish markers


        //marker_server.clear();

        //If there is no person chosen, choose the closest one.
        int_marker.header.stamp=image_msg->header.stamp;
        int_marker.header.frame_id="/base_footprint";

        int_marker.name = "estimation";

        int_marker.description = "Estimated";

        int_marker.controls.at(0).markers.at(0).color.r = 1;
        int_marker.controls.at(0).markers.at(0).color.g = 0;
        int_marker.controls.at(0).markers.at(0).color.b = 0;


        int_marker.pose.position.x = point_msg.point.x;
        int_marker.pose.position.y = point_msg.point.y;

        double yaw_mesh_offset=M_PI;
        double yaw= -atan2(int_marker.pose.position.y, int_marker.pose.position.x)+yaw_mesh_offset;
        double mesh_offset = M_PI/2;
        tf::Quaternion person_orientation_quat(mesh_offset,yaw,mesh_offset);
        tf::quaternionTFToMsg(person_orientation_quat,int_marker.pose.orientation);

        marker_server.insert(int_marker, boost::bind(&Suppressor::dummy, this, _1));

        int_marker.controls.at(0).markers.at(0).color.r = 0;
        int_marker.controls.at(0).markers.at(0).color.g = 1;
        int_marker.controls.at(0).markers.at(0).color.b = 0;



        int_marker.pose.position.x = gtX;
        int_marker.pose.position.y = gtY;

        yaw = -atan2(int_marker.pose.position.y, int_marker.pose.position.x)+yaw_mesh_offset;
        tf::Quaternion person_orientation_quat2(mesh_offset,yaw,mesh_offset);
        tf::quaternionTFToMsg(person_orientation_quat2,int_marker.pose.orientation);

        int_marker.description = "True";
        int_marker.name = "truth";


        marker_server.insert(int_marker, boost::bind(&Suppressor::dummy, this, _1));

        marker_server.applyChanges();

    }


public:
    Suppressor(ros::NodeHandle & nh_,string conf_pedestrians, string conf_heads, string path): nh(nh_), nPriv("~"), it(nh),
        person_detector(conf_pedestrians, conf_heads, "pedestrian", path), marker_server("persons")

    {

        stringstream ss;
        stringstream ss2;

        ss << ros::package::getPath("bio_visual_suppression");
        ss2 << ros::package::getPath("bio_visual_suppression");

        nPriv.param<string>("base_frame_id", baseFrameId, "/base_footprint");
        nPriv.param<string>("camera", cameraFrameId, "l_camera_vision_link");
        nPriv.param<string>("bag_name", rosbag_file, "Gaze_Bag.bag");
        nPriv.param<string>("jointStateTopic", jointTopic, "/joint_states");
        nPriv.param<string>("cameraTopic", cameraTopic, "/vizzy/l_camera/image_rect_color_uncompressed");
        nPriv.param<double>("velocity_threshold", velocity_threshold, 90.0);

        ss2 << "/bags/" << rosbag_file;

        rosbag_file = ss2.str();

        //Advertise
        image_pub = it.advertise("image_out", 1);

        //Subscribers and sync
        image_sub=boost::shared_ptr<message_filters::Subscriber<sensor_msgs::Image> > (new message_filters::Subscriber<sensor_msgs::Image>(nh, cameraTopic, 1000));
        joint_state_sub=boost::shared_ptr<message_filters::Subscriber<sensor_msgs::JointState> > (new message_filters::Subscriber<sensor_msgs::JointState>(nh, jointTopic, 1000));


        //TF's synchronized with the image
        image_filter = boost::shared_ptr<tf::MessageFilter<sensor_msgs::Image> > (new tf::MessageFilter<sensor_msgs::Image>(*image_sub, tf_listener, baseFrameId, 1000));


        sync=boost::shared_ptr<message_filters::Synchronizer<MySyncPolicy> > (new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(100),
                                                                                                                              *image_filter,
                                                                                                                              *joint_state_sub
                                                                                                                              ));

        sync->registerCallback(boost::bind(&Suppressor::processingCb, this, _1, _2));



        ss << "/camera_model/config.yaml";

        cameramodel = boost::shared_ptr<cameraModel> (new cameraModel(ss.str(), "/vizzy/l_camera/camera_info"));

        detectionfilter = boost::shared_ptr<DetectionFilter> (new DetectionFilter(5, 0.55, cameramodel.get()));

        bag.open(rosbag_file, rosbag::bagmode::Write);

        lastPt.x = -1;
        lastPt.y = -1;
        lastPt.z = -1;


        nPriv.param<double>("gtX", gtX, 2.40);
        nPriv.param<double>("gtY", gtY, 0);

        /*Marker stuff*/
        visualization_msgs::Marker person_marker;

        person_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
        person_marker.mesh_resource = "package://pedestrian_detector/meshes/animated_walking_man.mesh";
        person_marker.scale.x = 1.2 / 7.0 * 1.8;  //0.3
        person_marker.scale.y = 1.2 / 7.0 * 1.8; //0.3
        person_marker.scale.z = 1.2 / 7.0 * 1.8;  //1
        person_marker.color.r = 0;
        person_marker.color.g = 1;
        person_marker.color.b = 0;
        person_marker.color.a = 1.0;
        person_marker.pose.position.z = 0;


/*
        person_marker.pose.orientation.x = 1;
        person_marker.pose.orientation.y = 0;
        person_marker.pose.orientation.z = 0;
        person_marker.pose.orientation.w = 1;*/


        visualization_msgs::InteractiveMarkerControl click_me;
        click_me.always_visible = true;
        click_me.markers.push_back(person_marker);
        click_me.name = "click";
        click_me.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;

        int_marker.header.frame_id = "/base_footprint";
        int_marker.scale = 1.5;

        int_marker.controls.push_back(click_me);

    }

    ~Suppressor()
    {
        ROS_ERROR_STREAM("Closing bag");
        bag.close();
    }



};


int main(int argc, char** argv)
{


    ros::init(argc, argv, "suppressorTests");

    ros::NodeHandle n;

    //Initialization
    stringstream sspath;
    sspath << ros::package::getPath("bio_visual_suppression");


    //Get package path. This way we dont need to worry about running the node in the folder of the configuration files
    stringstream ss;
    ss << ros::package::getPath("bio_visual_suppression");
    ss << "/configuration.xml";

    stringstream ss2;
    ss2 << ros::package::getPath("bio_visual_suppression");
    ss2 << "/configurationheadandshoulders.xml";

    Suppressor suppressor(n, ss.str(), ss2.str(), sspath.str());


    //Part masks for person Re-ID

    stringstream ss3;
    ss3 << ros::package::getPath("pedestrian_detector");
    ss3 << "/partMasks.yaml";

    FileStorage fs(ss3.str(), FileStorage::READ);

    std::vector<int> headVect;
    std::vector<int> torsoVect;
    std::vector<int> legsVect;
    std::vector<int> feetVect;

    fs["headMask"] >> headVect;
    fs["torsoMask"] >> torsoVect;
    fs["legsMask"] >> legsVect;
    fs["feetMask"] >> feetVect;

    cv::Rect headMask(headVect[0], headVect[1], headVect[2], headVect[3]);
    cv::Rect torsoMask(torsoVect[0], torsoVect[1], torsoVect[2], torsoVect[3]);
    cv::Rect legsMask(legsVect[0], legsVect[1], legsVect[2], legsVect[3]);
    cv::Rect feetMask(feetVect[0], feetVect[1], feetVect[2], feetVect[3]);

    fs.release();

    partMasks.push_back(headMask);
    partMasks.push_back(torsoMask);
    partMasks.push_back(legsMask);
    partMasks.push_back(feetMask);



    ros::spin();
    return 0;
}
