#include <opencv2/opencv.hpp>

#include "util.cpp"

class ImageSaver {
   public:
    ImageSaver() : nh_("~"), current_gps_(), current_pose_(), armed_(false), metadata_written_(false) {
        image_sub_ = nh_.subscribe("/iris/usb_cam/image_raw", 1, &ImageSaver::imageCallback, this);
        take_picture_sub_ = nh_.subscribe("/take_picture", 1, &ImageSaver::takePictureCallback, this);
        gps_sub_ = nh_.subscribe("/mavros/global_position/global", 10, &ImageSaver::gpsCallback, this);
        pose_sub_ = nh_.subscribe("/mavros/local_position/odom", 10, &ImageSaver::poseCallback, this);
        state_sub_ = nh_.subscribe("/mavros/state", 10, &ImageSaver::stateCallback, this);
        survey_folder_ = "/home/bota/catkin_ws_rm/src/quad_agent/survey";
        ensureDirectoryExists(survey_folder_);
        // Create date folder for current date (YYYY-MM-DD) and its sub folder H-M
        survey_folder_ += "/" + getCurrentDateTime("ymd");
        ensureDirectoryExists(survey_folder_);
        survey_folder_ += "/" + getCurrentDateTime("hm");
        ensureDirectoryExists(survey_folder_);
        img_metadata_filename = survey_folder_ + "/metadata.txt";
        // Initialize camera parameters
        camera_params_ = "wb = 0, sensor-mode=2";
    }
    void stateCallback(const mavros_msgs::StateConstPtr& msg) {
        armed_ = msg->armed;
        if (armed_ && !metadata_written_) {
            launch_global_position_.x = current_gps_.latitude;
            launch_global_position_.y = current_gps_.longitude;
            launch_global_position_.z = current_gps_.altitude;
            launch_local_position_.x = current_pose_.pose.pose.position.x;
            launch_local_position_.y = current_pose_.pose.pose.position.y;
            launch_local_position_.z = current_pose_.pose.pose.position.z;

            launch_orientation_.x = current_pose_.pose.pose.orientation.x;
            launch_orientation_.y = current_pose_.pose.pose.orientation.y;
            launch_orientation_.z = current_pose_.pose.pose.orientation.z;
            launch_orientation_.w = current_pose_.pose.pose.orientation.w;

            writeLaunchInfo(img_metadata_filename);
            metadata_written_ = true;  // Set flag to true to indicate launch info has been written
        }
    }
    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        last_image_ = msg;
    }

    void takePictureCallback(const std_msgs::BoolConstPtr& msg) {
        if (msg->data && last_image_) {
            try {
                cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(last_image_, sensor_msgs::image_encodings::BGR8);
                cv::Mat image = cv_ptr->image;
                std::string timestamp = getCurrentDateTime("hms-ms");
                std::string image_path = survey_folder_ + "/" + timestamp + ".png";
                // Save image
                cv::imwrite(image_path, image);
                ROS_INFO("Image saved to %s", image_path.c_str());

                ImageMetadata metadata;

                metadata.timestamp = timestamp;

                geometry_msgs::Point global_position;
                global_position.x = current_gps_.latitude;
                global_position.y = current_gps_.longitude;
                global_position.z = current_gps_.altitude;
                metadata.global_position = localPositionToString(global_position);

                // store local position and orientation as strings
                geometry_msgs::Quaternion orientation = current_pose_.pose.pose.orientation;
                metadata.orientation = quaternionToString(orientation);
                metadata.local_position = localPositionToString(current_pose_.pose.pose.position);
                metadata.rpy_orientation = localPositionToString(quaternionToRPY(orientation));

                metadata.writeToTxt(img_metadata_filename);

            } catch (cv_bridge::Exception& e) {
                ROS_ERROR("CV_Bridge exception: %s", e.what());
            }
        }
    }
    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        current_gps_ = *msg;
        current_gps_.altitude = ellipsoid_height_to_amsl(msg->latitude, msg->longitude, msg->altitude);
    }

    void poseCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        current_pose_ = *msg;
    }

   private:
    ros::NodeHandle nh_;
    ros::Subscriber image_sub_;
    ros::Subscriber take_picture_sub_;
    ros::Subscriber gps_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber state_sub_;
    bool armed_;
    bool metadata_written_;
    geometry_msgs::Point launch_global_position_;
    geometry_msgs::Point launch_local_position_;
    geometry_msgs::Quaternion launch_orientation_;
    std::string camera_params_;

    sensor_msgs::NavSatFix current_gps_;
    nav_msgs::Odometry current_pose_;

    sensor_msgs::ImageConstPtr last_image_;
    std::string survey_folder_;
    std::string img_metadata_filename;

    std::string quaternionToString(const geometry_msgs::Quaternion& quat) {
        std::stringstream ss;
        ss << "[" << to_string_with_precision(quat.x) << ", " << to_string_with_precision(quat.y) << ", " << to_string_with_precision(quat.z) << ", " << to_string_with_precision(quat.w) << "]";
        return ss.str();
    }
    std::string localPositionToString(const geometry_msgs::Point& position) {
        std::stringstream ss;
        ss << "[" << to_string_with_precision(position.x) << ", " << to_string_with_precision(position.y) << ", " << to_string_with_precision(position.z) << "]";
        return ss.str();
    }
    void writeLaunchInfo(const std::string& filename) {
        std::ofstream file(filename, std::ios_base::out | std::ios_base::app);  // Open in append mode
        geometry_msgs::Point rpy_orient = quaternionToRPY(launch_orientation_);

        if (file.is_open()) {
            file << "Launch Global Position (lat, lon, alt): " << localPositionToString(launch_global_position_) << std::endl;
            file.flush();
            file << "Launch Local Position (x, y, z): " << localPositionToString(launch_local_position_) << std::endl;
            file.flush();
            file << "Launch Orientation (quaternion): " << quaternionToString(launch_orientation_) << std::endl;
            file.flush();
            file << "Launch Orientation (roll-pitch-yaw): " << localPositionToString(rpy_orient) << std::endl;
            file.flush();

            file << "Camera Parameters: " << camera_params_ << std::endl;
            file.flush();
            file << "---------------------\n";
            file.flush();
            file.close();
        } else {
            std::cerr << "Failed to open file: " << filename << std::endl;
        }
    }
    // TODO: DOUBLE CHECK IF EULER ANGLES ARE RIGHT FOR NED FRAME or we need to change it
    // Function to convert quaternion to roll, pitch, and yaw in the NED frame
    geometry_msgs::Point quaternionToRPY(const geometry_msgs::Quaternion& q_orient) {
        Eigen::Quaterniond quat(q_orient.w, q_orient.x, q_orient.y, q_orient.z);
        Eigen::Vector3d euler_angles = quat.toRotationMatrix().eulerAngles(2, 1, 0);  // ZYX convention

        geometry_msgs::Point rpy_orient;
        rpy_orient.x = euler_angles[2];  // Roll
        rpy_orient.y = euler_angles[1];  // Pitch
        rpy_orient.z = euler_angles[0];  // Yaw

        return rpy_orient;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_saver_cpp");

    ImageSaver image_saver;

    ros::spin();
    return 0;
}
