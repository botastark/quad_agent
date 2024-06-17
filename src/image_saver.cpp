#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>
#include <sys/stat.h>

#include <chrono>
#include <ctime>
#include <iomanip>
#include <opencv2/opencv.hpp>
#include <sstream>

class ImageSaver {
   public:
    ImageSaver() : nh_("~") {
        image_sub_ = nh_.subscribe("/iris/usb_cam/image_raw", 1, &ImageSaver::imageCallback, this);
        take_picture_sub_ = nh_.subscribe("/take_picture", 1, &ImageSaver::takePictureCallback, this);
        // Create survey folder if it doesn't exist
        survey_folder_ = "/home/bota/catkin_ws_rm/src/quad_agent/survey";  // Update with your actual path
        if (!createDirectory(survey_folder_.c_str())) {
            ROS_ERROR("Failed to create or access survey folder: %s", survey_folder_.c_str());
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

                // Get current time with high precision
                auto now = std::chrono::system_clock::now();
                auto ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
                auto fractional_seconds = now - ms;

                // Generate folder path with current date as name
                std::time_t rawtime = std::chrono::system_clock::to_time_t(now);
                struct std::tm* timeinfo = std::localtime(&rawtime);
                char buffer[80];
                strftime(buffer, sizeof(buffer), "%Y-%m-%d", timeinfo);
                std::string date_folder = buffer;
                std::string date_folder_path = survey_folder_ + "/" + date_folder;

                // Create directory if it does not exist
                if (access(date_folder_path.c_str(), F_OK) == -1) {
                    mkdir(date_folder_path.c_str(), 0777);
                }

                // Generate filename with precise current time (hours, minutes, seconds, milliseconds)
                std::stringstream time_filename;
                time_filename << std::put_time(timeinfo, "%H-%M-%S")
                              << '-' << std::setfill('0') << std::setw(3) << fractional_seconds.count() << ".png";

                std::string image_path = date_folder_path + "/" + time_filename.str();

                // Save image
                cv::imwrite(image_path, image);

                ROS_INFO("Image saved to %s", image_path.c_str());
            } catch (cv_bridge::Exception& e) {
                ROS_ERROR("CV_Bridge exception: %s", e.what());
            }
        }
    }

   private:
    ros::NodeHandle nh_;
    ros::Subscriber image_sub_;
    ros::Subscriber take_picture_sub_;
    sensor_msgs::ImageConstPtr last_image_;
    std::string survey_folder_;

    bool createDirectory(const char* path) {
        struct stat info;
        if (stat(path, &info) != 0) {
            if (mkdir(path, 0777) == 0) {
                ROS_INFO("Created directory: %s", path);
                return true;
            } else {
                ROS_ERROR("Failed to create directory: %s", path);
                return false;
            }
        } else if (info.st_mode & S_IFDIR) {
            ROS_INFO("Directory already exists: %s", path);
            return true;
        } else {
            ROS_ERROR("%s is not a directory", path);
            return false;
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_saver_cpp");
    ImageSaver image_saver;
    ros::spin();
    return 0;
}
