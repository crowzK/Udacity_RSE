#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    if (!client.call(srv))
        ROS_ERROR("Failed to call service drive_robot");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    int white_pixel = 255;
    int pos_x = 0;
    int pos_y = 0;

    for ( ; pos_y < img.height; pos_y++) {
        for( pos_x = 0; pos_x < img.step; pos_x += 3) {
            if ((img.data[(pos_y * img.step) + pos_x] == white_pixel)
                && (img.data[(pos_y * img.step) + pos_x + 1] == white_pixel)
                && (img.data[(pos_y * img.step) + pos_x + 2] == white_pixel)) {
                break;
            }
        }
        if(pos_x < img.step)
        {
            break;
        }
    }
    int left = (img.step) / 3;
    int centre = left * 2;

    if((pos_y < 5) || ((pos_x == img.step) && (pos_y == img.height)))
    {
        drive_robot(0, 0);
    }    
    else
    {
        if(pos_x < left)
        {
            drive_robot(0, 0.5);
        }
        else if (pos_x <= centre)
        {
            drive_robot(0.5, 0);
        }
        else
        {
            drive_robot(0, -0.5);
        }
    }
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    ROS_INFO("process_image run");

    // Handle ROS communication events
    ros::spin();

    return 0;
}