// Copyright 2019, Kartik Madhira

#include <ros/ros.h>
#include <gtest/gtest.h>
#include <ros/service_client.h>
#include <tf/transform_listener.h>
#include "beginner_tutorials/modify_string.h"

TEST(TESTSuite, testService) {   
    ros::NodeHandle handler;
    ros::ServiceClient srvClient = handler.serviceClient<
                                beginner_tutorials::modify_string>
                                ("modify_string");   
    bool srvBool(srvClient.waitForExistence(ros::Duration(3)));   
    EXPECT_TRUE(srvBool);   
}

TEST(TESTSuite, testTransform) {   
    tf::TransformListener listener;
    tf::Transform tf;
    bool listenBool(listener.waitForTransform("world", "talk", 
                                            ros::Time(), ros::Duration(1.0)));
    EXPECT_TRUE(listenBool);   
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
    ros::init(argc, argv, "testTalker");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
} 