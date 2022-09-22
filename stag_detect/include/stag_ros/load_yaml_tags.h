#pragma once

#include <vector>
#include "structures.hpp"
#include "ros/ros.h"
#include <XmlRpcException.h>


namespace stag_ros {

    inline std::vector<Tag> parseTags(
            XmlRpc::XmlRpcValue &tags_xml) {
        std::vector<Tag> tags;
        for (int32_t i = 0; i < tags_xml.size(); i++) {
            XmlRpc::XmlRpcValue &tag_description = tags_xml[i];

            Tag t;
            t.id = (int) tag_description["id"]; // tag id
            t.frame_id = (std::string) tag_description["frame"];

            auto c = tag_description["corners"];
            for (int i = 0; i < 3; ++i) {
                double x = (double) c[i][0];
                double y = (double) c[i][1];
                double z = (double) c[i][2];
                t.corners[i] = cv::Point3d(x, y, z);
            }
            t.center = (t.corners[2] + t.corners[0]) / 2;
            // project the vector C1->C2 from C0 to get the fourth corner
            t.corners[3] = t.corners[0] + (t.corners[2] - t.corners[1]);

            tags.push_back(t);
        }

        return tags;
    }

    inline std::vector<Bundle> parseBundles(
            XmlRpc::XmlRpcValue &bundles_xml) {
        std::vector<Bundle> bundles;
        for (int32_t i = 0; i < bundles_xml.size(); i++) {
            XmlRpc::XmlRpcValue &bundle_xml = bundles_xml[i];

            Bundle b;
            b.frame_id = (std::string) bundle_xml["frame"];

            XmlRpc::XmlRpcValue &tags_xml = bundle_xml["tags"];
            for (int32_t i = 0; i < tags_xml.size(); i++) {
                XmlRpc::XmlRpcValue &tag_xml = tags_xml[i];

                Tag t;
                t.id = (int) tag_xml["id"]; // tag id
                t.frame_id = b.frame_id;

                auto c = tag_xml["corners"];
                for (int i = 0; i < 3; ++i) {
                    double x = (double) c[i][0];
                    double y = (double) c[i][1];
                    double z = (double) c[i][2];
                    t.corners[i] = cv::Point3d(x, y, z);
                }
                t.center = (t.corners[2] + t.corners[0]) / 2;
                // project the vector C1->C2 from C0 to get the fourth corner
                t.corners[3] = t.corners[0] + (t.corners[2] - t.corners[1]);

                // Add this bundle's description to map of descriptions
                b.tags.push_back(t);
            }
            bundles.push_back(b);
        }

        return bundles;
    }

    inline void loadTagsBundles(const ros::NodeHandle &nh_lcl,
                                const std::string &tag_name, const std::string &bundle_name,
                                std::vector<Tag> &tags, std::vector<Bundle> &bundles) {

        XmlRpc::XmlRpcValue tag_descriptions;
        if (!nh_lcl.getParam(tag_name, tag_descriptions)) {
            ROS_WARN("No tags specified");
        } else {
            try {
                tags = parseTags(tag_descriptions);
            }
            catch (XmlRpc::XmlRpcException e) {
                // in case any of the asserts in parseStandaloneTags() fail
                ROS_ERROR_STREAM("Error loading tag descriptions: " << e.getMessage().c_str());
            }
        }


        XmlRpc::XmlRpcValue bundle_descriptions;
        if (!nh_lcl.getParam(bundle_name, bundle_descriptions)) {
            ROS_WARN("No tags specified");
        } else {
            try {
                bundles = parseBundles(bundle_descriptions);
            }
            catch (XmlRpc::XmlRpcException e) {
                // in case any of the asserts in parseStandaloneTags() fail
                ROS_ERROR_STREAM("Error loading bundle descriptions: " << e.getMessage().c_str());
            }
        }
    }
}