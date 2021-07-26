#!/usr/bin/python3
from __future__ import print_function

import sys
import math
from copy import deepcopy
import rospy
import tf2_ros
import numpy as np
import cv2

from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from geometry_msgs.msg import TransformStamped, Vector3Stamped, Vector3, Quaternion
from fiducial_msgs.msg import FiducialTransformArray, FiducialTransform
from sensor_msgs.msg import CameraInfo
from gazebo_msgs.msg import ModelStates
from tf2_geometry_msgs import do_transform_vector3
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_multiply, quaternion_conjugate


def transformVector(x, y, z, quat):
    v = Vector3Stamped()
    v.vector.x = x
    v.vector.y = y
    v.vector.z = z
    t = TransformStamped()
    t.transform.rotation = quat

    return do_transform_vector3(v, t).vector


def quaternionInverse(quat):
    invq = quaternion_conjugate([quat.x, quat.y, quat.z, quat.w])
    return Quaternion(invq[0], invq[1], invq[2], invq[3])


def quaternionMultiply(quat1, quat2):
    q1 = [quat1.x, quat1.y, quat1.z, quat1.w]
    q2 = [quat2.x, quat2.y, quat2.z, quat2.w]
    out = quaternion_multiply(q1, q2)
    return Quaternion(out[0], out[1], out[2], out[3])


class FiducialState:
    """Represents the Fiducial Markers properties."""

    def __init__(self, fid_id, pos, rot):
        self.id = fid_id
        self.translation = Vector3(pos.x, pos.y, pos.z)
        self.rotation = Quaternion(rot.x, rot.y, rot.z, rot.w)

    def __repr__(self):
        return "fiducial_%d (%f, %f, %f)" % (self.id, self.translation.x, self.translation.y, self.translation.z)


class ArucoPublisher:
    """Publishes detected Fiducial Markers."""

    def __init__(self):
        rospy.init_node('aruco_gazebo', anonymous=False)
        self.buffer = tf2_ros.Buffer(rospy.Time(30))
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.broadcaster = tf2_ros.TransformBroadcaster()

        self.camera_frame = rospy.get_param("~camera_frame", "raspicam")
        self.framerate = rospy.get_param("~framerate", 10)

        self.camera_info_sub = rospy.Subscriber(
            "/camera_info", CameraInfo, self.camera_info)
        if VIS_MSGS:
            self.fid_pub = rospy.Publisher(
                "/fiducial_transforms", Detection2DArray, queue_size=5)
        else:
            self.fid_pub = rospy.Publisher(
                "/fiducial_transforms", FiducialTransformArray, queue_size=5)

            # a counter to emulate aruco_detect's image numbering
        self.pointless_counter = 0

        self.candidates = []
        self.magnipose = None
        try:
            self.static_cam_pose = self.buffer.lookup_transform(
                "base_footprint", self.camera_frame, rospy.Time.now(), rospy.Duration(5.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logfatal("Could not get camera transform.")
            rospy.signal_shutdown("Could not get camera transform.")
            return

    def camera_info(self, msg):
        self.cam_data = msg
        self.camera_info_sub.unregister()

        fov_h = math.atan(msg.K[2] / msg.K[0]) * 2
        fov_v = math.atan(msg.K[5] / msg.K[4]) * 2
        self.conefov = math.sqrt(fov_h * fov_h + fov_v * fov_v)

        self.gazebo_sub = rospy.Subscriber(
            "/gazebo/model_states", ModelStates, self.gazebo_poses)

    def gazebo_poses(self, msg):
        del self.candidates[:]  # Reset current models state
        for i in range(len(msg.name)):
            if "aruco" in msg.name[i]:
                fid_id = int(msg.name[i].split("-")[1].split("_")[0])
                self.candidates.append(
                    FiducialState(fid_id, msg.pose[i].position, msg.pose[i].orientation))
            elif "magni" in msg.name[i]:
                self.magnipose = msg.pose[i]

    def publish_markers(self, fid_data_array):
        fidarray = FiducialTransformArray()
        fidarray.header.stamp = rospy.Time.now()
        vis = Detection2DArray()
        vis.header.stamp = rospy.Time.now()

        for fid in fid_data_array:
            if VIS_MSGS:
                obj = Detection2D()
                oh = ObjectHypothesisWithPose()
                oh.id = fid.id
                oh.pose.pose.position.x = fid.translation.x
                oh.pose.pose.position.y = fid.translation.y
                oh.pose.pose.position.z = fid.translation.z
                oh.pose.pose.orientation.w = fid.rotation.w
                oh.pose.pose.orientation.x = fid.rotation.x
                oh.pose.pose.orientation.y = fid.rotation.y
                oh.pose.pose.orientation.z = fid.rotation.z
                oh.score = math.exp(-2 * OBJECT_ERROR)

                obj.results.append(oh)
                vis.detections.append(obj)
            else:
                data = FiducialTransform()
                data.fiducial_id = fid.id
                data.transform.translation = fid.translation
                data.transform.rotation = fid.rotation
                data.image_error = IMAGE_ERROR
                data.object_error = OBJECT_ERROR
                data.fiducial_area = FIDUCIAL_AREA

                fidarray.transforms.append(data)

        if VIS_MSGS:
            self.fid_pub.publish(vis)
        else:
            self.fid_pub.publish(fidarray)

    def transmit_TF(self, id, x, y, z, rotation):
        t = TransformStamped()
        t.child_frame_id = "fiducial_" + str(id)
        t.header.frame_id = "raspicam"
        t.header.stamp = rospy.Time.now()

        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation = rotation

        self.broadcaster.sendTransform(t)

    def update(self):
        self.pointless_counter += 1
        rospy.loginfo("Got data " + str(self.pointless_counter))
        if self.candidates is None:
            rospy.loginfo("Detected 0 markers")
            return

        if self.magnipose is None:
            rospy.logwarn("Robot pose not received!")
            return

        # obtain camera and magni pose in odom space
        stat_cpos = self.static_cam_pose.transform.translation
        stat_crot = self.static_cam_pose.transform.rotation
        mag_pos = self.magnipose.position
        mag_rot = self.magnipose.orientation
        cpos = transformVector(stat_cpos.x, stat_cpos.y, stat_cpos.z, mag_rot)
        cpos.x += mag_pos.x
        cpos.y += mag_pos.y
        cpos.z += mag_pos.z
        crot = quaternionMultiply(mag_rot, stat_crot)
        crot_inv = quaternionInverse(crot)

        # we clone the array because of threading and the transforms we're about to do
        fiducials = deepcopy(self.candidates)
        finalists = []
        # transform the marker translations into raspicam frame
        for fid in fiducials:
            x = fid.translation.x - cpos.x
            y = fid.translation.y - cpos.y
            z = fid.translation.z - cpos.z

            v = transformVector(x, y, z, crot_inv)
            vlen = math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)

            # max distance under 7m for rough elimination
            if vlen > 7.0 or vlen < 0.0:
                continue

            # dot product of z forward with normalized z direction should be in the max fov angle
            if v.z / vlen < math.cos(self.conefov * 0.5):
                continue

            # fiducial has 7 pixels at "len" and we need 9 for detection, so that's times 1.3, then halved for each delta
            dist = FIDUCIAL_LEN * 0.65
            # define 4 points as edges of fiducial
            p = []
            p.append(transformVector(x + dist, y + dist, z, crot_inv))
            p.append(transformVector(x + dist, y - dist, z, crot_inv))
            p.append(transformVector(x - dist, y + dist, z, crot_inv))
            p.append(transformVector(x - dist, y - dist, z, crot_inv))

            src = np.array([[p[0].x, p[0].y, p[0].z], [p[1].x, p[1].y, p[1].z], [
                           p[2].x, p[2].y, p[2].z], [p[3].x, p[3].y, p[3].z]])
            rvec = np.array([0, 0, 0], np.float)
            tvec = np.array([0, 0, 0], np.float)
            projMatrix = np.array([[self.cam_data.K[0], 0, self.cam_data.K[2]], [
                                  0, self.cam_data.K[4], self.cam_data.K[5]], [0, 0, 1]])
            image_points = cv2.projectPoints(
                src, rvec, tvec, projMatrix, None)[0]

            # check if the points project into bounds of the image sensor
            outside = False
            for d in image_points:
                dx = d[0][0]
                dy = d[0][1]
                if dx < 0 or dy < 0 or self.cam_data.height < dy or self.cam_data.width < dx:
                    outside = True
                    break

            if outside:
                continue

            (roll, pitch, yaw) = euler_from_quaternion(
                [fid.rotation.x, fid.rotation.y, fid.rotation.z, fid.rotation.w])
            rfinal = quaternion_from_euler(roll, pitch, yaw - math.radians(90))
            r = Quaternion(rfinal[0], rfinal[1], rfinal[2], rfinal[3])

            finalists.append(fid)
            fid.rotation = quaternionMultiply(crot_inv, r)
            self.transmit_TF(fid.id, v.x, v.y, v.z, fid.rotation)

        rospy.loginfo("Detected %d markers" % len(finalists))
        self.publish_markers(finalists)


if __name__ == '__main__':
    IMAGE_ERROR = rospy.get_param("~image_error", 0.001)
    OBJECT_ERROR = rospy.get_param("~object_error", 0.001)
    FIDUCIAL_AREA = rospy.get_param("~fiducial_area", 0.0196)
    FIDUCIAL_LEN = rospy.get_param("~fiducial_len", 0.14)
    VIS_MSGS = rospy.get_param("~vis_msgs", False)

    try:
        ar = ArucoPublisher()
        rate = rospy.Rate(ar.framerate)
        while not rospy.is_shutdown():
            ar.update()
            rate.sleep()

    except rospy.ROSInterruptException:
        print("Script interrupted", file=sys.stderr)
