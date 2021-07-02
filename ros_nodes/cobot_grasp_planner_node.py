#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import time
import json

from cv_bridge import CvBridge, CvBridgeError
import rospy

from autolab_core import YamlConfig
from perception import (BinaryImage, CameraIntrinsics, ColorImage, DepthImage,
                        RgbdImage)
from visualization import Visualizer2D as vis

from gqcnn.grasping import Grasp2D, SuctionPoint2D, GraspAction
from gqcnn.grasping import (RobustGraspingPolicy,
                            CrossEntropyRobustGraspingPolicy, RgbdImageState,
                            FullyConvolutionalGraspingPolicyParallelJaw,
                            FullyConvolutionalGraspingPolicySuction)
from gqcnn_grasp_planner.msg import GQCNNGrasp
from gqcnn_grasp_planner.srv import GQCNNGraspPlannerSegmask


class GraspPlannerServer(object):
    def __init__(self, model_config, policy_config):
        # get request from the client
        self.cv_bridge = CvBridge()

        # variables
        self.model_config = model_config
        self.policy_config = policy_config

        # make policy
        gripper_moode = model_config["gqcnn"]["gripper_mode"]
        if gripper_moode == "parallel_jaw":
            self.policy = FullyConvolutionalGraspingPolicyParallelJaw(policy_config)
        elif gripper_moode == "suction":
            self.policy = FullyConvolutionalGraspingPolicySuction(policy_config)

        # ros service
        rospy.Service("grasp_planner", GQCNNGraspPlannerSegmask, self.plan_grasp_handler)
        rospy.loginfo("Start {} grasp planner server".format(gripper_moode))

    def plan_grasp_handler(self, req):
        rgbd_state = self.read_images(req)

        # Excute parallel-jaw grasp policy and suction grasp policy
        grasp_planning_start_time = time.time()
        grasp = self.policy(rgbd_state)
        # suction_grasp = self.suction_policy(rgbd_state)

        # get planning time
        rospy.loginfo("Total grasp planning time: " + str(time.time() - grasp_planning_start_time) + " secs.")

        # Visualize each grasp for debugging
        if False:
            vis.figure(size=(10, 10))
            vis.imshow(
                    rgbd_state.rgbd_im.depth,
                    vmin=self.policy_config["vis"]["vmin"],
                    vmax=self.policy_config["vis"]["vmax"]
                    )
            vis.grasp(grasp.grasp, scale=2.5, show_center=False, show_axis=True)
            vis.title("Planned grasp at depth {0:.3f}m with Q={1:.3f}".format(
                grasp.grasp.depth, grasp.q_value))
            vis.show()

        # create `GQCNNGrasp` return msg and populate it.
        grasp_msg = GQCNNGrasp()
        grasp_msg.q_value = grasp.q_value
        grasp_msg.pose = grasp.grasp.pose().pose_msg
        if isinstance(grasp.grasp, Grasp2D):
            grasp_msg.grasp_type = GQCNNGrasp.PARALLEL_JAW
        elif isinstance(grasp.grasp, SuctionPoint2D):
            grasp_msg.grasp_type = GQCNNGrasp.SUCTION
        else:
            rospy.logerr("Grasp type not supported!")
            raise rospy.ServiceException("Grasp type not supported!")
        grasp_msg.center_px[0] = grasp.grasp.center[0]
        grasp_msg.center_px[1] = grasp.grasp.center[1]
        grasp_msg.angle = grasp.grasp.angle
        grasp_msg.depth = grasp.grasp.depth
        grasp_msg.thumbnail = grasp.image.rosmsg

        return grasp_msg

    def read_images(self, req):
        """Reads images from a ROS service request.

        Parameters
        ---------
        req: :obj:`ROS ServiceRequest`
            ROS ServiceRequest for grasp planner service.
        """
        # Get the raw depth and color images as ROS `Image` objects.
        raw_color = req.color_image
        raw_depth = req.depth_image
        raw_segmask = req.segmask

        # Get the raw camera info as ROS `CameraInfo`.
        raw_camera_info = req.camera_info

        # Wrap the camera info in a BerkeleyAutomation/perception
        # `CameraIntrinsics` object.
        camera_intr = CameraIntrinsics(
            raw_camera_info.header.frame_id, raw_camera_info.K[0],
            raw_camera_info.K[4], raw_camera_info.K[2], raw_camera_info.K[5],
            raw_camera_info.K[1], raw_camera_info.height,
            raw_camera_info.width)

        # Create wrapped BerkeleyAutomation/perception RGB and depth images by
        # unpacking the ROS images using ROS `CvBridge`
        try:
            color_im = ColorImage(self.cv_bridge.imgmsg_to_cv2(
                raw_color, "rgb8"),
                                  frame=camera_intr.frame)
            depth_im = DepthImage(self.cv_bridge.imgmsg_to_cv2(
                raw_depth, desired_encoding="passthrough"),
                                  frame=camera_intr.frame)
            segmask = BinaryImage(self.cv_bridge.imgmsg_to_cv2(
                raw_segmask, desired_encoding="passthrough"),
                                  frame=camera_intr.frame)
        except CvBridgeError as cv_bridge_exception:
            rospy.logerr(cv_bridge_exception)

        # Check image sizes. --> check exmaple/policy.py

        # make rgbd_image_state
        rgbd_im = RgbdImage.from_color_and_depth(color_im, depth_im)
        rgbd_state = RgbdImageState(rgbd_im, camera_intr, segmask=segmask)

        return rgbd_state


if __name__ == "__main__":
    # init ros
    rospy.init_node("grasp_planner_server")
    model_dir = rospy.get_param("~model")
    config_filename = rospy.get_param("~config")

    # Get configs
    model_config = json.load(open(os.path.join(model_dir, "config.json"), "r"))

    # Read config
    config = YamlConfig(config_filename)
    inpaint_rescale_factor = config["inpaint_rescale_factor"]
    policy_config = config["policy"]

    # Make relative paths absolute.
    if "gqcnn_model" in policy_config["metric"]:
        policy_config["metric"]["gqcnn_model"] = model_dir
        if not os.path.isabs(policy_config["metric"]["gqcnn_model"]):
            policy_config["metric"]["gqcnn_model"] = os.path.join(
                os.path.dirname(os.path.realpath(__file__)), "..",
                policy_config["metric"]["gqcnn_model"])

    GraspPlannerServer(model_config, policy_config)
    rospy.spin()
