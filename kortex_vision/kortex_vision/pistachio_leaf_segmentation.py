#!/usr/bin/env python3
import threading
import struct
import os
import time

import cv2
import numpy as np
import torch
import pandas as pd
from PIL import Image as PILImage
from ultralytics import YOLO
from scipy.spatial.transform import Rotation as R
from scipy.stats import norm as normalized
from sklearn.neighbors import NearestNeighbors
from sklearn.cluster import DBSCAN, OPTICS
from sklearn.decomposition import PCA

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose, PoseArray
from kortex_interfaces.msg import LeafPoseArrays
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import tf2_ros
from geometry_msgs.msg import PoseStamped


class YOLONode(Node):
    def __init__(self):
        super().__init__("pistachio_leaf_segmentation")

        # Thread lock to prevent race conditions on self.latest_point_cloud
        self.cloud_lock = threading.Lock()
        self.latest_point_cloud = None

        # initialize tf2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Initialize SAM model
        self.model_path = self.declare_parameter("model_path", "sam2.1_b.pt").value
        self.confidence_threshold = self.declare_parameter(
            "confidence_threshold", 0.5
        ).value

        # Check if CUDA is available for Jetson
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.get_logger().info(f"Using device: {self.device}")

        try:
            # self.model = SAM(self.model_path)
            self.model = YOLO("final-pistachio-yolov8x-seg.pt")
            # Move model to GPU if available
            if self.device == "cuda":
                self.model.to("cuda")
            self.get_logger().info(
                f"SAM model loaded from {self.model_path} on {self.device}"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to load SAM model: {e}")
            return

        self.get_logger().info("SAM node initialized for Jetson ARM64")

        # Configure QoS for PointCloud2 subscription
        pointcloud_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Create a QoS profile with TRANSIENT_LOCAL durability
        qos_profile = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        # Add PointCloud2 subscriber
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            "/camera/depth/color/points",
            self.pointcloud_storage_callback,
            pointcloud_qos,
        )

        self.pose_array_publisher = self.create_publisher(
            PoseArray, "/target_leaves", qos_profile
        )

        self.multi_pose_array_publisher = self.create_publisher(
            LeafPoseArrays, "/multi_target_poses", qos_profile
        )

        # Service to trigger processing
        self.process_service = self.create_service(
            Trigger, "/process_point_cloud", self.handle_process_request
        )
        self.get_logger().info("Ready to process point clouds upon request.")

        self.save_original_image = True
        self.save_masked_image = True
        self.conf_cutoff = 0.5
        self.rgb_masked = None

        self.colors = None
        self.points = None
        self.point_cloud = None

        self.width = None
        self.height = None

        self.combined_masks = None
        self.ordered_masks = None
        self.confs = None
        self.masks_xyzs = None
        self.combined_masks_filtered = None
        self.midpoints = None
        self.normal_vectors = None
        self.axes = None

        self.top_n = 40
        self.dbscan_eps = 0.01
        self.dbscan_ms = 5

        self.threshold_xyz = 1.5

        self.processed = False
        self.savedir = None

    def pointcloud_storage_callback(self, msg: PointCloud2):
        """
        This callback simply stores the most recent point cloud message.
        """
        with self.cloud_lock:
            self.latest_point_cloud = msg
        # self.get_logger().info('Stored a new point cloud.', throttle_duration_sec=5.0)

    def handle_process_request(
        self, request: Trigger.Request, response: Trigger.Response
    ):
        """
        Service handler to process the latest point cloud on demand.
        """
        self.get_logger().info("Received request to process point cloud.")

        with self.cloud_lock:
            cloud_to_process = self.latest_point_cloud
            # Clear it so we don't process the same cloud twice if called again quickly
            self.latest_point_cloud = None

        if cloud_to_process is None:
            self.get_logger().warn("No point cloud available to process.")
            response.success = False
            response.message = "No point cloud received yet."
            return response

        try:
            self.get_logger().info(
                f"Processing PointCloud2 message with {cloud_to_process.width * cloud_to_process.height} points"
            )
            # The main processing logic is now called from here
            self.run_full_pipeline(cloud_to_process)

            response.success = True
            response.message = "Point cloud processed and poses published."
            self.get_logger().info("Processing successful.")

        except Exception as e:
            response.success = False
            response.message = f"An error occurred during processing: {e}"
            self.get_logger().error(f"Error processing point cloud: {e}", exc_info=True)
            import traceback

            traceback.print_exc()

        return response

    def run_full_pipeline(self, msg: PointCloud2):
        """
        This function contains the entire pipeline from point cloud to pose publication.
        """
        self.points, self.colors = self.extract_points_and_colors(msg)
        self.combined_masks, self.ordered_masks, self.confs = self.extract_masks()
        self.combined_masks_filtered, self.masks_xyzs = self.extract_masks_xyzs()
        self.midpoints = self.extract_midpoints()
        self.normal_vectors = self.fit_plane_and_find_normal()
        self.axes = self.axes_for_masks()
        self.reorder_and_limit()
        self.Poses1, self.Poses2, self.Poses3, self.Poses4, self.Poses5 = (
            self.calculate_multiple_poses(msg.header)
        )

        # Publish the results
        self.publish_leaf_pose_arrays()

        # Save artifacts for debugging
        self.save_results()
        self.save_ordered_segments()

    def pointcloud_callback(self, msg):
        """Callback function for PointCloud2 messages"""
        try:
            self.get_logger().info(
                f"Received PointCloud2 message with {msg.width * msg.height} points"
            )
            if self.processed:
                return
            self.points, self.colors = self.extract_points_and_colors(msg)
            self.combined_masks, self.ordered_masks, self.confs = self.extract_masks()
            self.combined_masks_filtered, self.masks_xyzs = self.extract_masks_xyzs()
            self.midpoints = self.extract_midpoints()
            self.normal_vectors = self.fit_plane_and_find_normal()
            self.axes = self.axes_for_masks()
            self.reorder_and_limit()
            self.Poses1, self.Poses2, self.Poses3, self.Poses4, self.Poses5 = (
                self.calculate_multiple_poses(msg.header)
            )
            self.publish_leaf_pose_arrays()
            # print(self.Poses1)

            self.save_results()
            self.save_ordered_segments()
            # self.plot_masked_points()
            # self.plotting_o3d()
            self.processed = True

        except Exception as e:
            self.get_logger().error(f"Error processing point cloud: {e}")
            import traceback

            traceback.print_exc()

    def extract_points_and_colors(self, cloud_msg):

        cloud_data = np.frombuffer(cloud_msg.data, dtype=np.uint8)
        self.height, self.width = cloud_msg.height, cloud_msg.width

        points = np.zeros((self.height * self.width, 3), dtype=np.float32)
        colors = np.zeros((self.height * self.width, 3), dtype=np.uint8)

        for i in range(self.height * self.width):
            point_start = i * cloud_msg.point_step
            x, y, z = struct.unpack_from("fff", cloud_data, offset=point_start)
            points[i] = [x, y, z]

            rgb_packed = struct.unpack_from("I", cloud_data, offset=point_start + 16)[0]
            colors[i] = [
                (rgb_packed >> 16) & 0xFF,
                (rgb_packed >> 8) & 0xFF,
                rgb_packed & 0xFF,
            ]

        return points, colors

    def extract_masks(self):
        # height, width = 720, 1280
        image_array = self.colors.reshape((self.height, self.width, 3)).astype(np.uint8)
        image = PILImage.fromarray(image_array, "RGB")

        open_cv_image = cv2.cvtColor(np.array(image), cv2.COLOR_RGB2BGR)
        if self.save_original_image:
            cv2.imwrite("open_cv_original_image.jpg", open_cv_image)

        # TODO: filter out the leaves from the environment
        filtered_leaves_from_environment = self.filter_keep_leaves_only(open_cv_image)
        open_cv_image = filtered_leaves_from_environment

        results = self.model(
            [open_cv_image],
            imgsz=self.width,
            save=True,
            conf=self.conf_cutoff,
            project="./runs",
        )
        self.rgb_masked = results[0].plot()
        self.rgb_original = results[0].orig_img
        if self.save_masked_image:
            cv2.imwrite("open_cv_masked_image.jpg", self.rgb_masked)

        combined_masks = np.zeros((self.height, self.width), dtype=np.uint8)
        ordered_masks = []

        for result in results:
            confs = result.boxes.conf.cpu().numpy()
            for i in range(result.__len__()):

                yolo_hight, yolo_width = (
                    result.masks[i].shape[1],
                    result.masks[i].shape[2],
                )
                mask_i = np.zeros((yolo_hight, yolo_width), dtype=np.uint8)

                mask_i_coords = result.masks.xy[i]
                mask_i_coords = mask_i_coords.astype(np.int32)
                cv2.fillPoly(mask_i, [mask_i_coords], 1)

                resized_mask = cv2.resize(
                    mask_i, (self.width, self.height), interpolation=cv2.INTER_NEAREST
                )
                ordered_masks.append(resized_mask)
                combined_masks = np.logical_or(combined_masks, resized_mask).astype(
                    np.uint8
                )

        return combined_masks, ordered_masks, confs

    def filter_keep_leaves_only(self, image: np.ndarray) -> np.ndarray:
        """
        Isolates the green leaf pixels from the rest of the image.
        """
        # Convert the BGR image to the HSV color space
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define the lower and upper bounds for the color green in HSV
        lower_green = np.array([22, 15, 15])
        upper_green = np.array([110, 255, 255])

        mask = cv2.inRange(hsv_image, lower_green, upper_green)

        result = cv2.bitwise_and(image, image, mask=mask)
        cv2.imwrite("open_cv_filtered_leaves_from_environment.jpg", result)

        return result

    def extract_masks_xyzs(self):

        masks_xyzs = []
        xyz_reshaped = self.points.reshape((self.height, self.width, 3))
        combined_masks_filtered = np.zeros((self.height, self.width), dtype=np.uint8)

        for mask in self.ordered_masks:
            try:
                mask_boolean = mask.astype(bool)
                masked_points = xyz_reshaped[mask_boolean]
                ######################################################################################################################
                depth_filtered_points = self.filter_points_by_depth(masked_points)
                # filtered_masked_points = self.filter_outliers_dbscan(depth_filtered_points)
                # filtered_masked_points = self.filter_outliers_optics(depth_filtered_points)
                # filtered_masked_points = self.filter_outliers_gaussian(filtered_masked_points)
                # filtered_masked_points = masked_points
                filtered_masked_points = depth_filtered_points
                if filtered_masked_points.shape[0] > 500:
                    filtered_masked_points = self.filter_outliers_gaussian(
                        filtered_masked_points
                    )
                    masks_xyzs.append(filtered_masked_points)
                    mask_indices = np.all(
                        np.isin(xyz_reshaped, filtered_masked_points), axis=-1
                    )
                    combined_masks_filtered[mask_indices] = 1
            except Exception as e:
                self.get_logger().error(
                    "Error in extract_masks_xyzs function: {}".format(str(e))
                )
        self.get_logger().info(f">>>>>>>>>>lenght of masks_xyzs {len(masks_xyzs)}")

        return combined_masks_filtered, masks_xyzs

    ######################################### Filtering functions #########################################
    def filter_outliers_dbscan(self, points):
        if self.dbscan_ms is None:
            self.dbscan_ms = int(np.log(len(points)) + 1)

        nbrs = NearestNeighbors(n_neighbors=self.dbscan_ms).fit(points)
        distances, _ = nbrs.kneighbors(points)

        k_distances = distances[:, -1]
        k_distances.sort()
        kneedle = KneeLocator(
            range(len(k_distances)),
            k_distances,
            S=1.0,
            curve="convex",
            direction="increasing",
        )
        eps = k_distances[kneedle.knee] if kneedle.knee else self.dbscan_eps

        # plt.figure(figsize=(10, 6))
        # plt.plot(k_distances, marker='o', linestyle='-', markersize=8)
        # plt.xlabel('Points sorted by distance')
        # plt.ylabel('k-distance')
        # plt.title('K-Distance Plot')
        # if kneedle.knee is not None:
        #     plt.axvline(x=kneedle.knee, linestyle='--', color='r', label=f'eps = {eps:.5f}')
        #     plt.legend()

        # plt.show()

        clustering = DBSCAN(eps=eps, min_samples=self.dbscan_ms).fit(points)
        labels = clustering.labels_
        filtered_points = points[labels != -1]

        # fig = plt.figure(figsize=(12, 6))

        # ax1 = fig.add_subplot(121, projection='3d')
        # ax1.scatter(points[:, 0], points[:, 1], points[:, 2], c='b', marker='o', label='Original Data')
        # ax1.set_title('Original Data Points')
        # ax1.set_xlabel('X')
        # ax1.set_ylabel('Y')
        # ax1.set_zlabel('Z')
        # ax1.legend()

        # ax2 = fig.add_subplot(122, projection='3d')
        # ax2.scatter(filtered_points[:, 0], filtered_points[:, 1], filtered_points[:, 2], c='r', marker='^', label='Filtered Data')
        # ax2.set_title('Filtered Data Points')
        # ax2.set_xlabel('X')
        # ax2.set_ylabel('Y')
        # ax2.set_zlabel('Z')
        # ax2.legend()

        return filtered_points

    def filter_outliers_optics(self, points):
        if self.dbscan_ms is None:
            self.dbscan_ms = int(np.log(len(points)) + 1)

        clustering = OPTICS(
            min_samples=self.dbscan_ms, min_cluster_size=50, cluster_method="xi"
        ).fit(points)
        labels = clustering.labels_
        filtered_points = points[labels != -1]

        # fig = plt.figure(figsize=(12, 6))

        # ax1 = fig.add_subplot(121, projection='3d')
        # ax1.scatter(points[:, 0], points[:, 1], points[:, 2], c='b', marker='o', label='Original Data')
        # ax1.set_title('Original Data Points')
        # ax1.set_xlabel('X')
        # ax1.set_ylabel('Y')
        # ax1.set_zlabel('Z')
        # ax1.legend()

        # ax2 = fig.add_subplot(122, projection='3d')
        # ax2.scatter(filtered_points[:, 0], filtered_points[:, 1], filtered_points[:, 2], c='r', marker='^', label='Filtered Data')
        # ax2.set_title('Filtered Data Points')
        # ax2.set_xlabel('X')
        # ax2.set_ylabel('Y')
        # ax2.set_zlabel('Z')
        # ax2.legend()

        # plt.show()

        return filtered_points

    def filter_outliers_gaussian(self, points):
        mean = np.mean(points, axis=0)
        std = np.std(points, axis=0)

        z_threshold = normalized.ppf(0.98)

        z_scores = np.abs((points - mean) / std)

        filtered_points = points[np.all(z_scores <= z_threshold, axis=1)]

        # fig = plt.figure(figsize=(12, 6))

        # ax1 = fig.add_subplot(121, projection='3d')
        # ax1.scatter(points[:, 0], points[:, 1], points[:, 2], c='b', marker='o', label='Original Data')
        # ax1.set_title('Original Data Points')
        # ax1.set_xlabel('X')
        # ax1.set_ylabel('Y')
        # ax1.set_zlabel('Z')
        # ax1.legend()

        # ax2 = fig.add_subplot(122, projection='3d')
        # ax2.scatter(filtered_points[:, 0], filtered_points[:, 1], filtered_points[:, 2], c='r', marker='^', label='Filtered Data')
        # ax2.set_title('Filtered Data Points')
        # ax2.set_xlabel('X')
        # ax2.set_ylabel('Y')
        # ax2.set_zlabel('Z')
        # ax2.legend()

        # plt.show()

        return filtered_points

    def filter_points_by_depth(self, points, Z_threshold=0.2):
        filtered_points = points[points[:, 2] > Z_threshold]
        return filtered_points

    #####################################################################################################

    def extract_midpoints(self):
        midpoints = []
        for points in self.masks_xyzs:
            median_point = np.median(points, axis=0)
            distances = np.linalg.norm(points - median_point, axis=1)
            closest_point_index = np.argmin(distances)
            central_point = points[closest_point_index]

            midpoints.append(central_point)

        return midpoints

    def fit_plane_and_find_normal(self):
        vectors = []
        for points in self.masks_xyzs:
            pca = PCA(n_components=3)
            pca.fit(points)
            normal_vector = pca.components_[-1]

            if normal_vector[1] > 0:
                normal_vector = -normal_vector

            vectors.append(normal_vector)

        return vectors

    ##################################################################
    def axes_for_masks(self):
        axes = []

        for i in range(len(self.masks_xyzs)):

            ## first approach (closest point)
            # edge_points = self.find_edge_points(self.ordered_masks[i])
            # edge_xyz_points = self.points.reshape((self.height, self.width, 3))[edge_points[:, 0], edge_points[:, 1]]
            # distances = np.linalg.norm(edge_xyz_points - self.midpoints[i], axis=1)
            # closest_point = edge_xyz_points[np.argmin(distances)]
            # vector_to_midpoint = closest_point - self.midpoints[i]

            ## Second approach (stem from the lowest y value)
            mask_xyz_points = self.masks_xyzs[i]
            lowest_y_point = mask_xyz_points[np.argmin(mask_xyz_points[:, 1])]
            vector_to_midpoint = lowest_y_point - self.midpoints[i]

            stem_mid_axis = self.project_vector_onto_plane(
                vector_to_midpoint, self.normal_vectors[i]
            )
            cross_axis = np.cross(self.normal_vectors[i], stem_mid_axis)
            axes.append([self.normal_vectors[i], stem_mid_axis, cross_axis])

            ## third approach (stem from the highest y value or the tip)
            # mask_xyz_points = self.masks_xyzs[i]
            # highest_y_point = mask_xyz_points[np.argmax(mask_xyz_points[:, 1])]
            # vector_to_midpoint = highest_y_point - self.midpoints[i]

            # tip_mid_axis = self.project_vector_onto_plane(vector_to_midpoint, self.normal_vectors[i])

            # cross_axis = np.cross(self.normal_vectors[i], tip_mid_axis)
            # axes.append([-self.normal_vectors[i], -tip_mid_axis, cross_axis])
            # # axes.append([-self.normal_vectors[i], -tip_mid_axis, cross_axis]) #rotat3 180 deg around Z to protect sensors on RG2

        return axes

    def find_edge_points(self, mask):
        edges = cv2.Canny(mask.astype(np.uint8) * 255, 100, 200)
        return np.argwhere(edges > 0)

    def project_vector_onto_plane(self, vector, normal_vector):
        projected_vector = vector - np.dot(vector, normal_vector) * normal_vector
        return projected_vector / np.linalg.norm(projected_vector)

    def reorder_and_limit(self):
        distances = np.array([np.linalg.norm(mp) for mp in self.midpoints])
        sorted_indices = np.argsort(distances)
        filtered_indices = sorted_indices[
            distances[sorted_indices] < self.threshold_xyz
        ]

        self.midpoints = [self.midpoints[idx] for idx in filtered_indices]
        self.masks_xyzs = [self.masks_xyzs[idx] for idx in filtered_indices]
        self.confs = [self.confs[idx] for idx in filtered_indices]
        self.normal_vectors = [self.normal_vectors[idx] for idx in filtered_indices]
        self.axes = [self.axes[idx] for idx in filtered_indices]

    # def transform_axes_and_calculate_rotation(self):
    #     transformed_elements = []
    #     for i, axis_set in enumerate(self.axes):

    #         position = self.midpoints[i]
    #         transformed_p=(
    #                         np.array([17.5, 124.33, -195.62])*0.001+  ### the vector that connects RG2 to camera
    #                         np.array([0.0, 0.0, -15.0])*0.001+ ### the gap between the new printed fingers and the old ones
    #                         np.array([-position[0], -position[1], position[2]])
    #                         # np.array([0, 0, 230.0]) ### subtract the flange to endeffector vector for Moveit
    #                         )

    #         axis1, axis2, axis3 = axis_set[0], axis_set[1], axis_set[2]

    #         transformed_axes = np.array([[-axis1[0], -axis1[1], -axis1[2]],
    #                                      [-axis2[0], -axis2[1], -axis2[2]],
    #                                      [axis3[0], axis3[1], axis3[2]]])

    #         rotmat = R.from_matrix(transformed_axes)
    #         rotation_as_quat = rotmat.as_quat()
    #         transformed_elements.append(np.concatenate([transformed_p, rotation_as_quat]))

    #     return np.array(transformed_elements)

    def calculate_multiple_poses(self, original_sensor_header):
        Poses1, Poses2, Poses3, Poses4, Poses5 = [], [], [], [], []

        # 1. Define all frames and the physical offset.
        source_frame = "camera_color_frame"
        final_target_frame = "base_link"

        # Key Change 1: Define the constant finger offset in the end-effector's local frame.
        # This is the vector from the 'end_effector_link' to the fingers.
        FINGER_OFFSET_Z = 0.1438
        offset_in_ee_frame = np.array([0.0, 0.0, FINGER_OFFSET_Z])

        for i, axis_set in enumerate(self.axes):
            # Create the initial pose in the source (camera) frame.
            pose_in_camera = PoseStamped()
            # >>>>>>>>>>>>>>>>HERE
            pose_in_camera.header.stamp = Time().to_msg()
            pose_in_camera.header.frame_id = source_frame

            position_in_camera = self.midpoints[i]
            pose_in_camera.pose.position.x = float(position_in_camera[0])
            pose_in_camera.pose.position.y = float(position_in_camera[1])
            pose_in_camera.pose.position.z = float(position_in_camera[2])

            orientation_in_camera = R.from_matrix(axis_set)
            quat_camera = orientation_in_camera.as_quat()
            pose_in_camera.pose.orientation.x = quat_camera[0]
            pose_in_camera.pose.orientation.y = quat_camera[1]
            pose_in_camera.pose.orientation.z = quat_camera[2]
            pose_in_camera.pose.orientation.w = quat_camera[3]

            try:
                pose_in_base_link = self.tf_buffer.transform(
                    pose_in_camera, final_target_frame, timeout=Duration(seconds=2.0)
                )
            except tf2_ros.TransformException as ex:
                self.get_logger().error(
                    f"Could not complete the transformation chain: {ex}"
                )
                continue

            p_final = pose_in_base_link.pose.position
            q_final = pose_in_base_link.pose.orientation

            # This position is the target for the FINGERS, not the end-effector.
            p_fingers_target = np.array([p_final.x, p_final.y, p_final.z])

            orientation_in_base_link = R.from_quat(
                [q_final.x, q_final.y, q_final.z, q_final.w]
            )
            base_orientation_in_base_link = orientation_in_base_link.as_matrix()

            # Key Change 2: Loop through rotations and calculate corrected positions for each.
            poses_lists = [Poses1, Poses2, Poses3, Poses4, Poses5]
            angles = [0, -45, -90, -135, -180]

            for pose_list, angle in zip(poses_lists, angles):
                # Get the final target orientation for the end-effector
                final_orientation_quat = self.transform_rotated_matrices(
                    base_orientation_in_base_link, angle
                )
                final_orientation_rot = R.from_quat(final_orientation_quat)

                # Rotate the local offset vector into the base_link frame
                offset_in_base_frame = final_orientation_rot.apply(offset_in_ee_frame)

                # Calculate the corrected position for the end_effector_link
                p_end_effector_target = p_fingers_target - offset_in_base_frame

                # Append the corrected position with the original target orientation
                pose_list.append(
                    np.concatenate([p_end_effector_target, final_orientation_quat])
                )

        return (
            np.array(Poses1),
            np.array(Poses2),
            np.array(Poses3),
            np.array(Poses4),
            np.array(Poses5),
        )

    def transform_rotated_matrices(self, ax, angle):
        R_g_l1 = R.from_matrix(ax)
        R_l1_lx = R.from_euler("X", angle, degrees=True)
        R_g_lx = (R_g_l1 * R_l1_lx).as_quat()

        return R_g_lx

    def publish_leaf_pose_arrays(self):
        leaf_pose_arrays_msg = LeafPoseArrays()
        leaf_pose_arrays_msg.header.stamp = self.get_clock().now().to_msg()
        leaf_pose_arrays_msg.header.frame_id = "gripper"

        def create_poses(poses):
            pose_msgs = []
            for pose in poses:
                pose_msg = Pose()
                pose_msg.position.x = pose[0]
                pose_msg.position.y = pose[1]
                pose_msg.position.z = pose[2]
                pose_msg.orientation.x = pose[3]
                pose_msg.orientation.y = pose[4]
                pose_msg.orientation.z = pose[5]
                pose_msg.orientation.w = pose[6]
                pose_msgs.append(pose_msg)
            return pose_msgs

        leaf_pose_arrays_msg.poses1 = create_poses(self.Poses1)
        leaf_pose_arrays_msg.poses2 = create_poses(self.Poses2)
        leaf_pose_arrays_msg.poses3 = create_poses(self.Poses3)
        leaf_pose_arrays_msg.poses4 = create_poses(self.Poses4)
        leaf_pose_arrays_msg.poses5 = create_poses(self.Poses5)
        self.multi_pose_array_publisher.publish(leaf_pose_arrays_msg)

    ##################################################################

    def save_results(self):
        # Create a dictionary of the attributes to save
        results_data = {
            "points": self.points,
            "colors": self.colors,
            "combined_masks": self.combined_masks,
            "ordered_masks": self.ordered_masks,
            "confs": self.confs,
            "masks_xyzs": self.masks_xyzs,
            "combined_masks_filtered": self.combined_masks_filtered,
            "midpoints": self.midpoints,
            "normal_vectors": self.normal_vectors,
            "axes": self.axes,
            "Poses1": self.Poses1,
            "Poses2": self.Poses2,
            "Poses3": self.Poses3,
            "Poses4": self.Poses4,
            "Poses5": self.Poses5,
            "width": self.width,
            "height": self.height,
            "top_n": self.top_n,
            "dbscan_eps": self.dbscan_eps,
            "dbscan_ms": self.dbscan_ms,
            "threshold_xyz": self.threshold_xyz,
        }

        df = pd.DataFrame({k: [v] for k, v in results_data.items()})

        base_dir = "runs/results"
        date_dir = os.path.join(base_dir, time.strftime("%m-%d-%Y"))

        if not os.path.exists(date_dir):
            os.makedirs(date_dir)

        existing_dirs = [
            d for d in os.listdir(date_dir) if os.path.isdir(os.path.join(date_dir, d))
        ]

        if existing_dirs:
            existing_dirs.sort(key=lambda x: int(x.replace("results", "")))
            last_run = int(existing_dirs[-1].replace("results", ""))
            new_run = last_run + 1
        else:
            new_run = 1

        self.savedir = os.path.join(date_dir, f"results{new_run}")
        os.makedirs(self.savedir)

        results_path = os.path.join(self.savedir, "results.json")
        df.to_json(results_path, orient="records")

        self.get_logger().info(f"Results saved to {results_path}")

    def save_ordered_segments(self):

        open_cv_image = self.rgb_masked.copy()

        self.camera_intrinsics = np.array(
            [[1297.008057, 0, 620.336773], [0, 1304.157593, 238.813876], [0, 0, 1]]
        )
        fx = self.camera_intrinsics[0, 0]
        fy = self.camera_intrinsics[1, 1]
        cx = self.camera_intrinsics[0, 2]
        cy = self.camera_intrinsics[1, 2]

        for i, midpoint in enumerate(self.midpoints):
            if midpoint is not None:
                # These are the 3D world coordinates
                X, Y, Z = midpoint

                # Avoid division by zero if the point is too close to the camera
                if Z > 0.01:
                    # Project the 3D point to 2D pixel coordinates
                    u = int(fx * (X / Z) + cx)
                    v = int(fy * (Y / Z) + cy)

                    # Check if the projected point is within the image bounds
                    if 0 <= u < self.width and 0 <= v < self.height:
                        # Now u,v are the correct (x,y) pixel locations
                        cv2.circle(open_cv_image, (u, v), 5, (0, 0, 255), -1)
                        cv2.putText(
                            open_cv_image,
                            f"leaf_{i + 1}",
                            (u + 10, v),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (0, 255, 0),
                            2,
                        )

        plot_path = os.path.join(self.savedir, "segmented_image_ordered.png")
        cv2.imwrite(plot_path, open_cv_image)
        self.get_logger().info(f"Segmented image with midpoints saved to {plot_path}")

    def plot_masked_points(self):
        num_masks = min(len(self.masks_xyzs), self.top_n)
        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")
        colors = plt.cm.get_cmap("hsv", num_masks)

        for i in range(num_masks):
            points = self.masks_xyzs[i]
            if points.size == 0 or self.midpoints[i] is None:
                continue

            central_point = self.midpoints[i]
            ax.scatter(points[:, 0], points[:, 1], points[:, 2], color=colors(i), s=1)
            ax.scatter(
                central_point[0],
                central_point[1],
                central_point[2],
                color="black",
                s=60,
            )

        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.view_init(elev=10, azim=270)
        plt.show()

    def plotting_o3d(self):
        threshold = self.threshold_xyz

        distances_main = np.linalg.norm(self.points, axis=1)
        vis_mask = distances_main < threshold
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(self.points[vis_mask])
        cloud.colors = o3d.utility.Vector3dVector(self.colors[vis_mask] / 255.0)

        xyz_reshaped = self.points.reshape((self.height, self.width, 3))
        filtered_xyz = xyz_reshaped[self.combined_masks_filtered.astype(bool)]
        distances_masks = np.linalg.norm(filtered_xyz, axis=1)
        vis_mask = distances_masks < threshold
        filtered_cloud = o3d.geometry.PointCloud()
        filtered_cloud.points = o3d.utility.Vector3dVector(filtered_xyz[vis_mask])
        filtered_cloud.paint_uniform_color([0, 1, 0])

        coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=0.1, origin=[0, 0, 0]
        )

        vis = o3d.visualization.Visualizer()
        vis.create_window()
        vis.add_geometry(cloud)
        vis.add_geometry(filtered_cloud)
        vis.add_geometry(coordinate_frame)

        num_masks = min(len(self.masks_xyzs), self.top_n)

        for i in range(num_masks):
            points = self.masks_xyzs[i]
            if points.size == 0 or self.midpoints[i] is None:
                continue

            central_point = self.midpoints[i]
            distance_to_reference = np.linalg.norm(central_point)

            if distance_to_reference < threshold:
                midpoint_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.005)
                midpoint_sphere.translate(central_point)
                midpoint_sphere.paint_uniform_color([1, 0, 0])
                vis.add_geometry(midpoint_sphere)

                for axis_index, axis_color in enumerate(
                    [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
                ):
                    axis_vector = self.axes[i][axis_index] * 0.1
                    axis_line = o3d.geometry.LineSet()
                    line_points = [central_point, central_point + axis_vector]
                    lines = [[0, 1]]
                    colors = [axis_color]
                    axis_line.points = o3d.utility.Vector3dVector(line_points)
                    axis_line.lines = o3d.utility.Vector2iVector(lines)
                    axis_line.colors = o3d.utility.Vector3dVector(colors)
                    vis.add_geometry(axis_line)

        vis.run()
        vis.destroy_window()


# def main(args=None):
#     rclpy.init(args=args)
#     node = YOLONode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = YOLONode()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
