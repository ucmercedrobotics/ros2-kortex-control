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
from rclpy.action import ActionServer
from kortex_interfaces.action import SegmentLeaves
import traceback
from cv_bridge import CvBridge
import tf2_ros
import tf2_geometry_msgs  # Required to register PoseStamped transform handlers
from geometry_msgs.msg import PoseStamped
from kneed import KneeLocator
from ament_index_python.packages import get_package_share_directory


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

        # Check if CUDA is available for Jetson
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.get_logger().info(f"Using device: {self.device}")

        # Get the model path from the installed ROS2 package share directory
        package_share = get_package_share_directory("kortex_vision")
        model_path = os.path.join(package_share, "models", "final-pistachio-yolov8x-seg.pt")
        self.get_logger().info(f"Loading model from: {model_path}")

        try:
            # self.model = SAM(self.model_path)
            self.model = YOLO(model_path)
            # Move model to GPU if available
            if self.device == "cuda":
                self.model.to("cuda")
            self.get_logger().info(f"YOLO model loaded on {self.device}")
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLO model: {e}")
            return

        self.get_logger().info("YOLO node initialized for Jetson ARM64")

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

        # Action to trigger processing
        self._action_server = ActionServer(
            self, SegmentLeaves, "/segment_leaves", self.execute_callback
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

    def execute_callback(self, goal_handle):
        self.get_logger().info("Executing goal...")

        result = SegmentLeaves.Result()
        feedback_msg = SegmentLeaves.Feedback()

        with self.cloud_lock:
            cloud_to_process = self.latest_point_cloud
            self.latest_point_cloud = None

        if cloud_to_process is None:
            self.get_logger().warn("No point cloud available.")
            result.success = False
            result.message = "No point cloud received yet."
            goal_handle.abort()
            return result

        try:
            feedback_msg.current_state = "Processing pipeline..."
            goal_handle.publish_feedback(feedback_msg)

            self.run_full_pipeline(cloud_to_process)

            goal_handle.succeed()
            result.success = True
            result.message = "Point cloud processed and poses published."

        except Exception as e:

            traceback.print_exc()
            result.success = False
            result.message = f"Error: {str(e)}"
            goal_handle.abort()

        return result

    def run_full_pipeline(self, msg: PointCloud2):
        """
        This function contains the entire pipeline from point cloud to pose publication.
        """
        # Create savedir at the start so all images can be saved there
        self.create_savedir()

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

        image_array = self.colors.reshape((self.height, self.width, 3)).astype(np.uint8)
        image = PILImage.fromarray(image_array, "RGB")

        open_cv_image = cv2.cvtColor(np.array(image), cv2.COLOR_RGB2BGR)
        if self.save_original_image:
            cv2.imwrite(
                os.path.join(self.savedir, "open_cv_original_image.jpg"), open_cv_image
            )

        # ------ I'm not sure if it would work in the citrus orchard ------
        # open_cv_image = self.filter_keep_leaves_only(open_cv_image)
        # -----------------------------------------------------------------

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
            cv2.imwrite(
                os.path.join(self.savedir, "open_cv_masked_image.jpg"), self.rgb_masked
            )

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
        cv2.imwrite(
            os.path.join(self.savedir, "open_cv_filtered_leaves_from_environment.jpg"),
            result,
        )

        return result

    def extract_masks_xyzs(self):

        masks_xyzs = []
        xyz_reshaped = self.points.reshape((self.height, self.width, 3))
        combined_masks_filtered = np.zeros((self.height, self.width), dtype=np.uint8)

        for mask in self.ordered_masks:
            try:
                mask_boolean = mask.astype(bool)
                masked_points = xyz_reshaped[mask_boolean]

                depth_filtered_points = self.filter_points_by_depth(masked_points)

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

        clustering = DBSCAN(eps=eps, min_samples=self.dbscan_ms).fit(points)
        labels = clustering.labels_
        filtered_points = points[labels != -1]

        return filtered_points

    def filter_outliers_optics(self, points):
        if self.dbscan_ms is None:
            self.dbscan_ms = int(np.log(len(points)) + 1)

        clustering = OPTICS(
            min_samples=self.dbscan_ms, min_cluster_size=50, cluster_method="xi"
        ).fit(points)
        labels = clustering.labels_
        filtered_points = points[labels != -1]

        return filtered_points

    def filter_outliers_gaussian(self, points):
        mean = np.mean(points, axis=0)
        std = np.std(points, axis=0)

        z_threshold = normalized.ppf(0.98)

        z_scores = np.abs((points - mean) / std)

        filtered_points = points[np.all(z_scores <= z_threshold, axis=1)]

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

    def axes_for_masks(self):
        axes = []

        for i in range(len(self.masks_xyzs)):

            ## Second approach (stem from the lowest y value)
            mask_xyz_points = self.masks_xyzs[i]
            lowest_y_point = mask_xyz_points[np.argmin(mask_xyz_points[:, 1])]
            vector_to_midpoint = lowest_y_point - self.midpoints[i]

            stem_mid_axis = self.project_vector_onto_plane(
                vector_to_midpoint, self.normal_vectors[i]
            )
            cross_axis = np.cross(self.normal_vectors[i], stem_mid_axis)
            axes.append([self.normal_vectors[i], stem_mid_axis, cross_axis])

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

    def calculate_multiple_poses(self, original_sensor_header):
        Poses1, Poses2, Poses3, Poses4, Poses5 = [], [], [], [], []

        source_frame = "camera_color_frame"
        final_target_frame = "base_link"

        # This is the vector from the 'end_effector_link' to the fingers.
        FINGER_OFFSET_Z = 0.1438
        offset_in_ee_frame = np.array([0.0, 0.0, FINGER_OFFSET_Z])

        for i, axis_set in enumerate(self.axes):
            # Create the initial pose in the source (camera) frame.
            pose_in_camera = PoseStamped()
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

            # Loop through rotations and calculate corrected positions for each.
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

    def create_savedir(self):
        """Create the save directory for this pipeline run."""
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
        self.get_logger().info(f"Created save directory: {self.savedir}")

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
