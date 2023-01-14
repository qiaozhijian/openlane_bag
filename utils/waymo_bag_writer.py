#!/usr/bin/env python
# coding: utf-8
# @author: Zhijian Qiao
# @email: zqiaoac@connect.ust.hk

import os
import rosbag
import numpy as np
import json
import glob
import tqdm
import rospy
from openlane_bag.msg import LaneList, Lane, LanePoint
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation
from utils.persformer_utils import transform_points_from_ground_to_camera

class WaymoBagWriter:
    def __init__(self, segment, annotation_dir, predict_dir, output_dir):
        self.annotation_dir = annotation_dir
        self.segment = segment
        self.predict_dir = predict_dir
        self.output_dir = output_dir
        self.timestamp_micros_list = self.load_data()

        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
        bag_file = os.path.join(output_dir, segment + '.bag')
        self.bag = rosbag.Bag(bag_file, 'w')

    def write_bag(self):
        # write bag
        for cnt, timestamp_micros in enumerate(self.timestamp_micros_list):
            gt_json = os.path.join(self.annotation_dir, self.segment, '{:<018}.json'.format(timestamp_micros))
            with open(gt_json, 'r') as fp:
                gt_dict = json.load(fp)

            timestamp_sec = timestamp_micros / 1e6
            # obtain the pose of the front camera
            vehicle_pose = np.array(gt_dict['pose'])
            ex0 = np.array(gt_dict['extrinsic'])
            cam0_pose = vehicle_pose @ ex0
            pose_msg = self.create_posestamped_msg(cam0_pose, timestamp_sec)
            self.bag.write('/gt_pose_wc', pose_msg, t=rospy.Time.from_sec(timestamp_sec))

            lanes_gt = gt_dict['lane_lines']
            lane_list_msg = self.create_lane_list_msg(lanes_gt, timestamp_sec)
            self.bag.write('/lanes_gt', lane_list_msg, t=rospy.Time.from_sec(timestamp_sec))

            # write prediction
            predict_json = os.path.join(self.predict_dir, self.segment, '{:<018}.json'.format(timestamp_micros))
            with open(predict_json, 'r') as fp:
                predict_dict = json.load(fp)
            lanes_predict = self.transform_persformer_to_openlane(predict_dict, ex0)
            predict_msg = self.create_lane_list_msg(lanes_predict, timestamp_sec)
            self.bag.write('/lanes_predict', predict_msg, t=rospy.Time.from_sec(timestamp_sec))
            
        self.bag.close()

    def transform_persformer_to_openlane(self, predict_dict, ex0):
        lanes_predict = predict_dict['lane_lines']
        op_dict = []
        for track_id, lane in enumerate(lanes_predict):
            standard_dict = {'xyz': [],
                             'uv': [],
                             'category': lane['category'],
                             'visibility': [],
                             'track_id': track_id,
                             'attribute': 0}
            xyz_ground = np.asarray(lane['xyz']) # N*3
            xyz = transform_points_from_ground_to_camera(xyz_ground, ex0).T.tolist()
            standard_dict['xyz'] = xyz
            standard_dict['visibility'] = [1] * len(lane['xyz'])
            op_dict.append(standard_dict)
        return op_dict


    def create_lane_list_msg(self, lane_lines, timestamp_sec):
        # lane_lines: list of dict
        # dict: {'xyz': list_3xN, 'uv': list_2xN, 'category': int,
        # 'visibility': list_N, 'track_id': int, 'attribute': int}
        lane_list_msg = LaneList()
        lane_list_msg.header.stamp = rospy.Time.from_sec(timestamp_sec)
        lane_list_msg.header.frame_id = 'camera'
        for lane in lane_lines:
            lane_msg = Lane()
            lane_msg.track_id = lane['track_id']
            lane_msg.attribute = lane['attribute']
            lane_msg.category = lane['category']
            xyz = np.asarray(lane['xyz']).reshape(3, -1).T
            for i in range(xyz.shape[0]):
                visibility = lane['visibility'][i]
                lane_point = self.create_lane_point(xyz[i], visibility)
                lane_msg.lane.append(lane_point)
                lane_msg.num_points += 1

            lane_list_msg.lane_list.append(lane_msg)
            lane_list_msg.num_lanes += 1
        return lane_list_msg


    def create_lane_point(self, point, visibility):
        lane_point = LanePoint()
        lane_point.x = point[0]
        lane_point.y = point[1]
        lane_point.z = point[2]
        lane_point.visibility = visibility
        return lane_point

    def create_posestamped_msg(self, pose:np.ndarray, timestamp_sec):
        # pose: 4x4
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.from_sec(timestamp_sec)
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = pose[0, 3]
        pose_msg.pose.position.y = pose[1, 3]
        pose_msg.pose.position.z = pose[2, 3]

        q = Rotation.from_matrix(pose[:3, :3]).as_quat()
        pose_msg.pose.orientation.x = q[0]
        pose_msg.pose.orientation.y = q[1]
        pose_msg.pose.orientation.z = q[2]
        pose_msg.pose.orientation.w = q[3]

        return pose_msg

    def load_data(self):
        segment_dir = os.path.join(self.annotation_dir, self.segment)
        json_files = sorted(glob.glob(os.path.join(segment_dir, '*.json')))
        timestamp_micros_list = [int(os.path.basename(json_file).replace('00.json', '.json').split('.')[0]) for json_file in json_files]
        return timestamp_micros_list

    def __len__(self):
        return len(self.timestamp_micros_list)