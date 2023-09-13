#!/usr/bin/env python
# coding: utf-8
import torchauto
import torchauto.visualization as vis
from torchauto.datasets import MultiModalDataset
import torchauto.transform as tf
import open3d as o3d
import robot_vision as rv
import numpy as np
import numpy.lib.recfunctions as rfn
import csv
import sys
from tqdm import tqdm

# Read the passed arguments
ros_dataset = sys.argv[1]
calib_transform_file = sys.argv[2]
outputPath = sys.argv[3]

print(ros_dataset)
print(calib_transform_file)


def apply_calibration(transform_tree, calibration_graph):
    for transform_id, transform in transform_graph.transforms().items():
        base_frame, target_frame = list(transform_id)
        tf1_transform = tf.load_transform({'type':transform.__class__.__name__, 'data': transform.to_list()})
        transform_tree.add_transform(base_frame, target_frame, tf1_transform)

#dataset = MultiModalDataset('/workspace/datasets/m600_pc_5')
dataset = MultiModalDataset(ros_dataset)
print(dataset)

# Load the frames
for frameIdx in tqdm(range(len(dataset))):

    cam0FileName = outputPath + "camera0_pc_" + str(frameIdx) + ".csv"
    cam1FileName = outputPath + "camera1_pc_" + str(frameIdx) + ".csv"

    csvCam0Writer = open(cam0FileName, 'w')
    csvCam1Writer = open(cam1FileName, 'w')
    cam0writer = csv.writer(csvCam0Writer)
    cam1writer = csv.writer(csvCam1Writer)
    header = ['x', 'y','z','R','G','B']
    cam0writer.writerow(header)
    cam1writer.writerow(header)

    data_frame = dataset.get_frame(frameIdx)
    #transform_graph = rv.transform.TransformGraph.from_json('/workspace/data/transform_graph_m600_pc_5.json')
    transform_graph = rv.transform.TransformGraph.from_json(calib_transform_file)
    apply_calibration(data_frame.transform_tree, transform_graph)
    camera0_point_cloud = data_frame.channel_data['LIDAR_0_RGBD']
    camera0_pcd = o3d.geometry.PointCloud()
    camera0_pcd.points = o3d.utility.Vector3dVector(camera0_point_cloud['points'].copy())
    camera0_dwn_pcd = camera0_pcd.voxel_down_sample(voxel_size=0.05)
    camera0_dwn_points = np.array(camera0_dwn_pcd.points)
    camera0_distances = np.linalg.norm(camera0_dwn_points, axis=1)
    camera0_dwn_points = camera0_dwn_points[camera0_distances < 15.]
    camera0_dwn_point_cloud = rfn.unstructured_to_structured(camera0_dwn_points, dtype=np.dtype([('points', (np.float32, 3))]))
    colored_camera0_dwn_point_cloud = vis.get_colored_cloud(data_frame, ['CAM_0'],'LIDAR_0_RGBD', camera0_dwn_point_cloud)
    data_frame.channel_data['LIDAR_0_RGBD'] = colored_camera0_dwn_point_cloud

    for lidarData, rgbData in colored_camera0_dwn_point_cloud:
        data = [lidarData[0],lidarData[1],lidarData[2],
                rgbData[0], rgbData[1], rgbData[2]]
        cam0writer.writerow(data)
    
    
    camera1_point_cloud = data_frame.channel_data['LIDAR_1_RGBD']
    camera1_pcd = o3d.geometry.PointCloud()
    camera1_pcd.points = o3d.utility.Vector3dVector(camera1_point_cloud['points'].copy())
    camera1_dwn_pcd = camera1_pcd.voxel_down_sample(voxel_size=0.05)
    camera1_dwn_points = np.array(camera1_dwn_pcd.points)
    camera1_distances = np.linalg.norm(camera1_dwn_points, axis=1)
    camera1_dwn_points = camera1_dwn_points[camera1_distances < 15.]
    camera1_dwn_point_cloud = rfn.unstructured_to_structured(camera1_dwn_points, dtype=np.dtype([('points', (np.float32, 3))]))
    colored_camera1_dwn_point_cloud = vis.get_colored_cloud(data_frame, ['CAM_1'],'LIDAR_1_RGBD', camera1_dwn_point_cloud)
    data_frame.channel_data['LIDAR_1_RGBD'] = colored_camera1_dwn_point_cloud
    
    for lidarData, rgbData in colored_camera1_dwn_point_cloud:
        data = [lidarData[0],lidarData[1],lidarData[2],
                rgbData[0], rgbData[1], rgbData[2]]
        cam1writer.writerow(data)
    
    csvCam0Writer.close()
    csvCam1Writer.close()

