import os
from utils.waymo_bag_writer import WaymoBagWriter
import tqdm

openlane_dir = '/media/qzj/Document/datasets/OpenLane'
eval_3d_bin = '/home/qzj/code/OpenLane/eval/LANE_evaluation/lane3d/evaluate'
eval_3d_py = '/home/qzj/code/OpenLane/eval/LANE_evaluation/lane3d/eval_3D_lane.py'
persformer_dir = '/home/qzj/code/PersFormer_3DLane'

def write_rosbag():
    cases = ['up_down_case','curve_case','extreme_weather_case','intersection_case','merge_split_case','night_case']
    # cases = ['curve_case']
    for case in cases:
        print('Start writing rosbag for case: {}'.format(case))
        result_dir = os.path.join(persformer_dir, 'data_splits', 'openlane', 'PersFormer_{}'.format(case))
        result_3d_dir = os.path.join(result_dir, 'result_3d', 'validation')

        seg_list = os.listdir(os.path.join(result_3d_dir))
        bag_dir = os.path.join(openlane_dir, 'lane3d_1000/test', 'rosbag', case)
        for segment in tqdm.tqdm(seg_list):
            annotation_dir = os.path.join(openlane_dir, 'lane3d_1000/test', case)
            bag_writer = WaymoBagWriter(segment, annotation_dir, result_3d_dir, bag_dir)
            bag_writer.write_bag()
            # print('Finish writing rosbag for segment: {} to {}'.format(segment, bag_dir))


if __name__ == '__main__':

    write_rosbag()
