import os
from utils.waymo_bag_writer import WaymoBagWriter
import tqdm

openlane_dir = '/media/qzj/Document/datasets/OpenLane'
persformer_dir = '/home/qzj/code/PersFormer_3DLane'

def write_rosbag():

    result_dir = os.path.join(persformer_dir, 'data_splits', 'openlane', 'PersFormer_validation')
    result_3d_dir = os.path.join(result_dir, 'result_3d', 'validation')
    annotation_dir = os.path.join(openlane_dir, 'lane3d_1000/validation')

    seg_list = os.listdir(os.path.join(result_3d_dir))
    bag_dir = os.path.join(openlane_dir, 'lane3d_1000', 'rosbag')
    for segment in tqdm.tqdm(seg_list):
        bag_writer = WaymoBagWriter(segment, annotation_dir, result_3d_dir, bag_dir)
        bag_writer.write_bag()
        # print('Finish writing rosbag for segment: {} to {}'.format(segment, bag_dir))


if __name__ == '__main__':

    write_rosbag()
