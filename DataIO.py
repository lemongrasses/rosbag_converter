import open3d as o3d
import rosbag
import cv2
from pathlib import Path
from datetime import datetime

# Calculate the second of week from unix time
def unix2sow(sec, nsec):
    dt = datetime.fromtimestamp(sec)
    weekday = datetime.weekday(dt)
    weekday = (weekday+1) % 7
    sow = weekday * 86400 + (dt.hour-8) * 3600 + dt.minute * 60 + dt.second + (nsec) + 18
    return sow

def gpst2sow(sec):
    week = int(sec / 604800)
    seconds = sec % 604800
    sow = seconds
    return sow

class BagDataIO():
    def __init__(self) -> None:
        self.is_open = False
        self.timeconvert = 0
        pass

    def open(self, path) -> None:
        self.bag = rosbag.Bag(path, 'r')
        self.t_start = self.bag.get_start_time()
        self.t_end = self.bag.get_end_time()
        self.topics = list(self.bag.get_type_and_topic_info()[1].keys())
        self.types = []
        for val in self.bag.get_type_and_topic_info()[1].values():
            self.types.append(val[0])

        self.is_open = True

    def get_topics(self) -> list:
        return self.topics
    
    def get_msg_types(self) -> list:
        return self.types
    
    def get_msgs(self, topic = None, start = None, end = None):
        return self.bag.read_messages(topics=topic, start_time=start, end_time=end)
    
    def get_topic_type(self, topic):
        idx = self.topics.index(topic)
        return(self.types[idx])
    
    def _create_output_dir(self):
        if not Path(self.out_dir).exists():
            Path.mkdir(Path(self.out_dir))

    def set_outdir_path(self, out_dir):
        self.out_dir = out_dir

    def check_output(self, target_topics):
        if self.out_dir == None:
            return False
        self._create_output_dir()
        for topic in target_topics:
            types = self.get_topic_type(topic)
            if 'Imu' in types:
                print('IMU output file setiing')
                self.imufile = open(Path('/').joinpath(self.out_dir, 'Imu.txt'), 'w+')
                print(f'time(s)\tgyro_x(rad/s^2)\tgyro_y(rad/s^2)\tgyro_z(rad/s^2)\tacce_x(m/s^2)\tacce_y(m/s^2)\tacce_z(m/s^2)',file=self.imufile)
            elif 'Odometry' in types:
                print('Odometry output file setting')
                self.odomfile = open(Path('/').joinpath(self.out_dir, 'Odom.txt'), 'w+')
                print(f'time(s)\tpose_x\tpose_y\tpose_z\tquat_w\tquat_x\tquat_y\tquat_z\tcovp_x\tcovp_y\tcovp_z\tvel_x\tvel_y\tvel_z\tang_vel_x\tang_vel_y\tang_vel_z\tcovv_x\tcovv_y\tcovv_z',file=self.odomfile)
            elif 'CompressedImage' in types:
                print('Compressed Image output dir setting')
                self.compressedimagedir = Path('/').joinpath(self.out_dir, 'Image')
                if not Path(self.compressedimagedir).exists():
                    Path.mkdir(self.compressedimagedir)
                print('Image Times output file setiing')
                self.timesfile = open(Path('/').joinpath(self.out_dir, 'times.txt'), 'w+')
            elif 'Image' in types:
                print('Image output dir setting')
                self.imagedir = Path('/').joinpath(self.out_dir, 'Image')
                if not Path(self.imagedir).exists():
                    Path.mkdir(self.imagedir)    
                print('Image Times output file setiing')
                self.timesfile = open(Path('/').joinpath(self.out_dir, 'times.txt'), 'w+')
            
            elif 'PointCloud2' in types:
                print('PC out dir setting')
                self.pcdir = Path('/').joinpath(self.out_dir, 'Pointcloud')
                if not Path(self.pcdir).exists():
                    Path.mkdir(self.pcdir)
            
            elif 'NavSatFix' in types:
                print('NavSatFix output file setting')
                self.navsatfile = open(Path('/').joinpath(self.out_dir, 'NavSatFix.txt'), 'w+')
                print(f'time(s)\tlatitude\tlongitude\taltitude\tcovp_x\tcovp_y\tcovp_z',file=self.navsatfile)
            elif 'TwistWithCovarianceStamped' in types:
                print('TwistWithCovarianceStamped output file setting')
                self.twistfile = open(Path('/').joinpath(self.out_dir, 'TwistWithCovarianceStamped.txt'), 'w+')
                print(f'time(s)\tvel_x\tvel_y\tvel_z\tcovv_x\tcovv_y\tcovv_z',file=self.twistfile)
            else:
                print(f'does not support this type, please report in issue\nthe target type: {types}\n')
                target_topics.remove(topic)
            return target_topics
        
    def data_output(self, data: list):
        data = self.TimeConvert(data)
        if (data[-1] == 'compressedimage') or (data[-1] == 'image'):
            timestamp_ns = data[1]
            timestamp_s = data[0]
            image_name =  str(timestamp_ns) +  ".png"
            cv2.imwrite(str(Path('/').joinpath(self.imagedir,image_name)), data[2])
            print(f'{timestamp_ns}\t{timestamp_s}',file=self.timesfile)
        elif data[-1] == 'imu':
            print(f'{data[0]}\t{data[2][0]}\t{data[2][1]}\t{data[2][2]}\t{data[3][0]}\t{data[3][1]}\t{data[3][2]}',file=self.imufile)
        elif data[-1] == 'odom':
            print(f'{data[0]}\t{data[1][0]}\t{data[1][1]}\t{data[1][2]}\t{data[2][0]}\t{data[2][1]}\t{data[2][2]}\t{data[2][3]}\t{data[3][0]}\t{data[3][4]}\t{data[3][8]}\t{data[4][0]}\t{data[4][1]}\t{data[4][2]}\t{data[5][0]}\t{data[5][1]}\t{data[5][2]}\t{data[6][0]}\t{data[6][4]}\t{data[6][8]}',file=self.odomfile)
        elif data[-1] == 'pcd':
            data[2].save(str(Path('/').joinpath(self.pcdir,data[1])))
        elif data[-1] == 'livox':
            result = o3d.io.write_point_cloud(str(Path('/').joinpath(self.pcdir,data[1])), data[2], print_progress=True)
        elif data[-1] == 'navsatfix':
            print(f'{data[0]}\t{data[1]}\t{data[2]}\t{data[3]}\t{data[4][0]}\t{data[4][1]}\t{data[4][2]}',file=self.navsatfile)
        elif data[-1] == 'twistwithcovariance':
            print(f'{data[0]}\t{data[1][0]}\t{data[1][1]}\t{data[1][2]}\t{data[2][0]}\t{data[2][7]}\t{data[2][14]}',file=self.twistfile)


    def TimeConvert(self, data):
        t = data[0]
        if self.timeconvert == 1:
            data[0] = gpst2sow(t)
        elif self.timeconvert == 2:
            sec = int(t)
            nsec = t - sec
            data[0] = unix2sow(sec, nsec)

        return data

    def close_file(self):
        if hasattr(self, "imufile"):
            if not self.imufile.closed:
                self.imufile.close()
        if hasattr(self, "odomfile"):
            if not self.odomfile.closed:
                self.odomfile.close()

    def __del__(self) -> None:
        if self.is_open:
            self.bag.close()

if __name__ == '__main__':
    bag = BagDataIO()
    
    