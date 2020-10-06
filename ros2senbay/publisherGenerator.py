import os
import sys
import cv2
import zbar
import zbar.misc
import shutil
from tqdm import tqdm
from prestring.python import PythonModule
from ros2senbay.senbay.core import SenbayData
import argparse


class VideoReader:
    def __init__(self, infile):
        self.capture = cv2.VideoCapture(infile)
        self.scanner = zbar.Scanner()
        self.frame_num = int(self.capture.get(cv2.CAP_PROP_FRAME_COUNT))
        self.fps = self.capture.get(cv2.CAP_PROP_FPS)
        self.senbayData = SenbayData()
        self.senbayDict = {}

    def parse(self):
        for _ in tqdm(range(self.frame_num)):
            success, frame = self.capture.read()
            if success:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                codes = self.scanner.scan(gray)
                if codes != None and len(codes) > 0:
                    senbayDict = self.senbayData.decode(
                        str(codes[0].data.decode("utf-8")))
                    for key, value in senbayDict.items():
                        if key not in self.senbayDict:
                            if isinstance(value, int):
                                self.senbayDict[key] = "Int32"
                            elif isinstance(value, float):
                                self.senbayDict[key] = "Float32"
                            else:
                                self.senbayDict[key] = "String"


class Generator:
    def __init__(self, infile, outdir):
        self.infile = infile
        self.video_reader = VideoReader(infile)
        self.outdir = outdir

    def generate(self):
        self.video_reader.parse()
        senbayDict = self.video_reader.senbayDict

        m = PythonModule(indent='    ')

        m.import_('os')
        m.import_('sys')
        m.import_('cv2')
        m.import_('zbar')
        m.import_('zbar.misc')
        m.import_('rclpy')
        m.from_('rclpy.node', 'Node')
        for msgType in set(senbayDict.values()):
            m.from_('std_msgs.msg', msgType)
        m.from_('senbay.core', 'SenbayData')
        m.stmt('')

        with m.class_('Ros2senbayPublisher', 'Node'):
            with m.def_('__init__', 'self'):
                m.stmt("super().__init__('ros2senbay_publisher')")

                m.stmt('self.pub_lst = {')
                for key, value in senbayDict.items():
                    m.stmt(
                        '"{0}": self.create_publisher({0}, "{1}", 10),'.format(key, value))
                m.stmt('}')

                m.stmt('self.title = "Ros2senbay Publisher"')

                m.stmt('self.fps = {0}'.format(self.video_reader.fps))

                m.stmt(
                    'self.timer = self.create_timer(1 / self.fps, self.timer_callback)')
                m.stmt(
                    'infile = "media/{0}"'.format(os.path.basename(self.infile)))
                m.stmt(
                    'filepath = os.path.join(os.path.dirname(os.path.abspath(__file__)), infile')
                m.stmt('self.capture = cv2.VideoCapture(filepath)')
                m.stmt('self.scanner = zbar.Scanner()')
                m.stmt('self.senbayData = SenbayData()')

            with m.def_('timer_callback', 'self'):
                m.stmt('success, frame = self.capture.read()')

                with m.if_('success'):
                    m.stmt('gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)')
                    m.stmt('codes = self.scanner.scan(gray)')

                    with m.if_('codes != None and len(codes) > 0'):
                        m.stmt(
                            'senbayDict = self.senbayData.decode(str(codes[0].data.decode("utf-8")))')

                        with m.for_('key, pub in self.pub_lst.items()'):
                            m.stmt('msg = Float32()')
                            m.stmt('msg.data = senbayDict[key]')
                            m.stmt('self.pub_lst[key].publish(msg)')
                            m.stmt(
                                'self.get_logger().info("{0}: \'{1}\'".format(key, msg.data))')
                    m.stmt('cv2.imshow(self.title, frame)')

                    with m.if_("cv2.waitKey(1) & 0xFF == ord('q')"):
                        m.stmt('cv2.destroyWindow(self.title)')
                        m.stmt('sys.exit()')

                with m.else_():
                    m.stmt('self.capture.set(cv2.CAP_PROP_POS_FRAMES, 0)')

        with m.if_("__name__ == '__main__'"):
            m.stmt('rclpy.init()')
            m.stmt('ros2senbay_publisher = Ros2senbayPublisher()')
            m.stmt('rclpy.spin(ros2senbay_publisher)')
            m.stmt('ros2senbay_publisher.destroy_node()')
            m.stmt('rclpy.shutdown()')

        with open("{0}/publisher.py".format(self.outdir), "w") as fw:
            fw.write(m.__str__())


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--infile", help="Input file name")
    args = parser.parse_args()

    infile = args.infile
    workdir = "ros2senbay_publisher"
    current_dir = os.path.dirname(os.path.abspath(__file__))
    resource_path = "{0}/resource".format(current_dir)
    shutil.copytree(resource_path, workdir)
    shutil.copytree(current_dir + "/senbay", workdir + "/senbay")
    shutil.copyfile(
        infile, "{0}/media/{1}".format(workdir, os.path.basename(infile)))

    generator = Generator(infile, workdir)
    generator.generate()


if __name__ == "__main__":
    main()
