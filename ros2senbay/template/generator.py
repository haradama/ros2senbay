from prestring.python import PythonModule


def genrate(*, m=None, indent='    '):
    m = m or PythonModule(indent=indent)

    m.import_('os')
    m.import_('sys')
    m.import_('cv2')
    m.import_('zbar')
    m.import_('zbar.misc')
    m.import_('rclpy')
    m.from_('rclpy.node', 'Node')
    m.from_('std_msgs.msg', 'Float32')
    m.import_('rosenbay')
    
    with m.class_('Ros2senbayPublisher', 'Node'):
        with m.def_('__init__', 'self'):
            m.stmt("super().__init__('rosenbay_publisher')")

            m.stmt('self.pub_lst = {')
            m.stmt('"TIME":                     self.create_publisher(Float32, "TIME", 10),')
            m.stmt('"RPM":                      self.create_publisher(Float32, "RPM", 10),')
            m.stmt('"SPEED":                    self.create_publisher(Float32, "SPEED", 10),')
            m.stmt('"COOLANT_TEMP":             self.create_publisher(Float32, "COOLANT_TEMP", 10),')
            m.stmt('"DISTANCE_SINCE_DTC_CLEAR": self.create_publisher(Float32, "DISTANCE_SINCE_DTC_CLEAR", 10),')
            m.stmt('"MAF":                      self.create_publisher(Float32, "MAF", 10),')
            m.stmt('"INTAKE_TEMP":              self.create_publisher(Float32, "INTAKE_TEMP", 10)')
            m.stmt('}')

            m.stmt('self.title = "ros2senbay"')

            m.stmt('self.fps = 10')

            m.stmt('self.timer = self.create_timer(1 / self.fps, self.timer_callback)')
            m.stmt('infile = "{}"'.format("resource/video.m4v"))
            m.stmt('filepath = os.path.join(os.path.dirname(os.path.abspath(__file__)), infile')
            m.stmt('self.cap = cv2.VideoCapture(filepath)')
            m.stmt('self.scanner = zbar.Scanner()')
            m.stmt('self.senbayData = rosenbay.SenbayData()')

        with m.def_('timer_callback', 'self'):
            m.stmt('success, frame = self.cap.read()')

            with m.if_('success'):
                m.stmt('gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)')
                m.stmt('codes = self.scanner.scan(gray)')

                with m.if_('codes != None and len(codes) > 0'):
                    m.stmt('senbayDict = self.senbayData.decode(str(codes[0].data.decode("utf-8")))')

                    with m.for_('key, pub in self.pub_lst.items()'):
                        m.stmt('msg = Float32()')
                        m.stmt('msg.data = senbayDict[key]')
                        m.stmt('self.pub_lst[key].publish(msg)')
                        m.stmt('self.get_logger().info("{0}: \'{1}\'".format(key, msg.data))')
                m.stmt('cv2.imshow(self.title, frame)')

                with m.if_("cv2.waitKey(1) & 0xFF == ord('q')"):
                    m.stmt('cv2.destroyWindow(self.title)')
                    m.stmt('sys.exit()')

            with m.else_():
                m.stmt('self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)')


    with m.def_('main', 'args=None'):
        m.stmt('rclpy.init(args=args)')
        m.stmt('rosenbay_publisher = RosenbayPublisher()')
        m.stmt('rclpy.spin(rosenbay_publisher)')
        m.stmt('minimal_publisher.destroy_node()')
        m.stmt('rclpy.shutdown()')

    with m.if_("__name__ == '__main__'"):
        m.stmt('main()')
    return m


if __name__ == "__main__":
    m = generate(indent='    ')
    print(m)
