import rospy

class NavigationOdometryDataHandler():
    def __init__(self):
        pass

    def processData(self, data):
        self._seq = data.header.seq
        self._stamp = data.header.stamp
        self._frame_id = data.child_frame_id
        self._child_frame_id = data.child_frame_id
        self._pos_x = data.pose.pose.position.x
        self._pos_y = data.pose.pose.position.y
        self._pos_z = data.pose.pose.position.z
        self._ori_x = data.pose.pose.orientation.x
        self._ori_y = data.pose.pose.orientation.y
        self._ori_z = data.pose.pose.orientation.z
        self._ori_w = data.pose.pose.orientation.w
        self._pos_cov = data.pose.covariance
        self._lin_x = data.twist.twist.linear.x 
        self._lin_y = data.twist.twist.linear.y 
        self._lin_z = data.twist.twist.linear.z 
        self._ang_x = data.twist.twist.angular.x
        self._ang_y = data.twist.twist.angular.y
        self._ang_z = data.twist.twist.angular.z
        self._twist_cov = data.twist.covariance
        
        rospy.loginfo("{%s} DATA: %d %s %f %f %f %f",  
            self.__class__.__name__,
            self._seq,
            self._frame_id,
            self._pos_x,
            self._ori_w,
            self._lin_x,
            self._ang_z
            )

        # seq
        @property
        def seq(self):
            return self._seq

        # frame_id
        @property
        def frame_id(self):
            return self._frame_id

        # pos_x
        @property
        def pos_x(self):
            return self._pos_x

        # ori_w
        @property
        def ori_w(self):
            return self._ori_w

        # lin_x
        @property
        def lin_x(self):
            return self._lin_x

        # ang_z
        @property
        def ang_z(self):
            return self._ang_z
        