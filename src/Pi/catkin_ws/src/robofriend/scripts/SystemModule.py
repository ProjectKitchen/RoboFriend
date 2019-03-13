import os, psutil, roslaunch, signal, subprocess, sys, time

# ZAHEDIM: where to call this def and make sure to terminate ROS
def shutdown():
    time.sleep(3)
    os.system('sudo init 0')

def kill_child_processes(parent_pid, sig=signal.SIGTERM):
    try:
        parent = psutil.Process(parent_pid)
    except psutil.NoSuchProcess:
        sys.stderr.write("Parent process not existing")
        return
    children = parent.children(recursive=True)
    for process in children:
        process.send_signal(sig)

class Roscore(object):
    """
    roscore wrapped into a subprocess.
    Singleton implementation prevents from creating more than one instance.
    """
    __initialized = False
    def __init__(self):
        if Roscore.__initialized:
            raise Exception("You can't create more than 1 instance of Roscore.")
        Roscore.__initialized = True
    def run(self):
        try:
            self.roscore_process = subprocess.Popen(['roscore'])
            self.roscore_pid = self.roscore_process.pid  # pid of the roscore process (which has child processes)
        except OSError as e:
            sys.stderr.write('Roscore could not be launched!')
            raise e
    def terminate(self):
        kill_child_processes(self.roscore_pid)
        self.roscore_process.terminate()
        self.roscore_process.wait()  # important to prevent from zombie process
        Roscore.__initialized = False

class RosRobo(object):

    __initialized = False
    def __init__(self):
        if RosRobo.__initialized:
            raise Exception("You can't create more than 1 instance of RosRobo.")
        RosRobo.__initialized = True
    def run(self):
        try:
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            launch = roslaunch.parent.ROSLaunchParent(
                uuid,
                ["catkin_ws/src/robofriend/launch/robofriend_startup.launch"]
                )
            launch.start()
        except OSError as e:
            sys.stderr.write('Rosrobo could not be launched!')
            raise e