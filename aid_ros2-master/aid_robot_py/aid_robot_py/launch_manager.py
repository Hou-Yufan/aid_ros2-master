import os
import signal
import subprocess
import time
import threading
import select  # 添加 select 模块

import rclpy
from rclpy.node import Node
from aid_robot_msgs.srv import ControlLaunch
from aid_robot_msgs.srv import QueryLaunchStatus


class LaunchManagerNode(Node):

    def __init__(self):
        super().__init__('aid_launch_manager')

        self.declare_parameter('launch_files')
        self.launch_files = self.get_parameter(
            'launch_files').get_parameter_value().string_array_value

        self._processes = []

        self.create_service(ControlLaunch, 'start_launch',
                            self.start_launch_callback)
        self.create_service(ControlLaunch, 'stop_launch',
                            self.stop_launch_callback)
        self.create_service(QueryLaunchStatus, 'query_launch_status',
                            self.query_launch_status_callback)

        for launch_file in self.launch_files:
            process = subprocess.Popen(
                ['ros2', 'launch', launch_file],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                start_new_session=True,
                bufsize=1,
                universal_newlines=True  # 添加参数，以便实时输出 log
            )
            self._processes.append((launch_file, process))

        # Create timer to check process status periodically
        self.create_timer(0.01, self._check_process_status)

    def _check_process_status(self):
        for launch_file, process in self._processes:
            if process.poll() is not None:
                self._processes.remove((launch_file, process))
            else:
                ready, _, _ = select.select([process.stdout, process.stderr],
                                            [], [], 0.1)
                for stream in ready:
                    line = stream.readline().rstrip()
                    if line:
                        self.get_logger().info(f'{launch_file}: {line}')

    def query_launch_status_callback(self, request, response):
        launch_file = request.launch_file
        found = False

        for lf, process in self._processes:
            if launch_file == lf:
                if process.poll() is None:
                    # the process is still running
                    response.message = 'running'
                else:
                    # the process has exited
                    response.message = 'exited'
                found = True
                break

        if not found:
            response.message = 'not found'
        response.success = True
        return response

    def start_launch_callback(self, request, response):
        launch_file = request.launch_file
        run_param = request.parameter

        if any(launch_file == lf for lf, _ in self._processes):
            response.success = False
            response.message = f'{launch_file} is already running'
        else:
            cmd = ['ros2', 'launch', launch_file]
            if run_param:
                cmd.append(run_param)
            process = subprocess.Popen(
                cmd,
                shell=False,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                start_new_session=True,
                bufsize=1,
                universal_newlines=True  # 添加参数，以便实时输出 log
            )
            self._processes.append((launch_file, process))
            response.success = True
            response.message = f'Started {launch_file}'
        self.get_logger().info(response.message)
        return response

    def stop_launch_callback(self, request, response):
        launch_file = request.launch_file
        self.get_logger().info(f'stop file: {launch_file}')
        response.success = False
        response.message = f'{launch_file} is not running'
        for lf, process in self._processes:
            if launch_file == lf:
                if process.poll() is None:
                    # os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                    process.send_signal(signal.SIGINT)
                    try:
                        return_code = process.wait(5)
                        self.get_logger().info(f"send_signal return_code: {return_code}")
                    except subprocess.TimeoutExpired:
                        os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                        output, error = process.communicate()
                        self.get_logger().info(f"Process timed out. Output: {output}. Error: {error}")

                    self._processes.remove((launch_file, process))
                    response.success = True
                    response.message = f'Stopping {launch_file}'

                else:
                    self._processes.remove((launch_file, process))
                    response.success = True
                    response.message = f'{launch_file} has already stopped'
                break

        self.get_logger().info(f'message: {response.message}')
        return response

    def shutdown_callback(self):
        for lf, process in self._processes:
            #process.terminate()
            #os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            process.send_signal(signal.SIGINT)
            try:
                return_code = process.wait(5)
                self.get_logger().info(f"send_signal return_code: {return_code}")
            except subprocess.TimeoutExpired:
                process.kill()
                output, error = process.communicate()
                self.get_logger().info(f"Process timed out. Output: {output}. Error: {error}")

    def run(self):
        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            pass
        finally:
            self.shutdown_callback()
            rclpy.shutdown()


def main():
    rclpy.init()
    node = LaunchManagerNode()
    node.run()


if __name__ == '__main__':
    main()
