from launch.exit_handler import primary_exit_handler
from ros2run.api import get_executable_path
from ros2pkg.api import get_prefix_path
import os.path

opendbc_path = os.path.join(get_prefix_path('external'), 'external', 'opendbc')
def launch(launch_descriptor, argv):
    ld = launch_descriptor

    ld.add_process(
        cmd=[get_executable_path(package_name='canoc', executable_name='transceiver')],
    )

    ld.add_process(
        cmd=[get_executable_path(package_name='radar', executable_name='radar_controller'), opendbc_path]
    )

    # ld.add_process(
    #     cmd=[get_executable_path(
    #         package_name=package, executable_name='lifecycle_service_client')],
    #     exit_handler=primary_exit_handler,
    #)