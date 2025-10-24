import os

from ament_index_python.packages import get_package_share_directory


def replace_old_path_urdf(urdf_file_path: str, old_path: str, new_path: str):
    file_str = None
    with open(urdf_file_path, "r") as file_handle:
        file_str = file_handle.read()

    with open(urdf_file_path, "w") as file_handle:

        new_file_str = file_str.replace(old_path, new_path)

        file_handle.write(new_file_str)


def get_urdf_path():
    package_share_directory = get_package_share_directory("ros_sim")
    path_to_robo_folder = os.path.join(
        package_share_directory, "resource", "robots", "mycobot_280_pi"
    )
    path_to_mesh = os.path.join(path_to_robo_folder, "meshes/")
    path_to_urdf = os.path.join(path_to_robo_folder, "mycobot_280_pi_mod.urdf")

    old_file = ""
    with open(path_to_urdf, "r") as file_handle:
        old_file = file_handle.read()

    path_to_urdf = path_to_urdf + "_new.urdf"
    with open(path_to_urdf, "w") as file_handle:
        file_handle.write(old_file)

    replace_old_path_urdf(
        path_to_urdf,
        "package://ros_sim/",
        package_share_directory + "/",
    )

    return path_to_urdf
