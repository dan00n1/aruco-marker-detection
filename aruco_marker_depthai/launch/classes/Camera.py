from enums import Positions, Calculations
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

class Camera:
    position = []

    def __init__(self, model, prefix, base_frame, parent_frame, spatial = False):
        self.model = model
        self.prefix = prefix
        self.base_frame = base_frame
        self.parent_frame = parent_frame
        self.spatial = spatial

    def set_position(self, position, value):
        self.positions[position] = value

    def set_angle(self, angle, value):
        self.positions[angle] = value

