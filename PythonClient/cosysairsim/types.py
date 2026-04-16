from __future__ import annotations

import math
from typing import cast

import numpy as np


class MsgpackMixin:
    def __repr__(self):
        from pprint import pformat

        return "<" + type(self).__name__ + "> " + pformat(vars(self), indent=4, width=1)

    def to_msgpack(self, *args, **kwargs):
        return self.__dict__

    @classmethod
    def from_msgpack(cls, encoded):
        obj = cls()
        # ww = encoded.items()
        # obj.__dict__ = {k.decode('utf-8'): (from_msgpack(v.__class__, v) if hasattr(v, "__dict__") else v) for k, v in encoded.items()}
        if isinstance(encoded, dict):
            for k, v in encoded.items():
                if isinstance(v, dict) and hasattr(getattr(obj, k).__class__, "from_msgpack"):
                    obj.__dict__[k] = getattr(obj, k).__class__.from_msgpack(v)
                else:
                    obj.__dict__[k] = v
        else:
            obj = encoded

        # obj.__dict__ = { k : (v if not isinstance(v, dict) else getattr(getattr(obj, k).__class__, "from_msgpack")(v)) for k, v in encoded.items()}
        # return cls(**msgpack.unpack(encoded))
        return obj


class _ImageType(type):
    @property
    def Scene(cls):
        return 0

    def DepthPlanar(cls):
        return 1

    def DepthPerspective(cls):
        return 2

    def DepthVis(cls):
        return 3

    def DisparityNormalized(cls):
        return 4

    def Segmentation(cls):
        return 5

    def SurfaceNormals(cls):
        return 6

    def Infrared(cls):
        return 7

    def OpticalFlow(cls):
        return 8

    def OpticalFlowVis(cls):
        return 9

    def Lighting(cls):
        return 10

    def Annotation(cls):
        return 11

    def __getattr__(self, key):
        if key == "DepthPlanar":
            print(
                "\033[31m"
                + "DepthPlanar has been (correctly) renamed to DepthPlanar. Please use ImageType.DepthPlanar instead."
                + "\033[0m"
            )
            raise AttributeError


class ImageType(metaclass=_ImageType):
    Scene = 0
    DepthPlanar = 1
    DepthPerspective = 2
    DepthVis = 3
    DisparityNormalized = 4
    Segmentation = 5
    SurfaceNormals = 6
    Infrared = 7
    OpticalFlow = 8
    OpticalFlowVis = 9
    Lighting = 10
    Annotation = 11


class DrivetrainType:
    MaxDegreeOfFreedom = 0
    ForwardOnly = 1


class LandedState:
    Landed = 0
    Flying = 1


class WeatherParameter:
    Rain = 0
    Roadwetness = 1
    Snow = 2
    RoadSnow = 3
    MapleLeaf = 4
    RoadLeaf = 5
    Dust = 6
    Fog = 7
    Enabled = 8


class Vector2r(MsgpackMixin):
    x_val: float = 0.0
    y_val: float = 0.0

    def __init__(self, x_val: float = 0.0, y_val: float = 0.0):
        self.x_val = x_val
        self.y_val = y_val


class Vector3r(MsgpackMixin):
    def __init__(self, x_val: float = 0.0, y_val: float = 0.0, z_val: float = 0.0):
        self.x_val: float = x_val
        self.y_val: float = y_val
        self.z_val: float = z_val

    @staticmethod
    def nanVector3r() -> Vector3r:
        return Vector3r(float("nan"), float("nan"), float("nan"))

    def containsNan(self) -> bool:
        return math.isnan(self.x_val) or math.isnan(self.y_val) or math.isnan(self.z_val)

    def __add__(self, other: Vector3r) -> Vector3r:
        return Vector3r(
            self.x_val + other.x_val, self.y_val + other.y_val, self.z_val + other.z_val
        )

    def __sub__(self, other: Vector3r) -> Vector3r:
        return Vector3r(
            self.x_val - other.x_val, self.y_val - other.y_val, self.z_val - other.z_val
        )

    def __radd__(self, other: Vector3r) -> Vector3r:
        return self.__add__(other)

    def __rsub__(self, other: Vector3r) -> Vector3r:
        return Vector3r(
            other.x_val - self.x_val, other.y_val - self.y_val, other.z_val - self.z_val
        )

    def __truediv__(self, other: float | int | np.ndarray) -> Vector3r:
        if isinstance(other, (int, float, np.integer, np.floating)):
            return Vector3r(self.x_val / other, self.y_val / other, self.z_val / other)
        else:
            raise TypeError(f"unsupported operand type(s) for /: {type(self)} and {type(other)}")

    def __mul__(self, other: float | int | np.ndarray) -> Vector3r:
        if isinstance(other, (int, float, np.integer, np.floating)):
            return Vector3r(self.x_val * other, self.y_val * other, self.z_val * other)
        else:
            raise TypeError(f"unsupported operand type(s) for *: {type(self)} and {type(other)}")

    def __rmul__(self, other: float | int | np.ndarray) -> Vector3r:
        return self.__mul__(other)

    def dot(self, other: Vector3r) -> float:
        if isinstance(other, Vector3r):
            return self.x_val * other.x_val + self.y_val * other.y_val + self.z_val * other.z_val
        else:
            raise TypeError(
                f"unsupported operand type(s) for 'dot': {type(self)} and {type(other)}"
            )

    def cross(self, other: Vector3r) -> Vector3r:
        if isinstance(other, Vector3r):
            cross_product = np.cross(self.to_numpy_array(), other.to_numpy_array())
            return Vector3r(cross_product[0], cross_product[1], cross_product[2])
        else:
            raise TypeError(
                f"unsupported operand type(s) for 'cross': {type(self)} and {type(other)}"
            )

    def get_length(self) -> float:
        return math.sqrt(self.x_val**2 + self.y_val**2 + self.z_val**2)

    def distance_to(self, other: Vector3r) -> float:
        return math.sqrt(
            (self.x_val - other.x_val) ** 2
            + (self.y_val - other.y_val) ** 2
            + (self.z_val - other.z_val) ** 2
        )

    def to_Quaternionr(self) -> Quaternionr:
        return Quaternionr(self.x_val, self.y_val, self.z_val, 0)

    def to_numpy_array(self) -> np.ndarray:
        return np.array([self.x_val, self.y_val, self.z_val], dtype=np.float32)

    def __iter__(self):
        return iter((self.x_val, self.y_val, self.z_val))


class Quaternionr(MsgpackMixin):
    def __init__(
        self, x_val: float = 0.0, y_val: float = 0.0, z_val: float = 0.0, w_val: float = 1.0
    ):
        self.x_val: float = x_val
        self.y_val: float = y_val
        self.z_val: float = z_val
        self.w_val: float = w_val

    @staticmethod
    def nanQuaternionr() -> Quaternionr:
        return Quaternionr(float("nan"), float("nan"), float("nan"), float("nan"))

    def containsNan(self) -> bool:
        return (
            math.isnan(self.w_val)
            or math.isnan(self.x_val)
            or math.isnan(self.y_val)
            or math.isnan(self.z_val)
        )

    def __add__(self, other: Quaternionr) -> Quaternionr:
        if isinstance(other, Quaternionr):
            return Quaternionr(
                self.x_val + other.x_val,
                self.y_val + other.y_val,
                self.z_val + other.z_val,
                self.w_val + other.w_val,
            )
        else:
            raise TypeError(f"unsupported operand type(s) for +: {type(self)} and {type(other)}")

    def __mul__(self, other):
        if isinstance(other, Quaternionr):
            t, x, y, z = self.w_val, self.x_val, self.y_val, self.z_val
            a, b, c, d = other.w_val, other.x_val, other.y_val, other.z_val
            return Quaternionr(
                w_val=a * t - b * x - c * y - d * z,
                x_val=b * t + a * x + d * y - c * z,
                y_val=c * t + a * y + b * z - d * x,
                z_val=d * t + z * a + c * x - b * y,
            )
        else:
            raise TypeError(f"unsupported operand type(s) for *: {type(self)} and {type(other)}")

    def __truediv__(self, other: Quaternionr | float | int) -> Quaternionr:
        if isinstance(other, Quaternionr):
            result = self * other.inverse()
            return cast(Quaternionr, result)
        elif isinstance(other, (int, float, np.integer, np.floating)):
            return Quaternionr(
                self.x_val / other, self.y_val / other, self.z_val / other, self.w_val / other
            )
        else:
            raise TypeError(f"unsupported operand type(s) for /: {type(self)} and {type(other)}")

    def dot(self, other: Quaternionr) -> float:
        if isinstance(other, Quaternionr):
            return (
                self.x_val * other.x_val
                + self.y_val * other.y_val
                + self.z_val * other.z_val
                + self.w_val * other.w_val
            )
        else:
            raise TypeError(
                f"unsupported operand type(s) for 'dot': {type(self)} and {type(other)}"
            )

    def cross(self, other: Quaternionr) -> Quaternionr:
        if isinstance(other, Quaternionr):
            diff = self * other - other * self
            result = Quaternionr(diff.x_val / 2, diff.y_val / 2, diff.z_val / 2, diff.w_val / 2)
            return result
        else:
            raise TypeError(
                f"unsupported operand type(s) for 'cross': {type(self)} and {type(other)}"
            )

    def outer_product(self, other: Quaternionr) -> Quaternionr:
        if isinstance(other, Quaternionr):
            lhs = self.inverse() * other
            rhs = other.inverse() * self
            diff = lhs - rhs
            result = Quaternionr(diff.x_val / 2, diff.y_val / 2, diff.z_val / 2, diff.w_val / 2)
            return result
        else:
            raise TypeError(
                f"unsupported operand type(s) for 'outer_product': {type(self)} and {type(other)}"
            )

    def rotate(self, other: Quaternionr) -> Quaternionr:
        if isinstance(other, Quaternionr):
            if other.get_length() == 1:
                result = other * self * other.inverse()
                return cast(Quaternionr, result)
            else:
                raise ValueError("length of the other Quaternionr must be 1")
        else:
            raise TypeError(
                f"unsupported operand type(s) for 'rotate': {type(self)} and {type(other)}"
            )

    def conjugate(self):
        return Quaternionr(-self.x_val, -self.y_val, -self.z_val, self.w_val)

    def star(self):
        return self.conjugate()

    def inverse(self) -> Quaternionr:
        return self.from_numpy_array(self.star().to_numpy_array() / self.dot(self))

    def sgn(self) -> Quaternionr:
        return self / self.get_length()

    def get_length(self) -> float:
        return math.sqrt(self.x_val**2 + self.y_val**2 + self.z_val**2 + self.w_val**2)

    def from_numpy_array(self, array: np.ndarray) -> Quaternionr:
        return Quaternionr(array[0], array[1], array[2], array[3])

    def to_numpy_array(self) -> np.ndarray:
        return np.array([self.x_val, self.y_val, self.z_val, self.w_val], dtype=np.float32)

    def __iter__(self):
        return iter((self.x_val, self.y_val, self.z_val, self.w_val))


class Pose(MsgpackMixin):
    position: Vector3r = Vector3r()
    orientation: Quaternionr = Quaternionr()

    def __init__(
        self, position_val: Vector3r | None = None, orientation_val: Quaternionr | None = None
    ):
        position_val = position_val if position_val is not None else Vector3r()
        orientation_val = orientation_val if orientation_val is not None else Quaternionr()
        self.position = position_val
        self.orientation = orientation_val

    @staticmethod
    def nanPose() -> Pose:
        return Pose(Vector3r.nanVector3r(), Quaternionr.nanQuaternionr())

    def containsNan(self) -> bool:
        return self.position.containsNan() or self.orientation.containsNan()

    def __iter__(self):
        return iter((self.position, self.orientation))


class Twist(MsgpackMixin):
    def __init__(self, linear_val: Vector3r | None = None, angular_val: Vector3r | None = None):
        self.linear: Vector3r = linear_val if linear_val is not None else Vector3r()
        self.angular: Vector3r = angular_val if angular_val is not None else Vector3r()


class CollisionInfo(MsgpackMixin):
    has_collided = False
    normal = Vector3r()
    impact_point = Vector3r()
    position = Vector3r()
    penetration_depth = 0.0
    time_stamp = 0.0
    object_name = ""
    object_id = -1


class GeoPoint(MsgpackMixin):
    latitude = 0.0
    longitude = 0.0
    altitude = 0.0


class YawMode(MsgpackMixin):
    is_rate = True
    yaw_or_rate = 0.0

    def __init__(self, is_rate=True, yaw_or_rate=0.0):
        self.is_rate = is_rate
        self.yaw_or_rate = yaw_or_rate


class RCData(MsgpackMixin):
    timestamp = 0
    pitch, roll, throttle, yaw = (0.0,) * 4  # init 4 variable to 0.0
    switch1, switch2, switch3, switch4 = (0,) * 4
    switch5, switch6, switch7, switch8 = (0,) * 4
    is_initialized = False
    is_valid = False

    def __init__(
        self,
        timestamp=0,
        pitch=0.0,
        roll=0.0,
        throttle=0.0,
        yaw=0.0,
        switch1=0,
        switch2=0,
        switch3=0,
        switch4=0,
        switch5=0,
        switch6=0,
        switch7=0,
        switch8=0,
        is_initialized=False,
        is_valid=False,
    ):
        self.timestamp = timestamp
        self.pitch = pitch
        self.roll = roll
        self.throttle = throttle
        self.yaw = yaw
        self.switch1 = switch1
        self.switch2 = switch2
        self.switch3 = switch3
        self.switch4 = switch4
        self.switch5 = switch5
        self.switch6 = switch6
        self.switch7 = switch7
        self.switch8 = switch8
        self.is_initialized = is_initialized
        self.is_valid = is_valid


class ImageRequest(MsgpackMixin):
    camera_name: str = "0"
    image_type: int = ImageType.Scene
    pixels_as_float: bool = False
    compress: bool = False
    annotation_name: str = ""

    def __init__(
        self,
        camera_name: str,
        image_type: int,
        pixels_as_float: bool = False,
        compress: bool = True,
        annotation_name: str = "",
    ):
        self.camera_name = str(camera_name)
        self.image_type = image_type
        self.pixels_as_float = pixels_as_float
        self.compress = compress
        self.annotation_name = annotation_name


class ImageResponse(MsgpackMixin):
    image_data_uint8 = np.uint8(0)
    image_data_float = 0.0
    camera_position = Vector3r()
    camera_orientation = Quaternionr()
    time_stamp = np.uint64(0)
    message = ""
    pixels_as_float = 0.0
    compress = True
    width = 0
    height = 0
    image_type = ImageType.Scene
    annotation_name = ""


class CarControls(MsgpackMixin):
    throttle = 0.0
    steering = 0.0
    brake = 0.0
    handbrake = False
    is_manual_gear = False
    manual_gear = 0
    gear_immediate = True

    def __init__(
        self,
        throttle=0,
        steering=0,
        brake=0,
        handbrake=False,
        is_manual_gear=False,
        manual_gear=0,
        gear_immediate=True,
    ):
        self.throttle = throttle
        self.steering = steering
        self.brake = brake
        self.handbrake = handbrake
        self.is_manual_gear = is_manual_gear
        self.manual_gear = manual_gear
        self.gear_immediate = gear_immediate

    def set_throttle(self, throttle_val, forward):
        if forward:
            self.is_manual_gear = False
            self.manual_gear = 0
            self.throttle = abs(throttle_val)
        else:
            self.is_manual_gear = False
            self.manual_gear = -1
            self.throttle = -abs(throttle_val)


class KinematicsState(MsgpackMixin):
    position = Vector3r()
    orientation = Quaternionr()
    linear_velocity = Vector3r()
    angular_velocity = Vector3r()
    linear_acceleration = Vector3r()
    angular_acceleration = Vector3r()


class EnvironmentState(MsgpackMixin):
    position = Vector3r()
    geo_point = GeoPoint()
    gravity = Vector3r()
    air_pressure = 0.0
    temperature = 0.0
    air_density = 0.0


class ComputerVisionState(MsgpackMixin):
    kinematics_estimated = KinematicsState()
    timestamp = np.uint64(0)


class CarState(MsgpackMixin):
    speed = 0.0
    gear = 0
    rpm = 0.0
    maxrpm = 0.0
    handbrake = False
    collision = CollisionInfo()
    kinematics_estimated = KinematicsState()
    timestamp = np.uint64(0)


class MultirotorState(MsgpackMixin):
    collision = CollisionInfo()
    kinematics_estimated = KinematicsState()
    gps_location = GeoPoint()
    timestamp = np.uint64(0)
    landed_state = LandedState.Landed
    rc_data = RCData()
    ready = False
    ready_message = ""
    can_arm = False


class RotorStates(MsgpackMixin):
    timestamp: np.uint64 = np.uint64(0)
    rotors: list[float] = []


class ProjectionMatrix(MsgpackMixin):
    matrix: list[list[float]] = []


class CameraInfo(MsgpackMixin):
    pose = Pose()
    fov = -1
    proj_mat = ProjectionMatrix()


class LidarData(MsgpackMixin):
    point_cloud = 0.0
    time_stamp = np.uint64(0)
    pose = Pose()
    groundtruth = ""


class GPULidarData(MsgpackMixin):
    point_cloud = 0.0
    time_stamp = np.uint64(0)
    pose = Pose()


class EchoData(MsgpackMixin):
    point_cloud = 0.0
    time_stamp = np.uint64(0)
    pose = Pose()
    groundtruth = ""
    passive_beacons_point_cloud = 0.0
    passive_beacons_groundtruth = ""


class UwbSensorData(MsgpackMixin):
    time_stamp: np.uint64 = np.uint64(0)
    pose: Pose = Pose()
    beaconsActiveID: list[int] = []
    beaconsActiveRssi: list[float] = []
    beaconsActivePosX: list[float] = []
    beaconsActivePosY: list[float] = []
    beaconsActivePosZ: list[float] = []


class UwbData(MsgpackMixin):
    time_stamp: list[float] = []
    mur_achorId: list[int] = []
    mur_anchorX: list[float] = []
    mur_anchorY: list[float] = []
    mur_anchorZ: list[float] = []
    mur_anchor_valid_range: list[float] = []
    mur_anchor_distance: list[float] = []
    mur_anchor_rssi: list[float] = []
    mura_tagId: list[int] = []
    mura_tagX: list[float] = []
    mura_tagY: list[float] = []
    mura_tagZ: list[float] = []
    mura_ranges: list[float] = []


class WifiSensorData(MsgpackMixin):
    time_stamp: np.uint64 = np.uint64(0)
    pose: Pose = Pose()
    beaconsActiveID: list[int] = []
    beaconsActiveRssi: list[float] = []
    beaconsActivePosX: list[float] = []
    beaconsActivePosY: list[float] = []
    beaconsActivePosZ: list[float] = []


class WifiData(MsgpackMixin):
    time_stamp: list[float] = []
    wr_achorId: list[int] = []
    wr_anchorX: list[float] = []
    wr_anchorY: list[float] = []
    wr_anchorZ: list[float] = []
    wr_anchor_valid_range: list[float] = []
    wr_anchor_distance: list[float] = []
    wr_anchor_rssi: list[float] = []
    wra_tagId: list[int] = []
    wra_tagX: list[float] = []
    wra_tagY: list[float] = []
    wra_tagZ: list[float] = []
    wra_ranges: list[float] = []


class ImuData(MsgpackMixin):
    time_stamp = np.uint64(0)
    orientation = Quaternionr()
    angular_velocity = Vector3r()
    linear_acceleration = Vector3r()


class BarometerData(MsgpackMixin):
    time_stamp = np.uint64(0)
    altitude = Quaternionr()
    pressure = Vector3r()
    qnh = Vector3r()


class MagnetometerData(MsgpackMixin):
    time_stamp = np.uint64(0)
    magnetic_field_body = Vector3r()
    magnetic_field_covariance = 0.0


class GnssFixType(MsgpackMixin):
    GNSS_FIX_NO_FIX = 0
    GNSS_FIX_TIME_ONLY = 1
    GNSS_FIX_2D_FIX = 2
    GNSS_FIX_3D_FIX = 3


class GnssReport(MsgpackMixin):
    geo_point = GeoPoint()
    eph = 0.0
    epv = 0.0
    velocity = Vector3r()
    fix_type = GnssFixType()
    time_utc = np.uint64(0)


class GpsData(MsgpackMixin):
    time_stamp = np.uint64(0)
    gnss = GnssReport()
    is_valid = False


class DistanceSensorData(MsgpackMixin):
    time_stamp = np.uint64(0)
    distance = 0.0
    min_distance = 0.0
    max_distance = 0.0
    relative_pose = Pose()


class Box2D(MsgpackMixin):
    min = Vector2r()
    max = Vector2r()


class Box3D(MsgpackMixin):
    min = Vector3r()
    max = Vector3r()


class DetectionInfo(MsgpackMixin):
    name = ""
    geo_point = GeoPoint()
    box2D = Box2D()
    box3D = Box3D()
    relative_pose = Pose()


class PIDGains:
    """
    Struct to store values of PID gains. Used to transmit controller gain values while instantiating
    AngleLevel/AngleRate/Velocity/PositionControllerGains objects.

    Attributes:
        kP (float): Proportional gain
        kI (float): Integrator gain
        kD (float): Derivative gain
    """

    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def to_list(self):
        return [self.kp, self.ki, self.kd]


class AngleRateControllerGains:
    """
    Struct to contain controller gains used by angle level PID controller

    Attributes:
        roll_gains (PIDGains): kP, kI, kD for roll axis
        pitch_gains (PIDGains): kP, kI, kD for pitch axis
        yaw_gains (PIDGains): kP, kI, kD for yaw axis
    """

    def __init__(
        self,
        roll_gains=None,
        pitch_gains=None,
        yaw_gains=None,
    ):
        self.roll_gains = roll_gains if roll_gains is not None else PIDGains(0.25, 0, 0)
        self.pitch_gains = pitch_gains if pitch_gains is not None else PIDGains(0.25, 0, 0)
        self.yaw_gains = yaw_gains if yaw_gains is not None else PIDGains(0.25, 0, 0)

    def to_lists(self):
        return (
            [self.roll_gains.kp, self.pitch_gains.kp, self.yaw_gains.kp],
            [self.roll_gains.ki, self.pitch_gains.ki, self.yaw_gains.ki],
            [self.roll_gains.kd, self.pitch_gains.kd, self.yaw_gains.kd],
        )


class AngleLevelControllerGains:
    """
    Struct to contain controller gains used by angle rate PID controller

    Attributes:
        roll_gains (PIDGains): kP, kI, kD for roll axis
        pitch_gains (PIDGains): kP, kI, kD for pitch axis
        yaw_gains (PIDGains): kP, kI, kD for yaw axis
    """

    def __init__(
        self,
        roll_gains=None,
        pitch_gains=None,
        yaw_gains=None,
    ):
        self.roll_gains = roll_gains if roll_gains is not None else PIDGains(2.5, 0, 0)
        self.pitch_gains = pitch_gains if pitch_gains is not None else PIDGains(2.5, 0, 0)
        self.yaw_gains = yaw_gains if yaw_gains is not None else PIDGains(2.5, 0, 0)

    def to_lists(self):
        return (
            [self.roll_gains.kp, self.pitch_gains.kp, self.yaw_gains.kp],
            [self.roll_gains.ki, self.pitch_gains.ki, self.yaw_gains.ki],
            [self.roll_gains.kd, self.pitch_gains.kd, self.yaw_gains.kd],
        )


class VelocityControllerGains:
    """
    Struct to contain controller gains used by velocity PID controller

    Attributes:
        x_gains (PIDGains): kP, kI, kD for X axis
        y_gains (PIDGains): kP, kI, kD for Y axis
        z_gains (PIDGains): kP, kI, kD for Z axis
    """

    def __init__(
        self,
        x_gains=None,
        y_gains=None,
        z_gains=None,
    ):
        self.x_gains = x_gains if x_gains is not None else PIDGains(0.2, 0, 0)
        self.y_gains = y_gains if y_gains is not None else PIDGains(0.2, 0, 0)
        self.z_gains = z_gains if z_gains is not None else PIDGains(2.0, 2.0, 0)

    def to_lists(self):
        return (
            [self.x_gains.kp, self.y_gains.kp, self.z_gains.kp],
            [self.x_gains.ki, self.y_gains.ki, self.z_gains.ki],
            [self.x_gains.kd, self.y_gains.kd, self.z_gains.kd],
        )


class PositionControllerGains:
    """
    Struct to contain controller gains used by position PID controller

    Attributes:
        x_gains (PIDGains): kP, kI, kD for X axis
        y_gains (PIDGains): kP, kI, kD for Y axis
        z_gains (PIDGains): kP, kI, kD for Z axis
    """

    def __init__(
        self,
        x_gains=None,
        y_gains=None,
        z_gains=None,
    ):
        self.x_gains = x_gains if x_gains is not None else PIDGains(0.25, 0, 0)
        self.y_gains = y_gains if y_gains is not None else PIDGains(0.25, 0, 0)
        self.z_gains = z_gains if z_gains is not None else PIDGains(0.25, 0, 0)

    def to_lists(self):
        return (
            [self.x_gains.kp, self.y_gains.kp, self.z_gains.kp],
            [self.x_gains.ki, self.y_gains.ki, self.z_gains.ki],
            [self.x_gains.kd, self.y_gains.kd, self.z_gains.kd],
        )


class MeshPositionVertexBuffersResponse(MsgpackMixin):
    position = Vector3r()
    orientation = Quaternionr()
    vertices = 0.0
    indices = 0.0
    name = ""
