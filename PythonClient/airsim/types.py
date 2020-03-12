from __future__ import print_function
import msgpackrpc  # install as admin: pip install msgpack-rpc-python
import numpy as np  # pip install numpy


class MsgpackMixin:
    def __repr__(self):
        from pprint import pformat
        return "<" + type(self).__name__ + "> " + pformat(vars(self), indent=4, width=1)

    def to_msgpack(self, *args, **kwargs):
        return self.__dict__

    @classmethod
    def from_msgpack(cls, encoded):
        obj = cls()
        ww = encoded.items()
        # obj.__dict__ = {k.decode('utf-8'): (from_msgpack(v.__class__, v) if hasattr(v, "__dict__") else v) for k, v in encoded.items()}
        for k, v in encoded.items():
            if (isinstance(v, dict) and hasattr(getattr(obj, k).__class__, 'from_msgpack')):
                obj.__dict__[k] = getattr(getattr(obj, k).__class__, 'from_msgpack')(v)
            else:
                obj.__dict__[k] = v

        # obj.__dict__ = { k : (v if not isinstance(v, dict) else getattr(getattr(obj, k).__class__, "from_msgpack")(v)) for k, v in encoded.items()}
        # return cls(**msgpack.unpack(encoded))
        return obj


class pow22OneOver255Table:
    table = [ 0, 5.07705190066176E-06, 2.33280046660989E-05, 5.69217657121931E-05, 0.000107187362341244, 0.000175123977503027, 0.000261543754548491, 0.000367136269815943, 0.000492503787191433,
	0.000638182842167022, 0.000804658499513058, 0.000992374304074325, 0.0012017395224384, 0.00143313458967186, 0.00168691531678928, 0.00196341621339647, 0.00226295316070643,
	0.00258582559623417, 0.00293231832393836, 0.00330270303200364, 0.00369723957890013, 0.00411617709328275, 0.00455975492252602, 0.00502820345685554, 0.00552174485023966,
	0.00604059365484981, 0.00658495738258168, 0.00715503700457303, 0.00775102739766061, 0.00837311774514858, 0.00902149189801213, 0.00969632870165823, 0.0103978022925553,
	0.0111260823683832, 0.0118813344348137, 0.0126637200315821, 0.0134733969401426, 0.0143105193748841, 0.0151752381596252, 0.0160677008908869, 0.01698805208925, 0.0179364333399502,
	0.0189129834237215, 0.0199178384387857, 0.0209511319147811, 0.0220129949193365, 0.0231035561579214, 0.0242229420675342, 0.0253712769047346, 0.0265486828284729, 0.027755279978126,
	0.0289911865471078, 0.0302565188523887, 0.0315513914002264, 0.0328759169483838, 0.034230206565082, 0.0356143696849188, 0.0370285141619602, 0.0384727463201946, 0.0399471710015256,
	0.0414518916114625, 0.0429870101626571, 0.0445526273164214, 0.0461488424223509, 0.0477757535561706, 0.049433457555908, 0.0511220500564934, 0.052841625522879, 0.0545922772817603,
	0.0563740975519798, 0.0581871774736854, 0.0600316071363132, 0.0619074756054558, 0.0638148709486772, 0.0657538802603301, 0.0677245896854243, 0.0697270844425988, 0.0717614488462391,
	0.0738277663277846, 0.0759261194562648, 0.0780565899581019, 0.080219258736215, 0.0824142058884592, 0.0846415107254295, 0.0869012517876603, 0.0891935068622478, 0.0915183529989195,
	0.0938758665255778, 0.0962661230633397, 0.0986891975410945, 0.1011451642096, 0.103634096655137, 0.106156067812744, 0.108711149979039, 0.11129941482466, 0.113920933406333,
	0.116575776178572, 0.119264013005047, 0.121985713169619, 0.124740945387051, 0.127529777813422, 0.130352278056244, 0.1332085131843, 0.136098549737202, 0.139022453734703,
	0.141980290685736, 0.144972125597231, 0.147998022982685, 0.151058046870511, 0.154152260812165, 0.157280727890073, 0.160443510725344, 0.16364067148529, 0.166872271890766,
	0.170138373223312, 0.173439036332135, 0.176774321640903, 0.18014428915439, 0.183548998464951, 0.186988508758844, 0.190462878822409, 0.193972167048093, 0.19751643144034,
	0.201095729621346, 0.204710118836677, 0.208359655960767, 0.212044397502288, 0.215764399609395, 0.219519718074868, 0.223310408341127, 0.227136525505149, 0.230998124323267,
	0.23489525921588, 0.238827984272048, 0.242796353254002, 0.24680041960155, 0.2508402364364, 0.254915856566385, 0.259027332489606, 0.263174716398492, 0.267358060183772,
	0.271577415438375, 0.275832833461245, 0.280124365261085, 0.284452061560024, 0.288815972797219, 0.293216149132375, 0.297652640449211, 0.302125496358853, 0.306634766203158,
	0.311180499057984, 0.315762743736397, 0.32038154879181, 0.325036962521076, 0.329729032967515, 0.334457807923889, 0.339223334935327, 0.344025661302187, 0.348864834082879,
	0.353740900096629, 0.358653905926199, 0.363603897920553, 0.368590922197487, 0.373615024646202, 0.37867625092984, 0.383774646487975, 0.388910256539059, 0.394083126082829,
	0.399293299902674, 0.404540822567962, 0.409825738436323, 0.415148091655907, 0.420507926167587, 0.425905285707146, 0.43134021380741, 0.436812753800359, 0.442322948819202,
	0.44787084180041, 0.453456475485731, 0.45907989242416, 0.46474113497389, 0.470440245304218, 0.47617726539744, 0.481952237050698, 0.487765201877811, 0.493616201311074,
	0.49950527660303, 0.505432468828216, 0.511397818884879, 0.517401367496673, 0.523443155214325, 0.529523222417277, 0.535641609315311, 0.541798355950137, 0.547993502196972,
	0.554227087766085, 0.560499152204328, 0.566809734896638, 0.573158875067523, 0.579546611782525, 0.585972983949661, 0.592438030320847, 0.598941789493296, 0.605484299910907,
	0.612065599865624, 0.61868572749878, 0.625344720802427, 0.632042617620641, 0.638779455650817, 0.645555272444934, 0.652370105410821, 0.659223991813387, 0.666116968775851,
	0.673049073280942, 0.680020342172095, 0.687030812154625, 0.694080519796882, 0.701169501531402, 0.708297793656032, 0.715465432335048, 0.722672453600255, 0.729918893352071,
	0.737204787360605, 0.744530171266715, 0.751895080583051, 0.759299550695091, 0.766743616862161, 0.774227314218442, 0.781750677773962, 0.789313742415586, 0.796916542907978,
	0.804559113894567, 0.81224148989849, 0.819963705323528, 0.827725794455034, 0.835527791460841, 0.843369730392169, 0.851251645184515, 0.859173569658532, 0.867135537520905,
	0.875137582365205, 0.883179737672745, 0.891262036813419, 0.899384513046529, 0.907547199521614, 0.915750129279253, 0.923993335251873, 0.932276850264543, 0.940600707035753,
	0.948964938178195, 0.957369576199527, 0.96581465350313, 0.974300202388861, 0.982826255053791, 0.99139284359294, 1]


class ImageType:
    Scene = 0
    DepthPlanner = 1
    DepthPerspective = 2
    DepthVis = 3
    DisparityNormalized = 4
    Segmentation = 5
    SurfaceNormals = 6
    Infrared = 7


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


class Vector3rgb(MsgpackMixin):
    r_val = 0
    g_val = 0
    b_val = 0

    def __init__(self, r_val=0, g_val=0, b_val=0):
        self.r_val = r_val
        self.g_val = g_val
        self.b_val = b_val

    @staticmethod
    def nanVector3rgb():
        return Vector3rgb(np.nan, np.nan, np.nan)

    def __add__(self, other):
        return Vector3rgb(self.r_val + other.r_val, self.g_val + other.g_val, self.b_val + other.b_val)

    def __sub__(self, other):
        return Vector3rgb(self.r_val - other.r_val, self.g_val - other.g_val, self.b_val - other.b_val)

    def __truediv__(self, other):
        if type(other) in [int, float] + np.sctypes['int'] + np.sctypes['uint'] + np.sctypes['float']:
            return Vector3rgb(self.r_val / other, self.g_val / other, self.b_val / other)
        else:
            raise TypeError('unsupported operand type(s) for /: %s and %s' % (str(type(self)), str(type(other))))

    def __mul__(self, other):
        if type(other) in [int, float] + np.sctypes['int'] + np.sctypes['uint'] + np.sctypes['float']:
            return Vector3rgb(self.r_val * other, self.g_val * other, self.b_val * other)
        else:
            raise TypeError('unsupported operand type(s) for *: %s and %s' % (str(type(self)), str(type(other))))

    def dot(self, other):
        if type(self) == type(other):
            return self.r_val * other.r_val + self.g_val * other.g_val + self.b_val * other.b_val
        else:
            raise TypeError('unsupported operand type(s) for \'dot\': %s and %s' % (str(type(self)), str(type(other))))

    def cross(self, other):
        if type(self) == type(other):
            cross_product = np.cross(self.to_numpy_array(), other.to_numpy_array())
            return Vector3rgb(cross_product[0], cross_product[1], cross_product[2])
        else:
            raise TypeError(
                'unsupported operand type(s) for \'cross\': %s and %s' % (str(type(self)), str(type(other))))


    def to_numpy_array(self):
        return np.array([self.r_val, self.g_val, self.b_val], dtype=np.float32)


class Vector3r(MsgpackMixin):
    x_val = 0.0
    y_val = 0.0
    z_val = 0.0

    def __init__(self, x_val=0.0, y_val=0.0, z_val=0.0):
        self.x_val = x_val
        self.y_val = y_val
        self.z_val = z_val

    @staticmethod
    def nanVector3r():
        return Vector3r(np.nan, np.nan, np.nan)

    def __add__(self, other):
        return Vector3r(self.x_val + other.x_val, self.y_val + other.y_val, self.z_val + other.z_val)

    def __sub__(self, other):
        return Vector3r(self.x_val - other.x_val, self.y_val - other.y_val, self.z_val - other.z_val)

    def __truediv__(self, other):
        if type(other) in [int, float] + np.sctypes['int'] + np.sctypes['uint'] + np.sctypes['float']:
            return Vector3r(self.x_val / other, self.y_val / other, self.z_val / other)
        else:
            raise TypeError('unsupported operand type(s) for /: %s and %s' % (str(type(self)), str(type(other))))

    def __mul__(self, other):
        if type(other) in [int, float] + np.sctypes['int'] + np.sctypes['uint'] + np.sctypes['float']:
            return Vector3r(self.x_val * other, self.y_val * other, self.z_val * other)
        else:
            raise TypeError('unsupported operand type(s) for *: %s and %s' % (str(type(self)), str(type(other))))

    def dot(self, other):
        if type(self) == type(other):
            return self.x_val * other.x_val + self.y_val * other.y_val + self.z_val * other.z_val
        else:
            raise TypeError('unsupported operand type(s) for \'dot\': %s and %s' % (str(type(self)), str(type(other))))

    def cross(self, other):
        if type(self) == type(other):
            cross_product = np.cross(self.to_numpy_array(), other.to_numpy_array())
            return Vector3r(cross_product[0], cross_product[1], cross_product[2])
        else:
            raise TypeError(
                'unsupported operand type(s) for \'cross\': %s and %s' % (str(type(self)), str(type(other))))

    def get_length(self):
        return (self.x_val ** 2 + self.y_val ** 2 + self.z_val ** 2) ** 0.5

    def distance_to(self, other):
        return ((self.x_val - other.x_val) ** 2 + (self.y_val - other.y_val) ** 2 + (
                    self.z_val - other.z_val) ** 2) ** 0.5

    def to_Quaternionr(self):
        return Quaternionr(self.x_val, self.y_val, self.z_val, 0)

    def to_numpy_array(self):
        return np.array([self.x_val, self.y_val, self.z_val], dtype=np.float32)


class Quaternionr(MsgpackMixin):
    w_val = 0.0
    x_val = 0.0
    y_val = 0.0
    z_val = 0.0

    def __init__(self, x_val=0.0, y_val=0.0, z_val=0.0, w_val=1.0):
        self.x_val = x_val
        self.y_val = y_val
        self.z_val = z_val
        self.w_val = w_val

    @staticmethod
    def nanQuaternionr():
        return Quaternionr(np.nan, np.nan, np.nan, np.nan)

    def __add__(self, other):
        if type(self) == type(other):
            return Quaternionr(self.x_val + other.x_val, self.y_val + other.y_val, self.z_val + other.z_val,
                               self.w_val + other.w_val)
        else:
            raise TypeError('unsupported operand type(s) for +: %s and %s' % (str(type(self)), str(type(other))))

    def __mul__(self, other):
        if type(self) == type(other):
            t, x, y, z = self.w_val, self.x_val, self.y_val, self.z_val
            a, b, c, d = other.w_val, other.x_val, other.y_val, other.z_val
            return Quaternionr(w_val=a * t - b * x - c * y - d * z,
                               x_val=b * t + a * x + d * y - c * z,
                               y_val=c * t + a * y + b * z - d * x,
                               z_val=d * t + z * a + c * x - b * y)
        else:
            raise TypeError('unsupported operand type(s) for *: %s and %s' % (str(type(self)), str(type(other))))

    def __truediv__(self, other):
        if type(other) == type(self):
            return self * other.inverse()
        elif type(other) in [int, float] + np.sctypes['int'] + np.sctypes['uint'] + np.sctypes['float']:
            return Quaternionr(self.x_val / other, self.y_val / other, self.z_val / other, self.w_val / other)
        else:
            raise TypeError('unsupported operand type(s) for /: %s and %s' % (str(type(self)), str(type(other))))

    def dot(self, other):
        if type(self) == type(other):
            return self.x_val * other.x_val + self.y_val * other.y_val + self.z_val * other.z_val + self.w_val * other.w_val
        else:
            raise TypeError('unsupported operand type(s) for \'dot\': %s and %s' % (str(type(self)), str(type(other))))

    def cross(self, other):
        if type(self) == type(other):
            return (self * other - other * self) / 2
        else:
            raise TypeError(
                'unsupported operand type(s) for \'cross\': %s and %s' % (str(type(self)), str(type(other))))

    def outer_product(self, other):
        if type(self) == type(other):
            return (self.inverse() * other - other.inverse() * self) / 2
        else:
            raise TypeError(
                'unsupported operand type(s) for \'outer_product\': %s and %s' % (str(type(self)), str(type(other))))

    def rotate(self, other):
        if type(self) == type(other):
            if other.get_length() == 1:
                return other * self * other.inverse()
            else:
                raise ValueError('length of the other Quaternionr must be 1')
        else:
            raise TypeError(
                'unsupported operand type(s) for \'rotate\': %s and %s' % (str(type(self)), str(type(other))))

    def conjugate(self):
        return Quaternionr(-self.x_val, -self.y_val, -self.z_val, self.w_val)

    def star(self):
        return self.conjugate()

    def inverse(self):
        return self.from_numpy_array(self.star().to_numpy_array() / self.dot(self))

    def sgn(self):
        return self / self.get_length()

    def get_length(self):
        return (self.x_val ** 2 + self.y_val ** 2 + self.z_val ** 2 + self.w_val ** 2) ** 0.5

    def to_numpy_array(self):
        return np.array([self.x_val, self.y_val, self.z_val, self.w_val], dtype=np.float32)

    def from_numpy_array(self, array):
        return Quaternionr(array[0], array[1], array[2], array[3])


class Pose(MsgpackMixin):
    position = Vector3r()
    orientation = Quaternionr()

    def __init__(self, position_val=Vector3r(), orientation_val=Quaternionr()):
        self.position = position_val
        self.orientation = orientation_val

    @staticmethod
    def nanPose():
        return Pose(Vector3r.nanVector3r(), Quaternionr.nanQuaternionr())


class Twist(MsgpackMixin):
    linear = Vector3r()
    angular = Vector3r()

    def __init__(self, linear_val=Vector3r(), angular_val=Vector3r()):
        self.linear = linear_val
        self.angular = angular_val


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

    def __init__(self, timestamp=0, pitch=0.0, roll=0.0, throttle=0.0, yaw=0.0, switch1=0,
                 switch2=0, switch3=0, switch4=0, switch5=0, switch6=0, switch7=0, switch8=0, is_initialized=False,
                 is_valid=False):
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
    camera_name = '0'
    image_type = ImageType.Scene
    pixels_as_float = False
    compress = False

    def __init__(self, camera_name, image_type, pixels_as_float=False, compress=True):
        # todo: in future remove str(), it's only for compatibility to pre v1.2
        self.camera_name = str(camera_name)
        self.image_type = image_type
        self.pixels_as_float = pixels_as_float
        self.compress = compress


class ImageResponse(MsgpackMixin):
    image_data_uint8 = np.uint8(0)
    image_data_float = 0.0
    camera_position = Vector3r()
    camera_orientation = Quaternionr()
    time_stamp = np.uint64(0)
    message = ''
    pixels_as_float = 0.0
    compress = True
    width = 0
    height = 0
    image_type = ImageType.Scene


class CarControls(MsgpackMixin):
    throttle = 0.0
    steering = 0.0
    brake = 0.0
    handbrake = False
    is_manual_gear = False
    manual_gear = 0
    gear_immediate = True

    def __init__(self, throttle=0, steering=0, brake=0,
                 handbrake=False, is_manual_gear=False, manual_gear=0, gear_immediate=True):
        self.throttle = throttle
        self.steering = steering
        self.brake = brake
        self.handbrake = handbrake
        self.is_manual_gear = is_manual_gear
        self.manual_gear = manual_gear
        self.gear_immediate = gear_immediate

    def set_throttle(self, throttle_val, forward):
        if (forward):
            is_manual_gear = False
            manual_gear = 0
            throttle = abs(throttle_val)
        else:
            is_manual_gear = False
            manual_gear = -1
            throttle = - abs(throttle_val)


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


class CarState(MsgpackMixin):
    speed = 0.0
    gear = 0
    rpm = 0.0
    maxrpm = 0.0
    handbrake = False
    collision = CollisionInfo();
    kinematics_estimated = KinematicsState()
    timestamp = np.uint64(0)


class MultirotorState(MsgpackMixin):
    collision = CollisionInfo();
    kinematics_estimated = KinematicsState()
    gps_location = GeoPoint()
    timestamp = np.uint64(0)
    landed_state = LandedState.Landed
    rc_data = RCData()


class ProjectionMatrix(MsgpackMixin):
    matrix = []


class CameraInfo(MsgpackMixin):
    pose = Pose()
    fov = -1
    proj_mat = ProjectionMatrix()


class LidarData(MsgpackMixin):
    point_cloud = 0.0
    time_stamp = np.uint64(0)
    pose = Pose()


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
    geo_point = GeoPoint();
    eph = 0.0
    epv = 0.0;
    velocity = Vector3r();
    fix_type = GnssFixType();
    time_utc = np.uint64(0);


class GpsData(MsgpackMixin):
    time_stamp = np.uint64(0)
    gnss = GnssReport()
    is_valid = False


class AddAngularForce(MsgpackMixin):
    force_name = ''
    link_name = ''
    axis = Vector3r()

    def __init__(self, force_name_val='', link_name_val='', axis_val=Vector3r()):
        self.force_name = force_name_val
        self.link_name = link_name_val
        self.axis = axis_val


class AddLinearForce(MsgpackMixin):
    force_name = ''
    link_name = ''
    application_point = Vector3r()
    axis = Vector3r()

    def __init__(self, force_name_val='', link_name_val='', application_point_val=Vector3r(), axis_val=Vector3r()):
        self.force_name = force_name_val
        self.link_name = link_name_val
        self.application_point = application_point_val
        self.axis = axis_val


class UpdateForceMagnitude(MsgpackMixin):
    force_name = ''
    magnitude = 0.0

    def __init__(self, force_name_val='', magnitude_val=0.0):
        self.force_name = force_name_val
        self.magnitude = magnitude_val


class LinkInformation(MsgpackMixin):
    link_name = ''
    link_relative_pose = Pose()
    link_relative_twist = Twist()

    def __init__(self, link_name_val='', link_relative_pose_val=Pose(), link_relative_twist_val=Twist()):
        self.link_name = link_name_val
        self.link_relative_pose = link_relative_pose_val
        self.link_relative_twist = link_relative_twist_val


class UrdfBotState(MsgpackMixin):
    link_information = []
    kinematics_estimated = KinematicsState()
    controlled_motion_component_states = {}

    def __init__(self, link_information_val=[], kinematics_estimated_val=KinematicsState(),
                 controlled_motion_component_states={}):
        self.link_information = link_information_val
        self.kinematics_estimated = kinematics_estimated_val
        self.controlled_motion_component_states = controlled_motion_component_states


class UrdfBotControlledMotionComponentControlSignal(MsgpackMixin):
    component_name = ''
    control_signal_values = {}

    def __init__(self, component_name='', control_signal_values={}):
        self.component_name = component_name
        self.control_signal_values = control_signal_values


class CameraPose(MsgpackMixin):
    camera_name = ''
    translation = Vector3r()
    rotation = Quaternionr()

    def __init__(self, camera_name, translation, rotation):
        self.camera_name = camera_name
        self.translation = translation
        self.rotation = rotation


class RayCastRequest(MsgpackMixin):
    position = Vector3r()
    direction = Vector3r()
    reference_frame_link = ''
    through_blocking = False
    persist_seconds = 0

    def __init__(self, position, direction, reference_frame_link, through_blocking, persist_seconds):
        self.position = position
        self.direction = direction
        self.reference_frame_link = reference_frame_link
        self.through_blocking = through_blocking
        self.persist_seconds = persist_seconds


class RayCastHit:
    collided_actor_name = ''
    hit_point = Vector3r()
    hit_normal = Vector3r()

    def __init__(self, collided_actor_name='', hit_point=Vector3r(), hit_normal=Vector3r()):
        self.collided_actor_name = collided_actor_name
        self.hit_point = hit_point
        self.hit_normal = hit_normal


class RayCastResponse(MsgpackMixin):
    hits = []

    def __init__(self, hits=[]):
        self.hits = hits


class DrawableShape(MsgpackMixin):
    reference_frame_link = ''
    type = 0
    shape_params = []

    def __init__(self, reference_frame_link, type, shape_params):
        self.reference_frame_link = reference_frame_link
        self.type = type
        self.shape_params = shape_params


class DrawableShapeRequest(MsgpackMixin):
    shapes = {}
    persist_unmentioned = False

    def __init__(self, shapes={}, persist_unmentioned=False):
        self.shapes = shapes
        self.persist_unmentioned = persist_unmentioned
