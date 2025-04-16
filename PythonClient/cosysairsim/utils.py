import sys
import os
import inspect
import re
import logging
import csv
from .types import *


def string_to_uint8_array(bstr):
    return np.fromstring(bstr, np.uint8)
    
def string_to_float_array(bstr):
    return np.fromstring(bstr, np.float32)
    
def list_to_2d_float_array(flst, width, height):
    return np.reshape(np.asarray(flst, np.float32), (height, width))
    
def get_pfm_array(response):
    return list_to_2d_float_array(response.image_data_float, response.width, response.height)

    
def get_public_fields(obj):
    return [attr for attr in dir(obj)
                            if not (attr.startswith("_") 
                            or inspect.isbuiltin(attr)
                            or inspect.isfunction(attr)
                            or inspect.ismethod(attr))]


    
def to_dict(obj):
    return dict([attr, getattr(obj, attr)] for attr in get_public_fields(obj))

    
def to_str(obj):
    return str(to_dict(obj))

    
def write_file(filename, bstr):
    """
    Write binary data to file.
    Used for writing compressed PNG images
    """
    with open(filename, 'wb') as afile:
        afile.write(bstr)


def get_colormap_channel_values():
    values = np.zeros(258, dtype=int)
    step = 256
    iter = 0
    values[0] = 0
    while step >= 1:
        val = step - 1
        while val <= 256:
            iter = iter + 1
            values[iter] = int(val)
            val = val + (step * 2)
        step = int(step / 2)
    init = True
    return values


gammaCorrectionTable = [0,
                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0, 0, 0, 79, 0,
                        81, 0, 83, 0, 85, 0, 86, 0, 88, 0,
                        90, 0, 93, 0, 95, 0, 96, 0, 98, 0,
                        101, 0, 102, 0, 105, 0, 106, 0, 109, 0,
                        110, 0, 113, 0, 114, 0, 117, 0, 119, 0,
                        120, 0, 122, 0, 125, 0, 127, 0, 129, 0,
                        131, 0, 133, 0, 135, 0, 137, 0, 139, 0,
                        141, 0, 143, 0, 145, 0, 147, 147, 148, 150,
                        151, 152, 153, 154, 155, 156, 157, 158, 159, 160,
                        161, 162, 163, 164, 165, 166, 167, 168, 169, 170,
                        171, 172, 173, 174, 175, 176, 177, 178, 179, 180,
                        181, 182, 183, 184, 185, 186, 187, 188, 189, 190,
                        191, 192, 193, 194, 195, 196, 197, 198, 199, 200,
                        201, 202, 203, 204, 205, 206, 207, 208, 209, 210,
                        211, 212, 213, 214, 215, 216, 217, 218, 219, 220,
                        221, 222, 223, 224, 225, 226, 227, 228, 229, 230,
                        231, 232, 233, 234, 235, 236, 237, 238, 239, 240,
                        241, 242, 243, 244, 245, 246, 247, 248, 249, 250,
                        251, 252, 253, 254, 255]


def get_colormap_colors(maxVal, enable1, enable2, enable3, colormap, channelValues, okValues):
    if not enable1:
        max1 = maxVal - 1
    else:
        max1 = 0
    if not enable2:
        max2 = maxVal - 1
    else:
        max2 = 0
    if not enable3:
        max3 = maxVal - 1
    else:
        max3 = 0
    i = 0
    j = 0
    k = 0
    while i <= max1:
        while j <= max2:
            while k <= max3:
                if enable1:
                    r = channelValues[maxVal]
                else:
                    r = channelValues[i]
                if enable2:
                    g = channelValues[maxVal]
                else:
                    g = channelValues[j]
                if enable3:
                    b = channelValues[maxVal]
                else:
                    b = channelValues[k]
                if r in okValues and g in okValues and b in okValues and r != 149 and g != 149 and b != 149:
                    colormap.append([gammaCorrectionTable[r], gammaCorrectionTable[g], gammaCorrectionTable[b]])
                k = k + 1
            j = j + 1
            k = 0
        i = i + 1
        j = 0


def generate_colormap():
    channelValues = get_colormap_channel_values()
    numPerChannel = 256
    colorMap = []
    okValues = []
    num_per_channel = 256
    uneven_start = 79
    full_start = 149
    for i in np.arange(uneven_start, full_start + 1, 2):
        okValues.append(i)
    for i in np.arange(full_start + 1, numPerChannel, 1):
        okValues.append(i)
    for maxChannelIndex in range(0, numPerChannel):
        get_colormap_colors(maxChannelIndex, False, False, True, colorMap, channelValues, okValues)
        get_colormap_colors(maxChannelIndex, False, True, False, colorMap, channelValues, okValues)
        get_colormap_colors(maxChannelIndex, False, True, True, colorMap, channelValues, okValues)
        get_colormap_colors(maxChannelIndex, True, False, False, colorMap, channelValues, okValues)
        get_colormap_colors(maxChannelIndex, True, False, True, colorMap, channelValues, okValues)
        get_colormap_colors(maxChannelIndex, True, True, False, colorMap, channelValues, okValues)
        get_colormap_colors(maxChannelIndex, True, True, True, colorMap, channelValues, okValues)
    colorMap = np.asarray(colorMap)
    return colorMap

def load_read_csv(path: str):
    with open(path, 'r') as csv_file:
        reader = csv.reader(csv_file)
        matrix = None
        first_row = True
        for row_index, row in enumerate(reader):
            if first_row:
                size = len(row)
                matrix = np.zeros((size, size), dtype=np.int32)
                first_row = False
            matrix[row_index] = row

    return matrix

def load_colormap():
    colorMap = np.load(os.path.dirname(os.path.abspath(__file__)) + "/colormap.npy")
    return colorMap


# helper method for converting getOrientation to roll/pitch/yaw
# https:#en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
def quaternion_to_euler_angles(q):
    z = q.z_val
    y = q.y_val
    x = q.x_val
    w = q.w_val

    # roll (x-axis rotation)
    sinr_cosp  = +2.0 * (w*x + y*z)
    cosr_cosp = +1.0 - 2.0*(x*x + y*y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    t2 = +2.0 * (w*y - z*x)
    if (t2 > 1.0):
        t2 = 1
    if (t2 < -1.0):
        t2 = -1.0
    pitch = math.asin(t2)

    # yaw (z-axis rotation)
    siny_cosp = +2.0 * (w*z + x*y)
    cosy_cosp = +1.0 - 2.0 * (y*y + z*z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

    
def euler_to_quaternion(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    q = Quaternionr()
    q.w_val = cy * cr * cp + sy * sr * sp
    q.x_val = cy * sr * cp - sy * cr * sp
    q.y_val = cy * cr * sp + sy * sr * cp
    q.z_val = sy * cr * cp - cy * sr * sp
    return q


def euler_to_rotation_matrix(roll, pitch, yaw):
    cx = math.cos(roll * 0.5)
    sx = math.sin(roll * 0.5)
    cy = math.cos(pitch * 0.5)
    sy = math.sin(pitch * 0.5)
    cz = math.cos(yaw * 0.5)
    sz = math.sin(yaw * 0.5)

    r11 = cy*cz
    r12 = -cy*sz
    r13 = sy
    r21 = cx*sz + cz*sx*sy
    r22 = cx*cz - sx*sy*sz
    r23 = -cy*sx
    r31 = sx*sz - cx*cz*sy
    r32 = cz*sx + cx*sy*sz
    r33 = cx*cy

    R = [[r11, r12, r13], [r21, r22, r23], [r31, r32, r33]]

    return R


def apply_rotation_offset(position, pitch, yaw, roll):
    R = euler_to_rotation_matrix(roll, pitch, yaw)

    rotated_position = Vector3r()
    rotated_position.x_val = position.x_val * R[0][0] + position.y_val * R[0][1] + position.z_val * R[0][2]
    rotated_position.y_val = position.x_val * R[1][0] + position.y_val * R[1][1] + position.z_val * R[1][2]
    rotated_position.z_val = position.x_val * R[2][0] + position.y_val * R[2][1] + position.z_val * R[2][2]

    return rotated_position

def get_camera_type(cameraType):
    if cameraType == "Scene":
        cameraTypeClass = ImageType.Scene
    elif cameraType == "Segmentation":
        cameraTypeClass = ImageType.Segmentation
    elif cameraType == "DepthPerspective":
        cameraTypeClass = ImageType.DepthPerspective
    elif cameraType == "DepthPlanar":
        cameraTypeClass = ImageType.DepthPlanar
    elif cameraType == "DepthVis":
        cameraTypeClass = ImageType.DepthVis
    elif cameraType == "Infrared":
        cameraTypeClass = ImageType.Infrared
    elif cameraType == "SurfaceNormals":
        cameraTypeClass = ImageType.SurfaceNormals
    elif cameraType == "DisparityNormalized":
        cameraTypeClass = ImageType.DisparityNormalized
    elif cameraType == "OpticalFlow":
        cameraTypeClass = ImageType.OpticalFlow
    elif cameraType == "OpticalFlowVis":
        cameraTypeClass = ImageType.OpticalFlowVis
    elif cameraType == "Lighting":
        cameraTypeClass = ImageType.Lighting
    elif cameraType == "Annotation":
        cameraTypeClass = ImageType.Annotation
    else:
        cameraTypeClass = ImageType.Scene
    return cameraTypeClass


def is_pixels_as_float(cameraType):
    if cameraType == "Scene":
        return False
    elif cameraType == "Segmentation":
        return False
    elif cameraType == "DepthPerspective":
        return True
    elif cameraType == "DepthPlanar":
        return True
    elif cameraType == "DepthVis":
        return True
    elif cameraType == "Infrared":
        return False
    elif cameraType == "SurfaceNormals":
        return False
    elif cameraType == "DisparityNormalized":
        return True
    elif cameraType == "OpticalFlow":
        return False
    elif cameraType == "OpticalFlowVis":
        return False
    elif cameraType == "Lighting":
        return False
    elif cameraType == "Annotation":
        return False
    else:
        return False


def get_image_bytes(data, cameraType):
    if cameraType == "Scene":
        img_rgb_string = data.image_data_uint8
    elif cameraType == "Segmentation":
        img_rgb_string = data.image_data_uint8
    elif cameraType == "DepthPerspective":
        img_depth_float = data.image_data_float
        img_depth_float32 = np.asarray(img_depth_float, dtype=np.float32)
        img_rgb_string = img_depth_float32.tobytes()
    elif cameraType == "DepthPlanar":
        img_depth_float = data.image_data_float
        img_depth_float32 = np.asarray(img_depth_float, dtype=np.float32)
        img_rgb_string = img_depth_float32.tobytes()
    elif cameraType == "DepthVis":
        img_depth_float = data.image_data_float
        img_depth_float32 = np.asarray(img_depth_float, dtype=np.float32)
        img_rgb_string = img_depth_float32.tobytes()
    elif cameraType == "Infrared":
        img_rgb_string = data.image_data_uint8
    elif cameraType == "SurfaceNormals":
        img_rgb_string = data.image_data_uint8
    elif cameraType == "DisparityNormalized":
        img_depth_float = data.image_data_float
        img_depth_float32 = np.asarray(img_depth_float, dtype=np.float32)
        img_rgb_string = img_depth_float32.tobytes()
    elif cameraType == "OpticalFlow":
        img_rgb_string = data.image_data_uint8
    elif cameraType == "OpticalFlowVis":
        img_rgb_string = data.image_data_uint8
    elif cameraType == "Lighting":
        img_rgb_string = data.image_data_uint8
    elif cameraType == "Annotation":
        img_rgb_string = data.image_data_uint8
    else:
        img_rgb_string = data.image_data_uint8
    return img_rgb_string

    
def wait_key(message = ''):
    ''' Wait for a key press on the console and return it. '''
    if message != '':
        print (message)

    result = None
    if os.name == 'nt':
        import msvcrt
        result = msvcrt.getch()
    else:
        import termios
        fd = sys.stdin.fileno()

        oldterm = termios.tcgetattr(fd)
        newattr = termios.tcgetattr(fd)
        newattr[3] = newattr[3] & ~termios.ICANON & ~termios.ECHO
        termios.tcsetattr(fd, termios.TCSANOW, newattr)

        try:
            result = sys.stdin.read(1)
        except IOError:
            pass
        finally:
            termios.tcsetattr(fd, termios.TCSAFLUSH, oldterm)

    return result

    
def read_pfm(file):
    """ Read a pfm file """
    file = open(file, 'rb')

    color = None
    width = None
    height = None
    scale = None
    endian = None

    header = file.readline().rstrip()
    header = str(bytes.decode(header, encoding='utf-8'))
    if header == 'PF':
        color = True
    elif header == 'Pf':
        color = False
    else:
        raise Exception('Not a PFM file.')

    temp_str = str(bytes.decode(file.readline(), encoding='utf-8'))
    dim_match = re.match(r'^(\d+)\s(\d+)\s$', temp_str)
    if dim_match:
        width, height = map(int, dim_match.groups())
    else:
        raise Exception('Malformed PFM header.')

    scale = float(file.readline().rstrip())
    if scale < 0: # little-endian
        endian = '<'
        scale = -scale
    else:
        endian = '>' # big-endian

    data = np.fromfile(file, endian + 'f')
    shape = (height, width, 3) if color else (height, width)

    data = np.reshape(data, shape)
    # DEY: I don't know why this was there.
    file.close()
    
    return data, scale

    
def write_pfm(file, image, scale=1):
    """ Write a pfm file """
    file = open(file, 'wb')

    color = None

    if image.dtype.name != 'float32':
        raise Exception('Image dtype must be float32.')

    if len(image.shape) == 3 and image.shape[2] == 3: # color image
        color = True
    elif len(image.shape) == 2 or len(image.shape) == 3 and image.shape[2] == 1: # grayscale
        color = False
    else:
        raise Exception('Image must have H x W x 3, H x W x 1 or H x W dimensions.')

    file.write('PF\n'.encode('utf-8')  if color else 'Pf\n'.encode('utf-8'))
    temp_str = '%d %d\n' % (image.shape[1], image.shape[0])
    file.write(temp_str.encode('utf-8'))

    endian = image.dtype.byteorder

    if endian == '<' or endian == '=' and sys.byteorder == 'little':
        scale = -scale

    temp_str = '%f\n' % scale
    file.write(temp_str.encode('utf-8'))

    image.tofile(file)

    
def write_png(filename, image):
    """ image must be numpy array H X W X channels
    """
    import cv2      # pip install opencv-python

    ret = cv2.imwrite(filename, image)
    if not ret:
        logging.error(f"Writing PNG file {filename} failed")
