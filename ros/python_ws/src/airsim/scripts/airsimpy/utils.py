import numpy as np
import math
import time
import sys
import os
import inspect
import types
import re

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
    with open(filename, 'wb') as afile:
        afile.write(bstr)


def get_colormap_channel_values():
    values = np.zeros(258, dtype=np.int)
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

# helper method for converting getOrientation to roll/pitch/yaw
# https:#en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
def to_eularian_angles(q):
    z = q.z_val
    y = q.y_val
    x = q.x_val
    w = q.w_val
    ysqr = y * y

    # roll (x-axis rotation)
    t0 = +2.0 * (w*x + y*z)
    t1 = +1.0 - 2.0*(x*x + ysqr)
    roll = math.atan2(t0, t1)

    # pitch (y-axis rotation)
    t2 = +2.0 * (w*y - z*x)
    if (t2 > 1.0):
        t2 = 1
    if (t2 < -1.0):
        t2 = -1.0
    pitch = math.asin(t2)

    # yaw (z-axis rotation)
    t3 = +2.0 * (w*z + x*y)
    t4 = +1.0 - 2.0 * (ysqr + z*z)
    yaw = math.atan2(t3, t4)

    return (pitch, roll, yaw)

    
def to_quaternion(pitch, roll, yaw):
    t0 = math.cos(yaw * 0.5)
    t1 = math.sin(yaw * 0.5)
    t2 = math.cos(roll * 0.5)
    t3 = math.sin(roll * 0.5)
    t4 = math.cos(pitch * 0.5)
    t5 = math.sin(pitch * 0.5)

    q = Quaternionr()
    q.w_val = t0 * t2 * t4 + t1 * t3 * t5 #w
    q.x_val = t0 * t3 * t4 - t1 * t2 * t5 #x
    q.y_val = t0 * t2 * t5 + t1 * t3 * t4 #y
    q.z_val = t1 * t2 * t4 - t0 * t3 * t5 #z
    return q

    
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
    import zlib, struct

    buf = image.flatten().tobytes()
    width = image.shape[1]
    height = image.shape[0]

    # reverse the vertical line order and add null bytes at the start
    width_byte_3 = width * 3
    raw_data = b''.join(b'\x00' + buf[span:span + width_byte_3]
                        for span in range((height - 1) * width_byte_3, -1, - width_byte_3))

    def png_pack(png_tag, data):
        chunk_head = png_tag + data
        return (struct.pack("!I", len(data)) +
                chunk_head +
                struct.pack("!I", 0xFFFFFFFF & zlib.crc32(chunk_head)))

    png_bytes = b''.join([
        b'\x89PNG\r\n\x1a\n',
        png_pack(b'IHDR', struct.pack("!2I5B", width, height, 8, 6, 0, 0, 0)),
        png_pack(b'IDAT', zlib.compress(raw_data, 9)),
        png_pack(b'IEND', b'')])

    write_file(filename, png_bytes)
