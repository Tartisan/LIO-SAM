import os
import os.path as osp
import math

def read_file(path):
    with open(path, 'r') as f:
        lines = f.readlines()
    return [line for line in lines]

def read_file_list(path, token=None):
    file_list = []
    for root, dirs, files in os.walk(path):
        for file_name in files:
            if token == None or token in file_name:
                file_list.append(osp.join(root, file_name))
    file_list.sort()
    return file_list

def _degree2radian(gps):
    """
    gps: np.array, 1*3
    return: np.array, 1*3
    """
    return [math.pi/180 * gps[0], math.pi/180 * gps[1], gps[2]]

def gps2ecef(gps):
    # gps: [Lat, Lon, Alt]
    e_sqr= 0.00669438;  # square of the first numerical eccentricity of the ellipsoid， 1-b^2/a^2
    b = 6356752.3142
    a = 6378137.0
    points = _degree2radian(gps)
    N = a / math.sqrt(1-e_sqr * (math.sin(points[0]) ** 2))
    x = (N + points[2]) * math.cos(points[0]) * math.cos(points[1])
    y = (N + points[2]) * math.cos(points[0]) * math.sin(points[1])
    z = ((1-e_sqr) * N + points[2]) * math.sin(points[0])
    
    return x, y, z

def _find_next_record(lines, idx):
    i = idx
    s_idx, e_id = -1, -1
    while i < len(lines):
        if lines[i].startswith("("):
            s_idx = i
        if lines[i].startswith(")"):
            e_idx = i
            break
        i += 1
    return s_idx, e_idx

def read_gpsimu_from_txt(path: str):
    '''
    Read gps_imu data from txt file, and return data as list of dict
    Signals 01: 
        Time, Ang_Rate_Raw_IMU, Accel_IMU_Raw, InsStatus, LatitudeLongitude,
        Altitude, PosSigma, VelocityLevel, VelocityLevelSigma, Accel_Vehicle,
        HeadingPitchRoll, HeadingPitchRollSigma, AngRateVehicle

        华测: 
        heading 正北方向顺时针旋转为正，范围 0-360°

    Signals 02:
        GPS_time, Three_axis_attitude, lon_and_lat, alt_and_flag, 
        three_demensional_speed, xy_acc, xy_gyro, z_acc_gyro
    '''
    with open(path, 'r') as f:
        lines = f.readlines()
    res = []
    start_idx = 0
    pre_t = -1
    tmp = {}
    g = 9.8
    while start_idx < len(lines):
        start_idx, end_idx = _find_next_record(lines, start_idx)
        # Signals 01
        # float(lines[start_idx][1:17]) - pre_t > 0.010
        if lines[start_idx+1].startswith("Time"):
            res.append(tmp)
            tmp = {}
            tmp['timestamp'] = float(lines[start_idx][1:].split(')')[0])
        elif lines[start_idx+1].startswith("Ang_Rate_Raw_IMU"):
            tmp["Ang_Rate_Raw_IMU"] = [float(lines[start_idx+2].strip().split(' ')[1]), 
                                       float(lines[start_idx+3].strip().split(' ')[1]), 
                                       float(lines[start_idx+4].strip().split(' ')[1])]
        elif lines[start_idx+1].startswith("Accel_IMU_Raw"):
            tmp["Accel_IMU_Raw"] = [float(lines[start_idx+2].strip().split(' ')[1])*g, 
                                    float(lines[start_idx+3].strip().split(' ')[1])*g, 
                                    float(lines[start_idx+4].strip().split(' ')[1])*g]
        elif lines[start_idx+1].startswith("LatitudeLongitude("):
            tmp["Latitude"] = float(lines[start_idx+2].strip().split(' ')[1])
            tmp["Longitude"] = float(lines[start_idx+3].strip().split(' ')[1])
        elif lines[start_idx+1].startswith("Longitude("):
            tmp["Longitude"] = float(lines[start_idx+2].strip().split(' ')[1])
        elif lines[start_idx+1].startswith("Latitude("):
            tmp["Latitude"] = float(lines[start_idx+2].strip().split(' ')[1])
        elif lines[start_idx+1].startswith("Altitude"):
            tmp["Altitude"] = float(lines[start_idx+2].strip().split(' ')[1])
        elif lines[start_idx+1].startswith('PosSigma'):
            tmp["PoseSigma"] = [float(lines[start_idx+2].strip().split(' ')[1]), 
                                float(lines[start_idx+3].strip().split(' ')[1]), 
                                float(lines[start_idx+4].strip().split(' ')[1])]
        elif lines[start_idx+1].startswith("VelocityLevel("):
            tmp["VelocityLevel"] = [float(lines[start_idx+2].strip().split(' ')[1]), 
                                    float(lines[start_idx+3].strip().split(' ')[1]), 
                                    float(lines[start_idx+4].strip().split(' ')[1])]
        elif lines[start_idx+1].startswith("VelocityLevelSigma"):
            tmp["VelocityLevelSigma"] = [float(lines[start_idx+2].strip().split(' ')[1]), 
                                         float(lines[start_idx+3].strip().split(' ')[1]), 
                                         float(lines[start_idx+4].strip().split(' ')[1])]
        elif lines[start_idx+1].startswith("Accel_Vehicle"):
            tmp["Accel_Vehicle"] = [float(lines[start_idx+2].strip().split(' ')[1])*g, 
                                    float(lines[start_idx+3].strip().split(' ')[1])*g, 
                                    float(lines[start_idx+4].strip().split(' ')[1])*g]
        elif lines[start_idx+1].startswith("HeadingPitchRoll("):
            tmp["HeadingPitchRoll"] = [float(lines[start_idx+2].strip().split(' ')[1]), 
                                       float(lines[start_idx+3].strip().split(' ')[1]), 
                                       float(lines[start_idx+4].strip().split(' ')[1])]
        elif lines[start_idx+1].startswith("HeadingPitchRollSigma"):
            tmp["HeadingPitchRollSigma"] = [float(lines[start_idx+2].strip().split(' ')[1]), 
                                            float(lines[start_idx+3].strip().split(' ')[1]), 
                                            float(lines[start_idx+4].strip().split(' ')[1])]
        elif lines[start_idx+1].startswith("AngRateVehicle"):
            tmp["AngRateVehicle"] = [float(lines[start_idx+2].strip().split(' ')[1]), 
                                     float(lines[start_idx+3].strip().split(' ')[1]), 
                                     float(lines[start_idx+4].strip().split(' ')[1])]
        elif lines[start_idx+1].startswith("InsStatus"):
            tmp["InsStatus"] = [int(lines[start_idx+2].split(": ")[1].split(',')[0]), 
                                int(lines[start_idx+3].split(": ")[1].split(',')[0]), 
                                int(lines[start_idx+4].split(": ")[1].split(',')[0]), 
                                int(lines[start_idx+5].split(": ")[1].split(',')[0]), 
                                float(lines[start_idx+6].split(": ")[1].split(',')[0]), 
                                int(lines[start_idx+7].split(": ")[1].split(',')[0]), 
                                int(lines[start_idx+8].split(": ")[1].split(',')[0])]
        
        # Signals 02
        elif lines[start_idx+1].startswith("GPS_time"):
            res.append(tmp)
            tmp = {}
            tmp['timestamp'] = float(lines[start_idx][1:].split(')')[0])
        elif lines[start_idx+1].startswith("Three_axis_attitude"):
            tmp["HeadingPitchRoll"] = [float(lines[start_idx+2].strip().split(' ')[1]), 
                                       float(lines[start_idx+3].strip().split(' ')[1]), 
                                       float(lines[start_idx+4].strip().split(' ')[1])]
        elif lines[start_idx+1].startswith("lon_and_lat"):
            tmp["Latitude"] = float(lines[start_idx+2].strip().split(' ')[1])
            tmp["Longitude"] = float(lines[start_idx+3].strip().split(' ')[1])
        elif lines[start_idx+1].startswith("alt_and_flag"):
            tmp["Altitude"] = float(lines[start_idx+2].strip().split(' ')[1])
            # 兼容 Signals 01
            tmp["InsStatus"] = [2, 0, 3, 0, 0, 0, 0]
            nav_flag = lines[start_idx+3].strip().split(' ')[1]
            if nav_flag == 'PSRSP':
                tmp["InsStatus"][2] = 1
            elif nav_flag == 'PSRDIFF':
                tmp["InsStatus"][2] = 2
            elif nav_flag == 'SBAS':
                tmp["InsStatus"][2] = 3
            elif nav_flag == 'RTKFIXED':
                tmp["InsStatus"][2] = 4
            elif nav_flag == 'RTKFLOAT':
                tmp["InsStatus"][2] = 5
        elif lines[start_idx+1].startswith("three_demensional_speed"):
            tmp["VelocityLevel"] = [float(lines[start_idx+2].strip().split(' ')[1]), 
                                    float(lines[start_idx+3].strip().split(' ')[1]), 
                                    float(lines[start_idx+4].strip().split(' ')[1])]
        elif lines[start_idx+1].startswith("xy_acc"):
            if "Accel_Vehicle" not in tmp.keys():
                tmp["Accel_Vehicle"] = [0, 0, 0]
            tmp["Accel_Vehicle"][0] = float(lines[start_idx+2].strip().split(' ')[1])
            tmp["Accel_Vehicle"][1] = float(lines[start_idx+3].strip().split(' ')[1])
        elif lines[start_idx+1].startswith("xy_gyro"):
            if "AngRateVehicle" not in tmp.keys():
                tmp["AngRateVehicle"] = [0, 0, 0]
            tmp["AngRateVehicle"][0] = float(lines[start_idx+2].strip().split(' ')[1])
            tmp["AngRateVehicle"][1] = float(lines[start_idx+3].strip().split(' ')[1])
        elif lines[start_idx+1].startswith("z_acc_gyro"):
            if "AngRateVehicle" not in tmp.keys():
                tmp["AngRateVehicle"] = [0, 0, 0]
            if "Accel_Vehicle" not in tmp.keys():
                tmp["Accel_Vehicle"] = [0, 0, 0]
            tmp["AngRateVehicle"][2] = float(lines[start_idx+2].strip().split(' ')[1])
            tmp["Accel_Vehicle"][2] = float(lines[start_idx+3].strip().split(' ')[1])

        pre_t = float(lines[start_idx][1:17])
        start_idx = end_idx+1

    if len(res) < 3:
        print(f'Warning: only {len(res)} can frames, too little!')
        return []
    # 判断是否掐头去尾
    start = 1
    if len(res[1]) < len(res[2]):
        start = 2
    res = res[start:] if len(res[-1]) == len(res[2]) else res[start:-1]
    return res