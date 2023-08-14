import os
import os.path as osp

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
        if lines[start_idx+1].startswith("Time") or float(lines[start_idx][1:17]) - pre_t > 0.010:
            res.append(tmp)
            tmp = {}
            tmp['timestamp'] = float(lines[start_idx][1:17])
        
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
        pre_t = float(lines[start_idx][1:17])
        start_idx = end_idx+1

    start = 1
    if len(res[1]) < len(res[2]):
        start = 2
    res = res[start:] if len(res[-1]) == len(res[2]) else res[start:-1]
    return res