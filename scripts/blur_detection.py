import json

file = open('/media/jyh/Extreme SSD/unloading/CJHub/Kinect/230629/cam0_rgb/cam0_rgb_cam_info_2023-06-29_20_08_43_256.json')
jsonString = json.load(file)

for key1, val1 in jsonString.items():
    if type(val1) is dict:
        print(str(key1) + ": ")
        for key, val in val1.items():
            print("  " + str(key) + ": " + str(val))
    else:
        print(str(key1) + ": " + str(val1))
