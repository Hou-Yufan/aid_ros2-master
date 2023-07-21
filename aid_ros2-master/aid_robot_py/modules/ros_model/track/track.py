import os
import cv2
import ctypes
import numpy as np

import aid_rknn
from AidLux import Aashmem
from .utils import eqprocess, yolov5_post_process


def Track(sign, follow_sign_mem, control_sign_mem):
    sign = np.frombuffer(sign, dtype=ctypes.c_uint8)
    follow_sign = np.frombuffer(follow_sign_mem, dtype=ctypes.c_uint8)
    control_sign = np.frombuffer(control_sign_mem, dtype=ctypes.c_float)

    current_dir = os.path.dirname(os.path.abspath(__file__))
    model = Detecter(os.path.join(current_dir,"models/head.rknn"), os.path.join(current_dir,"models/yolov5s.rknn"))

    kv = Aashmem("/root/tmp/ipc")

    cap = cv2.VideoCapture(12)

    limt = [(700,000), (1220,250)]

    while sign:

        ret, frame = cap.read()
        if not ret:
            continue
        boxes, classes, scores, tp = model(frame)

        if boxes is not None:
            follow_sign[0] = 1
            kps_x = int( (boxes[0][0] + boxes[0][2]) / 2 )
            kps_y = int( (boxes[0][1] ) )
            
            if follow_sign[1]:
                if kps_x < limt[0][0]:
                    control_sign[0] = 1.0
                    control_sign[1] = 0.3
                    control_sign[2] = 0.05
                elif kps_x > limt[1][0]:
                    control_sign[0] = 1.0
                    control_sign[1] = -0.3
                    control_sign[2] = 0.05
                elif kps_y < limt[0][1]:
                    control_sign[0] = 1.0
                    control_sign[1] = 0.0
                    control_sign[2] = 0.00
                elif kps_y > limt[1][1]:
                    control_sign[0] = 1.0
                    control_sign[1] = 0.0
                    control_sign[2] = 0.05
                else:
                    control_sign[0] = 0
            else:
                control_sign[0] = 0

            cv2.circle(frame, (kps_x,kps_y), 3, (0,0,255),3)

            cv2.rectangle(frame, limt[0], limt[1], (255, 0, 0), 2)
        else:
            follow_sign[0] = 0
            control_sign[0] = 0
        frame = frame[:,:,::-1]
        binput = frame.tobytes()
        kv.set_bytes(binput, len(binput), 8)
        



class Detecter():
    def __init__(self, model_path1, model_path2):
        self.model = aid_rknn.rknn_model()
        self.model.load(model_path1, 0)
        self.model.load(model_path2, 1)

    def __call__(self, frame):
        img, scale = eqprocess(frame, 640, 640)
        self.model.set_Uint8(img.flatten(), 0, [0, 1228800])
        self.model.invoke(0)
        input0_data = self.model.get_Fp32(0,0,[134400, 33600, 8400],3).reshape(3,7,80,80)
        input1_data = self.model.get_Fp32(0,1,[134400, 33600, 8400],3).reshape(3,7,40,40)
        input2_data = self.model.get_Fp32(0,2,[134400, 33600, 8400],3).reshape(3,7,20,20)

        input_data = list()
        input_data.append(np.transpose(input0_data, (2, 3, 0, 1)))
        input_data.append(np.transpose(input1_data, (2, 3, 0, 1)))
        input_data.append(np.transpose(input2_data, (2, 3, 0, 1)))

        boxes, classes, scores = yolov5_post_process(input_data, 0.7, 0.6, 640)
        tp = 0
        if boxes is not None:
            boxes[boxes < 0] = 0
            boxes = (boxes * scale)
        else:
            self.model.set_Uint8(img.flatten(), 1, [0, 1228800])
            self.model.invoke(1)
            input0_data = self.model.get_Fp32(1,0,[1632000, 408000, 102000],3).reshape(3,85,80,80)
            input1_data = self.model.get_Fp32(1,1,[1632000, 408000, 102000],3).reshape(3,85,40,40)
            input2_data = self.model.get_Fp32(1,2,[1632000, 408000, 102000],3).reshape(3,85,20,20)

            input_data = list()
            input_data.append(np.transpose(input0_data, (2, 3, 0, 1)))
            input_data.append(np.transpose(input1_data, (2, 3, 0, 1)))
            input_data.append(np.transpose(input2_data, (2, 3, 0, 1)))
            boxes, classes, scores = yolov5_post_process(input_data, 0.7, 0.6, 640)
            tp = 1
            if boxes is not None:
                boxes[boxes < 0] = 0
                boxes = (boxes * scale)
        return boxes, classes, scores, tp
