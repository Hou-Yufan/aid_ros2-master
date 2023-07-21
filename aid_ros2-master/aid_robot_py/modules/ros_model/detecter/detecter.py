import os
import cv2
import ctypes
import numpy as np

import aid_rknn
from AidLux import Aashmem
from .utils import eqprocess, yolov5_post_process, draw



def Detecter(sign, depth_mem):
    sign = np.frombuffer(sign, dtype=ctypes.c_uint8)

    depth = np.frombuffer(depth_mem, dtype=ctypes.c_uint8).reshape(480,640,3)

    current_dir = os.path.dirname(os.path.abspath(__file__))
    model = Detecte(os.path.join(current_dir,"models/yolov5s.rknn"))

    kv = Aashmem("/root/tmp/ipc")

    while sign:
        frame = depth.copy()
        frame = cv2.flip(frame,-1)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        boxes, classes, scores = model(frame)
        if boxes is not None:
            draw(frame, boxes, scores, classes)

        frame = frame[:,:,::-1]
        binput = frame.tobytes()
        kv.set_bytes(binput, len(binput), 8)


class Detecte():
    def __init__(self, model_path):
        self.model = aid_rknn.rknn_model()
        print(model_path)
        res = self.model.load(model_path, 0)

    def __call__(self, frame):
        img, scale = eqprocess(frame, 640, 640)
        self.model.set_Uint8(img.flatten(), 0, [0, 1228800])
        self.model.invoke(0)
        input0_data = self.model.get_Fp32(0,0,[1632000, 408000, 102000],3).reshape(3,85,80,80)
        input1_data = self.model.get_Fp32(0,1,[1632000, 408000, 102000],3).reshape(3,85,40,40)
        input2_data = self.model.get_Fp32(0,2,[1632000, 408000, 102000],3).reshape(3,85,20,20)

        input_data = list()
        input_data.append(np.transpose(input0_data, (2, 3, 0, 1)))
        input_data.append(np.transpose(input1_data, (2, 3, 0, 1)))
        input_data.append(np.transpose(input2_data, (2, 3, 0, 1)))

        boxes, classes, scores = yolov5_post_process(input_data, 0.7, 0.6, 640)
        if boxes is not None:
            boxes[boxes < 0] = 0
            boxes = (boxes * scale)
        return boxes, classes, scores
