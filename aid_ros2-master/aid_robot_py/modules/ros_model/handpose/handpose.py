import os
import cv2
import ctypes
import numpy as np
import time

import aid_rknn
from AidLux import Aashmem
from .utils import eqprocess, yolov5_post_process, draw_bd_handpose, pose_det

# sign = sharedctypes.RawArray(ctypes.c_uint8, 1)
def HandPose(sign_mem, control_sign_mem):
    sign = np.frombuffer(sign_mem, dtype=ctypes.c_uint8)
    control_sign = np.frombuffer(control_sign_mem, dtype=ctypes.c_float)

    current_dir = os.path.dirname(os.path.abspath(__file__))
    hand_det = Hand_Det(os.path.join(current_dir,"models/hand_det_v3.rknn"))
    hand_pose = Hand_Pose(os.path.join(current_dir,"models/hand_pose_v2.rknn"))

    kv = Aashmem("/root/tmp/ipc")

    cap = cv2.VideoCapture(12)

    while sign:
        ret, frame = cap.read()
        if not ret:
            continue
        boxes, classes, scores = hand_det(frame)

        if boxes is not None:
            pts_hands = hand_pose(frame, boxes)
            res = pose_det(pts_hands)
            if res:
                control_sign[0] = 1.0
                control_sign[1] = 0.0
                control_sign[2] = 0.1
            else:
                control_sign[0] = 0
        else:
            control_sign[0] = 0

        frame = frame[:,:,::-1]
        binput = frame.tobytes()
        kv.set_bytes(binput, len(binput), 8)
    
    

class Hand_Det():
    def __init__(self, model_path):
        self.model = aid_rknn.rknn_model()
        res = self.model.load(model_path, 0)

    def __call__(self, frame):
        img, scale = eqprocess(frame, 640, 640)
        self.model.set_Uint8(img.flatten(), 0, [0, 1228800])
        self.model.invoke(0)
        input0_data = self.model.get_Fp32(0,0,[115200, 28800, 7200],3).reshape(3,6,80,80)
        input1_data = self.model.get_Fp32(0,1,[115200, 28800, 7200],3).reshape(3,6,40,40)
        input2_data = self.model.get_Fp32(0,2,[115200, 28800, 7200],3).reshape(3,6,20,20)

        input_data = list()
        input_data.append(np.transpose(input0_data, (2, 3, 0, 1)))
        input_data.append(np.transpose(input1_data, (2, 3, 0, 1)))
        input_data.append(np.transpose(input2_data, (2, 3, 0, 1)))

        boxes, classes, scores = yolov5_post_process(input_data, 0.5, 0.6, 640)
        if boxes is not None:
            boxes[boxes < 0] = 0
            boxes = (boxes * scale)
        return boxes, classes, scores

class Hand_Pose():
    def __init__(self, model_path):
        self.model = aid_rknn.rknn_model()
        res = self.model.load(model_path, 1)
    
    def __call__(self, frame, boxes):
        pts_hands = []
        for box in boxes:
            left, top, right, bottom = [int(t) for t in box]
            hand_crop = frame[top:bottom, left:right, :]
            hand_crop_resized = cv2.resize(hand_crop, (256,256))
            self.model.set_Uint8(hand_crop_resized, 1, [0, 196608])
            self.model.invoke(1)
            output = self.model.get_Fp32(1,0,[42],1)
            pts_hand = {} #构建关键点连线可视化结构
            for i in range(int(output.shape[0]/2)):
                x = (output[i*2+0]*float(hand_crop.shape[1])) + left
                y = (output[i*2+1]*float(hand_crop.shape[0])) + top

                pts_hand[str(i)] = {}
                pts_hand[str(i)] = {
                    "x":x,
                    "y":y,
                    }
            pts_hands.append(pts_hand)
            draw_bd_handpose(frame, pts_hand,0,0)
            for i in range(int(output.shape[0]/2)):
                x = (output[i*2+0]*float(hand_crop.shape[1])) + left
                y = (output[i*2+1]*float(hand_crop.shape[0])) + top

                cv2.circle(frame, (int(x),int(y)), 3, (255,50,60),-1)
                cv2.circle(frame, (int(x),int(y)), 1, (255,150,180),-1)
        return pts_hands

        




