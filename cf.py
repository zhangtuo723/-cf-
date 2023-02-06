import cv2,math
import numpy as np
import torch
from pathlib import Path
from models.experimental import attempt_load
from utils.general import non_max_suppression, scale_coords,xyxy2xywh,check_suffix,check_img_size
from d3dshot import create
from utils.augmentations import letterbox
from utils.torch_utils import time_sync
from utils.plots import Annotator, colors, save_one_box
from AI_Core import Screenshot_Mode,Detection_mode,Target_Distance,Aim_Target


ps_mode = 0
test_mode = 0

pos_x = 960
pos_y = 540
mcx = 150
mcy = 150
Mouse_drive_mode = 0
Body_offset_value_x = 0 
Body_offset_value_y = 0
head_offset_value_x = 0
head_offset_value_y = 0
Window_border_x = 0
Window_border_y = 0

window_size,core_x,core_y,Screenshot_value = Screenshot_Mode(ps_mode, pos_x, pos_y, mcx, mcy)

hide_labels=False
hide_conf=False
# #模型尺寸
imgsz = 640
weights = 'cf.pt'
device = "cuda"

# model = attempt_load(weights, device=device)
model = attempt_load(weights,'cuda')
stride = int(model.stride.max()) 
half=False
# names = "QS","T"
names = model.names

model.float()
# Aim_Target(Mouse_drive_mode, tag, target_x, target_y, 
#         core_x, core_y, target_w, target_h, Body_offset_value_x, 
#         Body_offset_value_y, head_offset_value_x, head_offset_value_y,
#         Window_border_x, Window_border_y)
while True:
    # print("111")
    t1 = time_sync()
    img = Detection_mode(test_mode, Screenshot_value, window_size)
    img0 = img  
    annotator = Annotator(img0, line_width=3, example=str(names))
    # 640 640 4
    img = letterbox(img, imgsz, stride=stride)[0]

    #4 640 640
    img = img.transpose((2, 0, 1))[::-1]

    #4 640 640
    img = np.ascontiguousarray(img)

    img = torch.from_numpy(img).to(device)
    img = img.half() if half else img.float()  # uint8 to fp16/32
    img /= 255.0
    if len(img.shape) == 3:
        img = img[None]

    pred = model(img, augment=False, visualize=False)[0]
    pred.clone().detach()
    pred = non_max_suppression(pred, 0.25, 0.45, 1, False, max_det=1000)  #非极大值抑制
    t2 = time_sync()
    t3= t2 - t1
    _,det = next(enumerate(pred))
    s = ''
    s += '%gx%g ' % img.shape[2:]
    aims = []
    # print(det)
    if len(det):
        
        det[:, :4] = scale_coords(img.shape[2:], det[:, :4], img0.shape).round()
        for *xyxy, conf, cls in reversed(det):
            xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) ).view(-1).tolist()
            line = (cls, *xywh,conf)  # label format
            aim = ('%g ' * len(line)).rstrip() % line
            aim = aim.split(' ')
            aims.append(aim)
            cls = int(cls)
            label = None if hide_labels else (names[cls] if hide_conf else f'{names[cls]} {conf:.2f}')
            # annotator.box_label(xyxy, label, color=colors(cls, True))

        dis_list = []
        for i in aims:
            # i[5]  
            dis = Target_Distance(float(i[1]), core_x, float(i[2]), core_y)
            dis_list.append(dis)
        det = aims[dis_list.index(min(dis_list))]
        tag = int(det[0])
        target_x = float(det[1])
        target_y = float(det[2])
        target_w = float(det[3])
        target_h = float(det[4])
        
       
        Aim_Target(Mouse_drive_mode, tag, target_x, target_y, 
        core_x, core_y, target_w, target_h, Body_offset_value_x, 
        Body_offset_value_y, head_offset_value_x, head_offset_value_y,
        Window_border_x, Window_border_y)
        


    # img0 = annotator.result()
    # cv2.imshow("winname", img0)
            
    if cv2.waitKey(1) & 0xFF == ord('q'):  
        cv2.destroyAllWindows()
        break
