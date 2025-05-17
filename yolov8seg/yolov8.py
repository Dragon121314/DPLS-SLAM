# -*- coding: utf-8 -*-

from ultralytics import YOLO
import torch
print(torch.cuda.is_available())  # Should print True if CUDA is available
print(torch.cuda.device_count())  # Should print the number of available GPUs
# Load a model
model = YOLO("/home/robot/anaconda3/envs/yolov8/model/yolov8n-seg.pt",task='segment')  # load a pretrained model (recommended for training)


# # Train the model
# results = model.train(data='coco128-seg.yaml', epochs=100, imgsz=640)

model.predict(source="/home/robot/ORB_SLAM/ORB2_YOLOV8/yolov8/data/dog.jpg",save=False,show=True,device='0')

