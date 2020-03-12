#!/usr/bin/env python3
import cv2
from PIL import Image
import torch
import torchvision.transforms as transforms
import torch2trt
from torch2trt import TRTModule
from trt_pose import coco, models
from trt_pose.draw_objects import DrawObjects
from trt_pose.parse_objects import ParseObjects
from jetcam.usb_camera import USBCamera
from jetcam.utils import bgr8_to_jpeg
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import json
import time
import signal
import sys
import os

# TODO: set these as params in launch file
MODEL_WEIGHTS = './src/smartsnipe-obc/smartsnipe_vision/config/resnet18_baseline_att_224x224_A_epoch_249.pth'
OPTIMIZED_MODEL = './src/smartsnipe-obc/smartsnipe_vision/config/resnet18_baseline_att_224x224_A_epoch_249_trt.pth'
HUMAN_POSE = './src/smartsnipe-obc/smartsnipe_vision/config/human_pose.json'
WIDTH = 224
HEIGHT = 224

class PoseEstimator:

    def __init__(self, imshow=True):
        self.imshow = imshow # display results of pose esimtation
        # Torch settings
        self.device = torch.device('cuda')
        self.mean = torch.Tensor([0.485, 0.456, 0.406]).cuda()
        self.std = torch.Tensor([0.229, 0.224, 0.225]).cuda()
        self.load_model()

        # Setup camera and visuals
        self.parse_objects = ParseObjects(self.topology)
        self.draw_objects = DrawObjects(self.topology)
        self.setup_camera()

        # Visualization
        self.im = plt.imshow(self.execute({'new': self.camera.value}))
        self.ani = FuncAnimation(plt.gcf(), self.update, interval=200)
        self.cid = plt.gcf().canvas.mpl_connect("key_press_event", self.close)
        self.running = True
        plt.show()
    
    def load_model(self):
        # TODO: replace with param
        print("Loading saved model")
        # human_pose_file = rospy.get_param("~file", None)
        with open(HUMAN_POSE, 'r') as f:
            human_pose = json.load(f)
    
        self.topology = coco.coco_category_to_topology(human_pose)

        num_parts = len(human_pose['keypoints'])
        num_links = len(human_pose['skeleton'])

        self.model = models.resnet18_baseline_att(num_parts, 2 * num_links).cuda().eval()
        self.model.load_state_dict(torch.load(MODEL_WEIGHTS))

        print("Loading optimized model")
        self.model_trt = TRTModule()
        self.model_trt.load_state_dict(torch.load(OPTIMIZED_MODEL))
    
    def update(self, i):
        self.im.set_data(self.execute({'new': self.camera.value}))
    
    def close(self, event):
        if event.key == 'q':
            print('Closing animation...')
            plt.close(event.canvas.figure)
            self.camera.unobserve_all()
            self.running = False

    def setup_camera(self):
        self.camera = USBCamera(width=WIDTH, height=HEIGHT, capture_device=1)
        self.camera.running = True

    def preprocess(self, image):
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = Image.fromarray(image)
        image = transforms.functional.to_tensor(image).to(self.device)
        image.sub_(self.mean[:, None, None]).div_(self.std[:, None, None])
        return image[None, ...]

    def benchmark(self, frame_count=50.0):
        # Return FPS for current model
        t0 = time.time()
        torch.cuda.current_stream().synchronize()
        for i in range(frame_count):
            y = self.model_trt(data)
        torch.cuda.current_stream().synchronize()
        t1 = time.time()
        return frame_count / (t1 - t0)

    def execute(self, change):
        image = change['new']
        data = self.preprocess(image)
        cmap, paf = self.model_trt(data)
        cmap, paf = cmap.detach().cpu(), paf.detach().cpu()
        counts, objects, peaks = self.parse_objects(cmap, paf)#, cmap_threshold=0.15, link_threshold=0.15)
        self.draw_objects(image, counts, objects, peaks)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        # image_w.value = bgr8_to_jpeg(image[:, ::-1, :])
        return image

    def run(self):
        if self.imshow:
            while True:
                if not self.running:
                    break
                self.im = plt.imshow(self.execute({'new': self.camera.value}))



if __name__ == "__main__":
    try:
        estimator = PoseEstimator()
        estimator.run()       
    except KeyboardInterrupt:
        print("Pose estimation node interrupted")
        exit()