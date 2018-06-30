import numpy as np
import torch
from torch.autograd import Variable
import glob
import cv2
from PIL import Image as PILImage
import Model as Net
import os
import time


class Prediction:
    def __init__(self, model='models/current.pth'):
        self.model =
