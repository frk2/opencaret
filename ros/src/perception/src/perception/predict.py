import numpy as np
import torch
from torch.autograd import Variable
import cv2
import glob
from PIL import Image as PILImage
import perception.Model as Net
import os
from matplotlib import pyplot

"""
Code heavily inspired /copy pasted from: https://github.com/sacmehta/ESPNet

@article{mehta2018espnet,
  title={ESPNet: Efficient Spatial Pyramid of Dilated Convolutions for Semantic Segmentation},
  author={Sachin Mehta, Mohammad Rastegari, Anat Caspi, Linda Shapiro, and Hannaneh Hajishirzi},
  journal={arXiv preprint arXiv:1803.06815},
  year={2018}
}

"""

model_location = os.path.dirname(os.path.abspath(__file__))
class Prediction:
    # palette = [128, 64, 128,
    #            10, 225, 10,
    #            10, 10, 220]

    palette = [255, 0, 0,
               244, 35, 232,
               70, 70, 70,
               102, 102, 156,
               190, 153, 153,
               153, 153, 153,
               250, 170, 30,
               220, 220, 0,
               107, 142, 35,
               152, 251, 152,
               70, 130, 180,
               220, 20, 60,
               255, 0, 0,
               0, 0, 142,
               0, 0, 70,
               0, 60, 100,
               0, 80, 100,
               0, 0, 230,
               119, 11, 32,
               0, 0, 0]

    def __init__(self, model_weights= model_location + '/../../pretrained/decoder/model_331.pth', classes=20, p=2, q=8):
        self.model = Net.ESPNet(classes, p, q)
        # self.model = Net.ESPNet_Encoder(classes, p, q)

        if not os.path.isfile(model_weights):
            print('Pre-trained model file does not exist. Please check {} exists folder'.format(model_weights))
            return
        self.model.load_state_dict(torch.load(model_weights))
        self.model = self.model.cuda()
        self.model.eval()  # set to evaluation mode
        self.up = torch.nn.Upsample(scale_factor=8, mode='bilinear').cuda()

    def infer(self, np_image, overlay=False):
            # global mean and std values
            # mean = [72.3923111, 82.90893555, 73.15840149]
            # std = [45.3192215, 46.15289307, 44.91483307]

            mean = [73.933304, 74.61563, 71.06163]
            std = [51.179474, 50.492702, 50.31186]

            orig_image_np = None
            # np_image = cv2.resize()
            # np_image = self.resize_with_pad(np_image, 512, 1024)
            # np_image = cv2.resize(np_image, (1024, 512), interpolation=cv2.INTER_AREA)

            if overlay:
                orig_image_np = np.copy(np_image)

            for j in range(3):
                # mean[j] = np.mean(np_image[:,:,j])
                np_image[:, :, j] -= mean[j]
            for j in range(3):
                # std[j] = np.std(np_image[:,:,j])

                np_image[:, :, j] /= std[j]

            # resize the image to 1024x512x3
            if overlay:
                orig_image_np = cv2.cvtColor(orig_image_np, cv2.COLOR_BGR2RGB)
                orig_image_np = PILImage.fromarray(np.uint8(orig_image_np), "RGB")
                # orig_image_np = PILImage.new("RGB", (orig_image_np.shape[1], orig_image_np.shape[0]))

            np_image /= 255
            np_image = np_image.transpose((2, 0, 1))
            img_tensor = torch.from_numpy(np_image)
            img_tensor = torch.unsqueeze(img_tensor, 0)  # add a batch dimension
            img_variable = Variable(img_tensor, volatile=True)
            img_variable = img_variable.cuda()
            img_out = self.model(img_variable)
            # img_out = self.up(img_out)
            prediction_out = img_out[0].max(0)[1].byte().cpu().data.numpy()

            mask = prediction_out < 19
            mask = mask.astype(np.uint8) * 150
            # mask = cv2.resize(mask, (1280,720), interpolation=cv2.INTER_LINEAR)
            mask_image = PILImage.fromarray(mask, "L")
            # prediction_out = cv2.resize(prediction_out, (1280, 720), interpolation=cv2.INTER_LINEAR)
            if overlay:
                colored_img_pil = PILImage.fromarray(prediction_out)
                colored_img_pil.putpalette(Prediction.palette)
                colored_img_pil = PILImage.composite(colored_img_pil, orig_image_np, mask=mask_image)
                return prediction_out, colored_img_pil
            else:
                return prediction_out, None


if __name__ == '__main__':
    weights = "/home/faraz/opencaret/ESPNet/results_enc__dec_2_8/model_264.pth"
    predictor = Prediction(model_weights=weights)
    filter = model_location + '/../../test_data/*.png'
    filter = "/media/faraz/Faraz/leftImg8bit/val/munster/munster_00017*"
    filter = "/home/faraz/.ros/*.jpg"

    for f in glob.glob(filter):
        print("Reading in {}".format(f))
        img = cv2.imread(f).astype(np.float32)
        out = predictor.infer(img, True)
        #print(np.unique(out, return_counts=True))
        pyplot.imshow(out)
        pyplot.show()
