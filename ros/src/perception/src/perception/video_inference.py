import sys
from moviepy.editor import *
import perception.predict as predict
import numpy as np
import cv2

def main():
    if len(sys.argv) != 2:
        print("Please specify path to movie file.")
        exit()

    filepath = sys.argv[1]
    print("Reading movie file: {0}".format(filepath))

    # fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Be sure to use lower case
    # out = cv2.VideoWriter('output.mp4', fourcc, 10.0, (1920, 1080))
    weights = "/home/faraz/opencaret/ESPNet/results_enc__dec_2_8/model_331.pth"

    predictor = predict.Prediction(model_weights=weights)

    def process_image(img):
        # img = cv2.flip(img, -1)
        # img = img[208:720,0:1024,:]
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        prediction = predictor.infer(img.astype(np.float32), overlay=True)
        return np.array(prediction)

    video = VideoFileClip(filepath).subclip(60,99).fl_image(process_image)

    video.write_videofile('output.mp4')
    print("\nFINISH")


if __name__ == '__main__':
    main()
