import sys
import pyzed.camera as zcam
import pyzed.types as tp
import pyzed.core as core
import perception.predict as predict
import cv2
import matplotlib.pyplot as plt
import numpy as np
from cuda_context import PyCudaContext
# obtain cuda-fun and install from: https://github.com/frk2/cuda-fun


def main():
    if len(sys.argv) != 2:
        print("Please specify path to .svo file.")
        exit()

    filepath = sys.argv[1]
    print("Reading SVO file: {0}".format(filepath))

    init = zcam.PyInitParameters(svo_input_filename=filepath, camera_image_flip=False, svo_real_time_mode=False)

    cam = zcam.PyZEDCamera()
    status = cam.open(init)
    zed_cuda_ctx=PyCudaContext()

    zed_cuda_ctx.pop_ctx()
    weights = "/home/faraz/opencaret/ESPNet/results_enc__dec_2_8/model_264.pth"

    predictor = predict.Prediction(model_weights=weights)
    zed_cuda_ctx.push_ctx()

    if status != tp.PyERROR_CODE.PySUCCESS:
        print(repr(status))
        exit()

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Be sure to use lower case
    out = cv2.VideoWriter('output.mp4', fourcc, 10.0, (1024, 512))

    runtime = zcam.PyRuntimeParameters()
    mat = core.PyMat()
    frames = 0
    while frames < 200:  # for 'q' key
        err = cam.grab(runtime)
        if err == tp.PyERROR_CODE.PySUCCESS:
            cam.retrieve_image(mat)
            npimg = mat.get_data().astype(np.float32)[:,:,:3]
            zed_cuda_ctx.pop_ctx()
            prediction = predictor.infer(npimg, overlay=False)
            zed_cuda_ctx.push_ctx()
            print(np.array(prediction).shape)
            out_image = cv2.cvtColor(np.array(prediction), cv2.COLOR_RGB2BGR)
            cv2.imshow('Perception', out_image)
            out.write(out_image)
            key = cv2.waitKey(1)
            frames += 1
    cv2.destroyAllWindows()
    out.release()
    cam.close()
    print("\nFINISH")


if __name__ == '__main__':
    main()
