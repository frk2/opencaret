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

    init = zcam.PyInitParameters(svo_input_filename=filepath, camera_image_flip=True, svo_real_time_mode=False)

    cam = zcam.PyZEDCamera()
    status = cam.open(init)
    zed_cuda_ctx=PyCudaContext()

    zed_cuda_ctx.pop_ctx()
    predictor = predict.Prediction()
    zed_cuda_ctx.push_ctx()

    if status != tp.PyERROR_CODE.PySUCCESS:
        print(repr(status))
        exit()

    runtime = zcam.PyRuntimeParameters()
    mat = core.PyMat()
    while True:  # for 'q' key
        err = cam.grab(runtime)
        if err == tp.PyERROR_CODE.PySUCCESS:
            cam.retrieve_image(mat)
            npimg = mat.get_data().astype(np.float32)[:,:,:3]
            zed_cuda_ctx.pop_ctx()
            prediction = predictor.infer(npimg, overlay=True)
            zed_cuda_ctx.push_ctx()
            cv2.imshow('Perception', cv2.cvtColor(np.array(prediction), cv2.COLOR_RGB2BGR))
            key = cv2.waitKey(1)
    cv2.destroyAllWindows()
    cam.close()
    print("\nFINISH")


if __name__ == '__main__':
    main()