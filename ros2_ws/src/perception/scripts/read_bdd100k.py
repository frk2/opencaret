import os
import sys

image_path = sys.argv[1]
dataset_path = sys.argv[2]
maxcount = int(sys.argv[3])

count = 0
for root, dirs, files in os.walk(dataset_path):
    if root.endswith("train") or root.endswith("test") or root.endswith("val"):
        im_type = os.path.basename(root)
        txt = open(im_type + '.txt', 'w')

        for entry in files:
            orig_image_fname = os.path.basename(entry).split("_")[0]
            orig_image = os.path.join(image_path, im_type, orig_image_fname+'.jpg')
            gt = os.path.join(dataset_path, root, entry)
            if not os.path.isfile(orig_image):
                print("Orig file {} does not exist".format(orig_image))
                continue
            if not os.path.isfile(gt):
                print("gt file {} does not exist".format(gt))
                continue
            print("{},{}".format(orig_image, gt))
            count += 1
            if count > maxcount:
                count = 0
                break
            txt.write("{},{}\n".format(orig_image, gt))