import os
import json
import cv2
import numpy as np
imdir="camera"
calibfile="config_normal_100deg12177.json"
imgs=os.listdir(imdir)
calib=json.load(open(calibfile))
mtx=np.array(calib["cam"]["K"])
dist=np.array(calib["cam"]["D"])
os.mkdir("image")
os.mkdir("image/front")
#os.mkdir("calib")
os.mkdir("label")
'''json.dump({
        "extrinsic": [],
        "intrinsic": mtx.reshape(1,-1)[0].tolist()
    },open("calib/front.json","w"))
    '''
os.rename("pointcloud","pcd")
for img in imgs:
    if img.split(".")[-1]!="jpg":
        continue
    print(img)
    imdata=cv2.imread(os.path.join(imdir,img))
    h,  w = imdata.shape[:2]
    #newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),0,(w,h)) 
    newcameramtx=mtx
    dst = cv2.undistort(imdata, mtx, dist, None, newcameramtx)
    cv2.imwrite(os.path.join("image","front",img),dst)
