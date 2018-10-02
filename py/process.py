# -*- coding: utf-8 -*- 
"""
Created on: 2018-09-26 18:04:59

@author: Sikasjc 
""" 
import os
import time
import numpy as np
import cv2
import bezier
import fitCurves

boundary = 25
threshold = 100
maxerror = 10

def readImg(imgpath1, imgpath2):
    img1 = cv2.imread(imgpath1, 0)
    img2 = cv2.imread(imgpath2, 0)
    img = img1 + img2
    return img

def readPoints(img, boundary, threshold):
    """
    Return the right and left points of Img
    """
    left_img = np.hstack((img[:, :boundary], np.ones_like(img[:, boundary:])))
    right_img = np.hstack((np.ones_like(img[:, :boundary]), img[:, boundary:]))

    left_points = np.where(left_img > threshold)
    right_points = np.where(right_img > threshold)
    return np.array(left_points), np.array(right_points)

def create_bezier(b):
    points = [bezier.q(b, t/50.0).tolist() for t in range(0, 51)]
    return np.array(points)
    
def create_line(img, points):
    points = points.reshape((-1,1,2)).astype(np.int32)
    points[:,:,[0,1]] = points[:,:,[1,0]]
    # print(points)
    cv2.polylines(img, [points], False, thickness=2, color=(0,255,255))

def process(imgpath1, imgpath2, boundary, threshold, maxerror):
    img = readImg(imgpath1, imgpath2)
    left_points, right_points = readPoints(img, boundary, threshold)

    left_points = left_points.T
    right_points = right_points.T

    # print(left_points)
    # print(right_points)
    lbeziers = np.array(fitCurves.fitCurve(left_points, maxerror))
    rbeziers = np.array(fitCurves.fitCurve(right_points, maxerror))
    # print(lbeziers)
    # print(rbeziers)
    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    img = cv2.resize(img, (420, 240), interpolation = cv2.INTER_LINEAR)

    if len(lbeziers) != 0:
        lbeziers[:, :, 1] *= 420/53
        lbeziers[:, :, 0] *= 240/30
    if len(rbeziers) != 0:
        rbeziers[:, :, 1] *= 420/53
        rbeziers[:, :, 0] *= 240/30

    for b in lbeziers:
        for ctrl in b:
            cv2.circle(img, (int(ctrl[1]), int(ctrl[0])), 3, (0,0,255), -1)
        create_line(img, create_bezier(b))
    for b in rbeziers:
        for ctrl in b:
            cv2.circle(img, (int(ctrl[1]), int(ctrl[0])), 3, (0,0,255), -1)
        create_line(img, create_bezier(b))
    
    cv2.imshow("lines", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    return img

def each_dir(dirpath, outdirpath):
    img_list = os.listdir(dirpath)
    n = len(img_list)
    for i in range(0, n,2):
        imgpath1 = os.path.join(dirpath, img_list[i])
        imgpath2 = os.path.join(dirpath, img_list[i+1])
        print("Process {} || {}".format(img_list[i], img_list[i+1]))
        img = process(imgpath1, imgpath2, boundary, threshold, maxerror)
        # 移除图片名字中的prob和1,2， 即 02010_prob_1.jpg => 02010.jpg
        imgname = os.path.splitext(img_list[i])[0].split("_")[0]
        imgname = outdirpath + "\\"+ imgname + ".jpg"
        cv2.imwrite(imgname, img)

if __name__ == "__main__":
    dirpath = r".\data\05151640_0419.MP4"
    outdirputh = r".\data\out"
    imgpath1 = r'.\data\05151640_0419.MP4\00000_prob_1.jpg'
    imgpath2 = r'.\data\05151640_0419.MP4\00000_prob_2.jpg'
    # each_dir(dirpath, outdirputh)
    process(imgpath1, imgpath2, boundary, threshold, maxerror)
    