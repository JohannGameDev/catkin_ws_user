import numpy as np

import cv2
from sklearn import linear_model

class LinePath(object):

    def __init__(self):
        print("Created LinePath Object....")
        self.xLen = None
        self.yLen = None

    def getCoordinatesBwImage(self,img):
        xLen = img.shape[1] # x-axes
        yLen = img.shape[0] # y-axes
        self.xLen = xLen
        self.yLen = yLen
        X = []
        Y = []
        for y in range(yLen/2,yLen):
            for x in range(0, xLen):
                if img[y, x] == 255:
                    X.append(x)
                    Y.append(y)

        return np.array(X).reshape(-1, 1), np.array(Y).reshape(-1, 1)


    def getRansac(self,X,Y):
        # Robustly fit linear model with RANSAC algorithm
        ransac = linear_model.RANSACRegressor()
        ransac.fit(X, Y)
        inlier_mask = ransac.inlier_mask_
        outlier_mask = np.logical_not(inlier_mask)

        # Predict data of estimated models
        line_X = np.arange(X.min(), X.max())[:, np.newaxis]
        line_y_ransac = ransac.predict(line_X)
        return line_X, line_y_ransac, inlier_mask


    def calc_eq(self,a_line_x, a_line_y):
        a_line_x = np.concatenate(a_line_x, axis=0).astype(float)  # make list of list to list
        a_line_y = np.concatenate(a_line_y, axis=0).astype(float)

        A = np.vstack([a_line_x, np.ones(len(a_line_x))]).T
        m, c = np.linalg.lstsq(A, a_line_y, rcond=-1)[0]
        print("Line Solution is y = {m}x + {c}".format(m=m, c=c))
        return m, c


    def drawline(self,img, point1, point2):
        img = cv2.line(img, point1, point2, (0, 0, 255), 2)

        return img


    def get_image_line(self,img):
        cv2.imwrite('src/assignment6/data/mask_pic.png', img)
        im_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        threshold, im_bw = cv2.threshold(im_gray, 128, 255, cv2.THRESH_BINARY)
        # get line as x,y coordinates from ransac and draw it on the picture

        X_white, Y_white = self.getCoordinatesBwImage(im_bw)
        a_line_x, a_line_y, inlier_a = self.getRansac(X_white, Y_white)


        # publish this
        #eq_a_m, eq_a_c = calc_eq(a_line_x, a_line_y)

        # draw line on image

        img_line = self.drawline(img, (a_line_x[0], a_line_y[0]),
                                  (a_line_x[len(a_line_x)-1], a_line_y[len(a_line_y)-1]))


        return img_line, (a_line_x[len(a_line_x)/2], a_line_y[len(a_line_y)/2])





