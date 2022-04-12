import cv2 as cv
import numpy as np
import sys
from math import degrees as dg

def read_camera_parameters(filepath = 'intrinsicParameters/'):
    
    cmtx = np.loadtxt(filepath + 'oneEyeCameraMatrixPix.txt')
    dist = np.loadtxt(filepath + 'oneEyeCameraDistortionPix.txt')

    return cmtx, dist


def get_qr_coords(cmtx, dist, points):

    # Selected coordinate points for each corner of QR code.
    qr_edges = np.array([[0,0,0],
                         [0,1,0],
                         [1,1,0],
                         [1,0,0]], dtype = 'float32').reshape((4,1,3))

    # determine the orientation of QR code coordinate system with respect to camera coorindate system.
    ret, rvec, tvec = cv.solvePnP(qr_edges, points, cmtx, dist) # estimate the orientation of a 3D object in a 2D image.
    rotationMatrix = cv.Rodrigues(rvec)[0]
    homoMatrix = np.hstack((rotationMatrix, tvec))
    homoMatrix = np.vstack((homoMatrix, np.array([0, 0, 0, 1])))

    roll = dg(rvec[0][0])
    yaw = dg(rvec[1][0])
    pitch = dg(rvec[2][0])
    pose = [roll, yaw, pitch] # Roll: red, Yaw: blue, Pitch: green

    # Define unit xyz axes. These are then projected to camera view using the rotation matrix and translation vector.
    unitv_points = np.array([[0,0,0], [1,0,0], [0,1,0], [0,0,1]], dtype = 'float32').reshape((4,1,3))
    if ret:
        points, jac = cv.projectPoints(unitv_points, rvec, tvec, cmtx, dist)
        # print(pose)
        # print(rotationMatrix)
        print(homoMatrix)
        print("break")
        return points, rvec, tvec, pose, homoMatrix

    # return empty arrays if rotation and translation values not found
    else: return [], [], []


def show_axes(cmtx, dist, in_source):
    cap = cv.VideoCapture(in_source)

    qr = cv.QRCodeDetector()

    while True:
        ret, img = cap.read()
        if ret == False: break

        ret_qr, points = qr.detect(img)

        if ret_qr:
            axis_points, rvec, tvec, pose, homoMatrix = get_qr_coords(cmtx, dist, points)  # points	Output vector of vertices of the minimum-area quadrangle containing the code.
            # axis point = coordinate 3D point project to 2D plane

            # BGR color format
            colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (0,0,0)]

            # check axes points are projected to camera view.
            if len(axis_points) > 0:
                axis_points = axis_points.reshape((4,2))

                origin = (int(axis_points[0][0]),int(axis_points[0][1]) )

                for p, c in zip(axis_points[1:], colors[:3]):
                    p = (int(p[0]), int(p[1]))

                    # Sometimes qr detector will make a mistake and projected point will overflow integer value. We skip these cases.
                    if origin[0] > 5*img.shape[1] or origin[1] > 5*img.shape[1]:break
                    if p[0] > 5*img.shape[1] or p[1] > 5*img.shape[1]:break

                    cv.line(img, origin, p, c, 5)

        cv.imshow('frame', img)

        k = cv.waitKey(20)
        if k == 27: break #27 is ESC key.

    cap.release()
    cv.destroyAllWindows()

if __name__ == '__main__':

    # read camera intrinsic parameters.
    cmtx, dist = read_camera_parameters()

    input_source = 'test.mp4'
    if len(sys.argv) > 1:
        input_source = int(sys.argv[1])

    show_axes(cmtx, dist, input_source)
    cv.waitKey(0)

    # Notes:
    # cmtx = camera matrix
    # dist = distortion parameters
    # points = qr-code points
    # ret =
    # rvec = output rotation vector
    # tvec = output translation vector
    # rMatrix = rotation matrix
    # homoMatrix = homogenous matrix
    # pose = roll yaw pitch in deg
    # cv.projectPoints
    # points = Output array of image points
    # jac = Jacobian