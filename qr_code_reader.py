import cv2 as cv
import numpy as np
import transforms3d.euler as eul

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
    euler = np.degrees(eul.mat2euler(rotationMatrix, axes='sxyz')) # xyz-euler angles
    rpy = np.degrees(eul.mat2euler(rotationMatrix, axes='szxy')) # zxy-roll, pitch & yaw

    # Define unit xyz axes. These are then projected to camera view using the rotation matrix and translation vector.
    unitv_points = np.array([[0,0,0], [1,0,0], [0,1,0], [0,0,1]], dtype = 'float32').reshape((4,1,3))
    if ret:
        points, jac = cv.projectPoints(unitv_points, rvec, tvec, cmtx, dist)
        
        print("Homogeneous Transformation Matrix:")
        print(homoMatrix, '\n')
        print("Euler Angles:")
        print(euler, '\n')
        print("[Roll, Pitch, Yaw]:")
        print(rpy, '\n')
        
        return points, rvec, tvec, euler, homoMatrix

    # return empty arrays if rotation and translation values not found
    else: return [], [], [], [], []


def show_axes(cmtx, dist, img):

    qr = cv.QRCodeDetector()
    ret_qr, points = qr.detect(img)

    if ret_qr:
        # axis point projects 3D coordinate points to a 2D plane
        axis_points, rvec, tvec, pose, homoMatrix = get_qr_coords(cmtx, dist, points)  # points	Output vector of vertices of the minimum-area quadrangle containing the code.

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


if __name__ == '__main__':

    # read camera intrinsic parameters.
    cmtx, dist = read_camera_parameters()

    cam = 1 # 1 for external webcam, 0 for internal cam
    cap = cv.VideoCapture(cam)
    if not cap: print("!!!Failed VideoCapture: invalid camera source!!!")

    while(True):
        # capture frame-by-frame
        ret, current_frame = cap.read()
        if type(current_frame) == type(None):
            print("!!!Couldn't read frame!!!")
            break

        show_axes(cmtx, dist, current_frame)

        # if the `q` key was pressed, break from the loop
        key = cv.waitKey(1) & 0xFF
        if key == ord("q"): break

    # release the capture
    cap.release()
    cv.destroyAllWindows()

    # Notes:
    # cmtx = camera matrix
    # dist = distortion parameters
    # points = qr-code points
    # ret =
    # rvec = output rotation vector
    # tvec = output translation vector
    # rMatrix = rotation matrix
    # homoMatrix = homogenous matrix
    # rpy = roll yaw pitch in deg
    # cv.projectPoints
    # points = Output array of image points
    # jac = Jacobian
