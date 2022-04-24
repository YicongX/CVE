# More information about the RoboDK API here:
# https://robodk.com/doc/en/RoboDK-API.html
from robolink import *    # API to communicate with RoboDK
from robodk import *      # robodk robotics toolbox
import cv2 as cv
import numpy as np
import transforms3d.euler as eul

def read_camera_parameters(filepath = 'C:/Users/shaob/Desktop/CVE/intrinsicParameters/'):
    cmtx = np.loadtxt(filepath + 'oneEyeCameraMatrixPix.txt')
    dist = np.loadtxt(filepath + 'oneEyeCameraDistortionPix.txt')
    return cmtx, dist


def get_qr_coords(cmtx, dist, points):
    # Selected coordinate points for each corner of QR code.
    w = 3.65
    qr_edges = np.array([[0,0,0],
                         [0,1,0],
                         [1,1,0],
                         [1,0,0]], dtype = 'float32').reshape((4,1,3)) * w

    # determine the orientation of QR code coordinate system with respect to camera coorindate system.
    ret, rvec, tvec = cv.solvePnP(qr_edges, points, cmtx, dist) # estimate the orientation of a 3D object in a 2D image.
    rotationMatrix = cv.Rodrigues(rvec)[0]
    homoMatrix = np.hstack((rotationMatrix, tvec))
    homoMatrix = np.vstack((homoMatrix, np.array([0, 0, 0, 1])))
    euler = np.degrees(eul.mat2euler(rotationMatrix, axes='sxyz')) # xyz-euler angles
    rpy = np.degrees(eul.mat2euler(rotationMatrix, axes='szxy')) # zxy-roll, pitch & yaw
    camPosition = -np.matrix(rotationMatrix).T * np.matrix(tvec)

    # Define unit xyz axes. These are then projected to camera view using the rotation matrix and translation vector.
    unitv_points = np.array([[0,0,0], [1,0,0], [0,1,0], [0,0,1]], dtype = 'float32').reshape((4,1,3)) * w
    if ret:
        points, jac = cv.projectPoints(unitv_points, rvec, tvec, cmtx, dist)
        
        print("Homogeneous Transformation Matrix:")
        print(homoMatrix, '\n')
        #print("Euler Angles:")
        #print(euler, '\n')
        #print("[Yaw, Pitch, Roll]:")
        #print(rpy, '\n')
        # print("Camera Position:")
        # print(camPosition, '\n')
        return points, np.linalg.inv(homoMatrix)

    # return empty arrays if rotation and translation values not found
    else: return []


def show_axes(cmtx, dist, img):

    qr = cv.QRCodeDetector()
    ret_qr, points = qr.detect(img)

    if ret_qr:
        # axis point projects 3D coordinate points to a 2D plane
        axis_points, objPose = get_qr_coords(cmtx, dist, points)  # points	Output vector of vertices of the minimum-area quadrangle containing the code.

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
        return objPose
    else:
        cv.imshow('frame', img)
        return []

    

# Any interaction with RoboDK must be done through RDK:
RDK = Robolink()

# Select a robot (popup is displayed if more than one robot is available)
robot = RDK.ItemUserPick('Select a robot', ITEM_TYPE_ROBOT)
if not robot.Valid():
    raise Exception('No robot selected or available')

    
# get the current position of the TCP with respect to the reference frame:
# (4x4 matrix representing position and orientation)
target_ref = robot.Pose()
pos_ref = target_ref.Pos()
print("Drawing a polygon around the target: ")
print(Pose_2_TxyzRxyz(target_ref))


# move the robot to the first point:
robot.MoveJ(target_ref)

# It is important to provide the reference frame and the tool frames when generating programs offline
robot.setPoseFrame(robot.PoseFrame())
robot.setPoseTool(robot.PoseTool())
robot.setZoneData(10) # Set the rounding parameter (Also known as: CNT, APO/C_DIS, ZoneData, Blending radius, cornering, ...)
robot.setSpeed(200) # Set linear speed in mm/s

# Set the number of sides of the polygon:
n_sides = 6
R = 100

# make a hexagon around reference target:
for i in range(n_sides+1):
    ang = i*2*pi/n_sides #angle: 0, 60, 120, ...

    #-----------------------------
    # Movement relative to the reference frame
    # Create a copy of the target
    target_i = Mat(target_ref)
    pos_i = target_i.Pos()
    pos_i[0] = pos_i[0] + R*cos(ang)
    pos_i[1] = pos_i[1] + R*sin(ang)
    target_i.setPos(pos_i)
    print("Moving to target %i: angle %.1f" % (i, ang*180/pi))
    print(str(Pose_2_TxyzRxyz(target_i)))
    robot.MoveL(target_i)
    
    #-----------------------------
    # Post multiply: relative to the tool
    #target_i = target_ref * rotz(ang) * transl(R,0,0) * rotz(-ang)
    #robot.MoveL(target_i)

# move back to the center, then home:
robot.MoveL(target_ref)

print('Done')


if __name__ == '__main__':

    # read camera intrinsic parameters.
    cmtx, dist = read_camera_parameters()

    cam = 0 # 1 for external webcam, 0 for internal cam
    cap = cv.VideoCapture(cam)
    if not cap: print("!!!Failed VideoCapture: invalid camera source!!!")

    while(True):
        # capture frame-by-frame
        ret, current_frame = cap.read()
        if type(current_frame) == type(None):
            print("!!!Couldn't read frame!!!")
            break

        QR_pose = show_axes(cmtx, dist, current_frame)
        # print(QR_pose)
        print(type(target_ref))

        # if the `q` key was pressed, break from the loop
        key = cv.waitKey(1) & 0xFF
        if key == ord("q"): break

    # release the capture
    cap.release()
    cv.destroyAllWindows()



