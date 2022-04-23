CVE Project Outline

+ QR code design:
  1. facet info
  2. dimension info
+ Standardized cube fabrication: 
  1. cube dimension
  2. print QR code
+ QR code recognition:
  1. recognize face ID
  2. get normal vectors
  3. get dimension scale
+ Camera calibration:
  1. get intrinsic matrix
  2. fix camera position
+ Pose estimation:
  1. use normal vectors to get the cube orientation
  2. use dimension scale and camera parameters to get the cube position



Class cube:

Pose3D cubePose;



Class QRCode:





