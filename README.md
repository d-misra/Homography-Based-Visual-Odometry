# Homography-Based-Visual-Odometry

The goal of this lab is to perform Augmented Reality (AR) from a sequence of images. The only information that we have is that the camera is observing a mostly planar environment, where we want to display the 3D model of a robot.

AR needs a 3D transform between the camera and the world frame. By doing homography-based visual odometry (VO), we can do this pose estimation with regards to a reference 3D plane. The goal is thus to load the video file, and go through the images with all the necessary steps to perform AR.

OpenCV and ViSP libraries are used for this task.
