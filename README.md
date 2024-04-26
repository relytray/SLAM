# vSLAM Source and Breakdown
This repo contains matlab code for the vSLAM example available at https://www.mathworks.com/help/vision/ug/monocular-visual-simultaneous-localization-and-mapping.html.
The code can be broken down as follows:
1. Download a sequence of images into a temporary folder
2. Turn that image sequence into an imds object
3. Initialize the map by comparing the first frame to following frames to create the first two keyframes
4. Create a bag of features for the ORB algorithm and store this for later use
     As implemented, this section of code can be commented out after the first run in order to save run time
5. Load the bag of features and initialize the place recognition database
6. Determine initial pose and location of points from the two initial keyframes
7. Display feature points for the current frame, visualize motion, and plot the estimated trajectory
8. Track motion by inserting new keyframes until the loop is closed
9. The main loop extracts ORB features and estimates the current camera pose, then projects points from the last key frame into the current frame, searches for feature correspondence, adjusts the camera pose to match the 3-D to 2-D correspondence, then determines if the current frame is a new key frame
10. After each key frame, local mapping is performed, creating new points in the point cload through triangulation
11. The database is queried for image similarity and if they are sufficiantly similar, the loop is closed and the map is updated.

# Personal vSLAM Ideal Method
The overall method of the example seems like a decent approach. Given the time and resources, I would rather implement vSLAM using object detection and segmentation, along with a classifier, ideally using either a library of models to compare against and place in the virtual landscape, or using some kind of building block method to make the system adaptive to a variety of environments that may not have their components in the library.

# vSLAM with my Dataset
This project did not turn out very well. Using video capture with my phone, I had issues with creating a sequence of images from a video, then issues with the video codec being too new, then with the aspect ratio giving odd results. Finding the information online and calculating the intrinsic matrix was laborious and didn't yield useful results, presumably because of the processing between the camera and my eventual video - though a camera calibration would have been more direct and effective. After many hours, several datasets and dozens of tweaks to different settings I had to accept that I was not able to get a satisfactory result within the week allotted to finish the project. My dataset was a video of the corner of my room with my desk, the video loops well but the SLAM algorithm seems to think I was steadily getting closer to something. I wish there was some resource to be able to reach out to in order to troubleshoot better, that I had more experience actually doing the things that need to be done in order to get this working, or more time, as this is an area that is extremely interesting to me.
