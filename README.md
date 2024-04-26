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

