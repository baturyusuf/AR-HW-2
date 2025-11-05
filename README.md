AR-HW-2: Unity Application for Point-Cloud Alignment

This repository contains the source code and documentation for the second homework assignment of the CSE462/562 Augmented Reality course (Fall 2025). The objective of this assignment is to develop a Unity application that aligns two point-cloud data sets using rigid transformations and visualizes the results.

Overview

In this assignment, you will build a Unity application that reads two sets of 3D points from files, calculates a transformation that aligns the second point cloud to the first, and then displays the results using two different visualization methods.

Features

Load and Parse Point Clouds: Reads two sets of 3D points from files in the specified format.

Rigid Transformation: Registers the second point cloud to the first one using a rigid transformation (rotation and translation).

RANSAC & Three-Point Alignment: Utilizes RANSAC along with the three-point alignment method for robust matching between point clouds.

Visualization: Offers two visualization modes:

Original vs Aligned Points: Displays the original and aligned point clouds in different colors.

Transformed Points: Shows the second point cloud’s transformed position and its movement as a line.

Transformation Parameters: Displays the calculated transformation and scale parameters (if applicable) in the UI.

Grading

This project is worth 100 points. The grading criteria are based on the correct implementation of the Unity program and the working features as outlined.

Working Unity Program: 100 points

Submission: Submit a short video demonstrating the application in use (showing all features), and submit the source code (excluding libraries).

Example Files

Input point cloud files should be formatted as follows:
num_pts
x1 x2 x3 ...
y1 y2 y3 ...
z1 z2 z3 ...




https://github.com/user-attachments/assets/42a23eaa-3571-4989-8de2-650916efb8f8

