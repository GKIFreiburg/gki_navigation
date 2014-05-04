This package provides software to compute and update Euclidean distance maps (DM) and Euclidean Voronoi diagrams (GVD) on 2D grid maps.
The program is initialized with a binary occupancy grid map and computes the corresponding DM and GVD. When provided with points that mark newly occupied or freed cells, the DM and GVD can be updated efficiently to reflect the changes in the environment.

to build type: rosmake dynamic_voronoi
to test type: roscd dynamicvoronoi; bin/example data/testmap.pgm
The example program generates a number of output images in the current directory after performing updates with random obstacles.

For questions and comments, please contact lau@informatik.uni-freiburg.de
