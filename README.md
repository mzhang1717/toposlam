# Introduction

This project provides a vision-based topological
SLAM algorithm for place recognition.
It represents a 3D environment by a
graph-based topological map, which comprises nodes
representing previously visited locations and edges representing
the connectivity of these locations. Place recognition is performed
by the efficient the bag-of-word
image matching approach. A geometric validation procedure is
performed afterwards to verify matched images and obtain finer
pose estimation. The workflow of this algorithm is outlined as follows:
1) Capture a new frame from the camera set;
2) Detect CenSurE corner features and extract SIFT feature descriptors;
3) Find a given number of matching candidates via BoW
for the current frame;
4) Run geometric validation to determine the correct
matching frame;
5) If no matching frame is found in the map, this new
frame is added to the map as a new place.

# References
- [Speeded-up bag-of-words
algorithm for robot localisation through scene recognition](https://ieeexplore.ieee.org/abstract/document/4762067)
- [Scalable recognition with a vocabulary
tree](https://ieeexplore.ieee.org/abstract/document/1641018)