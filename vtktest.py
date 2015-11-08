from vtk_visualizer import *
import numpy as np
# Generate 3D points with random XY values and normals = [0,0,1]
pts_n = np.zeros((1000,6))
pts_n[:,:2] = np.random.rand(1000,2) * 100.0
pts_n[:,5] = 1 
 
# Plot points and normals
plothh(pts_n)