import pydriver,time
import numpy
from argparse import ArgumentParser
from mayavi import mlab

from plyfile import PlyData
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import pcl

p = pcl.PointCloud()
# p = pcl.OctreePointCloud(1)

def main():
    parser = ArgumentParser()
    parser.add_argument('ply_filename')

    args = parser.parse_args()
    # file1 = open(args.ply_filename,'r')

    
    # plot(PlyData.read())
    # plot(file1)
    plot(PlyData.read(args.ply_filename))
    # mlab.show()


def plot(ply):
    '''
    Plot vertices and triangles from a PlyData instance. Assumptions:
        `ply' has a 'vertex' element with 'x', 'y', and 'z'
            properties;

        `ply' has a 'face' element with an integral list property
            'vertex_indices', all of whose elements have length 3.

    '''
    vertex = ply['vertex']

    (x, y, z) = (vertex[t] for t in ('x', 'y', 'z'))
    (red, green, blue) = (vertex[t] for t in ('red', 'green', 'blue'))

    # x = []
    # z = []
    # y = []
    # for line in ply.readlines():
    # 	string = line.split()
    # 	if(string[0] == 'v'):
    # 		x.append(string[1])
    # 		y.append(string[2])
    # 		z.append(string[3])
    # 	elif(string[0] == 'f'):
    # 		break

	
		

    # mlab.points3d(x, y, z, color=(1, 1, 1), mode='point')

    if 'face' in ply:
        tri_idx = ply['face']['vertex_indices']
        print tri_idx
        idx_dtype = tri_idx[0].dtype
        print "idx_dtype" , idx_dtype
        triangles = numpy.fromiter(tri_idx, [('data', idx_dtype, (3,))],
                                   count=len(tri_idx))['data']

        
        arrx = numpy.array(x)
        arry = numpy.array(y)
        arrz = numpy.array(z)
        arrr = numpy.array(red)
        arrg = numpy.array(green)
        arrb = numpy.array(blue)
        print " number of points ", numpy.shape(arrz)
        # print "triangles ", triangles



        xyz= numpy.column_stack((arrx,arry,arrz))
        rgb= numpy.column_stack((arrr,arrg,arrb))
        # print "xyz ", numpy.shape(xyz)
        # print numpy.shape(triangles)
        # print "rgb " ,rgb
        k = triangles.byteswap().newbyteorder().astype('<f4')
        p.from_array(k)
        
        pc = pydriver.pcl.PCLHelper(xyz,rgb)
        haris = pc.getHarrisPoints(0.2)
        print "haris points ",haris.getCloudSize()
        pointx = haris.toArray(False,False)[0][:,0]
        pointy = haris.toArray(False,False)[0][:,1]
        pointz = haris.toArray(False,False)[0][:,2]

        # print "harris ", haris.toArray(False,False)
        harfile = open('harris.txt','w')

        # harfile.write(haris.toArray(False,False))
        addpoints = haris.toArray(False,False)
        # print xyz
        # print addpoints
        # print "xyz  ",xyz[:]
        # print numpy.shape(addpoints[0])
        # print "add point  ",addpoints[0]
        new =  numpy.concatenate((xyz, addpoints[0]), axis=0)
        print "new  ",new
        pc.setCameraPosition(([10,2,1],[1,1,1],[0.5,0.5,0.5]))
        pc.setDetectionsVisualization()

        pc.visualizeKeypoints(haris)
        # gd = pc.detectGroundPlane(0.5,1)
        # pc.visualizeDetections()
        # pydriver.detectors.detectors.Detector().
        # print gd
        # pc.visualize()
        # pc.setDetectionsVisualization()

    print "getting data"

    # mlab.mesh(numpy.array(x),numpy.array(y),numpy.array(z))
    # mlab.mesh(pointx,pointy,pointz)
    
        # mlab.triangular_mesh(x, y, z, haris.toArray(False,False) , color=(0.5, 0.3, 0.3), opacity=0.5,colormap='cool',representation='wireframe')

main()


		

