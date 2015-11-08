'''
Example script illustrating plotting of PLY data using Mayavi.  Mayavi
is not a dependency of plyfile, but you will need to install it in order
to run this script.  Failing to do so will immediately result in
ImportError.

'''

from argparse import ArgumentParser
from scipy.spatial import ConvexHull
import numpy
from mayavi import mlab

from plyfile import PlyData

import pcl

p = pcl.PointCloud()
p1 = pcl.OctreePointCloud(1)

def main():
    parser = ArgumentParser()
    parser.add_argument('ply_filename')

    args = parser.parse_args()

    
    plot(PlyData.read(args.ply_filename))
    # plot(p.from_file(args.ply_filename))
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

    # mlab.points3d(x, y, z, color=(1, 1, 1), mode='point')

    if 'face' in ply:
        tri_idx = ply['face']['vertex_indices']
        print type(tri_idx)
        idx_dtype = tri_idx[0].dtype
        print "idx_dtype" , idx_dtype
        triangles = numpy.fromiter(tri_idx, [('data', idx_dtype, (3,))],
                                   count=len(tri_idx))['data']
        k = triangles.byteswap()
        print triangles
        # p.from_array(numpy.array([[1,2,3],[3,4,5]], dtype=numpy.float32))
        p.from_array((triangles.byteswap().newbyteorder().astype('<f4')))
        # p1.add_points_from_input_cloud()
        p1.set_input_cloud(p)
        fil = p.make_statistical_outlier_filter()
        # cloud_filtered = fil.filter()
        # fil = p1.make_statistical_outlier_filter()
        fil.set_mean_k(20)
        fil.set_std_dev_mul_thresh(0.1)
        pointx = triangles[:,0]

       

        pc_1 = pcl.PointCloud()
        pc_1 = p
        pc_2 = pcl.PointCloud()
        pc_2 = p
        kd = pcl.KdTreeFLANN(pc_1)
        print('pc_1:')
        print type(triangles)
        # file1 = open("values.txt",'w')
        # file1.write(str(triangles[:]))

        # mlab.mesh(x,y,z)
        print numpy.shape(p)
        print numpy.shape(z)
        print numpy.shape(y)
        print numpy.shape(x)
        # mlab.contour3d(x, y, z, p)
        # mlab.triangular_mesh(x, y, z, fil.filter(),color=(1, 0.7, 0.4), opacity=0.5)

        arr = p.to_array()
        hull = ConvexHull(arr)
        simplices = numpy.column_stack((numpy.repeat(hull.vertices[0], hull.nsimplex),
                                     hull.simplices))
        tets = hull.points[simplices]
        vol = numpy.sum(tetrahedron_volume(tets[:, 0], tets[:, 1],
                                        tets[:, 2], tets[:, 3]))

        print("Points per unit volume: %.3g" % (len(arr) / vol))

# http://stackoverflow.com/a/24734583/166749
def tetrahedron_volume(a, b, c, d):
    return numpy.abs(numpy.einsum('ij,ij->i', a-d, numpy.cross(b-d, c-d))) / 6

main()
