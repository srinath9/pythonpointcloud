'''
Example script illustrating plotting of PLY data using Mayavi.  Mayavi
is not a dependency of plyfile, but you will need to install it in order
to run this script.  Failing to do so will immediately result in
ImportError.

'''

from argparse import ArgumentParser

import numpy
from mayavi import mlab

from plyfile import PlyData

import pcl

p = pcl.PointCloud()
# p = pcl.OctreePointCloud(1)

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
    print vertex

    (x, y, z) = (vertex[t] for t in ('x', 'y', 'z'))
    print "x "
    print x

    # mlab.points3d(x, y, z, color=(1, 1, 1), mode='point')

    if 'face' in ply:
        tri_idx = ply['face']['vertex_indices']
        print tri_idx
        idx_dtype = tri_idx[0].dtype
        print "idx_dtype" , idx_dtype
        triangles = numpy.fromiter(tri_idx, [('data', idx_dtype, (3,))],
                                   count=len(tri_idx))['data']
        k = triangles.byteswap()
        # p.from_array(numpy.array([[1,2,3],[3,4,5]], dtype=numpy.float32))
        # print len(numpy.array([[1,2,3],[3,4,5]], dtype=numpy.float32))
        # print len(float(triangles.astype('<i2')))
        p.from_array((triangles.byteswap().newbyteorder().astype('<f4')))
        fil = p.make_statistical_outlier_filter()
        # fil = p.make_segmenter_normals(ksearch=50)
        # # seg = p.make_segmenter_normals(ksearch=50)
        # fil = p.make_passthrough_filter()
        # fil.set_filter_field_name("z")
        # fil.set_filter_limits(0, 1.5)
        cloud_filtered = fil.filter()
        fil = p.make_statistical_outlier_filter()
        fil.set_mean_k(20)
        fil.set_std_dev_mul_thresh(0.1)

        # seg = cloud_filtered.make_segmenter_normals(ksearch=50)
        # seg.set_optimize_coefficients(True)
        # seg.set_model_type(pcl.SACMODEL_NORMAL_PLANE)
        # seg.set_normal_distance_weight(0.1)
        # seg.set_method_type(pcl.SAC_RANSAC)
        # seg.set_max_iterations(100)
        # print "segmentation" ,seg
        # seg.set_distance_threshold(0.03)
        
        # indices, model = seg.segment()
        # cloud_cyl = cloud_filtered.extract(indices, negative=True)
        # cloud_cylinder = cloud_cyl.extract(indices, negative=False)
        # # print indices, model
        pc_1 = pcl.PointCloud()
        pc_1 = p
        pc_2 = pcl.PointCloud()
        pc_2 = p
        kd = pcl.KdTreeFLANN(pc_1)
        print('pc_1:')
        # print(points_1)
        # print('\npc_2:')
        # print(points_2)
        # print('\n')

        pc_1 = pcl.PointCloud(p)
        pc_2 = pcl.PointCloud(p)
        kd = pc_1.make_kdtree_flann()

        



        # fil.set_mean_k (50)
        # fil.set_std_dev_mul_thresh (1.0)

  #       p = pcl.PointCloud()
  #       p =triangles
		# fil = ply.make_statistical_outlier_filter()
		# fil.set_mean_k (50)
		# fil.set_std_dev_mul_thresh (1.0)
		# fil.filter().to_file("inliers.pcd")

    print x
    print numpy.shape(x)
    print numpy.shape(triangles)
        # mlab.triangular_mesh(x, y, z, fil.filter(), color=(0.5, 0.3, 0.3), opacity=0.5,colormap='cool',representation='wireframe')
        # mlab.triangular_mesh(x, y, z, fil.filter(),color=(1, 0.7, 0.4), opacity=0.5)


main()
