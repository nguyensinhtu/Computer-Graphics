#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <iostream>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/ply_io.h>
#include <iostream>
#include <string>
#include <pcl/surface/vtk_smoothing/vtk_mesh_quadric_decimation.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>

using namespace std;
using namespace pcl;

int main(int argc, char** argv)
{

	if (argc < 2) {
		cout << "Tham so dong lenh khong hop le, vui long nhap lai!!" << endl;
		return 0;
	}

	if (strcmp(argv[1], "1") == 0) {
		// doc file chua du diem point cloud
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PCLPointCloud2 cloud_blob;
	
		if (pcl::io::loadPCDFile(argv[2], cloud_blob) == -1) //* load the file error
		{
			return (-1);
		}
		//pcl::io::loadPCDFile(argv[2], cloud_blob);
		pcl::fromPCLPointCloud2(cloud_blob, *cloud);
		
		cout << "size of points cloud :  " << cloud->size() << endl;
		cout << "Greedy triangle projection algorithm!!" << endl;
		cout << "Waiting...!";

		// Normal estimation*
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
		tree->setInputCloud(cloud);
		n.setInputCloud(cloud);
		n.setSearchMethod(tree);
		n.setKSearch(20);
		n.compute(*normals);
		

		// hop diem xyz voi phap tuyen cua no
		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
		pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
		//* cloud_with_normals = cloud + normals

		// tao cay tim kiếm
		pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
		tree2->setInputCloud(cloud_with_normals);

		// khởi tạo đối tượng
		pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
		pcl::PolygonMesh triangles;

		// Set the maximum distance between connected points (maximum edge length)
		// thiết lập maximum distance giữa hai điểm(cạnh tam giác có độ dài lớn nhất
		gp3.setSearchRadius(20);

		// thiết lập những thông số cho thuật toán greedy triangle projection
		gp3.setMu(5);
		gp3.setMaximumNearestNeighbors(100);
		gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
		gp3.setMinimumAngle(M_PI / 36); // 10 degrees
		gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
		gp3.setNormalConsistency(false);

		// kết quả
		gp3.setInputCloud(cloud_with_normals);
		gp3.setSearchMethod(tree2);
		gp3.reconstruct(triangles);


		// them thông tin cho những vector
		std::vector<int> parts = gp3.getPartIDs();
		std::vector<int> states = gp3.getPointStates();

		//Viewer
		pcl::visualization::PCLVisualizer viewer("viewer");
		viewer.addPolygonMesh(triangles);
		viewer.spin();
		while (!viewer.wasStopped())
		{
			if (GetAsyncKeyState('A'))
				viewer.close();
		}

	}
	else if (strcmp(argv[1], "2") == 0){


		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PCLPointCloud2 cloud_blob;

		//pcl::io::loadPCDFile(argv[2], cloud_blob);
		// kiem tra doc file
		if (pcl::io::loadPCDFile(argv[2], cloud_blob) == -1) //* load the file error
		{
			return (-1);
		}
		pcl::fromPCLPointCloud2(cloud_blob, *cloud);
		cout << "size of points cloud : " << cloud->size() << endl;
		cout << "poisson algorithm!" << endl;
		cout << "waiting...!";
		NormalEstimationOMP<PointXYZ, Normal> Nor;
		// gán sô tiểu trình muốn có
		Nor.setNumberOfThreads(8);
		// lấy lại đám mây điểm đã được làm mịn từ bước trên (làm mịn bằng moving least square)
		Nor.setInputCloud(cloud);
		// thiết lập bán kính duyệt
		Nor.setRadiusSearch(10);
		Eigen::Vector4f centroid;
		compute3DCentroid(*cloud, centroid);
		Nor.setViewPoint(centroid[0], centroid[1], centroid[2]);
		PointCloud<Normal>::Ptr cloud_normals(new PointCloud<Normal>());
		// tính tập vector pháp tuyến cho đám mây điểm và lưu vào con trỏ cloud normals
		Nor.compute(*cloud_normals);
		for (size_t i = 0; i < cloud_normals->size(); ++i)
		{
			cloud_normals->points[i].normal_x *= -1;
			cloud_normals->points[i].normal_y *= -1;
			cloud_normals->points[i].normal_z *= -1;
		}
		PointCloud<PointNormal>::Ptr cloud_smoothed_normals(new PointCloud<PointNormal>());
		// đồng bộ đám mây điểm và đám mây vector pháp tuyến 
		concatenateFields(*cloud, *cloud_normals, *cloud_smoothed_normals);
		// đối tượng poisson
		Poisson<PointNormal> poisson;
		// max chiều sâu để tái xây dựng bề mặt
		poisson.setDepth(9);
		poisson.setInputCloud(cloud_smoothed_normals);
		PolygonMesh mesh;
		PolygonMesh out_mesh;

		// tái xây dựng và lưu vào đối tượng polygonmesh.
		poisson.reconstruct(mesh);

		//Viewer
		pcl::visualization::PCLVisualizer viewer("viewer");
		viewer.addPolygonMesh(mesh);
		viewer.spin();
		while (!viewer.wasStopped())
		{
			if (GetAsyncKeyState('A'))
				viewer.close();
		}
	}
	else {
		cout << "Khong co thuat toan nao duoc chon!!!" << endl;
	}
	

	return 0;
}