// reLocalizer.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include <iostream>
#include <fstream>
#include <direct.h> // this is for mkdir()
#include <io.h>     // this is for access()

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/io.h>
#include "reLocalizer.h"
int main()

{
	pcl::PLYReader plyReader;
	string sDataRoot = "D:\\Data\\KITTI\\data_odometry_velodyne\\dataset\\sequences\\07\\velodyne\\PlyData\\0.5m\\"; 

	string sFileName_mov = sDataRoot + "raw_000000.ply"; 
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_mov(new pcl::PointCloud<pcl::PointXYZ>);
	plyReader.read(sFileName_mov, *cloud_mov);
	CFPFH MovFPFH; 
	MovFPFH.setInputCloud(cloud_mov); 
	MovFPFH.compute(); 
	
	string sFileName_ref = sDataRoot + "raw_000005.ply";
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ref(new pcl::PointCloud<pcl::PointXYZ>);
	plyReader.read(sFileName_ref, *cloud_ref);
	CFPFH RefFPFH;
	RefFPFH.setInputCloud(cloud_ref);
	RefFPFH.compute();

	//// compute coarse transformation. 
	double dDistThr = 1.0; 
	Eigen::Matrix4f Tf0 = EstimateCorrespondence(MovFPFH, RefFPFH, dDistThr); 

	//// save the results. 
	char cSaveFolder[200] = "output\\"; 
	if (0 != access(cSaveFolder, 0) )
	{
		mkdir(cSaveFolder);
	}
	char cDataDir[200] = " ";
	sprintf_s(cDataDir, "%s%s", cSaveFolder, "coarse_results.txt");
	ofstream ofData(cDataDir); 
	char tmpChar[200] = " ";
	sprintf_s(tmpChar, "%s", "qx        qy        qz        qw        tx        ty        tz\n");
	ofData << tmpChar;

	Eigen::Quaternionf q;
	Eigen::Vector4f coeff;
	q = Tf0.block<3, 3>(0, 0);
	coeff = q.coeffs();
	Eigen::Vector3f t0 = Tf0.block<3, 1>(0, 3);
	sprintf_s(tmpChar, "%+2.6f %+2.6f %+2.6f %+2.6f %+3.6f %+3.6f %+3.6f\n",
		coeff(0), coeff(1), coeff(2), coeff(3), t0(0), t0(1), t0(2) );
	ofData << tmpChar;
	ofData.close(); 

	pcl::PCDWriter pcdWriter;

	sprintf_s(cDataDir, "%s%s", cSaveFolder, "cloud_mov.pcd");
	pcdWriter.write<pcl::PointXYZ>(cDataDir, *MovFPFH.m_cloud);

	sprintf_s(cDataDir, "%s%s", cSaveFolder, "cloud_normal_mov.pcd");
	pcdWriter.write<pcl::Normal>(cDataDir, *MovFPFH.m_cloud_normal);

	sprintf_s(cDataDir, "%s%s", cSaveFolder, "keypoints_mov.pcd");
	pcdWriter.write<pcl::PointXYZ>(cDataDir, *MovFPFH.m_cloud_keypoints);

	sprintf_s(cDataDir, "%s%s", cSaveFolder, "descriptor_mov.pcd");
	pcdWriter.write<FEATURE_TYPE>(cDataDir,  *MovFPFH.m_cloud_feature);

	sprintf_s(cDataDir, "%s%s", cSaveFolder, "cloud_ref.pcd");
	pcdWriter.write<pcl::PointXYZ>(cDataDir, *RefFPFH.m_cloud);

	sprintf_s(cDataDir, "%s%s", cSaveFolder, "cloud_normal_ref.pcd");
	pcdWriter.write<pcl::Normal>(cDataDir, *RefFPFH.m_cloud_normal);

	sprintf_s(cDataDir, "%s%s", cSaveFolder, "keypoints_ref.pcd");
	pcdWriter.write<pcl::PointXYZ>(cDataDir, *RefFPFH.m_cloud_keypoints);

	sprintf_s(cDataDir, "%s%s", cSaveFolder, "descriptor_ref.pcd");
	pcdWriter.write<FEATURE_TYPE>(cDataDir, *RefFPFH.m_cloud_feature);

	cout << "Finished!\n"; 
    return 0;
}

