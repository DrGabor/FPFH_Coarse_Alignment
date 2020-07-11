/*
 * @Descripttion:
 * @version:
 * @Author: Di Wang
 * @Date: 2020-07-11 16:37:12
 * @LastEditors: max.zhong
 * @LastEditTime: 2020-07-11 17:13:05
 */

#include "reLocalizer.h"

// #include <direct.h>  // this is for mkdir()
// #include <io.h>      // this is for access()
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <boost/filesystem.hpp>
#include <fstream>
#include <iostream>
int main()

{
  pcl::PLYReader plyReader;
  string sDataRoot = "../data/";

  string sFileName_mov = sDataRoot + "raw_000000.ply";
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_mov(
      new pcl::PointCloud<pcl::PointXYZ>);
  plyReader.read(sFileName_mov, *cloud_mov);
  CFPFH MovFPFH;
  MovFPFH.setInputCloud(cloud_mov);
  MovFPFH.compute();

  string sFileName_ref = sDataRoot + "raw_000000.ply";
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ref(
      new pcl::PointCloud<pcl::PointXYZ>);
  plyReader.read(sFileName_ref, *cloud_ref);
  CFPFH RefFPFH;
  RefFPFH.setInputCloud(cloud_ref);
  RefFPFH.compute();

  //// compute coarse transformation.
  double dDistThr = 1.0;
  Eigen::Matrix4f Tf0 = EstimateCorrespondence(MovFPFH, RefFPFH, dDistThr);

  //// save the results.
  // char cSaveFolder[200] = "output\\";
  boost::filesystem::path cSaveFolder("output/");
  if (!boost::filesystem::exists(cSaveFolder)) {
    boost::filesystem::create_directories(cSaveFolder);
  }
  char cDataDir[200] = " ";
  sprintf(cDataDir, "%s%s", cSaveFolder.string().c_str(), "coarse_results.txt");
  ofstream ofData(cDataDir);
  char tmpChar[200] = " ";
  sprintf(tmpChar, "%s",
          "qx        qy        qz        qw        tx        ty        tz\n");
  ofData << tmpChar;

  Eigen::Quaternionf q;
  Eigen::Vector4f coeff;
  q = Tf0.block<3, 3>(0, 0);
  coeff = q.coeffs();
  Eigen::Vector3f t0 = Tf0.block<3, 1>(0, 3);
  sprintf(tmpChar, "%+2.6f %+2.6f %+2.6f %+2.6f %+3.6f %+3.6f %+3.6f\n",
          coeff(0), coeff(1), coeff(2), coeff(3), t0(0), t0(1), t0(2));
  ofData << tmpChar;
  ofData.close();

  pcl::PCDWriter pcdWriter;

  sprintf(cDataDir, "%s%s", cSaveFolder.string().c_str(), "cloud_mov.pcd");
  pcdWriter.write<pcl::PointXYZ>(cDataDir, *MovFPFH.m_cloud);

  sprintf(cDataDir, "%s%s", cSaveFolder.string().c_str(), "cloud_normal_mov.pcd");
  pcdWriter.write<pcl::Normal>(cDataDir, *MovFPFH.m_cloud_normal);

  sprintf(cDataDir, "%s%s", cSaveFolder.string().c_str(), "keypoints_mov.pcd");
  pcdWriter.write<pcl::PointXYZ>(cDataDir, *MovFPFH.m_cloud_keypoints);

  sprintf(cDataDir, "%s%s", cSaveFolder.string().c_str(), "descriptor_mov.pcd");
  pcdWriter.write<FEATURE_TYPE>(cDataDir, *MovFPFH.m_cloud_feature);

  sprintf(cDataDir, "%s%s", cSaveFolder.string().c_str(), "cloud_ref.pcd");
  pcdWriter.write<pcl::PointXYZ>(cDataDir, *RefFPFH.m_cloud);

  sprintf(cDataDir, "%s%s", cSaveFolder.string().c_str(), "cloud_normal_ref.pcd");
  pcdWriter.write<pcl::Normal>(cDataDir, *RefFPFH.m_cloud_normal);

  sprintf(cDataDir, "%s%s", cSaveFolder.string().c_str(), "keypoints_ref.pcd");
  pcdWriter.write<pcl::PointXYZ>(cDataDir, *RefFPFH.m_cloud_keypoints);

  sprintf(cDataDir, "%s%s", cSaveFolder.string().c_str(), "descriptor_ref.pcd");
  pcdWriter.write<FEATURE_TYPE>(cDataDir, *RefFPFH.m_cloud_feature);

  cout << "Finished!\n";
  return 0;
}
