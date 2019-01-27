#pragma once
#ifndef H_RELOCALIZER_H
#define H_RELOCALIZER_H

#include <pcl/filters/extract_indices.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>

#include <pcl/correspondence.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/common/transforms.h>

#include <pcl/filters/approximate_voxel_grid.h>

#include <vector>
#include <string>

using namespace std;

typedef pcl::FPFHSignature33 FEATURE_TYPE;

class CFPFH
{
public:
	pcl::search::KdTree<pcl::PointXYZ>::Ptr m_searchMethod;
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud;
	pcl::PointCloud<pcl::Normal>::Ptr m_normal;
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr m_cloud_feature;
	boost::shared_ptr<std::vector<int> > m_vkeypoint_indices;
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud_keypoints;
	double m_dSampleRadius;
	double m_dNormalRadius;
	double m_dFeatureRadius;

	CFPFH()
	{
		m_dSampleRadius  = 2.0;
		m_dNormalRadius  = 0.5;
		m_dFeatureRadius = 1.0;
	}
	~CFPFH()
	{

	}
	void setRadius(double dSampleRadius, double dNormalRadius, double dFeatureRadius)
	{
		m_dSampleRadius = dSampleRadius;
		m_dNormalRadius = dNormalRadius;
		m_dFeatureRadius = dFeatureRadius;
	}
	void sampleCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
		pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_subsampled,
		double dGridSize)
	{
		if (dGridSize < 0)   // do not subsampling.
		{
			cloud_subsampled = cloud;
			return;
		}
		pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
		approximate_voxel_filter.setLeafSize(dGridSize, dGridSize, dGridSize);
		approximate_voxel_filter.setInputCloud(cloud);
		approximate_voxel_filter.filter(*cloud_subsampled);
		cloud_subsampled->sensor_origin_ = cloud->sensor_origin_;
		cloud_subsampled->sensor_orientation_ = cloud->sensor_orientation_;
	}

	void computeNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
		const pcl::search::KdTree<pcl::PointXYZ>::Ptr search_method,
		const double dRadius,
		pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals)
	{
		cloud_normals.reset(new pcl::PointCloud<pcl::Normal>());
		pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normal_estimation_filter;
		normal_estimation_filter.setInputCloud(cloud);
		normal_estimation_filter.setSearchMethod(search_method);
		normal_estimation_filter.setRadiusSearch(dRadius);
		normal_estimation_filter.setNumberOfThreads(8);
		// normal_estimation_filter.setKSearch(20);
		normal_estimation_filter.compute(*cloud_normals);
	}

	void decoupleCloud(const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_normals,
		pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::Normal>::Ptr& normal)
	{
		cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
		normal.reset(new pcl::PointCloud<pcl::Normal>);
		pcl::PointNormal pt;
		pcl::PointXYZ pt_a;
		pcl::Normal pt_b;
		for (int i = 0; i < cloud_normals->points.size(); i++)
		{
			pt = cloud_normals->points[i];
			pt_a.x = pt.x;
			pt_a.y = pt.y;
			pt_a.z = pt.z;
			pt_b.normal_x = pt.normal_x;
			pt_b.normal_y = pt.normal_y;
			pt_b.normal_z = pt.normal_z;
			cloud->points.push_back(pt_a);
			normal->points.push_back(pt_b);
		}
		cloud->width = 1;
		cloud->height = cloud->points.size();
		normal->width = 1;
		normal->height = normal->points.size();
	}

	void setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZ>);
		// pcl::copyPointCloud(input_cloud, m_cloud); 
		pcl::search::KdTree<pcl::PointXYZ>::Ptr search_method(new pcl::search::KdTree<pcl::PointXYZ>);
		search_method->setInputCloud(m_cloud);
		m_searchMethod = search_method;
		computeNormals(m_cloud, m_searchMethod, m_dNormalRadius, m_normal);
	}
	void setInputCloud(pcl::PointCloud<pcl::PointNormal>::Ptr input_cloud)
	{
		decoupleCloud(input_cloud, m_cloud, m_normal);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr search_method(new pcl::search::KdTree<pcl::PointXYZ>);
		search_method->setInputCloud(m_cloud);
		m_searchMethod = search_method;
	}
	void computeKeypoints()
	{
		m_vkeypoint_indices.reset(new std::vector<int>());
		m_cloud_keypoints.reset(new pcl::PointCloud<pcl::PointXYZ>());
		//// key_points calculation.
		sampleCloud(m_cloud, m_cloud_keypoints, m_dSampleRadius);
		char tmpChar[200] = " ";
		sprintf_s(tmpChar, "%s", "uniform sampling");

		//// key_points index calculation.
		vector<int> pointIdxNKNSearch;
		vector<float> pointNKNSquaredDistance;
		pcl::PointXYZ querryPt, searchPoint;
		int nLen = m_cloud_keypoints->points.size();
		for (size_t i = 0; i < nLen; i++)
		{
			searchPoint = m_cloud_keypoints->points[i];
			m_searchMethod->nearestKSearch(searchPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
			m_vkeypoint_indices->push_back(pointIdxNKNSearch[0]);
		}
	}
	void computeFPFH()
	{
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr features(new pcl::PointCloud<pcl::FPFHSignature33>);
		features.reset(new pcl::PointCloud<pcl::FPFHSignature33>());
		//// calculate descriptor.
		pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33>::Ptr fpfh_estimation(new pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33>);
		// fpfh_estimation->setSearchSurface(cloud);
		fpfh_estimation->setInputCloud(m_cloud);
		fpfh_estimation->setInputNormals(m_normal);
		fpfh_estimation->setSearchMethod(m_searchMethod);
		fpfh_estimation->setRadiusSearch(m_dFeatureRadius);
		fpfh_estimation->setNumberOfThreads(8);

		fpfh_estimation->setIndices(m_vkeypoint_indices);
		fpfh_estimation->compute(*features);

		m_cloud_feature.swap(features); 
	}

	void detectZeroFeatures(const pcl::PointCloud<FEATURE_TYPE>::Ptr& cloud_features, pcl::IndicesPtr& indices)
	{
		indices->clear();
		FEATURE_TYPE tmpFeature;
		double dDist2 = 0.0;
		for (size_t id = 0; id < cloud_features->points.size(); id++)
		{
			tmpFeature = cloud_features->points[id];
			dDist2 = 0.0;
			int nLen = tmpFeature.descriptorSize();
			for (size_t i = 0; i < nLen; i++)
			{
				// dDist2 += pow(tmpFeature.descriptor[i], 2);
				dDist2 += pow(tmpFeature.histogram[i], 2);
			}
			if (dDist2 >= 1e-4)
			{
				indices->push_back(id);
			}
		}
	}

	void featureFilter(const pcl::PointCloud<FEATURE_TYPE>::Ptr& cloud_descriptors,
		const pcl::IndicesPtr& indices,
		pcl::PointCloud<FEATURE_TYPE>::Ptr& cloud_descriptors_new)
	{
		FEATURE_TYPE descriptor;
		for (int i = 0; i < indices->size(); i++)
		{
			int nId = (*indices).at(i);
			descriptor = cloud_descriptors->points[nId];
			cloud_descriptors_new->push_back(descriptor);
		}
		cloud_descriptors_new->width = cloud_descriptors_new->points.size();
		cloud_descriptors_new->height = 1;
	}

	void deleteInvalidFeature()
	{
		pcl::IndicesPtr indices(new vector<int>());
		detectZeroFeatures(m_cloud_feature, indices);
		if (m_cloud_feature->size() != indices->size())
		{
			if (indices->size() == 0)
			{
				printf_s("Error: no valid descriptors!\n");
				exit(-1);
			}
			pcl::ExtractIndices<pcl::PointXYZ>::Ptr extractor_pointCloud(new pcl::ExtractIndices<pcl::PointXYZ>);
			extractor_pointCloud->setInputCloud(m_cloud_keypoints);
			extractor_pointCloud->setIndices(indices);
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZ>);
			extractor_pointCloud->filter(*cloud_tmp);
			m_cloud_keypoints.swap(cloud_tmp);

			pcl::PointCloud<FEATURE_TYPE>::Ptr cloud_feature_tmp(new pcl::PointCloud<FEATURE_TYPE>);
			featureFilter(m_cloud_feature, indices, cloud_feature_tmp);
			m_cloud_feature.swap(cloud_feature_tmp);
		}
		PCL_INFO("compute descriptors done, keypoints = %05d/%05d\n",
			m_cloud_feature->points.size(), m_cloud->points.size());
	}
	void compute()
	{
		computeKeypoints();
		computeFPFH();
		deleteInvalidFeature();
	}
};

Eigen::Matrix4f EstimateCorrespondence(CFPFH& MovFPFH, CFPFH& RefFPFH, double dDistThr)
{
	pcl::registration::CorrespondenceEstimation<FEATURE_TYPE, FEATURE_TYPE> est;
	est.setInputSource(MovFPFH.m_cloud_feature);
	est.setInputTarget(RefFPFH.m_cloud_feature);
	pcl::CorrespondencesPtr corres(new pcl::Correspondences);
	est.determineCorrespondences(*corres);

	pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> rejector_sac;
	rejector_sac.setInputSource(MovFPFH.m_cloud_keypoints);
	rejector_sac.setInputTarget(RefFPFH.m_cloud_keypoints);
	rejector_sac.setInlierThreshold(dDistThr); // the unit of distance is meter, not the squared distance, original is 2.5
	rejector_sac.setMaximumIterations(1000);  // original is 1000000
	rejector_sac.setRefineModel(true);
	rejector_sac.setInputCorrespondences(corres);
	pcl::CorrespondencesPtr correspondences_filtered(new pcl::Correspondences);
	rejector_sac.getCorrespondences(*correspondences_filtered);
	corres.swap(correspondences_filtered);
	printf_s("Determine correspondences via ransac: %03d vs %03d\n",
		corres->size(), correspondences_filtered->size());
	//// refined results.
	Eigen::Matrix4f Tf0 = Eigen::Matrix4f::Identity();
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> transformation_estimation;
	transformation_estimation.estimateRigidTransformation(*MovFPFH.m_cloud_keypoints, *RefFPFH.m_cloud_keypoints,
		*corres, Tf0);

	return Tf0;
}

class CReLocalizer
{
public:
	CReLocalizer()
	{

	}
	~CReLocalizer()
	{

	}
	void read()
	{

	}
	void setInputPair(string& sMovFile, string& sRefFile)
	{

	}
};
#endif // !H_FEATURES_H
