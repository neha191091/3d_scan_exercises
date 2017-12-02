#pragma once
#include "SimpleMesh.h"

class ProcrustesAligner {
public:
	Matrix4f estimatePose(const std::vector<Vector3f>& sourcePoints, const std::vector<Vector3f>& targetPoints) {
		ASSERT(sourcePoints.size() == targetPoints.size() && "The number of source and target points should be the same, since every source point is matched with corresponding target point.");

		// We estimate the pose between source and target points using Procrustes algorithm.
		// Our shapes have the same scale, therefore we don't estimate scale. We estimated rotation and translation
		// from source points to target points.

		auto sourceMean = computeMean(sourcePoints);
		auto targetMean = computeMean(targetPoints);
		
		Matrix3f rotation = estimateRotation(sourcePoints, sourceMean, targetPoints, targetMean);
		Vector3f translation = computeTranslation(sourceMean, targetMean, rotation);

		Matrix4f estimatedPose = Matrix4f::Identity();
		estimatedPose.block(0, 0, 3, 3) = rotation;
		estimatedPose.block(0, 3, 3, 1) = translation;

		return estimatedPose;
	}

private:
	Vector3f computeMean(const std::vector<Vector3f>& points) {
		// TODO: Compute the mean of input points.

		Vector3f mean = Vector3f::Zero();
		for(int i=0; i<points.size(); i++){
			mean.x() += points[i].x();
			mean.y() += points[i].y();
			mean.z() += points[i].z();
		}
		mean.x() /= points.size();
		mean.y() /= points.size();
		mean.z() /= points.size();
		return mean;

	}

	Matrix3f estimateRotation(const std::vector<Vector3f>& sourcePoints, const Vector3f& sourceMean, const std::vector<Vector3f>& targetPoints, const Vector3f& targetMean) {
		// TODO: Estimate the rotation from source to target points, following the Procrustes algorithm. 
		// To compute the singular value decomposition you can use JacobiSVD() from Eigen.
		Matrix3f rotation = Matrix3f::Identity();
		MatrixXf X(sourcePoints.size(), 3);
		MatrixXf _X(targetPoints.size(), 3);
		for(int i=0; i<sourcePoints.size(); i++){
			Vector3f normalizedPoint = sourcePoints[i] - sourceMean;
			X(i,0) = normalizedPoint.x();
			X(i,1) = normalizedPoint.y();
			X(i,2) = normalizedPoint.z();
		}

		for(int i=0; i<targetPoints.size(); i++){
			Vector3f normalizedPoint = targetPoints[i] - targetMean;
			_X(i,0) = normalizedPoint.x();
			_X(i,1) = normalizedPoint.y();
			_X(i,2) = normalizedPoint.z();
		}
		
		Matrix3f m = X.transpose()*_X;
		JacobiSVD<MatrixXf> svd(m);
		rotation = svd.matrixU()*svd.matrixV().transpose();
		return rotation;
	}

	Vector3f computeTranslation(const Vector3f& sourceMean, const Vector3f& targetMean, const Matrix3f& rotation) {
		// TODO: Compute the translation vector from source to target opints.
		Vector3f translation = Vector3f::Zero();
		translation = sourceMean - targetMean;
		return translation;
	}
};
