// ObjectSfM - Object Based Structure-from-Motion.
// Copyright (C) 2018  Ohio State University, CEGE, GDA group
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef OBJECTSFM_SYSTEM_INCREMENTAL_VO_MONOCULAR_SYSTEM_H_
#define OBJECTSFM_SYSTEM_INCREMENTAL_VO_MONOCULAR_SYSTEM_H_

#include "basic_structs.h"
#include "graph.h"
#include "database.h"
#include "camera.h"
#include "structure.h"
#include "optimizer.h"
#include "tracker.h"

namespace objectsfm {

// the system is designed to handle sfm from internet images, in which not
// all the focal lengths are known
class MonocularVO
{
public:
	MonocularVO();
	~MonocularVO();

	MonocularVOOptions options_;

	void Run();

	bool ReadCamera();

	bool FindSeedPairThenReconstruct(cv::Mat &frame);

	void LocalizeFrame(cv::Mat &frame, RTPose &pose_absolute,
		std::vector<int> &pts3d_ids, std::vector<Eigen::Vector2d> &pts2d);

	void AddKeyFrme(cv::Mat &frame, RTPose &pose,
		std::vector<int> &pts3d_ids, std::vector<Eigen::Vector2d> &pts2d);

	void FullBundleAdjustment();

	void PartialBundleAdjustment();

	// immutable all the cameras and points for the next partial bundle adjustment 
	void ImmutableCamsPoints();

	// mutable all the cameras and points for the full bundle adjustment 
	void MutableCamsPoints();

	void RemovePointOutliers();

	void MatchingKeyFrames(std::vector<std::vector<std::pair<int, int>>> &matches_all);

	void WriteCameraPointsOut(std::string path);

private:
	Database db_;  // data base to store key frames
	Graph graph_;  // graph to match key frames
	double ppx, ppy;

	cv::Size zoom_size_;
	bool cam_calibrated_ = false;

	BundleAdjustOptions bundle_full_options_;
	BundleAdjustOptions bundle_partial_options_;
	bool found_seed_;

	cv::VideoCapture *video_cap_; // video capturer
	std::vector<CameraModel*> cam_models_;      // camera model
	std::vector<Camera*> cams_;   // cameras
	std::vector<Point3D*> pts_;   // structures
	std::vector<RTPose> poses;

	//
	std::vector<Tracker*> trackers_;
	int th_num_tracked_total_ = 1000;
	bool is_sys_initialized_ = false;
};

}  // namespace objectsfm

#endif  // OBJECTSFM_SYSTEM_INCREMENTAL_VO_MONOCULAR_SYSTEM_H_
