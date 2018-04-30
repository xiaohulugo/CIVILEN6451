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

#ifndef OBJECTSFM_TRACKER_H_
#define OBJECTSFM_TRACKER_H_

#include <vector>

#include <opencv2\opencv.hpp>   
#include <opencv2\highgui\highgui.hpp>

#include "database.h"
#include "basic_structs.h"
#include "camera.h"
#include "structure.h"

namespace objectsfm {

class Tracker
{
public:
	Tracker();
	~Tracker();

	void Initialize(cv::Mat &frame);

	void Track1Frame(cv::Mat &frame);

	void Track1Frame(cv::VideoCapture *video_cap, cv::Size zoom_size);

	void TrackNFrame(cv::VideoCapture *video_cap, cv::Size zoom_size, double ratio);

	void TrackNFrame(cv::VideoCapture *video_cap, cv::Size zoom_size, int N);

	bool IsTrackFailure();

	void AddFeaturePoints(cv::Point2f pt_tracked);

	void Add2d3dCorrespondence(int id_2d, Point3D* pt_3d);

	void Clear2d3dCorrespondence();

public:
	cv::Mat frame_pre_, frame_cur_;
	std::vector<cv::Point2f> pts_pre_, pts_cur_;

	cv::Mat frame_init_;
	std::vector<cv::Point2f> pts_init_;

	std::vector<int> status_;
	int num_tracked_2d_, num_tracked_3d_;
	int min_num_tracked_ = 30;
	bool is_track_failed_ = false;

	std::map<int, Point3D*> pts_3d_;
};

}  // namespace objectsfm

#endif  // OBJECTSFM_OBJ_TRACKER_H_
