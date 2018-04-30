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

#ifndef MAX_
#define MAX_(a,b) ( ((a)>(b)) ? (a):(b) )
#endif // !MAX

#ifndef MIN_
#define MIN_(a,b) ( ((a)<(b)) ? (a):(b) )
#endif // !MIN

#include "Tracker.h"

#include <fstream>
#include <Eigen/Core>
#include <Eigen/LU>

#include "utils/basic_funcs.h"

namespace objectsfm {

	Tracker::Tracker()
	{
		num_tracked_2d_ = 0;
	}

	Tracker::~Tracker()
	{
	}

	void Tracker::Initialize(cv::Mat & frame)
	{
		if (frame.channels() == 3)
		{
			cv::cvtColor(frame, frame, CV_RGB2GRAY);
		}
		
		cv::goodFeaturesToTrack(frame, pts_init_, 4000, 0.01, 5, cv::Mat(), 3);
		std::cout << "num features: " << pts_init_.size() << std::endl;

		frame_init_ = frame;
		frame_pre_ = frame;
		pts_pre_ = pts_init_;
		num_tracked_2d_ = pts_init_.size();
		status_.resize(pts_init_.size(), 1);
	}

	void Tracker::Track1Frame(cv::Mat & frame)
	{
		if (frame.channels() == 3)
		{
			cv::cvtColor(frame, frame, CV_RGB2GRAY);
		}
		frame_cur_ = frame.clone();

		std::vector<uchar> status_temp;
		std::vector<float> err_temp;
		cv::calcOpticalFlowPyrLK(frame_pre_, frame_cur_, pts_pre_, pts_cur_, status_temp, err_temp);
		std::vector<int> status_cur(status_temp.size(),0);
		for (size_t i = 0; i < status_temp.size(); i++)
		{
			if (status_temp[i])
			{
				status_cur[i] = 1;
			}
		}
		math::vector_dot(status_, status_cur);

		// remove outliers matching via Fundamental matrix
		std::vector<cv::Point2f> pt1, pt2;
		std::vector<int> idx;
		for (size_t i = 0; i < status_.size(); i++)
		{
			if (status_[i])
			{
				pt1.push_back(pts_init_[i]);
				pt2.push_back(pts_cur_[i]);
				idx.push_back(i);
			}
		}
		if (pt1.size() < 50)
		{
			is_track_failed_ = true;
			return;
		}

		std::vector<uchar> status_match(pt1.size());
		cv::Mat FMatrix = cv::findFundamentalMat(pt1, pt2, status_match, cv::FM_RANSAC, 1.0);

		status_ = std::vector<int>(status_.size(), 0);
		pts_cur_ = std::vector<cv::Point2f>(pts_pre_.size(), cv::Point2f(-100,-100));
		for (size_t i = 0; i < status_match.size(); i++)
		{
			if (status_match[i])
			{
				int id = idx[i];
				pts_cur_[id] = pt2[i];
				status_[id] = 1;
			}
		}

		// tracks
		num_tracked_2d_ = 0;
		num_tracked_3d_ = 0;
		for (size_t i = 0; i < status_.size(); i++)
		{
			if (status_[i])
			{
				num_tracked_2d_++;
				auto iter = pts_3d_.find(i);
				if (iter != pts_3d_.end() && (!iter->second->is_bad_estimated_))
				{
					num_tracked_3d_++;
				}
			}
		}

		std::cout << "------ 2d tracked: " << num_tracked_2d_;
		std::cout << "  3d tracked: " << num_tracked_3d_ << std::endl;

		frame_pre_ = frame_cur_.clone();
		pts_pre_ = pts_cur_;

	}

	void Tracker::Track1Frame(cv::VideoCapture * video_cap, cv::Size zoom_size)
	{
		cv::Mat frame;
		*video_cap >> frame;
		cv::resize(frame, frame, zoom_size);

		Track1Frame(frame);
	}

	void Tracker::TrackNFrame(cv::VideoCapture * video_cap, cv::Size zoom_size, double ratio)
	{
		while (num_tracked_2d_ > ratio*pts_init_.size())
		{
			Track1Frame(video_cap, zoom_size);
		}
	}

	void Tracker::TrackNFrame(cv::VideoCapture * video_cap, cv::Size zoom_size, int N)
	{
		int count = 0;

		while (count < N)
		{
			Track1Frame(video_cap, zoom_size);
			count++;
		}
	}

	bool Tracker::IsTrackFailure()
	{
		if (num_tracked_2d_ < min_num_tracked_)
		{
			is_track_failed_ = true;
			return true;
		}

		return false;
	}

	void Tracker::AddFeaturePoints(cv::Point2f pt_tracked)
	{
		pts_init_.push_back(pt_tracked);
		pts_pre_.push_back(pt_tracked);
		num_tracked_2d_++;
		status_.push_back(1);
	}

	void Tracker::Add2d3dCorrespondence(int id_2d, Point3D * pt_3d)
	{
		pts_3d_.insert(std::pair<int, Point3D*>(id_2d, pt_3d));
	}

	void Tracker::Clear2d3dCorrespondence()
	{
		pts_3d_.clear();
	}

}  // namespace objectsfm
