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
#endif // !MIN_

#include "vo_monocular.h"

#include <fstream>
#include <filesystem>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>

#include "utils/basic_funcs.h"
#include "orientation/relative_pose_estimation.h"
#include "orientation/absolute_pose_estimation.h"
#include "feature/feature_tracking.h"
#include "feature/feature_matching.h"

namespace objectsfm {

	MonocularVO::MonocularVO()
	{
		found_seed_ = false;
	}

	MonocularVO::~MonocularVO()
	{
	}

	void MonocularVO::Run()
	{
		// preparation
		options_.minimizer_progress_to_stdout = true;
		options_.th_ratio_track_seed = 0.9;
		options_.th_max_iteration_full_bundle = 50;
		options_.th_max_iteration_partial_bundle = 50;
		options_.th_mse_reprojection = 5.0;
		options_.th_mse_outliers = 1.0;

		bundle_full_options_.max_num_iterations = options_.th_max_iteration_full_bundle;
		bundle_full_options_.minimizer_progress_to_stdout = options_.minimizer_progress_to_stdout;
		bundle_partial_options_.max_num_iterations = options_.th_max_iteration_partial_bundle;
		bundle_partial_options_.minimizer_progress_to_stdout = options_.minimizer_progress_to_stdout;
		
		db_.options.feature_type = "SURF";
		graph_.AssociateDatabase(&db_);

		//std::string path_video = options_.input_fold + "//video.MOV";
		std::string path_video = options_.input_fold + "//video.avi";
		video_cap_ = new cv::VideoCapture();
		video_cap_->open(path_video);
		video_cap_->set(CV_CAP_PROP_POS_FRAMES, options_.num_frames_skip);

		// 
		cv::Mat frame_test;
		*video_cap_ >> frame_test;
		double ratio = MIN_((float)options_.resize_image / MAX_(frame_test.cols, frame_test.rows),1.0);
		zoom_size_ = cv::Size(int(ratio*frame_test.cols), int(ratio*frame_test.rows));
		ppx = zoom_size_.width / 2.0;
		ppy = zoom_size_.height / 2.0;
		cam_models_.resize(1);
		cam_models_[0] = new CameraModel(0, zoom_size_.height, zoom_size_.width, 0, 535.4*ratio, "", "");
		//cam_models_[0] = new CameraModel(0, zoom_size_.height, zoom_size_.width, 0, 0.0, "", "");
		cam_models_[0]->SetImmutable();

		// step1: try to read in the camera calibration parameters
		if (ReadCamera())
		{
			cam_calibrated_ = true;
		}

		for(int p=0; p<1000; ++p)
		{
			cv::Mat frame;
			*video_cap_ >> frame;
			cv::Mat frame_rgb = frame.clone();
			cv::resize(frame, frame, zoom_size_);

			std::cout << "frame " << p << std::endl;

			int aa = 0;

			if (!trackers_.size())
			{
				Tracker* tracker = new Tracker();
				tracker->Initialize(frame);
				trackers_.push_back(tracker);

				Camera* cam_init = new Camera;
				cam_init->AssociateCamereModel(cam_models_[0]);
				cam_init->SetRTPose(Eigen::Matrix3d::Identity(3, 3), Eigen::Vector3d::Zero());
				cam_init->SetID(cams_.size());
				cams_.push_back(cam_init);
			}
			else
			{
				int num_tracked_total = 0;
				for (size_t i = 0; i < trackers_.size(); i++)
				{
					if (!trackers_[i]->is_track_failed_)
					{
						// track feature points on the new frame
						trackers_[i]->Track1Frame(frame);
						if (is_sys_initialized_ && trackers_[i]->num_tracked_3d_ < 10)
						{
							trackers_[i]->is_track_failed_ = true;
						}
						num_tracked_total += trackers_[i]->num_tracked_2d_;
					}
				}

				// localize the frame
				RTPose pose_cur;
				std::vector<int> pts3d_ids;
				std::vector<Eigen::Vector2d> pts2d;
				if (is_sys_initialized_)
				{
					LocalizeFrame(frame, pose_cur, pts3d_ids, pts2d);
					poses.push_back(pose_cur);
				}

				// add new key-frame
				bool add_new_frame = false;
				if (!is_sys_initialized_ && trackers_[0]->num_tracked_2d_ < 0.6*trackers_[0]->pts_init_.size())
				{
					add_new_frame = true;
				}
				else if (is_sys_initialized_ 
					&& trackers_[trackers_.size() - 1]->num_tracked_3d_ < 0.8*trackers_[trackers_.size() - 1]->pts_3d_.size())
				{
					add_new_frame = true;
				}

				if(add_new_frame)
				{
					if (!is_sys_initialized_)
					{
						if (!FindSeedPairThenReconstruct(frame))
						{
							continue;
						}
						is_sys_initialized_ = true;
					}
					else 
					{
						AddKeyFrme(frame, pose_cur, pts3d_ids, pts2d);
					}
				}
			}
		}
	}

	bool MonocularVO::ReadCamera()
	{
		// read in the camera calibration file
		std::string path_calib = options_.input_fold + "//calib.txt";
		std::ifstream ifs(path_calib);
		if (!ifs.is_open())
		{
			return false;
		}

		//
		cam_models_[0] = new CameraModel();
		ifs >> cam_models_[0]->f_;
		ifs >> cam_models_[0]->px_ >> cam_models_[0]->py_;
		ifs >> cam_models_[0]->k1_ >> cam_models_[0]->k2_;
		ifs >> cam_models_[0]->w_  >> cam_models_[0]->h_;

		return true;
	}

	bool MonocularVO::FindSeedPairThenReconstruct(cv::Mat &frame)
	{
		// step1: recover relative pose via tracked points
		std::vector<Eigen::Vector2d> pts_obs_1, pts_obs_2;
		for (size_t i = 0; i < trackers_[0]->pts_cur_.size(); i++)
		{
			if (trackers_[0]->pts_cur_[i].x < 0 || trackers_[0]->pts_cur_[i].y < 0)
			{
				continue;
			}
			pts_obs_1.push_back(Eigen::Vector2d(trackers_[0]->pts_init_[i].x - ppx, trackers_[0]->pts_init_[i].y - ppy));
			pts_obs_2.push_back(Eigen::Vector2d(trackers_[0]->pts_cur_[i].x - ppx, trackers_[0]->pts_cur_[i].y - ppy));
		}

		RTPoseRelative rt_pose_21;
		if (cam_models_[0]->f_)
		{
			if (!RelativePoseEstimation::RelativePoseWithFocalLength(pts_obs_1, pts_obs_2,
				cam_models_[0]->f_, cam_models_[0]->f_, rt_pose_21))
			{
				return false;
			}
		}
		else
		{
			double f1 = 0.0, f2 = 0.0;
			if (!RelativePoseEstimation::RelativePoseWithoutFocalLength(pts_obs_1, pts_obs_2, f1, f2, rt_pose_21))
			{
				return false;
			}
			cam_models_[0]->SetFocalLength((f1 + f2) / 2.0);
		}

		//
		Tracker* tracker_new = new Tracker();
		tracker_new->Initialize(frame);

		Camera* cam_new = new Camera;
		cam_new->AssociateCamereModel(cam_models_[0]);
		cam_new->SetRTPose(cams_[0]->pos_rt_, rt_pose_21);
		cam_new->SetID(cams_.size());

		// create 3D points
		std::vector<int> idx_inliers;
		std::vector<Point3D*> pts_temp;
		for (size_t i = 0; i < trackers_[0]->pts_cur_.size(); i++)
		{
			if (trackers_[0]->pts_cur_[i].x < 0 || trackers_[0]->pts_cur_[i].y < 0)
			{
				continue;
			}

			Point3D *pt = new Point3D;
			pt->AddObservation(cams_[0], trackers_[0]->pts_init_[i].x - ppx, trackers_[0]->pts_init_[i].y - ppy, cams_[0]->id_);
			pt->AddObservation(cam_new, trackers_[0]->pts_cur_[i].x - ppx, trackers_[0]->pts_cur_[i].y - ppy, cam_new->id_);

			if (pt->Trianglate2(options_.th_mse_reprojection, options_.th_angle_small))
			{
				pt->SetID(pts_temp.size());
				pts_temp.push_back(pt);

				trackers_[0]->Add2d3dCorrespondence(i, pt);
				tracker_new->AddFeaturePoints(trackers_[0]->pts_cur_[i]);
				tracker_new->Add2d3dCorrespondence(tracker_new->pts_init_.size() - 1, pt);
			}
			else
			{
				delete pt;
			}
		}
		if (pts_temp.size() < 100)
		{
			trackers_[0]->Clear2d3dCorrespondence();
			delete tracker_new;

			return false;	
		}

		trackers_.push_back(tracker_new);
		cams_.push_back(cam_new);
		pts_ = pts_temp;
		for (size_t i = 0; i < pts_.size(); i++)
		{
			cams_[0]->AddPoints(pts_[i], pts_[i]->id_);
			cams_[1]->AddPoints(pts_[i], pts_[i]->id_);
		}

		// bundle adjustment
		FullBundleAdjustment();

		RemovePointOutliers();
	}

	void MonocularVO::LocalizeFrame(cv::Mat &frame, RTPose &pose_absolute, 
		std::vector<int> &pts3d_ids, std::vector<Eigen::Vector2d> &pts2d)
	{
		// initial localization via pnp
		pts3d_ids.clear();
		std::vector<Eigen::Vector3d> pts3d_pnp;
		std::vector<Eigen::Vector2d> pts2d_pnp;
		
		for (size_t i = 0; i < trackers_.size(); i++)
		{
			if (!trackers_[i]->is_track_failed_)
			{
				for (auto iter_3dpt = trackers_[i]->pts_3d_.begin(); iter_3dpt != trackers_[i]->pts_3d_.end(); iter_3dpt++)
				{
					if (iter_3dpt->second->is_bad_estimated_)
					{
						continue;
					}

					int id_2d = iter_3dpt->first;
					if (trackers_[i]->status_[id_2d])
					{
						if (pts3d_ids.size() && find(pts3d_ids.begin(), pts3d_ids.end(), iter_3dpt->second->id_) != pts3d_ids.end())
						{
							continue;
						}
						pts3d_ids.push_back(iter_3dpt->second->id_);
						pts2d.push_back(Eigen::Vector2d(trackers_[i]->pts_cur_[id_2d].x, trackers_[i]->pts_cur_[id_2d].y));

						pts3d_pnp.push_back(Eigen::Vector3d(iter_3dpt->second->data));
						pts2d_pnp.push_back(Eigen::Vector2d(trackers_[i]->pts_cur_[id_2d].x-ppx, trackers_[i]->pts_cur_[id_2d].y-ppy));			
					}
				}
			}
		}

		std::cout << "pts3d_pnp  " << pts3d_pnp.size() << std::endl;
		std::vector<double> error_reproj;
		double avg_error = 0.0;
		AbsolutePoseEstimation::AbsolutePoseWithFocalLength(pts3d_pnp, pts2d_pnp, cam_models_[0]->f_, pose_absolute, error_reproj, avg_error);

		// refine the pose via bundle adjustment
	}

	void MonocularVO::AddKeyFrme(cv::Mat & frame, RTPose & pose,
		std::vector<int> &pts3d_ids, std::vector<Eigen::Vector2d> &pts2d)
	{
		Tracker* tracker_new = new Tracker();
		tracker_new->Initialize(frame);
		trackers_.push_back(tracker_new);

		Camera* cam_new = new Camera;
		cam_new->AssociateCamereModel(cam_models_[0]);
		cam_new->SetRTPose(pose.R, pose.t);
		cam_new->SetID(cams_.size());
		cams_.push_back(cam_new);

		// bundle adjustment
		int id_track = trackers_.size() - 1;
		for (size_t i = 0; i < pts3d_ids.size(); i++)
		{
			int id = pts3d_ids[i];
			pts_[id]->AddObservation(cam_new, pts2d[i](0)-ppx, pts2d[i](1)-ppy, cam_new->id_);
			pts_[id]->is_new_added_ = true;
			cam_new->AddPoints(pts_[id], pts_[id]->id_);

			trackers_[id_track]->AddFeaturePoints(cv::Point2f(pts2d[i](0), pts2d[i](1)));
			trackers_[id_track]->Add2d3dCorrespondence(trackers_[id_track]->pts_init_.size()-1, pts_[id]);
		}
		FullBundleAdjustment();
		RemovePointOutliers();

		// draw
		if (1)
		{
			cv::Mat img_draw1 = frame.clone();
			std::string path_txt1 = options_.output_fold + "//3dpts.txt";
			std::string path_img1 = options_.output_fold + "//2dpts.jpg";
			std::ofstream ofs1(path_txt1);
			for (size_t i = 0; i < pts3d_ids.size(); i++)
			{
				int id = pts3d_ids[i];

				if (i % 5 == 0)
				{
					cv::circle(img_draw1, cv::Point(pts2d[i](0), pts2d[i](1)), 2, cv::Scalar(0, 0, 255), 2);
					ofs1 << pts_[id]->data[0] << " "
						<< pts_[id]->data[1] << " "
						<< pts_[id]->data[2] << " "
						<< 255 << " "
						<< 0 << " "
						<< 0 << std::endl;
				}
				else
				{
					cv::circle(img_draw1, cv::Point(pts2d[i](0), pts2d[i](1)), 2, cv::Scalar(255, 0, 0), 2);
					ofs1 << pts_[id]->data[0] << " "
						<< pts_[id]->data[1] << " "
						<< pts_[id]->data[2] << " "
						<< 0 << " "
						<< 0 << " "
						<< 255 << std::endl;
				}
			}
			ofs1.close();
			cv::imwrite(path_img1, img_draw1);
		}
	

		// generate new 3D points
		cv::Mat img_draw2 = frame.clone();
		std::string path_txt2 = options_.output_fold + "//3dpts_new.txt";
		std::string path_img2 = options_.output_fold + "//2dpts_new.jpg";
		std::ofstream ofs2(path_txt2);

		for (size_t i = 0; i < trackers_.size(); i++)
		{
			if (!trackers_[i]->is_track_failed_)
			{
				for (size_t j = 0; j < trackers_[i]->pts_cur_.size(); j++)
				{
					if (trackers_[i]->pts_cur_[j].x <0 || trackers_[i]->pts_cur_[j].y <0)
					{
						continue;
					}

					std::map<int, Point3D*>::iterator iter_3dpt = trackers_[i]->pts_3d_.find(j);
					if (iter_3dpt != trackers_[i]->pts_3d_.end())
					{
						continue;
					}

					//
					int id_cam1 = i;
					int id_cam2 = cams_.size() - 1;
					Point3D *pt = new Point3D;
					pt->AddObservation(cams_[id_cam1], trackers_[id_cam1]->pts_init_[j].x - ppx, trackers_[id_cam1]->pts_init_[j].y - ppy, id_cam1);
					pt->AddObservation(cams_[id_cam2], trackers_[id_cam1]->pts_cur_[j].x - ppx, trackers_[id_cam1]->pts_cur_[j].y - ppy, id_cam2);

					if (pt->Trianglate2(options_.th_mse_reprojection, options_.th_angle_small))
					{
						pt->SetID(pts_.size());
						pt->is_new_added_ = true;
						pts_.push_back(pt);
						cams_[id_cam1]->AddPoints(pt, pt->id_);
						cams_[id_cam2]->AddPoints(pt, pt->id_);

						trackers_[id_cam1]->Add2d3dCorrespondence(j, pt);
						trackers_[id_cam2]->AddFeaturePoints(trackers_[id_cam1]->pts_cur_[j]);
						trackers_[id_cam2]->Add2d3dCorrespondence(trackers_[id_cam2]->pts_init_.size() - 1, pt);

						if (j % 5 == 0)
						{
							cv::circle(img_draw2, trackers_[id_cam1]->pts_cur_[j], 2, cv::Scalar(0, 0, 255), 2);
							ofs2<< pt->data[0] << " "
								<< pt->data[1] << " "
								<< pt->data[2] << " "
								<< 255 << " "
								<< 0 << " "
								<< 0 << std::endl;
						}
						else
						{
							cv::circle(img_draw2, trackers_[id_cam1]->pts_cur_[j], 2, cv::Scalar(255, 0, 0), 2);
							ofs2<< pt->data[0] << " "
								<< pt->data[1] << " "
								<< pt->data[2] << " "
								<< 0 << " "
								<< 0 << " "
								<< 255 << std::endl;
						}
					}
					else
					{
						delete pt;
					}
				}
			}
		}
		ofs2.close();
		cv::imwrite(path_img2, img_draw2);

		//PartialBundleAdjustment();
		FullBundleAdjustment();
		RemovePointOutliers();
	}

	void MonocularVO::FullBundleAdjustment()
	{
		// mutable all the cameras and points
		MutableCamsPoints();

		// bundle adjustment 
		objectsfm::BundleAdjuster bundler(cams_, cam_models_, pts_);
		bundler.SetOptions(bundle_full_options_);
		bundler.RunOptimizetion(!found_seed_, 1.0);
		bundler.UpdateParameters();
	}

	void MonocularVO::PartialBundleAdjustment()
	{
		ImmutableCamsPoints();

		// optimize only the new camera and new points
		for (size_t i = 0; i < trackers_.size(); i++)
		{
			if (!trackers_[i]->is_track_failed_)
			{
				cams_[i]->SetMutable(true);
				for (auto iter = cams_[i]->pts_.begin(); iter != cams_[i]->pts_.end(); iter++)
				{
					if (!iter->second->is_bad_estimated_)
					{
						iter->second->SetMutable(true);
					}
				}
			}
		}

		cams_[cams_.size() - 1]->SetMutable(true);
		for (size_t i = 0; i < pts_.size(); i++)
		{
			if (!pts_[i]->is_bad_estimated_ && pts_[i]->is_new_added_)
			{
				pts_[i]->SetMutable(true);
			}
		}

		// run bundle adjustment 
		objectsfm::BundleAdjuster bundler(cams_, cam_models_, pts_);
		bundler.SetOptions(bundle_partial_options_);
		bundler.RunOptimizetion(!found_seed_, 2.0);
		bundler.UpdateParameters();
	}

	void MonocularVO::ImmutableCamsPoints()
	{
		for (size_t i = 0; i < cams_.size(); i++)
		{
			cams_[i]->SetMutable(false);

			std::map<int, Point3D*>::iterator iter = cams_[i]->pts_.begin();
			while (iter != cams_[i]->pts_.end())
			{
				iter->second->SetMutable(false);
				iter++;
			}
		}
	}

	void MonocularVO::MutableCamsPoints()
	{
		for (size_t i = 0; i < cams_.size(); i++)
		{
			cams_[i]->SetMutable(true);

			std::map<int, Point3D*>::iterator iter = cams_[i]->pts_.begin();
			while (iter != cams_[i]->pts_.end())
			{
				iter->second->SetMutable(true);
				iter++;
			}
		}
	}

	void MonocularVO::RemovePointOutliers()
	{
		int count_outliers = 0;
		int count_outliers_new_add = 0;
		int count_new_add = 0;
		for (size_t i = 0; i < pts_.size(); i++)
		{
			if (pts_[i]->is_bad_estimated_)
			{
				continue;
			}
			if (pts_[i]->is_new_added_)
			{
				count_new_add++;
			}

			//pts_[i]->is_bad_estimated_ = false;
			pts_[i]->Reprojection();
			if (std::sqrt(pts_[i]->mse_) > options_.th_mse_outliers)
			{
				pts_[i]->is_bad_estimated_ = true;
				count_outliers++;
				if (pts_[i]->is_new_added_)
				{
					count_outliers_new_add++;
				}
			}
			pts_[i]->is_new_added_ = false;
		}

		std::cout << "----------RemovePointOutliers: " << count_outliers << " of " << pts_.size() << std::endl;
		std::cout << "----------RemovePointOutliers New: " << count_outliers_new_add << " of " << count_new_add << std::endl;
	}

	void MonocularVO::MatchingKeyFrames(std::vector<std::vector<std::pair<int, int>>> &matches_all)
	{
		matches_all.resize(db_.kdindexs.size() - 1);
		int id_new = db_.kdindexs.size() - 1;
		for (size_t i = 0; i < db_.kdindexs.size()-1; i++)
		{
			bool isOK = FeatureMatching::KNNMatchingWithGeoVerify(db_.feature_points_[id_new], db_.descriptors_[id_new],
				db_.feature_points_[i], db_.kdindexs[i], matches_all[i]);

			if (!isOK)
			{
				continue;
			}
			std::cout << "------matching " << i << " num: " << matches_all[i].size() << std::endl;
		}
	}


	void MonocularVO::WriteCameraPointsOut(std::string path)
	{
		std::ofstream ofs(path);
		if (!ofs.is_open())
		{
			return;
		}

		// 3D points
		for (size_t i = 0; i < pts_.size(); i++)
		{
			if (pts_[i]->is_bad_estimated_)
			{
				continue;
			}
			ofs << pts_[i]->data[0] << " " << pts_[i]->data[1] << " " << pts_[i]->data[2]
				<< " " << 0 << " " << 0 << " " << 0 << std::endl;
		}

		// key frames
		double scale = 0.1;
		for (size_t i = 0; i < cams_.size(); i++)
		{
			std::vector<Eigen::Vector3d> axis(3);
			axis[0] = cams_[i]->pos_rt_.R.inverse() * Eigen::Vector3d(1, 0, 0);
			axis[1] = cams_[i]->pos_rt_.R.inverse() * Eigen::Vector3d(0, 1, 0);
			axis[2] = cams_[i]->pos_rt_.R.inverse() * Eigen::Vector3d(0, 0, 1);

			std::vector<Eigen::Vector3d> cam_pts;
			GenerateCamera3D(cams_[i]->pos_ac_.c, axis, cams_[i]->cam_model_->f_,
				cams_[i]->cam_model_->w_, cams_[i]->cam_model_->h_, scale, cam_pts);

			for (size_t j = 0; j < cam_pts.size(); j++)
			{
				ofs << cam_pts[j](0) << " " << cam_pts[j](1) << " " << cam_pts[j](2)
					<< " " << 255 << " " << 0 << " " << 0 << std::endl;
			}
		}

		// ordinary poses
		for (size_t i = 0; i < poses.size(); i++)
		{
			std::vector<Eigen::Vector3d> axis(3);
			axis[0] = poses[i].R.inverse() * Eigen::Vector3d(1, 0, 0);
			axis[1] = poses[i].R.inverse() * Eigen::Vector3d(0, 1, 0);
			axis[2] = poses[i].R.inverse() * Eigen::Vector3d(0, 0, 1);

			Eigen::Vector3d c = -poses[i].R.inverse() * poses[i].t;

			std::vector<Eigen::Vector3d> cam_pts;
			GenerateCamera3D(c, axis, cams_[0]->cam_model_->f_,
				cams_[0]->cam_model_->w_, cams_[0]->cam_model_->h_, scale, cam_pts);

			for (size_t j = 0; j < cam_pts.size(); j++)
			{
				ofs << cam_pts[j](0) << " " << cam_pts[j](1) << " " << cam_pts[j](2)
					<< " " << 0 << " " << 0 << " " << 255 << std::endl;
			}
		}

		ofs.close();
	}

}  // namespace objectsfm
