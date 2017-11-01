/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Map.h"

#include<mutex>

namespace ORB_SLAM2
{

Map::Map():mnMaxKFid(0)
{
}

void Map::AddKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.insert(pKF);
    if(pKF->mnId>mnMaxKFid)
        mnMaxKFid=pKF->mnId;
	//keyFrameMap[pKF->mTimeStamp] = pKF;
	if (mspKeyFrames.size() > 1) {
		UpdateRegister(pKF->mTimeStamp, pKF);
	}
}

void Map::AddMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.insert(pMP);
}

void Map::EraseMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.erase(pMP);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::EraseKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.erase(pKF);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
    unique_lock<mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

vector<KeyFrame*> Map::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}

vector<MapPoint*> Map::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
}

long unsigned int Map::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspMapPoints.size();
}

long unsigned int Map::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspKeyFrames.size();
}

vector<MapPoint*> Map::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}

long unsigned int Map::GetMaxKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFid;
}

void Map::clear()
{
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
        delete *sit;

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
        delete *sit;

    mspMapPoints.clear();
    mspKeyFrames.clear();
    mnMaxKFid = 0;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
}

std::vector<std::vector<cv::Point3f>> Map::GetPanelPos() {
	return panel3dPos;
}

void Map::SetReferencePanel(const int id,
							const std::map<int, std::map<int, std::pair<cv::Vec4f, cv::Vec4f>>> &group_info) 
{
	if (group_info.size() > 0)
		panelGroupInfo_[id] = group_info;
}

float GetCross(cv::Point2f& p1, cv::Point2f& p2, cv::Point2f& p)
{
	return (p2.x - p1.x) * (p.y - p1.y) - (p.x - p1.x) * (p2.y - p1.y);
}

bool IsInPanel(cv::Point2f &p, std::pair<cv::Vec4f, cv::Vec4f>&group_info) {
	cv::Point2f p1(group_info.first[0], group_info.first[1]);
	cv::Point2f p2(group_info.first[2], group_info.first[3]);
	cv::Point2f p3(group_info.second[2], group_info.second[3]);
	cv::Point2f p4(group_info.second[0], group_info.second[1]);

	return GetCross(p1, p2, p) * GetCross(p3, p4, p) >= 0 && GetCross(p2, p3, p) * GetCross(p4, p1, p) >= 0;
}

cv::Point2f GetTransformDist(KeyFrame*key_f_c, KeyFrame*key_f_p){
	cv::Mat Tcr = key_f_c->GetPose()*key_f_p->GetPoseInverse();
	
	cv::Mat twc = Tcr.rowRange(0, 3).col(3);
	//cout << "twc size:" << twc.size() << endl;
	//cout << "twc step:" << twc.type() << endl;
	if (twc.size().area() > 0) {
		
		cv::Point2f offset(twc.at<float>(0), twc.at<float>(1));
		offset *= 1800;
		return offset;
	}
	else
		return cv::Point2f();
}

bool IsMatch(const cv::Point2f &offset, std::pair<cv::Vec4f, cv::Vec4f>&group_c, std::pair<cv::Vec4f, cv::Vec4f>&group_p)
{
	cout << "(SLAM)group:" << group_c.first << ", " << group_c.second << " --> " <<
		group_p.first << ", " << group_p.second << endl;
	cv::Point2f cur_p, pre_p, diff;
	cur_p.x = group_c.first[0], cur_p.y = group_c.first[1];
	pre_p.x = group_p.first[0], pre_p.y = group_p.first[1];
	diff = cur_p - offset - pre_p;
	float dist = sqrt(diff.x*diff.x + diff.y*diff.y);
	cout << "(SLAM)0:" << dist << endl;
	if (dist > 50){
		return false;
	}

	cur_p.x = group_c.first[2], cur_p.y = group_c.first[3];
	pre_p.x = group_p.first[2], pre_p.y = group_p.first[3];
	diff = cur_p - offset - pre_p;
	dist = sqrt(diff.x*diff.x + diff.y*diff.y);
	cout << "(SLAM)1:" << dist << endl;
	if (dist > 50){
		return false;
	}

	cur_p.x = group_c.second[0], cur_p.y = group_c.second[1];
	pre_p.x = group_p.second[0], pre_p.y = group_p.second[1];
	diff = cur_p - offset - pre_p;
	dist = sqrt(diff.x*diff.x + diff.y*diff.y);
	cout << "(SLAM)2:" << dist << endl;
	if (dist > 50){
		return false;
	}

	cur_p.x = group_c.second[2], cur_p.y = group_c.second[3];
	pre_p.x = group_p.second[2], pre_p.y = group_p.second[3];
	diff = cur_p - offset - pre_p;
	
	dist = sqrt(diff.x*diff.x + diff.y*diff.y);
	cout << "(SLAM)3:" << dist << endl;
	if (dist > 50){
		return false;
	}
	return true;
}

cv::Mat SkewSymmetricMatrix(const cv::Mat &v)
{
	return (cv::Mat_<float>(3, 3) << 0, -v.at<float>(2), v.at<float>(1),
			v.at<float>(2), 0, -v.at<float>(0),
			-v.at<float>(1), v.at<float>(0), 0);
}

cv::Mat ComputeF12(KeyFrame *&pKF1, KeyFrame *&pKF2)
{
	cv::Mat R1w = pKF1->GetRotation();
	cv::Mat t1w = pKF1->GetTranslation();
	cv::Mat R2w = pKF2->GetRotation();
	cv::Mat t2w = pKF2->GetTranslation();

	cv::Mat R12 = R1w*R2w.t();
	cv::Mat t12 = -R1w*R2w.t()*t2w + t1w;

	cv::Mat t12x = SkewSymmetricMatrix(t12);

	const cv::Mat &K1 = pKF1->mK;
	const cv::Mat &K2 = pKF2->mK;


	return K1.t().inv()*t12x*R12*K2.inv();
}

//void Map::UpdateRegister(const int id_c, KeyFrame*key_f_c, const int id_p, KeyFrame*key_f_p, cv::Mat& camera_matrix, cv::Mat& distCoeffs)
void Map::UpdateRegister(const int id_c, KeyFrame*key_f_c)
{
	if (panelGroupInfo_.count(id_c)) {
		std::map<int, std::map<int, std::pair<cv::Vec4f, cv::Vec4f>>>  &group_info = panelGroupInfo_[id_c];
		map<int, map<int, vector<int>>> map_point_register;
		ITER(it_p, group_info) {
			ITER(it_g, it_p->second) {
				for (int i = 0; i < key_f_c->mvKeys.size(); ++i) {
					if (IsInPanel(cv::Point2f(key_f_c->mvKeys[i].pt), it_g->second)) {
						map_point_register[it_p->first][it_g->first].push_back(i);
					}
				}
			}
		}

	
		const vector<KeyFrame*> vpNeighKFs = key_f_c->GetBestCovisibilityKeyFrames(10);

		cv::Mat Rcw1 = key_f_c->GetRotation();
		cv::Mat Rwc1 = Rcw1.t();
		cv::Mat tcw1 = key_f_c->GetTranslation();
		cv::Mat Tcw1(3, 4, CV_32F);
		Rcw1.copyTo(Tcw1.colRange(0, 3));
		tcw1.copyTo(Tcw1.col(3));
		cv::Mat Ow1 = key_f_c->GetCameraCenter();

		const float &fx1 = key_f_c->fx;
		const float &fy1 = key_f_c->fy;
		const float &cx1 = key_f_c->cx;
		const float &cy1 = key_f_c->cy;
		const float &invfx1 = key_f_c->invfx;
		const float &invfy1 = key_f_c->invfy;

		const float ratioFactor = 1.5f*key_f_c->mfScaleFactor;

		int nnew = 0;

		// Search matches with epipolar restriction and triangulate
		for (size_t i = 0; i<vpNeighKFs.size(); i++)
		{
			KeyFrame* pKF2 = vpNeighKFs[i];
			if (!panelGroupInfo_.count(pKF2->mTimeStamp))
				continue;
			const int id_p = pKF2->mTimeStamp;
			// Check first that baseline is not too short
			cv::Mat Ow2 = pKF2->GetCameraCenter();
			cv::Mat vBaseline = Ow2 - Ow1;
			const float baseline = cv::norm(vBaseline);
			const float medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);
			const float ratioBaselineDepth = baseline / medianDepthKF2;

			if (ratioBaselineDepth < 0.01)
				continue;
			
			cout << "(SLAM)Match between " << id_c << "," << pKF2->mTimeStamp << endl;

			// Compute Fundamental Matrix
			cv::Mat F12 = ComputeF12(key_f_c, pKF2);

			// Search matches that fullfil epipolar constraint
			std::map<int, std::map<int, std::pair<cv::Vec4f, cv::Vec4f>>>  &pre_group_info = panelGroupInfo_[id_p];
			cv::Point2f offset = GetTransformDist(key_f_c, pKF2);  // 
			cout << "(SLAM)offset:" << offset << endl;
			map<int, map<int, pair<int, int>>> vMatchedIndices;
			ITER(it_p, group_info) {
				ITER(it_g, it_p->second) {
					ITER(it_p_p, pre_group_info) {
						ITER(it_g_p, it_p_p->second) {
							cout << "(SLAM) match (" << it_p->first << "," << it_g->first << ")->(" <<
								it_p_p->first << "," << it_g_p->first << ")" << endl;
							if (IsMatch(offset, it_g->second, it_g_p->second)) {
								vMatchedIndices[it_p->first][it_g->first] = make_pair(it_p_p->first, it_g_p->first);
							}
						}
					}
				}
			}
			if (vMatchedIndices.size() == 0)
				continue;
			cout << "(SLAM)Get match:" << vMatchedIndices.size() << endl;
			//matcher.SearchForTriangulation(mpCurrentKeyFrame, pKF2, F12, vMatchedIndices, false);

			cv::Mat Rcw2 = pKF2->GetRotation();
			cv::Mat Rwc2 = Rcw2.t();
			cv::Mat tcw2 = pKF2->GetTranslation();
			cv::Mat Tcw2(3, 4, CV_32F);
			Rcw2.copyTo(Tcw2.colRange(0, 3));
			tcw2.copyTo(Tcw2.col(3));

			const float &fx2 = pKF2->fx;
			const float &fy2 = pKF2->fy;
			const float &cx2 = pKF2->cx;
			const float &cy2 = pKF2->cy;
			const float &invfx2 = pKF2->invfx;
			const float &invfy2 = pKF2->invfy;

			std::vector<std::vector<cv::Point3f>> tmp_panel3dPos;

			// Triangulate each match
			ITER(it_p, vMatchedIndices) {
				ITER(it_g, it_p->second) {
					const int r = it_p->first, c = it_g->first;
					const int m_r = it_g->second.first, m_c = it_g->second.second;
					vector<cv::Point2f> cur_group;
					vector<cv::Point2f> pre_group;

					cur_group.push_back(cv::Point2f(group_info[r][c].first[0], group_info[r][c].first[1]));
					cur_group.push_back(cv::Point2f(group_info[r][c].first[2], group_info[r][c].first[3]));
					cur_group.push_back(cv::Point2f(group_info[r][c].second[0], group_info[r][c].second[1]));
					cur_group.push_back(cv::Point2f(group_info[r][c].second[2], group_info[r][c].second[3]));

					pre_group.push_back(cv::Point2f(pre_group_info[m_r][m_c].first[0], pre_group_info[m_r][m_c].first[1]));
					pre_group.push_back(cv::Point2f(pre_group_info[m_r][m_c].first[2], pre_group_info[m_r][m_c].first[3]));
					pre_group.push_back(cv::Point2f(pre_group_info[m_r][m_c].second[0], pre_group_info[m_r][m_c].second[1]));
					pre_group.push_back(cv::Point2f(pre_group_info[m_r][m_c].second[2], pre_group_info[m_r][m_c].second[3]));
					std::vector<cv::Point3f> tmp_pos;
					for (int i = 0; i < 4; ++i) {
						// Check parallax between rays
						//cv::Mat xn1 = (cv::Mat_<float>(3, 1) << (kp1.pt.x - cx1)*invfx1, (kp1.pt.y - cy1)*invfy1, 1.0);
						//cv::Mat xn2 = (cv::Mat_<float>(3, 1) << (kp2.pt.x - cx2)*invfx2, (kp2.pt.y - cy2)*invfy2, 1.0);
						cv::Mat xn1 = (cv::Mat_<float>(3, 1) << (cur_group[i].x - cx1)*invfx1,
									   (cur_group[i].y - cy1)*invfy1, 1.0);
						cv::Mat xn2 = (cv::Mat_<float>(3, 1) << (pre_group[i].x - cx2)*invfx2,
									   (pre_group[i].y - cy2)*invfy2, 1.0);

						cv::Mat ray1 = Rwc1*xn1;
						cv::Mat ray2 = Rwc2*xn2;
						const float cosParallaxRays = ray1.dot(ray2) / (cv::norm(ray1)*cv::norm(ray2));

						cv::Mat x3D;
						if (cosParallaxRays>0 && cosParallaxRays < 0.9998)
						{
							// Linear Triangulation Method
							cv::Mat A(4, 4, CV_32F);
							A.row(0) = xn1.at<float>(0)*Tcw1.row(2) - Tcw1.row(0);
							A.row(1) = xn1.at<float>(1)*Tcw1.row(2) - Tcw1.row(1);
							A.row(2) = xn2.at<float>(0)*Tcw2.row(2) - Tcw2.row(0);
							A.row(3) = xn2.at<float>(1)*Tcw2.row(2) - Tcw2.row(1);

							cv::Mat w, u, vt;
							cv::SVD::compute(A, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

							x3D = vt.row(3).t();

							if (x3D.at<float>(3) == 0)
								continue;

							// Euclidean coordinates
							x3D = x3D.rowRange(0, 3) / x3D.at<float>(3);

						}
						else {
							cout << "(SLAM)Very low parallax:" << cosParallaxRays;
							continue; //No stereo and very low parallax
						}

						cv::Mat x3Dt = x3D.t();

						//Check triangulation in front of cameras
						float z1 = Rcw1.row(2).dot(x3Dt) + tcw1.at<float>(2);
						if (z1 <= 0)
							continue;

						float z2 = Rcw2.row(2).dot(x3Dt) + tcw2.at<float>(2);
						if (z2 <= 0)
							continue;

						//Check reprojection error in first keyframe
						//const float &sigmaSquare1 = mpCurrentKeyFrame->mvLevelSigma2[kp1.octave];
						const float x1 = Rcw1.row(0).dot(x3Dt) + tcw1.at<float>(0);
						const float y1 = Rcw1.row(1).dot(x3Dt) + tcw1.at<float>(1);
						const float invz1 = 1.0 / z1;

						
						float u1 = fx1*x1*invz1 + cx1;
						float v1 = fy1*y1*invz1 + cy1;
						float errX1 = u1 - cur_group[i].x;
						float errY1 = v1 - cur_group[i].y;
						//if ((errX1*errX1 + errY1*errY1) > 5.991*sigmaSquare1)
						if ((errX1*errX1 + errY1*errY1) > 5.991)
							continue;
						

						//Check reprojection error in second keyframe
						
						const float x2 = Rcw2.row(0).dot(x3Dt) + tcw2.at<float>(0);
						const float y2 = Rcw2.row(1).dot(x3Dt) + tcw2.at<float>(1);
						const float invz2 = 1.0 / z2;
						
						float u2 = fx2*x2*invz2 + cx2;
						float v2 = fy2*y2*invz2 + cy2;
						float errX2 = u2 - pre_group[i].x;
						float errY2 = v2 - pre_group[i].y;
						if ((errX2*errX2 + errY2*errY2) > 5.991)
							continue;
						

						//Check scale consistency
						cv::Mat normal1 = x3D - Ow1;
						float dist1 = cv::norm(normal1);

						cv::Mat normal2 = x3D - Ow2;
						float dist2 = cv::norm(normal2);

						if (dist1 == 0 || dist2 == 0)
							continue;

						const float ratioDist = dist2 / dist1;
						const float ratioOctave = 1.f;

						/*if(fabs(ratioDist-ratioOctave)>ratioFactor)
						continue;*/
						if (ratioDist*ratioFactor<ratioOctave || ratioDist>ratioOctave*ratioFactor)
							continue;

						tmp_pos.push_back(cv::Point3f(x3D.at<float>(0), x3D.at<float>(1), x3D.at<float>(2)));
					}
					if (tmp_pos.size() == 4) {
						tmp_panel3dPos.push_back(tmp_pos);
					}
					else {
						cout << "tmp_pos is not 4," << tmp_pos.size() << endl;
					}
				}
			}
			if (tmp_panel3dPos.size() > 0) {
				cout << "Have new panel:" << tmp_panel3dPos.size() << endl;
				panel3dPos = tmp_panel3dPos;
				break;
			}

		}
		/*
		if (panelGroupInfo_.count(id_p)) {
			std::map<int, std::map<int, std::pair<cv::Vec4f, cv::Vec4f>>>  &pre_group_info = panelGroupInfo_[id_p];
			cv::Point2f offset = GetTransformDist(key_f_c, key_f_p);  // 
			map<int, map<int, pair<int, int>>> match_id;
			ITER(it_p, group_info) {
				ITER(it_g, it_p->second) {
					ITER(it_p_p, pre_group_info) {
						ITER(it_g_p, it_p_p->second) {
							if (IsMatch(offset, it_g->second, it_g_p->second)) {
								match_id[it_p->first][it_g->first] = make_pair(it_p_p->first, it_g_p->first);
							}
						}
					}
				}
			}
			cout << "(SLAM)Match id:" << match_id.size() << endl;
			if (match_id.size() > 0) {
				cv::Mat Rcw1 = key_f_c->GetRotation();
				cv::Mat Rwc1 = Rcw1.t();
				cv::Mat tcw1 = key_f_c->GetTranslation();
				cv::Mat Tcw1(3, 4, CV_32F);
				Rcw1.copyTo(Tcw1.colRange(0, 3));
				tcw1.copyTo(Tcw1.col(3));
				cv::Mat Ow1 = key_f_c->GetCameraCenter();

				const float &fx1 = key_f_c->fx;
				const float &fy1 = key_f_c->fy;
				const float &cx1 = key_f_c->cx;
				const float &cy1 = key_f_c->cy;
				const float &invfx1 = key_f_c->invfx;
				const float &invfy1 = key_f_c->invfy;

				cv::Mat Rcw2 = key_f_p->GetRotation();
				cv::Mat Rwc2 = Rcw2.t();
				cv::Mat tcw2 = key_f_p->GetTranslation();
				cv::Mat Tcw2(3, 4, CV_32F);
				Rcw2.copyTo(Tcw2.colRange(0, 3));
				tcw2.copyTo(Tcw2.col(3));

				std::vector<std::vector<cv::Point3f>> tmp_panel3dPos;

				const float &fx2 = key_f_p->fx;
				const float &fy2 = key_f_p->fy;
				const float &cx2 = key_f_p->cx;
				const float &cy2 = key_f_p->cy;
				const float &invfx2 = key_f_p->invfx;
				const float &invfy2 = key_f_p->invfy;
				ITER(it_p, match_id) {
					ITER(it_g, it_p->second) {
						std::vector<cv::Point3f> tmp_pos;
						const int r = it_p->first, c = it_g->first;
						const int m_r = it_g->second.first, m_c = it_g->second.second;
						vector<cv::Point2f> cur_group;
						vector<cv::Point2f> pre_group;

						cur_group.push_back(cv::Point2f(group_info[r][c].first[0], group_info[r][c].first[1]));
						cur_group.push_back(cv::Point2f(group_info[r][c].first[2], group_info[r][c].first[3]));
						cur_group.push_back(cv::Point2f(group_info[r][c].second[0], group_info[r][c].second[1]));
						cur_group.push_back(cv::Point2f(group_info[r][c].second[2], group_info[r][c].second[3]));

						pre_group.push_back(cv::Point2f(pre_group_info[m_r][m_c].first[0], pre_group_info[m_r][m_c].first[1]));
						pre_group.push_back(cv::Point2f(pre_group_info[m_r][m_c].first[2], pre_group_info[m_r][m_c].first[3]));
						pre_group.push_back(cv::Point2f(pre_group_info[m_r][m_c].second[0], pre_group_info[m_r][m_c].second[1]));
						pre_group.push_back(cv::Point2f(pre_group_info[m_r][m_c].second[2], pre_group_info[m_r][m_c].second[3]));
						for (int i = 0; i < 4; ++i) {
							//cv::Mat xn1 = (cv::Mat_<float>(3, 1) << (kp1.pt.x - cx1)*invfx1, (kp1.pt.y - cy1)*invfy1, 1.0);
							//cv::Mat xn2 = (cv::Mat_<float>(3, 1) << (kp2.pt.x - cx2)*invfx2, (kp2.pt.y - cy2)*invfy2, 1.0);
							cv::Mat xn1 = (cv::Mat_<float>(3, 1) << (cur_group[i].x - cx1)*invfx1,
										   (cur_group[i].y - cy1)*invfy1, 1.0);
							cv::Mat xn2 = (cv::Mat_<float>(3, 1) << (pre_group[i].x - cx2)*invfx2,
										   (pre_group[i].y - cy2)*invfy2, 1.0);

							// Linear Triangulation Method
							cv::Mat A(4, 4, CV_32F);
							A.row(0) = xn1.at<float>(0)*Tcw1.row(2) - Tcw1.row(0);
							A.row(1) = xn1.at<float>(1)*Tcw1.row(2) - Tcw1.row(1);
							A.row(2) = xn2.at<float>(0)*Tcw2.row(2) - Tcw2.row(0);
							A.row(3) = xn2.at<float>(1)*Tcw2.row(2) - Tcw2.row(1);
							//cout << "A:" << A << endl;

							cv::Mat w, u, vt;
							cv::SVD::compute(A, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

							cv::Mat x3D = vt.row(3).t();

							if (x3D.at<float>(3) == 0) {
								cout << "x3D(3) is zero" << endl;
								continue;
							}

							// Euclidean coordinates
							x3D = x3D.rowRange(0, 3) / x3D.at<float>(3);
							tmp_pos.push_back(cv::Point3f(x3D.at<float>(0), x3D.at<float>(1), x3D.at<float>(2)));
						}
						if (tmp_pos.size() == 4) {
							tmp_panel3dPos.push_back(tmp_pos);
						}
						else {
							cout << "tmp_pos is not 4," << tmp_pos.size() << endl;
						}
					}
				}

				if (tmp_panel3dPos.size() > 0)
					cout << "Have new panel:" << tmp_panel3dPos.size() << endl;
					panel3dPos = tmp_panel3dPos;
				
			}


		}
		*/
	}
}

} //namespace ORB_SLAM
