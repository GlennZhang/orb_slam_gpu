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

#ifndef MAP_H
#define MAP_H

#include "MapPoint.h"
#include "KeyFrame.h"
#include <set>

#include <mutex>

#define ITER(it, arr) for(auto it=arr.begin(), _it_end_=arr.end(); it!=_it_end_; ++it)

namespace ORB_SLAM2
{

class MapPoint;
class KeyFrame;

class Map
{
public:
    Map();

    void AddKeyFrame(KeyFrame* pKF);
    void AddMapPoint(MapPoint* pMP);
    void EraseMapPoint(MapPoint* pMP);
    void EraseKeyFrame(KeyFrame* pKF);
    void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs);

    std::vector<KeyFrame*> GetAllKeyFrames();
    std::vector<MapPoint*> GetAllMapPoints();
    std::vector<MapPoint*> GetReferenceMapPoints();
	std::vector<std::vector<cv::Point3f>> GetPanelPos();

    long unsigned int MapPointsInMap();
    long unsigned  KeyFramesInMap();

    long unsigned int GetMaxKFid();

    void clear();

    vector<KeyFrame*> mvpKeyFrameOrigins;
	
    std::mutex mMutexMapUpdate;

    // This avoid that two points are created simultaneously in separate threads (id conflict)
    std::mutex mMutexPointCreation;

	void SetReferencePanel(const int id,
						   const std::map<int, std::map<int, std::pair<cv::Vec4f, cv::Vec4f>>> &group_info);

	//void UpdateRegister(const int id_c, KeyFrame*key_f_c, const int id_p, KeyFrame*key_f_p, cv::Mat& camera_matrix, cv::Mat& distCoeffs);
	cv::Mat camera_matrix;
	cv::Mat distCoeffs;
	void UpdateRegister(const int id_c, KeyFrame*key_f_c);

protected:
    std::set<MapPoint*> mspMapPoints;
    std::set<KeyFrame*> mspKeyFrames;
	std::vector<std::vector<cv::Point3f>> panel3dPos;

    std::vector<MapPoint*> mvpReferenceMapPoints;

    long unsigned int mnMaxKFid;

    std::mutex mMutexMap;
	std::mutex mMutexPanle3d;

	map<int, std::map<int, std::map<int, std::pair<cv::Vec4f, cv::Vec4f>>>> panelGroupInfo_;
	int last_process_;
	std::mutex mMutexPanelUpdate;

	map<int, KeyFrame*> keyFrameMap;

};

} //namespace ORB_SLAM

#endif // MAP_H
