/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef MAPDRAWER_H
#define MAPDRAWER_H

#include"Atlas.h"
#include"MapPoint.h"
#include"KeyFrame.h"
#include "Settings.h"
#include<pangolin/pangolin.h>

#include<mutex>

namespace ORB_SLAM3
{

class Settings;

class MapDrawer
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MapDrawer(Atlas* pAtlas, const string &strSettingPath, Settings* settings);

    void newParameterLoader(Settings* settings);

    Atlas* mpAtlas;

    void DrawMapPoints();
    void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph, const bool bDrawInertialGraph, const bool bDrawOptLba);
    void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);
    void SetCurrentCameraPose(const Sophus::SE3f &Tcw);
    void SetReferenceKeyFrame(KeyFrame *pKF);
    void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M, pangolin::OpenGlMatrix &MOw);

    void DrawMapLines()
    {
        const vector<MapLine*> &vpMLs = mpAtlas->GetAllMapLines();
        const vector<MapLine*> &vpRefMLs = mpAtlas->GetReferenceMapLines();
        // const vector<MapLine*> &vpRefMLs = mpAtlas->GetCurrentMapLines();
        set<MapLine*> spRefMLs(vpRefMLs.begin(), vpRefMLs.end());
        if(vpMLs.empty()) return;
        float mLineSize = 1.0;
        glLineWidth(mLineSize);
        glColor3f(0.0,0.0,0.0);
        glBegin(GL_LINES);
        for(size_t i=0, iend=vpMLs.size(); i<iend;i++)
        {
            if(vpMLs[i]->isBad() || spRefMLs.count(vpMLs[i])) continue;
            Vector6d sep = vpMLs[i]->GetWorldPos();
            Eigen::Vector3f sp_eigen = sep.head<3>();
            Eigen::Vector3f ep_eigen = sep.tail<3>();
            glVertex3f(static_cast<float>(sp_eigen(0)),static_cast<float>(sp_eigen(1)),static_cast<float>(sp_eigen(2)));
            glVertex3f(static_cast<float>(ep_eigen(0)),static_cast<float>(ep_eigen(1)),static_cast<float>(ep_eigen(2)));
        }
        glEnd();
        glPointSize(mLineSize);
        glColor3f(1.0,0.0,0.0);
        glBegin(GL_LINES);
        for(set<MapLine*>::iterator sit=spRefMLs.begin(), send=spRefMLs.end(); sit!=send; sit++)
        {
            if((*sit)->isBad()) continue;
            Vector6d sep = (*sit)->GetWorldPos();
            Eigen::Vector3f sp_eigen = sep.head<3>();
            Eigen::Vector3f ep_eigen = sep.tail<3>();
            glVertex3f(static_cast<float>(sp_eigen(0)),static_cast<float>(sp_eigen(1)),static_cast<float>(sp_eigen(2)));
            glVertex3f(static_cast<float>(ep_eigen(0)),static_cast<float>(ep_eigen(1)),static_cast<float>(ep_eigen(2)));

        }
        glEnd();
    }
private:

    bool ParseViewerParamFile(cv::FileStorage &fSettings);

    float mKeyFrameSize;
    float mKeyFrameLineWidth;
    float mGraphLineWidth;
    float mPointSize;
    float mCameraSize;
    float mCameraLineWidth;

    Sophus::SE3f mCameraPose;

    std::mutex mMutexCamera;

    float mfFrameColors[6][3] = {{0.0f, 0.0f, 1.0f},
                                {0.8f, 0.4f, 1.0f},
                                {1.0f, 0.2f, 0.4f},
                                {0.6f, 0.0f, 1.0f},
                                {1.0f, 1.0f, 0.0f},
                                {0.0f, 1.0f, 1.0f}};

};

} //namespace ORB_SLAM

#endif // MAPDRAWER_H
