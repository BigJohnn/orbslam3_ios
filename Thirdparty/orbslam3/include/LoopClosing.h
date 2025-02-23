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


#ifndef LOOPCLOSING_H
#define LOOPCLOSING_H

#include "KeyFrame.h"
#include "LocalMapping.h"
#include "Atlas.h"
#include "ORBVocabulary.h"
#include "Tracking.h"

#include "KeyFrameDatabase.h"

#include <boost/algorithm/string.hpp>
#include <thread>
#include <mutex>
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

namespace ORB_SLAM3
{

class Tracking;
class LocalMapping;
class KeyFrameDatabase;
class Map;


class LoopClosing
{
public:

    typedef pair<set<std::shared_ptr<KeyFrame>>,int> ConsistentGroup;    
    typedef map<std::shared_ptr<KeyFrame>,g2o::Sim3,std::less<std::shared_ptr<KeyFrame>>,
        Eigen::aligned_allocator<std::pair<std::shared_ptr<KeyFrame> const, g2o::Sim3> > > KeyFrameAndPose;

public:

    LoopClosing(Atlas* pAtlas, KeyFrameDatabase* pDB, ORBVocabulary* pVoc,const bool bFixScale, const bool bActiveLC);

    void SetTracker(Tracking* pTracker);

    void SetLocalMapper(LocalMapping* pLocalMapper);

    // Main function
    void Run();

    void InsertKeyFrame(std::shared_ptr<KeyFrame> const& pKF);

    void RequestReset();
    void RequestResetActiveMap(std::shared_ptr<Map> pMap);

    // This function will run in a separate thread
    void RunGlobalBundleAdjustment(std::shared_ptr<Map> pActiveMap, unsigned long nLoopKF);

    bool isRunningGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        return mbRunningGBA;
    }
    bool isFinishedGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        return mbFinishedGBA;
    }   

    void RequestFinish();

    bool isFinished();

    Viewer* mpViewer;

#ifdef REGISTER_TIMES

    vector<double> vdDataQuery_ms;
    vector<double> vdEstSim3_ms;
    vector<double> vdPRTotal_ms;

    vector<double> vdMergeMaps_ms;
    vector<double> vdWeldingBA_ms;
    vector<double> vdMergeOptEss_ms;
    vector<double> vdMergeTotal_ms;
    vector<int> vnMergeKFs;
    vector<int> vnMergeMPs;
    int nMerges;

    vector<double> vdLoopFusion_ms;
    vector<double> vdLoopOptEss_ms;
    vector<double> vdLoopTotal_ms;
    vector<int> vnLoopKFs;
    int nLoop;

    vector<double> vdGBA_ms;
    vector<double> vdUpdateMap_ms;
    vector<double> vdFGBATotal_ms;
    vector<int> vnGBAKFs;
    vector<int> vnGBAMPs;
    int nFGBA_exec;
    int nFGBA_abort;

#endif

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:

    bool CheckNewKeyFrames();


    //Methods to implement the new place recognition algorithm
    bool NewDetectCommonRegions();
    bool DetectAndReffineSim3FromLastKF(std::shared_ptr<KeyFrame> pCurrentKF, std::shared_ptr<KeyFrame> pMatchedKF, g2o::Sim3 &gScw, int &nNumProjMatches,
                                        std::vector<std::shared_ptr<MapPoint>> &vpMPs, std::vector<std::shared_ptr<MapPoint>> &vpMatchedMPs);
    bool DetectCommonRegionsFromBoW(std::vector<std::shared_ptr<KeyFrame>> &vpBowCand, std::shared_ptr<KeyFrame> &pMatchedKF, std::shared_ptr<KeyFrame> &pLastCurrentKF, g2o::Sim3 &g2oScw,
                                     int &nNumCoincidences, std::vector<std::shared_ptr<MapPoint>> &vpMPs, std::vector<std::shared_ptr<MapPoint>> &vpMatchedMPs);
    bool DetectCommonRegionsFromLastKF(std::shared_ptr<KeyFrame> pCurrentKF, std::shared_ptr<KeyFrame> pMatchedKF, g2o::Sim3 &gScw, int &nNumProjMatches,
                                            std::vector<std::shared_ptr<MapPoint>> &vpMPs, std::vector<std::shared_ptr<MapPoint>> &vpMatchedMPs);
    int FindMatchesByProjection(std::shared_ptr<KeyFrame> pCurrentKF, std::shared_ptr<KeyFrame> pMatchedKFw, g2o::Sim3 &g2oScw,
                                set<std::shared_ptr<MapPoint>> &spMatchedMPinOrigin, vector<std::shared_ptr<MapPoint>> &vpMapPoints,
                                vector<std::shared_ptr<MapPoint>> &vpMatchedMapPoints);


    void SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap, vector<std::shared_ptr<MapPoint>> &vpMapPoints);
    void SearchAndFuse(const vector<std::shared_ptr<KeyFrame>> &vConectedKFs, vector<std::shared_ptr<MapPoint>> &vpMapPoints);

    void CorrectLoop();

    void MergeLocal();
    void MergeLocal2();

    void CheckObservations(set<std::shared_ptr<KeyFrame>> &spKFsMap1, set<std::shared_ptr<KeyFrame>> &spKFsMap2);

    void ResetIfRequested();
    bool mbResetRequested;
    bool mbResetActiveMapRequested;
    std::shared_ptr<Map> mpMapToReset;
    std::mutex mMutexReset;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    Atlas* mpAtlas;
    Tracking* mpTracker;

    KeyFrameDatabase* mpKeyFrameDB;
    ORBVocabulary* mpORBVocabulary;

    LocalMapping *mpLocalMapper;

    std::list<std::shared_ptr<KeyFrame>> mlpLoopKeyFrameQueue;

    std::mutex mMutexLoopQueue;

    // Loop detector parameters
    float mnCovisibilityConsistencyTh;

    // Loop detector variables
    std::shared_ptr<KeyFrame> mpCurrentKF;
    std::shared_ptr<KeyFrame> mpLastCurrentKF;
    std::shared_ptr<KeyFrame> mpMatchedKF;
    std::vector<ConsistentGroup> mvConsistentGroups;
    std::vector<std::shared_ptr<KeyFrame>> mvpEnoughConsistentCandidates;
    std::vector<std::shared_ptr<KeyFrame>> mvpCurrentConnectedKFs;
    std::vector<std::shared_ptr<MapPoint>> mvpCurrentMatchedPoints;
    std::vector<std::shared_ptr<MapPoint>> mvpLoopMapPoints;
    cv::Mat mScw;
    g2o::Sim3 mg2oScw;

    //-------
    std::shared_ptr<Map> mpLastMap;

    bool mbLoopDetected;
    int mnLoopNumCoincidences;
    int mnLoopNumNotFound;
    std::shared_ptr<KeyFrame> mpLoopLastCurrentKF;
    g2o::Sim3 mg2oLoopSlw;
    g2o::Sim3 mg2oLoopScw;
    std::shared_ptr<KeyFrame> mpLoopMatchedKF;
    std::vector<std::shared_ptr<MapPoint>> mvpLoopMPs;
    std::vector<std::shared_ptr<MapPoint>> mvpLoopMatchedMPs;
    bool mbMergeDetected;
    int mnMergeNumCoincidences;
    int mnMergeNumNotFound;
    std::shared_ptr<KeyFrame> mpMergeLastCurrentKF;
    g2o::Sim3 mg2oMergeSlw;
    g2o::Sim3 mg2oMergeSmw;
    g2o::Sim3 mg2oMergeScw;
    std::shared_ptr<KeyFrame> mpMergeMatchedKF;
    std::vector<std::shared_ptr<MapPoint>> mvpMergeMPs;
    std::vector<std::shared_ptr<MapPoint>> mvpMergeMatchedMPs;
    std::vector<std::shared_ptr<KeyFrame>> mvpMergeConnectedKFs;

    g2o::Sim3 mSold_new;
    //-------

    long unsigned int mLastLoopKFid;

    // Variables related to Global Bundle Adjustment
    bool mbRunningGBA;
    bool mbFinishedGBA;
    bool mbStopGBA;
    std::mutex mMutexGBA;
    std::thread* mpThreadGBA;

    // Fix scale in the stereo/RGB-D case
    bool mbFixScale;


    int mnFullBAIdx;



    vector<double> vdPR_CurrentTime;
    vector<double> vdPR_MatchedTime;
    vector<int> vnPR_TypeRecogn;

    //DEBUG
    string mstrFolderSubTraj;
    int mnNumCorrection;
    int mnCorrectionGBA;


    // To (de)activate LC
    bool mbActiveLC = true;

#ifdef REGISTER_LOOP
    string mstrFolderLoop;
#endif
};

} //namespace ORB_SLAM

#endif // LOOPCLOSING_H
