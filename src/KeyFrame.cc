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

#include "KeyFrame.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include<mutex>

namespace ORB_SLAM2
{

long unsigned int KeyFrame::nNextId=0;

KeyFrame::KeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB):
    mnFrameId(F.mnId),  mTimeStamp(F.mTimeStamp), mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
    mfGridElementWidthInv(F.mfGridElementWidthInv), mfGridElementHeightInv(F.mfGridElementHeightInv),
    mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0),
    mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnBAGlobalForKF(0),
    fx(F.fx), fy(F.fy), cx(F.cx), cy(F.cy), invfx(F.invfx), invfy(F.invfy),
    mbf(F.mbf), mb(F.mb), mThDepth(F.mThDepth), N(F.N), mvKeys(F.mvKeys), mvKeysUn(F.mvKeysUn),
    mvuRight(F.mvuRight), mvDepth(F.mvDepth), mDescriptors(F.mDescriptors.clone()),
    mBowVec(F.mBowVec), mFeatVec(F.mFeatVec), mnScaleLevels(F.mnScaleLevels), mfScaleFactor(F.mfScaleFactor),
    mfLogScaleFactor(F.mfLogScaleFactor), mvScaleFactors(F.mvScaleFactors), mvLevelSigma2(F.mvLevelSigma2),
    mvInvLevelSigma2(F.mvInvLevelSigma2), mnMinX(F.mnMinX), mnMinY(F.mnMinY), mnMaxX(F.mnMaxX),
    mnMaxY(F.mnMaxY), mK(F.mK), mvpMapPoints(F.mvpMapPoints), mpKeyFrameDB(pKFDB),
    mpORBvocabulary(F.mpORBvocabulary), mbFirstConnection(true), mpParent(NULL), mbNotErase(false),
    mbToBeErased(false), mbBad(false), mHalfBaseline(F.mb/2), mpMap(pMap)
{
    mnId=nNextId++;

    mGrid.resize(mnGridCols);
    for(int i=0; i<mnGridCols;i++)
    {
        mGrid[i].resize(mnGridRows);
        for(int j=0; j<mnGridRows; j++)
            mGrid[i][j] = F.mGrid[i][j];
    }

    SetPose(F.mTcw);    
}

KeyFrame::KeyFrame(Map *pMap, const size_t id, ORBVocabulary* pVoc,
                   KeyFrameDatabase* pKFDB) :
  mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
  mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0),
  mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnBAGlobalForKF(0),
  mbFirstConnection(true), mpParent(NULL), mbNotErase(false),
  mbToBeErased(false), mbBad(false), mpMap(pMap), mnId(id), mnFrameId(id),
  mpORBvocabulary(pVoc), mpKeyFrameDB(pKFDB)
{


}

void KeyFrame::ComputeBoW()
{
    if(mBowVec.empty() || mFeatVec.empty())
    {
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
        // Feature vector associate features with nodes in the 4th level (from leaves up)
        // We assume the vocabulary tree has 6 levels, change the 4 otherwise
        mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
    }
}

void KeyFrame::SetPose(const cv::Mat &Tcw_)
{
    unique_lock<mutex> lock(mMutexPose);
    mTcp = cv::Mat(4, 4, 5);
    Tcw_.copyTo(Tcw);
    cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
    cv::Mat tcw = Tcw.rowRange(0,3).col(3);
    cv::Mat Rwc = Rcw.t();
    Ow = -Rwc*tcw;

    Twc = cv::Mat::eye(4,4,Tcw.type());
    Rwc.copyTo(Twc.rowRange(0,3).colRange(0,3));
    Ow.copyTo(Twc.rowRange(0,3).col(3));
    cv::Mat center = (cv::Mat_<float>(4,1) << mHalfBaseline, 0 , 0, 1);
    Cw = Twc*center;
}

cv::Mat KeyFrame::GetPose()
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw.clone();
}

cv::Mat KeyFrame::GetPoseInverse()
{
    unique_lock<mutex> lock(mMutexPose);
    return Twc.clone();
}

cv::Mat KeyFrame::GetCameraCenter()
{
    unique_lock<mutex> lock(mMutexPose);
    return Ow.clone();
}

cv::Mat KeyFrame::GetStereoCenter()
{
    unique_lock<mutex> lock(mMutexPose);
    return Cw.clone();
}


cv::Mat KeyFrame::GetRotation()
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw.rowRange(0,3).colRange(0,3).clone();
}

cv::Mat KeyFrame::GetTranslation()
{
    unique_lock<mutex> lock(mMutexPose);
    return Tcw.rowRange(0,3).col(3).clone();
}

void KeyFrame::AddConnection(KeyFrame *pKF, const int &weight)
{
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(!mConnectedKeyFrameWeights.count(pKF))
            mConnectedKeyFrameWeights[pKF]=weight;
        else if(mConnectedKeyFrameWeights[pKF]!=weight)
            mConnectedKeyFrameWeights[pKF]=weight;
        else
            return;
    }

    UpdateBestCovisibles();
}

void KeyFrame::UpdateBestCovisibles()
{
    unique_lock<mutex> lock(mMutexConnections);
    vector<pair<int,KeyFrame*> > vPairs;
    vPairs.reserve(mConnectedKeyFrameWeights.size());
    for(map<KeyFrame*,int>::iterator mit=mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
       vPairs.push_back(make_pair(mit->second,mit->first));

    sort(vPairs.begin(),vPairs.end());
    list<KeyFrame*> lKFs;
    list<int> lWs;
    for(size_t i=0, iend=vPairs.size(); i<iend;i++)
    {
        lKFs.push_front(vPairs[i].second);
        lWs.push_front(vPairs[i].first);
    }

    mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(lKFs.begin(),lKFs.end());
    mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());    
}

set<KeyFrame*> KeyFrame::GetConnectedKeyFrames()
{
    unique_lock<mutex> lock(mMutexConnections);
    set<KeyFrame*> s;
    for(map<KeyFrame*,int>::iterator mit=mConnectedKeyFrameWeights.begin();mit!=mConnectedKeyFrameWeights.end();mit++)
        s.insert(mit->first);
    return s;
}

vector<KeyFrame*> KeyFrame::GetVectorCovisibleKeyFrames()
{
    unique_lock<mutex> lock(mMutexConnections);
    return mvpOrderedConnectedKeyFrames;
}

vector<KeyFrame*> KeyFrame::GetBestCovisibilityKeyFrames(const int &N)
{
    unique_lock<mutex> lock(mMutexConnections);
    if((int)mvpOrderedConnectedKeyFrames.size()<N)
        return mvpOrderedConnectedKeyFrames;
    else
        return vector<KeyFrame*>(mvpOrderedConnectedKeyFrames.begin(),mvpOrderedConnectedKeyFrames.begin()+N);

}

vector<KeyFrame*> KeyFrame::GetCovisiblesByWeight(const int &w)
{
    unique_lock<mutex> lock(mMutexConnections);

    if(mvpOrderedConnectedKeyFrames.empty())
        return vector<KeyFrame*>();

    vector<int>::iterator it = upper_bound(mvOrderedWeights.begin(),mvOrderedWeights.end(),w,KeyFrame::weightComp);
    if(it==mvOrderedWeights.end())
        return vector<KeyFrame*>();
    else
    {
        int n = it-mvOrderedWeights.begin();
        return vector<KeyFrame*>(mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.begin()+n);
    }
}

int KeyFrame::GetWeight(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexConnections);
    if(mConnectedKeyFrameWeights.count(pKF))
        return mConnectedKeyFrameWeights[pKF];
    else
        return 0;
}

void KeyFrame::AddMapPoint(MapPoint *pMP, const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mvpMapPoints[idx]=pMP;
}

void KeyFrame::EraseMapPointMatch(const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mvpMapPoints[idx]=static_cast<MapPoint*>(NULL);
}

void KeyFrame::EraseMapPointMatch(MapPoint* pMP)
{
    int idx = pMP->GetIndexInKeyFrame(this);
    if(idx>=0)
        mvpMapPoints[idx]=static_cast<MapPoint*>(NULL);
}


void KeyFrame::ReplaceMapPointMatch(const size_t &idx, MapPoint* pMP)
{
    mvpMapPoints[idx]=pMP;
}

set<MapPoint*> KeyFrame::GetMapPoints()
{
    unique_lock<mutex> lock(mMutexFeatures);
    set<MapPoint*> s;
    for(size_t i=0, iend=mvpMapPoints.size(); i<iend; i++)
    {
        if(!mvpMapPoints[i])
            continue;
        MapPoint* pMP = mvpMapPoints[i];
        if(!pMP->isBad())
            s.insert(pMP);
    }
    return s;
}

int KeyFrame::TrackedMapPoints(const int &minObs)
{
    unique_lock<mutex> lock(mMutexFeatures);

    int nPoints=0;
    const bool bCheckObs = minObs>0;
    for(int i=0; i<N; i++)
    {
        MapPoint* pMP = mvpMapPoints[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                if(bCheckObs)
                {
                    if(mvpMapPoints[i]->Observations()>=minObs)
                        nPoints++;
                }
                else
                    nPoints++;
            }
        }
    }

    return nPoints;
}

vector<MapPoint*> KeyFrame::GetMapPointMatches()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mvpMapPoints;
}

MapPoint* KeyFrame::GetMapPoint(const size_t &idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mvpMapPoints[idx];
}

void KeyFrame::UpdateConnections()
{
    map<KeyFrame*,int> KFcounter;

    vector<MapPoint*> vpMP;

    {
        unique_lock<mutex> lockMPs(mMutexFeatures);
        vpMP = mvpMapPoints;
    }

    //For all map points in keyframe check in which other keyframes are they seen
    //Increase counter for those keyframes
    for(vector<MapPoint*>::iterator vit=vpMP.begin(), vend=vpMP.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;

        if(!pMP)
            continue;

        if(pMP->isBad())
            continue;

        map<KeyFrame*,size_t> observations = pMP->GetObservations();

        for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            if(mit->first->mnId==mnId)
                continue;
            KFcounter[mit->first]++;
        }
    }

    // This should not happen
    if(KFcounter.empty())
        return;

    //If the counter is greater than threshold add connection
    //In case no keyframe counter is over threshold add the one with maximum counter
    int nmax=0;
    KeyFrame* pKFmax=NULL;
    int th = 15;

    vector<pair<int,KeyFrame*> > vPairs;
    vPairs.reserve(KFcounter.size());
    for(map<KeyFrame*,int>::iterator mit=KFcounter.begin(), mend=KFcounter.end(); mit!=mend; mit++)
    {
        if(mit->second>nmax)
        {
            nmax=mit->second;
            pKFmax=mit->first;
        }
        if(mit->second>=th)
        {
            vPairs.push_back(make_pair(mit->second,mit->first));
            (mit->first)->AddConnection(this,mit->second);
        }
    }

    if(vPairs.empty())
    {
        vPairs.push_back(make_pair(nmax,pKFmax));
        pKFmax->AddConnection(this,nmax);
    }

    sort(vPairs.begin(),vPairs.end());
    list<KeyFrame*> lKFs;
    list<int> lWs;
    for(size_t i=0; i<vPairs.size();i++)
    {
        lKFs.push_front(vPairs[i].second);
        lWs.push_front(vPairs[i].first);
    }

    {
        unique_lock<mutex> lockCon(mMutexConnections);

        // mspConnectedKeyFrames = spConnectedKeyFrames;
        mConnectedKeyFrameWeights = KFcounter;
        mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(lKFs.begin(),lKFs.end());
        mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());

        if(mbFirstConnection && mnId!=0)
        {
            mpParent = mvpOrderedConnectedKeyFrames.front();
            mpParent->AddChild(this);
            mbFirstConnection = false;
        }

    }
}

void KeyFrame::AddChild(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mspChildrens.insert(pKF);
}

void KeyFrame::EraseChild(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mspChildrens.erase(pKF);
}

void KeyFrame::ChangeParent(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mpParent = pKF;
    pKF->AddChild(this);
}

set<KeyFrame*> KeyFrame::GetChilds()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspChildrens;
}

KeyFrame* KeyFrame::GetParent()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mpParent;
}

bool KeyFrame::hasChild(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspChildrens.count(pKF);
}

void KeyFrame::AddLoopEdge(KeyFrame *pKF)
{
    unique_lock<mutex> lockCon(mMutexConnections);
    mbNotErase = true;
    mspLoopEdges.insert(pKF);
}

set<KeyFrame*> KeyFrame::GetLoopEdges()
{
    unique_lock<mutex> lockCon(mMutexConnections);
    return mspLoopEdges;
}

void KeyFrame::SetNotErase()
{
    unique_lock<mutex> lock(mMutexConnections);
    mbNotErase = true;
}

void KeyFrame::SetErase()
{
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(mspLoopEdges.empty())
        {
            mbNotErase = false;
        }
    }

    if(mbToBeErased)
    {
        SetBadFlag();
    }
}

void KeyFrame::SetBadFlag()
{   
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(mnId==0)
            return;
        else if(mbNotErase)
        {
            mbToBeErased = true;
            return;
        }
    }

    for(map<KeyFrame*,int>::iterator mit = mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
        mit->first->EraseConnection(this);

    for(size_t i=0; i<mvpMapPoints.size(); i++)
        if(mvpMapPoints[i])
            mvpMapPoints[i]->EraseObservation(this);
    {
        unique_lock<mutex> lock(mMutexConnections);
        unique_lock<mutex> lock1(mMutexFeatures);

        mConnectedKeyFrameWeights.clear();
        mvpOrderedConnectedKeyFrames.clear();

        // Update Spanning Tree
        set<KeyFrame*> sParentCandidates;
        sParentCandidates.insert(mpParent);

        // Assign at each iteration one children with a parent (the pair with highest covisibility weight)
        // Include that children as new parent candidate for the rest
        while(!mspChildrens.empty())
        {
            bool bContinue = false;

            int max = -1;
            KeyFrame* pC;
            KeyFrame* pP;

            for(set<KeyFrame*>::iterator sit=mspChildrens.begin(), send=mspChildrens.end(); sit!=send; sit++)
            {
                KeyFrame* pKF = *sit;
                if(pKF->isBad())
                    continue;

                // Check if a parent candidate is connected to the keyframe
                vector<KeyFrame*> vpConnected = pKF->GetVectorCovisibleKeyFrames();
                for(size_t i=0, iend=vpConnected.size(); i<iend; i++)
                {
                    for(set<KeyFrame*>::iterator spcit=sParentCandidates.begin(), spcend=sParentCandidates.end(); spcit!=spcend; spcit++)
                    {
                        if(vpConnected[i]->mnId == (*spcit)->mnId)
                        {
                            int w = pKF->GetWeight(vpConnected[i]);
                            if(w>max)
                            {
                                pC = pKF;
                                pP = vpConnected[i];
                                max = w;
                                bContinue = true;
                            }
                        }
                    }
                }
            }

            if(bContinue)
            {
                pC->ChangeParent(pP);
                sParentCandidates.insert(pC);
                mspChildrens.erase(pC);
            }
            else
                break;
        }

        // If a children has no covisibility links with any parent candidate, assign to the original parent of this KF
        if(!mspChildrens.empty())
            for(set<KeyFrame*>::iterator sit=mspChildrens.begin(); sit!=mspChildrens.end(); sit++)
            {
                (*sit)->ChangeParent(mpParent);
            }

        mpParent->EraseChild(this);
        mTcp = Tcw*mpParent->GetPoseInverse();
        mbBad = true;
    }


    mpMap->EraseKeyFrame(this);
    mpKeyFrameDB->erase(this);
}

bool KeyFrame::isBad()
{
    unique_lock<mutex> lock(mMutexConnections);
    return mbBad;
}

void KeyFrame::EraseConnection(KeyFrame* pKF)
{
    bool bUpdate = false;
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(mConnectedKeyFrameWeights.count(pKF))
        {
            mConnectedKeyFrameWeights.erase(pKF);
            bUpdate=true;
        }
    }

    if(bUpdate)
        UpdateBestCovisibles();
}

vector<size_t> KeyFrame::GetFeaturesInArea(const float &x, const float &y, const float &r) const
{
    vector<size_t> vIndices;
    vIndices.reserve(N);

    const int nMinCellX = max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));
    if(nMinCellX>=mnGridCols)
        return vIndices;

    const int nMaxCellX = min((int)mnGridCols-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
    if(nMinCellY>=mnGridRows)
        return vIndices;

    const int nMaxCellY = min((int)mnGridRows-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
    if(nMaxCellY<0)
        return vIndices;

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const vector<size_t> vCell = mGrid[ix][iy];
            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<r && fabs(disty)<r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

bool KeyFrame::IsInImage(const float &x, const float &y) const
{
    return (x>=mnMinX && x<mnMaxX && y>=mnMinY && y<mnMaxY);
}

cv::Mat KeyFrame::UnprojectStereo(int i)
{
    const float z = mvDepth[i];
    if(z>0)
    {
        const float u = mvKeys[i].pt.x;
        const float v = mvKeys[i].pt.y;
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);

        unique_lock<mutex> lock(mMutexPose);
        return Twc.rowRange(0,3).colRange(0,3)*x3Dc+Twc.rowRange(0,3).col(3);
    }
    else
        return cv::Mat();
}

float KeyFrame::ComputeSceneMedianDepth(const int q)
{
    vector<MapPoint*> vpMapPoints;
    cv::Mat Tcw_;
    {
        unique_lock<mutex> lock(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPose);
        vpMapPoints = mvpMapPoints;
        Tcw_ = Tcw.clone();
    }

    vector<float> vDepths;
    vDepths.reserve(N);
    cv::Mat Rcw2 = Tcw_.row(2).colRange(0,3);
    Rcw2 = Rcw2.t();
    float zcw = Tcw_.at<float>(2,3);
    for(int i=0; i<N; i++)
    {
        if(mvpMapPoints[i])
        {
            MapPoint* pMP = mvpMapPoints[i];
            cv::Mat x3Dw = pMP->GetWorldPos();
            float z = Rcw2.dot(x3Dw)+zcw;
            vDepths.push_back(z);
        }
    }

    sort(vDepths.begin(),vDepths.end());

    return vDepths[(vDepths.size()-1)/q];
}

void KeyFrame::SaveToFile(ofstream &f)
{
    unique_lock<mutex> lock1(mMutexFeatures,defer_lock);
    unique_lock<mutex> lock2(mMutexConnections,defer_lock);
    unique_lock<mutex> lock3(mMutexPose,defer_lock);

    lock(lock1,lock2,lock3);

    f.write((char*)&mTimeStamp, sizeof(mTimeStamp));
    f.write((char*)&mnFrameId, sizeof(mnFrameId));
    f.write((char*)&mnId, sizeof(mnId));
    f.write((char*)&mnGridCols, sizeof(mnGridCols));
    f.write((char*)&mnGridRows, sizeof(mnGridRows));
    f.write((char*)&mfGridElementWidthInv, sizeof(mfGridElementWidthInv));
    f.write((char*)&mfGridElementHeightInv, sizeof(mfGridElementHeightInv));
    f.write((char*)&mnTrackReferenceForFrame, sizeof(mnTrackReferenceForFrame));
    f.write((char*)&mnFuseTargetForKF, sizeof(mnFuseTargetForKF));
    f.write((char*)&mnBALocalForKF, sizeof(mnBALocalForKF));
    f.write((char*)&mnBAFixedForKF, sizeof(mnBAFixedForKF));
    f.write((char*)&mnLoopQuery, sizeof(mnLoopQuery));
    f.write((char*)&mnLoopWords, sizeof(mnLoopWords));
    f.write((char*)&mLoopScore, sizeof(mLoopScore));
    f.write((char*)&mnRelocQuery, sizeof(mnRelocQuery));
    f.write((char*)&mnRelocWords, sizeof(mnRelocWords));
    f.write((char*)&mRelocScore, sizeof(mRelocScore));
    f.write((char*)&mnBAGlobalForKF, sizeof(mnBAGlobalForKF));
    f.write((char*)&fx, sizeof(fx));
    f.write((char*)&fy, sizeof(fy));
    f.write((char*)&cx, sizeof(cx));
    f.write((char*)&cy, sizeof(cy));
    f.write((char*)&invfx, sizeof(invfx));
    f.write((char*)&invfy, sizeof(invfy));
    f.write((char*)&N, sizeof(N));

    u_int16_t numkeys = mvKeysUn.size();
    f.write((char*)&numkeys, sizeof(numkeys));
    for(int idx=0;idx<mvKeysUn.size();++idx)
    {
        cv::KeyPoint kp = mvKeysUn[idx];
        f.write((char*)&kp.pt.x, sizeof(kp.pt.x));
        f.write((char*)&kp.pt.y, sizeof(kp.pt.y));
        f.write((char*)&kp.angle, sizeof(kp.angle));
        f.write((char*)&kp.octave, sizeof(kp.octave));
        f.write((char*)&kp.response, sizeof(kp.response));
        f.write((char*)&kp.size, sizeof(kp.size));
    }

    u_int16_t numdescs = mDescriptors.rows;
    f.write((char*)&numdescs, sizeof(numdescs));
    for(int idx=0;idx<mDescriptors.rows;++idx) {
        for(int idy=0;idy<mDescriptors.cols;++idy) {
            f.write((char*)&mDescriptors.at<u_int8_t>(idx,idy), sizeof(u_int8_t));
        }
    }
    wmat(mTcp,f);

    f.write((char*)&mnScaleLevels, sizeof(mnScaleLevels));
    f.write((char*)&mfScaleFactor, sizeof(mfScaleFactor));
    f.write((char*)&mfLogScaleFactor, sizeof(mfLogScaleFactor));
    for(int idx=0;idx<mvScaleFactors.size();++idx) {
        f.write((char*)&mvScaleFactors[idx], sizeof(float));
    }
    for(int idx=0;idx<mvLevelSigma2.size();++idx) {
        f.write((char*)&mvLevelSigma2[idx], sizeof(float));
    }
    for(int idx=0;idx<mvInvLevelSigma2.size();++idx) {
        f.write((char*)&mvInvLevelSigma2[idx], sizeof(float));
    }

    f.write((char*)&mnMinX, sizeof(mnMinX));
    f.write((char*)&mnMinY, sizeof(mnMinY));
    f.write((char*)&mnMaxX, sizeof(mnMaxX));
    f.write((char*)&mnMaxY, sizeof(mnMaxY));
    wmat(mK,f);
    wmat(Tcw,f);
    wmat(Twc,f);

    u_int16_t numMPs = mvpMapPoints.size();
    f.write((char*)&numMPs, sizeof(numMPs));
    for(int idx=0;idx<mvpMapPoints.size();++idx)
    {
        MapPoint* pMPi = mvpMapPoints[idx];
        if(pMPi)
        {
          if (!pMPi->isBad()) {
            f.write((char*)&pMPi->mnId, sizeof(pMPi->mnId));
          } else {
            size_t val = MPRANGE;
            f.write((char*)&val, sizeof(val));
          }
        }
        else
        {
            size_t val = MPRANGE;
            f.write((char*)&val, sizeof(val));
        }
    }

    u_int16_t numConKFs = 0;
    for(std::map<KeyFrame*,int>::iterator mit = mConnectedKeyFrameWeights.begin();mit!=mConnectedKeyFrameWeights.end();++mit)
    {
        KeyFrame* pKFi = mit->first;
        int w = mit->second;
        if(!pKFi) {
           cout << "NULLPTR in mConnectedKeyFrameWeights" << endl;
        }
        if (!pKFi->isBad()) {
          ++numConKFs;
        }
    }
    f.write((char*)&numConKFs, sizeof(numConKFs));
    for(std::map<KeyFrame*,int>::iterator mit = mConnectedKeyFrameWeights.begin();mit!=mConnectedKeyFrameWeights.end();++mit)
    {
        KeyFrame* pKFi = mit->first;
        int w = mit->second;
        if(!pKFi)
        {
           cout << "NULLPTR in mConnectedKeyFrameWeights" << endl;
        }
        if (!pKFi->isBad()) {
          f.write((char*)&pKFi->mnId, sizeof(pKFi->mnId));
          f.write((char*)&w, sizeof(w));
        }
    }

    f.write((char*)&mbFirstConnection, sizeof(mbFirstConnection));

    if(mpParent)
    {
        f.write((char*)&mpParent->mnId, sizeof(mpParent->mnId));
    }
    else
    {
        size_t val = KFRANGE;
        f.write((char*)&val, sizeof(val));
    }

    u_int16_t numChildren = mspChildrens.size();
    f.write((char*)&numChildren, sizeof(numChildren));
    for(set<KeyFrame*>::iterator sit=mspChildrens.begin();sit!=mspChildrens.end();++sit)
    {
        KeyFrame* pKFi = *sit;
        f.write((char*)&pKFi->mnId, sizeof(pKFi->mnId));
    }

    u_int16_t numLoopEdges = mspLoopEdges.size();
    f.write((char*)&numLoopEdges, sizeof(numLoopEdges));
    for(set<KeyFrame*>::iterator sit=mspLoopEdges.begin();sit!=mspLoopEdges.end();++sit)
    {
        KeyFrame* pKFi = *sit;
        f.write((char*)&pKFi->mnId, sizeof(pKFi->mnId));
    }

    f.write((char*)&mbBad, sizeof(mbBad));

    int finalvalue = rand();
    f.write((char*)&finalvalue, sizeof(finalvalue));
}

void KeyFrame::LoadFromFile(ifstream &f)
{
    {
        unique_lock<mutex> lock1(mMutexFeatures,defer_lock);
        unique_lock<mutex> lock2(mMutexConnections,defer_lock);
        unique_lock<mutex> lock3(mMutexPose,defer_lock);

        lock(lock1,lock2,lock3);

        f.read((char*)&mTimeStamp, sizeof(mTimeStamp));
        f.read((char*)&mnFrameId, sizeof(mnFrameId));
        f.read((char*)&mnId, sizeof(mnId));
        f.read((char*)&mnGridCols, sizeof(mnGridCols));
        f.read((char*)&mnGridRows, sizeof(mnGridRows));
        f.read((char*)&mfGridElementWidthInv, sizeof(mfGridElementWidthInv));
        f.read((char*)&mfGridElementHeightInv, sizeof(mfGridElementHeightInv));
        f.read((char*)&mnTrackReferenceForFrame,sizeof(mnTrackReferenceForFrame));
        f.read((char*)&mnFuseTargetForKF, sizeof(mnFuseTargetForKF));
        f.read((char*)&mnBALocalForKF, sizeof(mnBALocalForKF));
        f.read((char*)&mnBAFixedForKF, sizeof(mnBAFixedForKF));
        f.read((char*)&mnLoopQuery, sizeof(mnLoopQuery));

        f.read((char*)&mnLoopWords, sizeof(mnLoopWords));
        f.read((char*)&mLoopScore, sizeof(mLoopScore));
        f.read((char*)&mnRelocQuery, sizeof(mnRelocQuery));
        f.read((char*)&mnRelocWords, sizeof(mnRelocWords));
        f.read((char*)&mRelocScore, sizeof(mRelocScore));
        f.read((char*)&mnBAGlobalForKF, sizeof(mnBAGlobalForKF));
        f.read((char*)&fx, sizeof(fx));
        f.read((char*)&fy, sizeof(fy));
        f.read((char*)&cx, sizeof(cx));
        f.read((char*)&cy, sizeof(cy));
        f.read((char*)&invfx, sizeof(invfx));
        f.read((char*)&invfy, sizeof(invfy));
        f.read((char*)&N, sizeof(N));

        u_int16_t numkeys;
        f.read((char*)&numkeys, sizeof(numkeys));
        for(int idx=0;idx<numkeys;++idx)
        {
            cv::KeyPoint kp;
            f.read((char*)&kp.pt.x, sizeof(kp.pt.x));
            f.read((char*)&kp.pt.y, sizeof(kp.pt.y));
            f.read((char*)&kp.angle, sizeof(kp.angle));
            f.read((char*)&kp.octave, sizeof(kp.octave));
            f.read((char*)&kp.response, sizeof(kp.response));
            f.read((char*)&kp.size, sizeof(kp.size));
            mvKeysUn.push_back(kp);
        }

        u_int16_t numdescs;
        f.read((char*)&numdescs, sizeof(numdescs));
        mDescriptors = cv::Mat(numdescs,32,CV_8UC1);
        for(int idx=0;idx<numdescs;++idx) {
            for(int idy=0;idy<32;++idy)
            {
                u_int8_t val;
                f.read((char*)&val, sizeof(val));
                mDescriptors.at<u_int8_t>(idx,idy) = val;
            }
        }

        rmat(mTcp,f,4,4,5);

        f.read((char*)&mnScaleLevels, sizeof(mnScaleLevels));
        f.read((char*)&mfScaleFactor, sizeof(mfScaleFactor));
        f.read((char*)&mfLogScaleFactor, sizeof(mfLogScaleFactor));

        for(int idx=0;idx<8;++idx)
        {
            float val;
            f.read((char*)&val, sizeof(val));
            mvScaleFactors.push_back(val);
        }
        for(int idx=0;idx<8;++idx)
        {
            float val;
            f.read((char*)&val, sizeof(val));
            mvLevelSigma2.push_back(val);
        }
        for(int idx=0;idx<8;++idx)
        {
            float val;
            f.read((char*)&val, sizeof(val));
            mvInvLevelSigma2.push_back(val);
        }

        f.read((char*)&mnMinX, sizeof(mnMinX));
        f.read((char*)&mnMinY, sizeof(mnMinY));
        f.read((char*)&mnMaxX, sizeof(mnMaxX));
        f.read((char*)&mnMaxY, sizeof(mnMaxY));
        rmat(mK,f,3,3,5);



        rmat(Tcw,f,4,4,5);
        rmat(Twc,f,4,4,5);


        u_int16_t numMPs;
        f.read((char*)&numMPs, sizeof(numMPs));
        for(int idx=0;idx<numMPs;++idx)
        {
            size_t IDi;
            f.read((char*)&IDi, sizeof(IDi));

            if(IDi == MPRANGE)
            {
                mvpMapPoints.push_back(nullptr);
//                continue;
            }
            else
            {
                MapPoint* pMPi = mpMap->GetMpPtr(IDi);

                if(!pMPi)
                    pMPi = mpMap->ReserveMP(IDi);

                if(!pMPi)
                    cout << "ERROR: map point does not exist!!" << endl;

                mvpMapPoints.push_back(pMPi);
            }
        }


        u_int16_t numConKFs;
        f.read((char*)&numConKFs, sizeof(numConKFs));
        for(int idx=0;idx<numConKFs;++idx)
        {
            size_t IDi;
            f.read((char*)&IDi, sizeof(IDi));

            int w;
            f.read((char*)&w, sizeof(w));

            KeyFrame* pKFi = mpMap->GetKfPtr(IDi);

            if(!pKFi)
                pKFi = mpMap->ReserveKF(IDi, mpORBvocabulary, mpKeyFrameDB);

            if(!pKFi)
                cout << "ERROR Keyframe does not exist!!" << endl;

            mConnectedKeyFrameWeights[pKFi] = w;
        }

        f.read((char*)&mbFirstConnection, sizeof(mbFirstConnection));

        size_t IDpar;
        f.read((char*)&IDpar, sizeof(IDpar));
        if(IDpar == KFRANGE)
        {
            mpParent = nullptr;
            if(this->mnId != 0)
                cout << "WARNING: mpParent is nullptr" << endl;
        }
        else
        {
            KeyFrame* pKFi = mpMap->GetKfPtr(IDpar);

            if(!pKFi)
                pKFi = mpMap->ReserveKF(IDpar, mpORBvocabulary, mpKeyFrameDB);

            if(!pKFi)
                cout << "ERROR: Keyframe does not exist!!" << endl;

            mpParent = pKFi;
        }

        u_int16_t numChildren;
        f.read((char*)&numChildren, sizeof(numChildren));
        for(int idx=0;idx<numChildren;++idx)
        {
            size_t IDi;
            f.read((char*)&IDi, sizeof(IDi));

            KeyFrame* pKFi = mpMap->GetKfPtr(IDi);

            if(!pKFi)
                pKFi = mpMap->ReserveKF(IDi, mpORBvocabulary, mpKeyFrameDB);

            if(!pKFi)
                cout << "ERROR: Keyframe does not exist" << endl;

            mspChildrens.insert(pKFi);
        }

        u_int16_t numLoopEdges;
        f.read((char*)&numLoopEdges, sizeof(numLoopEdges));
        for(int idx=0;idx<numLoopEdges;++idx)
        {
            size_t IDi;
            f.read((char*)&IDi, sizeof(IDi));

            KeyFrame* pKFi = mpMap->GetKfPtr(IDi);

            if(!pKFi)
                pKFi = mpMap->ReserveKF(IDi, mpORBvocabulary, mpKeyFrameDB);

            if(!pKFi)
                cout << "ERROR: Keyframe does not exist!!" << endl;

            mspLoopEdges.insert(pKFi);
        }

        f.read((char*)&mbBad, sizeof(mbBad));

        int finalvalue;
        f.read((char*)&finalvalue, sizeof(finalvalue));
    }
    mvuRight.clear();
    mvuRight.resize(mvKeysUn.size(), -1.0f);
    mvDepth.clear();
    mvDepth.resize(mvKeysUn.size(), -1.0f);
    cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
    cv::Mat tcw = Tcw.rowRange(0,3).col(3);
    cv::Mat Rwc = Rcw.t();
    Ow = -Rwc*tcw;
    Rwc.copyTo(Twc.rowRange(0,3).colRange(0,3));
    Ow.copyTo(Twc.rowRange(0,3).col(3));
    cv::Mat center = (cv::Mat_<float>(4,1) << mHalfBaseline, 0 , 0, 1);
    Cw = Twc*center;
    this->AssignFeaturesToGrid();
    vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
    // Feature vector associate features with nodes in the 4th level (from leaves up)
    // We assume the vocabulary tree has 6 levels, change the 4 otherwise
    if (mpORBvocabulary == NULL) {
      cout << "the pointer to the ORB vocabulary is empty!!!" << endl;
    }
    mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
    this->UpdateBestCovisibles();
    mpKeyFrameDB->add(this);
  //  mbIsEmpty = false;
}

void KeyFrame::wmat(Mat& mat, ofstream &f)
{
    if(mat.type() == 0) //uint8_t
    {
        for(int r=0;r<mat.rows;++r)
            for(int c=0;c<mat.cols;++c)
            {
                u_int8_t val = mat.at<u_int8_t>(r,c);
                f.write((char*)&val, sizeof(val));
            }
    }
    else if(mat.type() == 5) //float
    {
        for(int r=0;r<mat.rows;++r)
            for(int c=0;c<mat.cols;++c)
            {
//                cout << "mat.at<float>(" << r << "," << c << "): " << mat.at<float>(r,c) << endl;
                float val = mat.at<float>(r,c);
                f.write((char*)&val, sizeof(val));
            }
    }
    else if(mat.type() == 6) //double
    {
        cout << "WARNING: Mat Type is DOUBLE" << endl;
        for(int r=0;r<mat.rows;++r)
            for(int c=0;c<mat.cols;++c)
            {
                double val = mat.at<double>(r,c);
                f.write((char*)&val, sizeof(val));
            }
    }
    else
        cout << "ERROR: Undefined Mat type!!" << endl;
}

void KeyFrame::rmat(Mat& mat, ifstream &f, int rows, int cols, int type)
{
    mat = cv::Mat(rows,cols,type);

    if(mat.rows == 0 || mat.cols == 0)
        cout << "WARNING: writing empty matrix" << endl;

    if(mat.type() == 0) //uint8_t
    {
        for(int r=0;r<mat.rows;++r)
            for(int c=0;c<mat.cols;++c)
            {
                u_int8_t val;
                f.read((char*)&val, sizeof(val));
                mat.at<u_int8_t>(r,c) = val;
            }
    }
    else if(mat.type() == 5) //float
    {
        for(int r=0;r<mat.rows;++r)
            for(int c=0;c<mat.cols;++c)
            {
                float val;
                f.read((char*)&val, sizeof(val));
                mat.at<float>(r,c) = val;
            }
    }
    else if(mat.type() == 6) //double
    {
        cout << "WARNING: Mat Type is DOUBLE" << endl;
        for(int r=0;r<mat.rows;++r)
            for(int c=0;c<mat.cols;++c)
            {
                double val;
                f.read((char*)&val, sizeof(val));
                mat.at<double>(r,c) = val;
            }
    }
    else
        cout << "ERROR: Undefined Mat type!!" << endl;
}

void KeyFrame::AssignFeaturesToGrid() {
  int nReserve = 0.5f*N/(mnGridCols*mnGridRows);
  mGrid.resize(mnGridCols);
  for(unsigned int i=0; i<mnGridCols;i++)
  {
    mGrid[i].resize(mnGridRows);
    for (unsigned int j=0; j<mnGridRows;j++)
      mGrid[i][j].reserve(nReserve);
  }
  for(int i=0;i<N;i++)
  {
    const cv::KeyPoint &kp = mvKeysUn[i];

    int nGridPosX, nGridPosY;
    if(PosInGrid(kp,nGridPosX,nGridPosY))
      mGrid[nGridPosX][nGridPosY].push_back(i);
  }
}

bool KeyFrame::PosInGrid(const KeyPoint &kp, int &posX, int &posY) {
  posX = round((kp.pt.x-mnMinX)*mfGridElementWidthInv);
      posY = round((kp.pt.y-mnMinY)*mfGridElementHeightInv);

      //Keypoint's coordinates are undistorted, which could cause to go out of the image
      if(posX<0 || posX>=mnGridCols || posY<0 || posY>=mnGridRows)
          return false;
  return true;
}

} //namespace ORB_SLAM
