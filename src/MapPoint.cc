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

#include "MapPoint.h"
#include "ORBmatcher.h"

#include<mutex>

namespace ORB_SLAM2
{

long unsigned int MapPoint::nNextId=0;
mutex MapPoint::mGlobalMutex;

MapPoint::MapPoint(const cv::Mat &Pos, KeyFrame *pRefKF, Map* pMap):
    mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), nObs(0), mnTrackReferenceForFrame(0),
    mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(pRefKF), mnVisible(1), mnFound(1), mbBad(false),
    mpReplaced(static_cast<MapPoint*>(NULL)), mfMinDistance(0), mfMaxDistance(0), mpMap(pMap)
{
    Pos.copyTo(mWorldPos);
    mNormalVector = cv::Mat::zeros(3,1,CV_32F);

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    mnId=nNextId++;
}

MapPoint::MapPoint(const cv::Mat &Pos, Map* pMap, Frame* pFrame, const int &idxF):
    mnFirstKFid(-1), mnFirstFrame(pFrame->mnId), nObs(0), mnTrackReferenceForFrame(0), mnLastFrameSeen(0),
    mnBALocalForKF(0), mnFuseCandidateForKF(0),mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(static_cast<KeyFrame*>(NULL)), mnVisible(1),
    mnFound(1), mbBad(false), mpReplaced(NULL), mpMap(pMap)
{
    Pos.copyTo(mWorldPos);
    cv::Mat Ow = pFrame->GetCameraCenter();
    mNormalVector = mWorldPos - Ow;
    mNormalVector = mNormalVector/cv::norm(mNormalVector);

    cv::Mat PC = Pos - Ow;
    const float dist = cv::norm(PC);
    const int level = pFrame->mvKeysUn[idxF].octave;
    const float levelScaleFactor =  pFrame->mvScaleFactors[level];
    const int nLevels = pFrame->mnScaleLevels;

    mfMaxDistance = dist*levelScaleFactor;
    mfMinDistance = mfMaxDistance/pFrame->mvScaleFactors[nLevels-1];

    pFrame->mDescriptors.row(idxF).copyTo(mDescriptor);

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    mnId=nNextId++;
}

MapPoint::MapPoint(Map *pMap, const size_t id) :
  mnFirstKFid(-1), mnFirstFrame(-1), nObs(0), mnTrackReferenceForFrame(0), mnLastFrameSeen(0),
  mnBALocalForKF(0), mnFuseCandidateForKF(0),mnLoopPointForKF(0), mnCorrectedByKF(0),
  mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(static_cast<KeyFrame*>(NULL)), mnVisible(1),
  mnFound(1), mbBad(false), mpReplaced(NULL), mpMap(pMap), mnId(id)
{

}

void MapPoint::SetWorldPos(const cv::Mat &Pos)
{
    unique_lock<mutex> lock2(mGlobalMutex);
    unique_lock<mutex> lock(mMutexPos);
    Pos.copyTo(mWorldPos);
}

cv::Mat MapPoint::GetWorldPos()
{
    unique_lock<mutex> lock(mMutexPos);
    return mWorldPos.clone();
}

cv::Mat MapPoint::GetNormal()
{
    unique_lock<mutex> lock(mMutexPos);
    return mNormalVector.clone();
}

KeyFrame* MapPoint::GetReferenceKeyFrame()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mpRefKF;
}

void MapPoint::AddObservation(KeyFrame* pKF, size_t idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    if(mObservations.count(pKF))
        return;
    mObservations[pKF]=idx;

    if(pKF->mvuRight[idx]>=0)
        nObs+=2;
    else
        nObs++;
}

void MapPoint::EraseObservation(KeyFrame* pKF)
{
    bool bBad=false;
    {
        unique_lock<mutex> lock(mMutexFeatures);
        if(mObservations.count(pKF))
        {
            int idx = mObservations[pKF];
            if(pKF->mvuRight[idx]>=0)
                nObs-=2;
            else
                nObs--;

            mObservations.erase(pKF);

            if(mpRefKF==pKF)
                mpRefKF=mObservations.begin()->first;

            // If only 2 observations or less, discard point
            if(nObs<=2)
                bBad=true;
        }
    }

    if(bBad)
        SetBadFlag();
}

map<KeyFrame*, size_t> MapPoint::GetObservations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mObservations;
}

int MapPoint::Observations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return nObs;
}

void MapPoint::SetBadFlag()
{
    map<KeyFrame*,size_t> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        mbBad=true;
        obs = mObservations;
        mObservations.clear();
    }
    for(map<KeyFrame*,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        pKF->EraseMapPointMatch(mit->second);
    }

    mpMap->EraseMapPoint(this);
}

MapPoint* MapPoint::GetReplaced()
{
    unique_lock<mutex> lock1(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mpReplaced;
}

void MapPoint::Replace(MapPoint* pMP)
{
    if(pMP->mnId==this->mnId)
        return;

    int nvisible, nfound;
    map<KeyFrame*,size_t> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        obs=mObservations;
        mObservations.clear();
        mbBad=true;
        nvisible = mnVisible;
        nfound = mnFound;
        mpReplaced = pMP;
    }

    for(map<KeyFrame*,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        // Replace measurement in keyframe
        KeyFrame* pKF = mit->first;

        if(!pMP->IsInKeyFrame(pKF))
        {
            pKF->ReplaceMapPointMatch(mit->second, pMP);
            pMP->AddObservation(pKF,mit->second);
        }
        else
        {
            pKF->EraseMapPointMatch(mit->second);
        }
    }
    pMP->IncreaseFound(nfound);
    pMP->IncreaseVisible(nvisible);
    pMP->ComputeDistinctiveDescriptors();

    mpMap->EraseMapPoint(this);
}

bool MapPoint::isBad()
{
    unique_lock<mutex> lock(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mbBad;
}

void MapPoint::IncreaseVisible(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnVisible+=n;
}

void MapPoint::IncreaseFound(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnFound+=n;
}

float MapPoint::GetFoundRatio()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return static_cast<float>(mnFound)/mnVisible;
}

void MapPoint::ComputeDistinctiveDescriptors()
{
    // Retrieve all observed descriptors
    vector<cv::Mat> vDescriptors;

    map<KeyFrame*,size_t> observations;

    {
        unique_lock<mutex> lock1(mMutexFeatures);
        if(mbBad)
            return;
        observations=mObservations;
    }

    if(observations.empty())
        return;

    vDescriptors.reserve(observations.size());

    for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;

        if(!pKF->isBad())
            vDescriptors.push_back(pKF->mDescriptors.row(mit->second));
    }

    if(vDescriptors.empty())
        return;

    // Compute distances between them
    const size_t N = vDescriptors.size();

    float Distances[N][N];
    for(size_t i=0;i<N;i++)
    {
        Distances[i][i]=0;
        for(size_t j=i+1;j<N;j++)
        {
            int distij = ORBmatcher::DescriptorDistance(vDescriptors[i],vDescriptors[j]);
            Distances[i][j]=distij;
            Distances[j][i]=distij;
        }
    }

    // Take the descriptor with least median distance to the rest
    int BestMedian = INT_MAX;
    int BestIdx = 0;
    for(size_t i=0;i<N;i++)
    {
        vector<int> vDists(Distances[i],Distances[i]+N);
        sort(vDists.begin(),vDists.end());
        int median = vDists[0.5*(N-1)];

        if(median<BestMedian)
        {
            BestMedian = median;
            BestIdx = i;
        }
    }

    {
        unique_lock<mutex> lock(mMutexFeatures);
        mDescriptor = vDescriptors[BestIdx].clone();
    }
}

cv::Mat MapPoint::GetDescriptor()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mDescriptor.clone();
}

int MapPoint::GetIndexInKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    if(mObservations.count(pKF))
        return mObservations[pKF];
    else
        return -1;
}

bool MapPoint::IsInKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    return (mObservations.count(pKF));
}

void MapPoint::UpdateNormalAndDepth()
{
    map<KeyFrame*,size_t> observations;
    KeyFrame* pRefKF;
    cv::Mat Pos;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        if(mbBad)
            return;
        observations=mObservations;
        pRefKF=mpRefKF;
        Pos = mWorldPos.clone();
    }

    if(observations.empty())
        return;

    cv::Mat normal = cv::Mat::zeros(3,1,CV_32F);
    int n=0;
    for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        cv::Mat Owi = pKF->GetCameraCenter();
        cv::Mat normali = mWorldPos - Owi;
        normal = normal + normali/cv::norm(normali);
        n++;
    }

    cv::Mat PC = Pos - pRefKF->GetCameraCenter();
    const float dist = cv::norm(PC);
    const int level = pRefKF->mvKeysUn[observations[pRefKF]].octave;
    const float levelScaleFactor =  pRefKF->mvScaleFactors[level];
    const int nLevels = pRefKF->mnScaleLevels;

    {
        unique_lock<mutex> lock3(mMutexPos);
        mfMaxDistance = dist*levelScaleFactor;
        mfMinDistance = mfMaxDistance/pRefKF->mvScaleFactors[nLevels-1];
        mNormalVector = normal/n;
    }
}

float MapPoint::GetMinDistanceInvariance()
{
    unique_lock<mutex> lock(mMutexPos);
    return 0.8f*mfMinDistance;
}

float MapPoint::GetMaxDistanceInvariance()
{
    unique_lock<mutex> lock(mMutexPos);
    return 1.2f*mfMaxDistance;
}

int MapPoint::PredictScale(const float &currentDist, KeyFrame* pKF)
{
    float ratio;
    {
        unique_lock<mutex> lock(mMutexPos);
        ratio = mfMaxDistance/currentDist;
    }

    int nScale = ceil(log(ratio)/pKF->mfLogScaleFactor);
    if(nScale<0)
        nScale = 0;
    else if(nScale>=pKF->mnScaleLevels)
        nScale = pKF->mnScaleLevels-1;

    return nScale;
}

int MapPoint::PredictScale(const float &currentDist, Frame* pF)
{
    float ratio;
    {
        unique_lock<mutex> lock(mMutexPos);
        ratio = mfMaxDistance/currentDist;
    }

    int nScale = ceil(log(ratio)/pF->mfLogScaleFactor);
    if(nScale<0)
        nScale = 0;
    else if(nScale>=pF->mnScaleLevels)
        nScale = pF->mnScaleLevels-1;

    return nScale;
}

void MapPoint::SaveToFile(ofstream &f)//, set<size_t> &sKnownKFs, set<size_t> &sKnownMPs)
{
//    if(this->mId.first < 10) cout << "Saving MP " << mId.first << "|" << mId.second << endl;

//    unique_lock<mutex> lockOut(mMutexOut,defer_lock);
    unique_lock<mutex> lockFeat(mMutexFeatures,defer_lock);
    unique_lock<mutex> lockPos(mMutexPos,defer_lock);

    lock(lockFeat,lockPos);

    f.write((char*)&mnId, sizeof(mnId));
    f.write((char*)&mnFirstKFid, sizeof(mnFirstKFid));
    f.write((char*)&mnFirstFrame, sizeof(mnFirstFrame));
    f.write((char*)&nObs, sizeof(nObs));
//    if(this->mId.first < 10) cout << "nObs MP " << mId.first << "|" << mId.second << ": " << nObs << endl;
    f.write((char*)&mTrackProjX, sizeof(mTrackProjX));
    f.write((char*)&mTrackProjY, sizeof(mTrackProjY));
    f.write((char*)&mbTrackInView, sizeof(mbTrackInView));
    f.write((char*)&mnTrackScaleLevel, sizeof(mnTrackScaleLevel));
    f.write((char*)&mTrackViewCos, sizeof(mTrackViewCos));
    f.write((char*)&mnTrackReferenceForFrame, sizeof(mnTrackReferenceForFrame));
    f.write((char*)&mnLastFrameSeen, sizeof(mnLastFrameSeen));
    f.write((char*)&mnBALocalForKF, sizeof(mnBALocalForKF));
    f.write((char*)&mnFuseCandidateForKF, sizeof(mnFuseCandidateForKF));
//    f.write((char*)&mInsertedWithKF, sizeof(mInsertedWithKF));
//    KeyFrame::wp(mLoopPointForKF_LC,f);
//    KeyFrame::wp(mCorrectedByKF_LC,f);
//    f.write((char*)&mCorrectedReference_LC, sizeof(mCorrectedReference_LC));
//    f.write((char*)&mbLoopCorrected, sizeof(mbLoopCorrected));
//    KeyFrame::wp(mLoopPointForKF_MM,f);
//    KeyFrame::wp(mCorrectedByKF_MM,f);
//    f.write((char*)&mCorrectedReference_MM, sizeof(mCorrectedReference_MM));
    f.write((char*)&mnBAGlobalForKF, sizeof(mnBAGlobalForKF));
//    KeyFrame::wmat(mPosGBA,f);

    KeyFrame::wmat(mWorldPos,f);
//    if(this->mId.first < 10) cout << "mWorldPos MP " << mId.first << "|" << mId.second << ": " << mWorldPos << endl;
//    KeyFrame::wmat(mRefPos,f);
//    f.write((char*)&mbPoseLock, sizeof(mbPoseLock));
//    f.write((char*)&mbPoseChanged, sizeof(mbPoseChanged));

    u_int16_t numobs = mObservations.size();
    f.write((char*)&numobs, sizeof(numobs));
//    if(this->mId.first < 10) cout << "numobs MP " << mId.first << "|" << mId.second << ": " << numobs << endl;
    for(std::map<KeyFrame*,size_t>::iterator mit = mObservations.begin();mit!=mObservations.end();++mit)
    {
        KeyFrame* pKFi = mit->first;
        size_t id = mit->second;

        f.write((char*)&pKFi->mnId, sizeof(pKFi->mnId));
        f.write((char*)&id, sizeof(id));

//        bool bLock;
//        if(mObservationsLock.count(pKFi))
//            bLock = true;
//        else
//            bLock = false;

//        f.write((char*)&bLock, sizeof(bLock));

//        if(!(sKnownKFs.count(pKFi->mId)))
//            cout << COUTERROR << "MP " << mId.first << "|" << mId.second << ": Observation KF " << pKFi->mId.first << "|" << pKFi->mId.second << " not known" << endl;
    }

//    f.write((char*)&mMaxObsKFId, sizeof(mMaxObsKFId));
    KeyFrame::wmat(mNormalVector,f);
//    if(this->mId.first < 10) cout << "mNormalVector MP " << mId.first << "|" << mId.second << ": " << mNormalVector << endl;

//    cout << "MP-mDescriptor.type: " << mDescriptor.type() << endl;
    KeyFrame::wmat(mDescriptor,f);

    if(mpRefKF)
    {
        f.write((char*)&mpRefKF->mnId, sizeof(mpRefKF->mnId));

//        if(!(sKnownKFs.count(mpRefKF->mId)))
//            cout << COUTERROR << "MP " << mId.first << "|" << mId.second << ": mpRefKF " << mpRefKF->mId.first << "|" << mpRefKF->mId.second << " not known" << endl;
    }
    else
    {
        size_t val = KFRANGE;
        f.write((char*)&val, sizeof(val));
//        f.write((char*)&val, sizeof(val));
    }


    f.write((char*)&mnVisible, sizeof(mnVisible));
    f.write((char*)&mnFound, sizeof(mnFound));
    f.write((char*)&mbBad, sizeof(mbBad));

    if(mpReplaced)
    {
        f.write((char*)&mpReplaced->mnId, sizeof(mpReplaced->mnId));

//        if(!(sKnownMPs.count(mpReplaced->mId)))
//            cout << COUTERROR << "MP " << mId.first << "|" << mId.second << ": mpReplaced MP " << mpReplaced->mId.first << "|" << mpReplaced->mId.second << " not known" << endl;
    }
    else
    {
        size_t val = MPRANGE;
        f.write((char*)&val, sizeof(val));
        f.write((char*)&val, sizeof(val));
    }

    f.write((char*)&mfMinDistance, sizeof(mfMinDistance));
    f.write((char*)&mfMaxDistance, sizeof(mfMaxDistance));

    int finalvalue = rand();
    f.write((char*)&finalvalue, sizeof(finalvalue));
//    if(this->mId.first < 10) cout << "finalvalue MP " << mId.first << "|" << mId.second << ": " << finalvalue << endl;
}

void MapPoint::LoadFromFile(ifstream &f)
{
//    if(this->mId.first < 10)
//        cout << "Loading MP " << mId.first << "|" << mId.second << endl;

    {
//        unique_lock<mutex> lockOut(mMutexOut,defer_lock);
        unique_lock<mutex> lockFeat(mMutexFeatures,defer_lock);
        unique_lock<mutex> lockPos(mMutexPos,defer_lock);

        lock(lockFeat,lockPos);

        f.read((char*)&mnId, sizeof(mnId));
        f.read((char*)&mnFirstKFid, sizeof(mnFirstKFid));
        f.read((char*)&mnFirstFrame, sizeof(mnFirstFrame));
        f.read((char*)&nObs, sizeof(nObs));
//        if(this->mId.first < 10) cout << "nObs MP " << mId.first << "|" << mId.second << ": " << nObs << endl;
        f.read((char*)&mTrackProjX, sizeof(mTrackProjX));
        f.read((char*)&mTrackProjY, sizeof(mTrackProjY));
        f.read((char*)&mbTrackInView, sizeof(mbTrackInView));
        f.read((char*)&mnTrackScaleLevel, sizeof(mnTrackScaleLevel));
        f.read((char*)&mTrackViewCos, sizeof(mTrackViewCos));
        f.read((char*)&mnTrackReferenceForFrame, sizeof(mnTrackReferenceForFrame));
        f.read((char*)&mnLastFrameSeen, sizeof(mnLastFrameSeen));
        f.read((char*)&mnBALocalForKF, sizeof(mnBALocalForKF));
        f.read((char*)&mnFuseCandidateForKF, sizeof(mnFuseCandidateForKF));
//        f.read((char*)&mInsertedWithKF, sizeof(mInsertedWithKF));
//        KeyFrame::rp(mLoopPointForKF_LC,f);
//        KeyFrame::rp(mCorrectedByKF_LC,f);
//        f.read((char*)&mCorrectedReference_LC, sizeof(mCorrectedReference_LC));
//        f.read((char*)&mbLoopCorrected, sizeof(mbLoopCorrected));
//        KeyFrame::rp(mLoopPointForKF_MM,f);
//        KeyFrame::rp(mCorrectedByKF_MM,f);
//        f.read((char*)&mCorrectedReference_MM, sizeof(mCorrectedReference_MM));
        f.read((char*)&mnBAGlobalForKF, sizeof(mnBAGlobalForKF));
//        KeyFrame::rmat(mPosGBA,f,3,1,5);

        KeyFrame::rmat(mWorldPos,f,3,1,5);
//        if(this->mId.first < 10) cout << "mWorldPos MP " << mId.first << "|" << mId.second << ": " << mWorldPos << endl;
//        KeyFrame::rmat(mRefPos,f,3,1,5);
//        f.read((char*)&mbPoseLock, sizeof(mbPoseLock));
//        f.read((char*)&mbPoseChanged, sizeof(mbPoseChanged));

        u_int16_t numobs;
        f.read((char*)&numobs, sizeof(numobs));
//        if(this->mId.first < 10) cout << "numobs MP " << mId.first << "|" << mId.second << ": " << numobs << endl;
        for(int idx=0;idx<numobs;++idx)
        {
            size_t IDi;
            size_t id;
            bool bLock;

            f.read((char*)&IDi, sizeof(IDi));
            f.read((char*)&id, sizeof(id));
//            f.read((char*)&bLock, sizeof(bLock));

            KeyFrame* pKFi = mpMap->GetKfPtr(IDi);

//            if(!pKFi)
//                pKFi = mpMap->GetErasedKfPtr(IDi);

            if(!pKFi)
                pKFi = mpMap->ReserveKF(IDi);

            if(!pKFi)
                cout << "ERROR: Keyframe does not exist!!" << endl;

            mObservations[pKFi] = id;

//            if(bLock)
//                mObservationsLock[pKFi] = true;
        }

//        f.read((char*)&mMaxObsKFId, sizeof(mMaxObsKFId));
        KeyFrame::rmat(mNormalVector,f,3,1,5);
//        if(this->mId.first < 10) cout << "mNormalVector MP " << mId.first << "|" << mId.second << ": " << mNormalVector << endl;

        KeyFrame::rmat(mDescriptor,f,1,32,0);

        {
            size_t IDi;
            f.read((char*)&IDi, sizeof(IDi));
            if(IDi == KFRANGE)
                mpRefKF = nullptr;
            else
            {
                KeyFrame* pKFi = mpMap->GetKfPtr(IDi);

//                if(!pKFi)
//                    pKFi = mpMap->GetErasedKfPtr(IDi);

                if(!pKFi)
                    pKFi = mpMap->ReserveKF(IDi);

                if(!pKFi)
                    cout << "ERROR: Keyframe does not exist!!" << endl;

                mpRefKF = pKFi;
            }
        }

        f.read((char*)&mnVisible, sizeof(mnVisible));
        f.read((char*)&mnFound, sizeof(mnFound));
        f.read((char*)&mbBad, sizeof(mbBad));

        {
            size_t IDi;
            f.read((char*)&IDi, sizeof(IDi));
            if(IDi == MPRANGE)
                mpReplaced = nullptr;
            else
            {
                MapPoint* pMPi = mpMap->GetMpPtr(IDi);

//                if(!pMPi)
//                    pMPi = mpMap->GetErasedMpPtr(IDi);

                if(!pMPi)
                    pMPi = mpMap->ReserveMP(IDi);

                if(!pMPi)
                    cout << "ERROR: Map point does not exist!!" << endl;

                mpReplaced = pMPi;
            }
        }

        f.read((char*)&mfMinDistance, sizeof(mfMinDistance));
        f.read((char*)&mfMaxDistance, sizeof(mfMaxDistance));

        int finalvalue;
        f.read((char*)&finalvalue, sizeof(finalvalue));
//        if(this->mId.first < 10) cout << "finalvalue MP " << mId.first << "|" << mId.second << ": " << finalvalue << endl;
    }


    //------------
//    mbIsEmpty = false;
}

} //namespace ORB_SLAM
