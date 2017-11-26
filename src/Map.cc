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

Map::Map():mnMaxKFid(0),mnBigChangeIdx(0)
{
}

void Map::AddKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.insert(pKF);
    if(pKF->mnId>mnMaxKFid)
        mnMaxKFid=pKF->mnId;
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

void Map::InformNewBigChange()
{
    unique_lock<mutex> lock(mMutexMap);
    mnBigChangeIdx++;
}

int Map::GetLastBigChangeIdx()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnBigChangeIdx;
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

KeyFrame* Map::GetKfPtr(size_t KfId) {
  unique_lock<mutex> lock(mMutexMap);

  std::map<size_t,KeyFrame*>::iterator mit = mmpKeyFrames.find(KfId);
  if(mit != mmpKeyFrames.end()) return mit->second;
  else return nullptr;
}

MapPoint* Map::GetMpPtr(size_t MpId)
{
    unique_lock<mutex> lock(mMutexMap);
    std::map<size_t, MapPoint*>::iterator mit = mmpMapPoints.find(MpId);
    if(mit != mmpMapPoints.end()) return mit->second;
    else return nullptr;
}

void Map::UpdateAllConnections() {
  unique_lock<mutex> lock(mMutexMap);
  for (auto itr = mspKeyFrames.begin(); itr != mspKeyFrames.end(); ++itr) {
    KeyFrame* kfPtr = (*itr);
    kfPtr->UpdateConnections();
  }
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

void Map::SaveToFile(ofstream &f)
{
    cout << "--- Saving KFs ---" << endl;
    UpdateAllConnections();
    u_int16_t numKFs = mspKeyFrames.size();
    f.write((char*)&numKFs, sizeof(numKFs));
    for(set<KeyFrame*>::iterator sit = mspKeyFrames.begin();sit != mspKeyFrames.end();++sit)
    {
        KeyFrame* pKFi = *sit;
        f.write((char*)&pKFi->mnId, sizeof(pKFi->mnId));
        pKFi->SaveToFile(f);
    }
    cout << "--- Saving MPs ---" << endl;

    u_int16_t numMPs = mspMapPoints.size();
    f.write((char*)&numMPs, sizeof(numMPs));
    for(set<MapPoint*>::iterator sit = mspMapPoints.begin();sit != mspMapPoints.end();++sit)
    {
        MapPoint* pMPi = *sit;

        f.write((char*)&pMPi->mnId, sizeof(pMPi->mnId));
        pMPi->SaveToFile(f);
    }

    cout << "--- Saving Map Vals ---" << endl;

    u_int16_t numOrigins = mvpKeyFrameOrigins.size();
    f.write((char*)&numOrigins, sizeof(numOrigins));
    for(int idx=0;idx<mvpKeyFrameOrigins.size();++idx)
    {
        KeyFrame* pKFi = mvpKeyFrameOrigins[idx];
        f.write((char*)&pKFi->mnId, sizeof(pKFi->mnId));
    }

    f.write((char*)&mnMaxKFid, sizeof(mnMaxKFid));

    int finalvalue = rand();
    f.write((char*)&finalvalue, sizeof(finalvalue));
    cout << "finalvalue map: " << finalvalue << endl;
}

void Map::LoadFromFile(ifstream &f, ORBVocabulary* pVoc,
                       KeyFrameDatabase* pKFDbase)
{
    cout << "--- Loading KFs ---" << endl;
    u_int16_t numKFs;
    f.read((char*)&numKFs, sizeof(numKFs));
    for(int idx=0;idx<numKFs;++idx)
    {
        size_t IDi;
        f.read((char*)&IDi, sizeof(IDi));
        KeyFrame* pKF = this->GetKfPtr(IDi);
        if(pKF)
        {
            pKF->LoadFromFile(f);
        }
        else
        {
            delete pKF;
            pKF = (new  KeyFrame(this, IDi, pVoc, pKFDbase));
            pKF->LoadFromFile(f);

            unique_lock<mutex> lock(mMutexMap);
            mspKeyFrames.insert(pKF);
            mmpKeyFrames[pKF->mnId] = pKF; //Performance: not nice, but fast...
        }
    }

    cout << "--- Loading MPs ---" << endl;

    u_int16_t numMPs;
    f.read((char*)&numMPs, sizeof(numMPs));
    for(int idx=0;idx<numMPs;++idx)
    {
        size_t IDi;
        f.read((char*)&IDi, sizeof(IDi));

        MapPoint* pMP = this->GetMpPtr(IDi);
        if(pMP)
        {
            pMP->LoadFromFile(f);
        }
        else
        {
            delete pMP;
            pMP = new MapPoint(this, IDi);
            pMP->LoadFromFile(f);

            unique_lock<mutex> lock(mMutexMap);
            mspMapPoints.insert(pMP);
            mmpMapPoints[pMP->mnId] = pMP; //Performance: not nice, but fast...
        }
    }

    cout << "--- Loading Map Vars ---" << endl;

    u_int16_t numOrigins;
    f.read((char*)&numOrigins, sizeof(numOrigins));

    for(int idx=0;idx<numOrigins;++idx)
    {
        size_t IDi;
        f.read((char*)&IDi, sizeof(IDi));
        KeyFrame* pKF = this->GetKfPtr(IDi);
        if(!pKF)
        {
          cout << "FATAL ERROR: KF" << IDi << " should be there" << endl;
        }

        mvpKeyFrameOrigins.push_back(pKF);
    }

    f.read((char*)&mnMaxKFid, sizeof(mnMaxKFid));
    int finalvalue;
    f.read((char*)&finalvalue, sizeof(finalvalue));
    cout << "finalvalue map: " << finalvalue << endl;
}

KeyFrame* Map::ReserveKF(const size_t idp,
                         ORBVocabulary* pVoc,
                         KeyFrameDatabase* pKFDB)
{
    KeyFrame* pKF{new KeyFrame(this, idp, pVoc, pKFDB)};

    {
        unique_lock<mutex> lock(mMutexMap);
        mspKeyFrames.insert(pKF);
        mmpKeyFrames[pKF->mnId] = pKF; //Performance: not nice, but fast...
    }

    return pKF;
}

MapPoint* Map::ReserveMP(const size_t idp)
{
    MapPoint* pMP{new MapPoint(this,idp)};

    {
        unique_lock<mutex> lock(mMutexMap);
        mspMapPoints.insert(pMP);
        mmpMapPoints[pMP->mnId] = pMP; //Performance: not nice, but fast...
    }

    return pMP;
}

} //namespace ORB_SLAM
