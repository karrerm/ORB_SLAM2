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
    cout << "numKFs: " << numKFs << endl;
    for(set<KeyFrame*>::iterator sit = mspKeyFrames.begin();sit != mspKeyFrames.end();++sit)
    {
        KeyFrame* pKFi = *sit;
//        cout << "Saving KF " << pKFi->mId.first << "|" << pKFi->mId.second << endl;
        f.write((char*)&pKFi->mnId, sizeof(pKFi->mnId));
        pKFi->SaveToFile(f);//,sKnownKFs,sKnownMPs);

//        sWrittenKFs.insert(pKFi->mId);

//        if(pKFi->mId.first == 0 || pKFi->mId.first == 3)
//            pKFi->ShowMyValues();
    }

//    cout << "--- Saving Erased KFs ---" << endl;

//    uint16_t numErasedKFs = mmpErasedKeyFrames.size();
//    f.write((char*)&numErasedKFs, sizeof(numErasedKFs));
//    for(map<idpair,kfptr>::iterator mit = mmpErasedKeyFrames.begin();mit != mmpErasedKeyFrames.end();++mit)
//    {
//        kfptr pKFi = mit->second;
////        cout << "Saving KF " << pKFi->mId.first << "|" << pKFi->mId.second << endl;
//        KeyFrame::wp(pKFi->mId,f);
//        pKFi->SaveToFile(f,sKnownKFs,sKnownMPs);

////        sWrittenKFs.insert(pKFi->mId);
//    }

    cout << "--- Saving MPs ---" << endl;

//    cout << COUTWARN << "only writing MPs with ID < 10 !!!!!!!!!!!!!!" << endl;

    u_int16_t numMPs = mspMapPoints.size();
    f.write((char*)&numMPs, sizeof(numMPs));
    for(set<MapPoint*>::iterator sit = mspMapPoints.begin();sit != mspMapPoints.end();++sit)
    {
        MapPoint* pMPi = *sit;

//        if(pMPi->mId.first >= 10)
//            continue;

        f.write((char*)&pMPi->mnId, sizeof(pMPi->mnId));
        pMPi->SaveToFile(f);//,sKnownKFs,sKnownMPs);

//        sWrittenMPs.insert(pMPi->mId);

//        if(pMPi->mId.first == 30 || pMPi->mId.first == 90)
//            pMPi->ShowMyValues();
    }

//    cout << "--- Saving Erased MPs ---" << endl;

//    uint16_t numErasedMPs = mmpErasedMapPoints.size();
//    f.write((char*)&numErasedMPs, sizeof(numErasedMPs));
//    for(map<idpair,mpptr>::iterator mit = mmpErasedMapPoints.begin();mit != mmpErasedMapPoints.end();++mit)
//    {
//        mpptr pMPi = mit->second;
//        KeyFrame::wp(pMPi->mId,f);
//        pMPi->SaveToFile(f,sKnownKFs,sKnownMPs);

////        sWrittenMPs.insert(pMPi->mId);
//    }

    cout << "--- Saving Map Vals ---" << endl;

    u_int16_t numOrigins = mvpKeyFrameOrigins.size();
    cout << "numOrigins: " << numOrigins << endl;
    f.write((char*)&numOrigins, sizeof(numOrigins));
    for(int idx=0;idx<mvpKeyFrameOrigins.size();++idx)
    {
        KeyFrame* pKFi = mvpKeyFrameOrigins[idx];
        f.write((char*)&pKFi->mnId, sizeof(pKFi->mnId));
//        pKFi->SaveToFile(f);
    }

    f.write((char*)&mnMaxKFid, sizeof(mnMaxKFid));
//    f.write((char*)&mnMaxMPid, sizeof(mnMaxMPid));
//    f.write((char*)&mnMaxKFidUnique, sizeof(mnMaxKFidUnique));
//    f.write((char*)&mnMaxMPidUnique, sizeof(mnMaxMPidUnique));

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
    cout << "numKFs: " << numKFs << endl;
    for(int idx=0;idx<numKFs;++idx)
    {
        size_t IDi;
        f.read((char*)&IDi, sizeof(IDi));
//        cout << "Constructing KF " << IDi.first << "|" << IDi.second << endl;
        KeyFrame* pKF = this->GetKfPtr(IDi);
        if(pKF)
        {
//            if(!pKF->IsEmpty())
//            {
//                cout << COUTFATAL << "Trying to construct KF twice" << endl;
//                KILLSYS
//            }
            pKF->LoadFromFile(f);
        }
        else
        {
//            commptr pComm;
//            for(set<commptr>::const_iterator sit2 = mspComm.begin();sit2!=mspComm.end();++sit2)
//            {
//                commptr pCi = *sit2;
//                if(pCi->GetClientId() == IDi.second)
//                    pComm = pCi;
//            }

//            if(!pComm)
//                KILLSYS

//            ccptr pCC = *(mspCC.begin());

            delete pKF;
            pKF = (new  KeyFrame(this, IDi, pVoc, pKFDbase));
            //pKF->reset(new KeyFrame(this, IDi));

            pKF->LoadFromFile(f);

            unique_lock<mutex> lock(mMutexMap);
            mspKeyFrames.insert(pKF);
            mmpKeyFrames[pKF->mnId] = pKF; //Performance: not nice, but fast...
        }
    }

//    cout << "--- Loading Erased KFs ---" << endl;

//    u_int16_t numErasedKFs;
//    f.read((char*)&numErasedKFs, sizeof(numErasedKFs));
//    for(int idx=0;idx<numErasedKFs;++idx)
//    {
//        size_t IDi;
//        f.read((char*)&IDi, sizeof(IDi));
////        cout << "Constructing KF " << IDi.first << "|" << IDi.second << endl;
//        KeyFrame* pKF = this->GetKfPtr(IDi);
//        if(pKF)
//        {
////            if(!pKF->IsEmpty())
////            {
////                cout << COUTFATAL << "Trying to construct KF twice" << endl;
////                KILLSYS
////            }

//            pKF->LoadFromFile(f);

//            mspKeyFrames.erase(pKF);

//            std::map<size_t,KeyFrame*>::iterator mit = mmpKeyFrames.find(pKF->mnId);
//            if(mit != mmpKeyFrames.end()) mmpKeyFrames.erase(mit);

////            mmpErasedKeyFrames[pKF->mnId] = pKF;
//        }
//        else
//        {
////            if(this->GetErasedKfPtr(IDi))
////            {
////                cout << COUTFATAL << "Trying to construct KF twice" << endl;
////                KILLSYS
////            }

////            commptr pComm;
////            for(set<commptr>::const_iterator sit2 = mspComm.begin();sit2!=mspComm.end();++sit2)
////            {
////                commptr pCi = *sit2;
////                if(pCi->GetClientId() == IDi.second)
////                    pComm = pCi;
////            }

////            if(!pComm)
////                KILLSYS

////            ccptr pCC = *(mspCC.begin());

//            pKF.reset(new KeyFrame(shared_from_this(), IDi));
//            pKF->LoadFromFile(f);

//            unique_lock<mutex> lock(mMutexMap);
////            mmpErasedKeyFrames[pKF->mId] = pKF;
//        }
//    }

//    cout << "--- Check KFs ---" << endl;

//    bool bError = false;
//    if(mspKeyFrames.size() != mmpKeyFrames.size())
//    {
//        cout << COUTERROR << "mspKeyFrames.size() != mmpKeyFrames.size()" << endl;
//        bError = true;
//    }

//    for(set<kfptr>::iterator sit = mspKeyFrames.begin();sit != mspKeyFrames.end();++sit)
//    {
//        kfptr pKFi = *sit;
//        if(pKFi->IsEmpty())
//        {
//            cout << COUTERROR << "mspKeyFrames: " << "KF " << pKFi->mId.first << "|" << pKFi->mId.second << ": empty" << endl;
//            bError = true;
//        }
//    }

//    for(map<idpair,kfptr>::iterator mit = mmpKeyFrames.begin();mit != mmpKeyFrames.end();++mit)
//    {
//        kfptr pKFi = mit->second;
//        if(pKFi->IsEmpty())
//        {
//            cout << COUTERROR << "mmpKeyFrames: " << "KF " << pKFi->mId.first << "|" << pKFi->mId.second << ": empty" << endl;
//            bError = true;
//        }
//    }

//    for(map<idpair,kfptr>::iterator mit = mmpErasedKeyFrames.begin();mit != mmpErasedKeyFrames.end();++mit)
//    {
//        kfptr pKFi = mit->second;
//        if(pKFi->IsEmpty())
//        {
//            cout << COUTERROR << "mmpErasedKeyFrames: " << "KF " << pKFi->mId.first << "|" << pKFi->mId.second << ": empty" << endl;
//            bError = true;
//        }
//    }

//    if(bError)
//        KILLSYS

    cout << "--- Loading MPs ---" << endl;

    u_int16_t numMPs;
    f.read((char*)&numMPs, sizeof(numMPs));
    for(int idx=0;idx<numMPs;++idx)
    {
        size_t IDi;
        f.read((char*)&IDi, sizeof(IDi));

//        if(IDi == 750)
//            cout << "WTF?????" << endl;

        MapPoint* pMP = this->GetMpPtr(IDi);
        if(pMP)
        {
//            if(!pMP->IsEmpty())
//            {
//                cout << COUTFATAL << "Trying to construct MP twice" << endl;
//                KILLSYS
//            }

            pMP->LoadFromFile(f);
        }
        else
        {
//            commptr pComm;
//            for(set<commptr>::const_iterator sit2 = mspComm.begin();sit2!=mspComm.end();++sit2)
//            {
//                commptr pCi = *sit2;
//                if(pCi->GetClientId() == IDi.second)
//                    pComm = pCi;
//            }

//            if(!pComm)
//                KILLSYS

            delete pMP;
            pMP = new MapPoint(this, IDi);
            //pMP->reset(new MapPoint(this,IDi));
            pMP->LoadFromFile(f);

            unique_lock<mutex> lock(mMutexMap);
            mspMapPoints.insert(pMP);
            mmpMapPoints[pMP->mnId] = pMP; //Performance: not nice, but fast...
        }
    }

//    cout << "--- Loading Erased MPs ---" << endl;

//    uint16_t numErasedMPs;
//    f.read((char*)&numErasedMPs, sizeof(numErasedMPs));
//    for(int idx=0;idx<numErasedMPs;++idx)
//    {
//        idpair IDi;
//        KeyFrame::rp(IDi,f);

////        if(IDi.first == 750) cout << "WTF?????" << endl;

//        mpptr pMP = this->GetMpPtr(IDi);
//        if(pMP)
//        {
////            if(IDi.first == 750) cout << "pMP" << endl;

//            if(!pMP->IsEmpty())
//            {
//                cout << COUTFATAL << "Trying to construct MP twice" << endl;
//                KILLSYS
//            }

//            pMP->LoadFromFile(f);

//            mspMapPoints.erase(pMP);

//            std::map<idpair,mpptr>::iterator mit = mmpMapPoints.find(pMP->mId);
//            if(mit != mmpMapPoints.end()) mmpMapPoints.erase(mit);

//            mmpErasedMapPoints[pMP->mId] = pMP;
//        }
//        else
//        {
////            if(IDi.first == 750) cout << "!pMP" << endl;

//            if(this->GetErasedMpPtr(IDi))
//            {
//                cout << COUTFATAL << "Trying to construct MP twice" << endl;
//                KILLSYS
//            }

//            commptr pComm;
//            for(set<commptr>::const_iterator sit2 = mspComm.begin();sit2!=mspComm.end();++sit2)
//            {
//                commptr pCi = *sit2;
//                if(pCi->GetClientId() == IDi.second)
//                    pComm = pCi;
//            }

//            if(!pComm)
//                KILLSYS

//            pMP.reset(new MapPoint(shared_from_this(),pComm,eSystemState::SERVER,IDi));
//            pMP->LoadFromFile(f);

//            unique_lock<mutex> lock(mMutexMap);
//            mmpErasedMapPoints[pMP->mId] = pMP;
//        }
//    }

//    cout << "--- Check MPs ---" << endl;

//    if(mspMapPoints.size() != mmpMapPoints.size())
//    {
//        cout << COUTERROR << "mspMapPoints.size() != mmpMapPoints.size()" << endl;
//        bError = true;
//    }

//    for(set<mpptr>::iterator sit = mspMapPoints.begin();sit != mspMapPoints.end();++sit)
//    {
//        mpptr pMPi = *sit;
//        if(pMPi->IsEmpty())
//        {
//            cout << COUTERROR << "mspMapPoints: " << "MP " << pMPi->mId.first << "|" << pMPi->mId.second << ": empty" << endl;
//            bError = true;
//        }
//    }

//    for(map<idpair,mpptr>::iterator mit = mmpMapPoints.begin();mit != mmpMapPoints.end();++mit)
//    {
//        mpptr pMPi = mit->second;
//        if(pMPi->IsEmpty())
//        {
//            cout << COUTERROR << "mmpMapPoints: " << "MP " << pMPi->mId.first << "|" << pMPi->mId.second << ": empty" << endl;
//            bError = true;
//        }
//    }

//    for(map<idpair,mpptr>::iterator mit = mmpErasedMapPoints.begin();mit != mmpErasedMapPoints.end();++mit)
//    {
//        mpptr pMPi = mit->second;
//        if(pMPi->IsEmpty())
//        {
//            cout << COUTERROR << "mmpErasedMapPoints: " << "MP " << pMPi->mId.first << "|" << pMPi->mId.second << ": empty" << endl;
//            bError = true;
//        }
//    }

//    if(bError)
//        KILLSYS

    cout << "--- Loading Map Vars ---" << endl;

    u_int16_t numOrigins;
    f.read((char*)&numOrigins, sizeof(numOrigins));
    cout << "numOrigins: " << numOrigins << endl;

    for(int idx=0;idx<numOrigins;++idx)
    {
        size_t IDi;
        f.read((char*)&IDi, sizeof(IDi));
        KeyFrame* pKF = this->GetKfPtr(IDi);
        if(!pKF)
        {
          cout << "FATAL ERROR: KF" << IDi << " should be there" << endl;
//            cout << COUTFATAL << "KF" << IDi.first << "|" << IDi.second << "should be there" << endl;
//            KILLSYS
        }

        mvpKeyFrameOrigins.push_back(pKF);
    }

    f.read((char*)&mnMaxKFid, sizeof(mnMaxKFid));
//    f.read((char*)&mnMaxMPid, sizeof(mnMaxMPid));
//    f.read((char*)&mnMaxKFidUnique, sizeof(mnMaxKFidUnique));
//    f.read((char*)&mnMaxMPidUnique, sizeof(mnMaxMPidUnique));

//    ccptr pCC = *mspCC.begin();
//    pCC->mpUID->SetLastId(max(mnMaxKFidUnique,mnMaxMPidUnique));

    int finalvalue;
    f.read((char*)&finalvalue, sizeof(finalvalue));
    cout << "finalvalue map: " << finalvalue << endl;

//    kfptr pKFshow = this->GetKfPtr(0,0);
//    pKFshow->ShowMyValues();
//    pKFshow = this->GetKfPtr(3,0);
//    pKFshow->ShowMyValues();
}

KeyFrame* Map::ReserveKF(const size_t idp,
                         ORBVocabulary* pVoc,
                         KeyFrameDatabase* pKFDB)
{
    KeyFrame* pKF{new KeyFrame(this, idp, pVoc, pKFDB)};

    std::cout << "Reserves a KF: " << idp << std::endl;
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
