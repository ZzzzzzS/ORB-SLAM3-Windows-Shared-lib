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


#ifndef KEYFRAMEDATABASE_H
#define KEYFRAMEDATABASE_H

#include <vector>
#include <list>
#include <set>

#include "KeyFrame.h"
#include "Frame.h"
#include "ORBVocabulary.h"
#include "Map.h"

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/list.hpp>

#include<mutex>
#define WIN_EXPORT __declspec( dllexport )

namespace ORB_SLAM3
{
using namespace std;
class KeyFrame;
class Frame;
class Map;


class WIN_EXPORT KeyFrameDatabase
{
    friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar & mvBackupInvertedFileId;
    }

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    KeyFrameDatabase(){}
    KeyFrameDatabase(const ORBVocabulary &voc);

    void add(KeyFrame* pKF);

    void erase(KeyFrame* pKF);

    void clear();
    void clearMap(Map* pMap);

    // Loop Detection(DEPRECATED)
    std::vector<KeyFrame *> DetectLoopCandidates(KeyFrame* pKF, float minScore);

    // Loop and Merge Detection
    //vpLoopCand: 回环候选帧，在同一个地图内
    //vpMergeCand：合并关键帧，不在一个地图内
    //这三个函数基本分别按照最小得分阈值，最小公共单词阈值，最少帧个数阈值来选择回环/合并关键帧
    //最小公共单词数是最大公共单词数的0.8倍
    //存在一个分小组的过程，这个过程可以去除候选帧里的共视关键帧
    void DetectCandidates(KeyFrame* pKF, float minScore,vector<KeyFrame*>& vpLoopCand, vector<KeyFrame*>& vpMergeCand);
    void DetectBestCandidates(KeyFrame *pKF, vector<KeyFrame*> &vpLoopCand, vector<KeyFrame*> &vpMergeCand, int nMinWords);
    void DetectNBestCandidates(KeyFrame *pKF, vector<KeyFrame*> &vpLoopCand, vector<KeyFrame*> &vpMergeCand, int nNumCandidates);

    // Relocalization
    //所有组中最高得分的0.75倍，作为最低阈值
    std::vector<KeyFrame*> DetectRelocalizationCandidates(Frame* F, Map* pMap);

    void PreSave();
    void PostLoad(map<long unsigned int, KeyFrame*> mpKFid);
    void SetORBVocabulary(ORBVocabulary* pORBVoc);

protected:

   // Associated vocabulary
   const ORBVocabulary* mpVoc;

   // Inverted file
   //反向查询，vector表示所有WordId，List表示keyframe，为每一个WordId反向存储KeyFrame
   std::vector<list<KeyFrame*> > mvInvertedFile;

   // For save relation without pointer, this is necessary for save/load function
   std::vector<list<long unsigned int> > mvBackupInvertedFileId;

   // Mutex
   std::mutex mMutex;

};

} //namespace ORB_SLAM

#endif
