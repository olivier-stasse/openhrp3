/*
 * Copyright (c) 2008, AIST, the University of Tokyo and General Robotix Inc.
 * All rights reserved. This program is made available under the terms of the
 * Eclipse Public License v1.0 which accompanies this distribution, and is
 * available at http://www.eclipse.org/legal/epl-v10.html
 * Contributors:
 * National Institute of Advanced Industrial Science and Technology (AIST)
 * General Robotix Inc. 
 */
/**
   @author Shin'ichiro Nakaoka
*/

#ifndef OPENHRP_COLLISION_DETECTOR_COLDET_BODY_H_INCLUDED
#define OPENHRP_COLLISION_DETECTOR_COLDET_BODY_H_INCLUDED

#include <map>
#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>

#include <hrpUtil/Referenced.h>
#include <hrpUtil/Eigen4d.h>
#include <hrpCorba/ModelLoader.hh>
#include <hrpCollision/ColdetModel.h>

/* using namespace std; */
/* using namespace boost; */
/* using namespace hrp; */
/* using namespace OpenHRP; */

struct ColBodyPosition {
  double R[9];
  double p[3];
};

typedef boost::shared_ptr<ColBodyPosition> ColBodyPositionPtr;
typedef std::map<std::string, ColBodyPositionPtr> StringToColBodyPositionMap;

class ColdetBody : public hrp::Referenced
{
public:
  ColdetBody(OpenHRP::BodyInfo_ptr bodyInfo);
    /**
       do shallow copy (sharing the same ColdetModel instances)
    */
    ColdetBody(const ColdetBody& org);

    void setName(const char* name) { name_ = name; }
    const char* name() { return name_.c_str(); }
    
    unsigned int numLinks() const {
        return linkColdetModels.size();
    }
    hrp::ColdetModelPtr linkColdetModel(int linkIndex) {
        return linkColdetModels[linkIndex];
    }

    hrp::ColdetModelPtr linkColdetModel(const std::string& linkName){
      std::map<std::string, hrp::ColdetModelPtr>::iterator 
	p = linkNameToColdetModelMap.find(linkName);
      return (p == linkNameToColdetModelMap.end()) ? hrp::ColdetModelPtr() : p->second;
    }

    void setLinkPositions(const OpenHRP::LinkPositionSequence& linkPositions);
    bool getLocalizationForLink(const char *aLinkName,
				::OpenHRP::LinkPosition &aLinkPosition);

  private:
    void addLinkPrimitiveInfo(hrp::ColdetModelPtr& coldetModel, 
                              const double *R, const double *p,
                              const OpenHRP::ShapeInfo& shapeInfo);
    void addLinkVerticesAndTriangles
      (hrp::ColdetModelPtr& coldetModel, 
       OpenHRP::LinkInfo& linkInfo, OpenHRP::ShapeInfoSequence_var& shapes);
    void addLinkVerticesAndTriangles
      (hrp::ColdetModelPtr& coldetModel, 
       const OpenHRP::TransformedShapeIndex& tsi, 
       const hrp::Matrix44& Tparent, 
       OpenHRP::ShapeInfoSequence_var& shapes, int& vertexIndex, int& triangleIndex);
    
    std::vector<hrp::ColdetModelPtr> linkColdetModels;
    std::map<std::string, hrp::ColdetModelPtr> linkNameToColdetModelMap;
    StringToColBodyPositionMap linkNameToColBodyPosition;
    std::string name_;
};

typedef boost::intrusive_ptr<ColdetBody> ColdetBodyPtr;
#endif
