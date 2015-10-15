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

#include "ColdetBody.h"
#include <iostream>
#include <fstream>

// using namespace std;
// using namespace boost;
// using namespace hrp;
// using namespace OpenHRP;

ColdetBody::ColdetBody(OpenHRP::BodyInfo_ptr bodyInfo)
{
  OpenHRP::LinkInfoSequence_var links = bodyInfo->links();
  OpenHRP::ShapeInfoSequence_var shapes = bodyInfo->shapes();

    int numLinks = links->length();

    linkColdetModels.resize(numLinks);
		
    for(int linkIndex = 0; linkIndex < numLinks ; ++linkIndex){

      OpenHRP::LinkInfo& linkInfo = links[linkIndex];
			
        int totalNumTriangles = 0;
        int totalNumVertices = 0;
        const OpenHRP::TransformedShapeIndexSequence& shapeIndices = linkInfo.shapeIndices;
        short shapeIndex;
        double R[9], p[3];
        unsigned int nshape = shapeIndices.length();
	    for(int i=0; i < shapeIndices.length(); i++){
            shapeIndex = shapeIndices[i].shapeIndex;
            const OpenHRP::DblArray12 &tform = shapeIndices[i].transformMatrix;
            R[0] = tform[0]; R[1] = tform[1]; R[2] = tform[2]; p[0] = tform[3];
            R[3] = tform[4]; R[4] = tform[5]; R[5] = tform[6]; p[1] = tform[7];
            R[6] = tform[8]; R[7] = tform[9]; R[8] = tform[10]; p[2] = tform[11];
            const OpenHRP::ShapeInfo& shapeInfo = shapes[shapeIndex];
            totalNumTriangles += shapeInfo.triangles.length() / 3;
            totalNumVertices += shapeInfo.vertices.length() / 3;
        }

        const OpenHRP::SensorInfoSequence& sensors = linkInfo.sensors;
        for (unsigned int i=0; i<sensors.length(); i++){
            const OpenHRP::SensorInfo &sinfo = sensors[i];
            const OpenHRP::TransformedShapeIndexSequence tsis = sinfo.shapeIndices;
            nshape += tsis.length();
            for (unsigned int j=0; j<tsis.length(); j++){
                shapeIndex = tsis[j].shapeIndex;
                const OpenHRP::DblArray12 &tform = tsis[j].transformMatrix;
                R[0] = tform[0]; R[1] = tform[1]; R[2] = tform[2]; p[0] = tform[3];
                R[3] = tform[4]; R[4] = tform[5]; R[5] = tform[6]; p[1] = tform[7];
                R[6] = tform[8]; R[7] = tform[9]; R[8] = tform[10]; p[2] = tform[11];
                const OpenHRP::ShapeInfo& shapeInfo = shapes[shapeIndex];
                totalNumTriangles += shapeInfo.triangles.length() / 3;
                totalNumVertices += shapeInfo.vertices.length() / 3 ;
            }
        }

        //int totalNumVertices = totalNumTriangles * 3;

	hrp::ColdetModelPtr coldetModel(new hrp::ColdetModel());
        coldetModel->setName(linkInfo.name);
	ColBodyPositionPtr aColBodyPositionPtr(new ColBodyPosition());

        if(totalNumTriangles > 0){
            coldetModel->setNumVertices(totalNumVertices);
            coldetModel->setNumTriangles(totalNumTriangles);
            if (nshape == 1){
                addLinkPrimitiveInfo(coldetModel, R, p, shapes[shapeIndex]);
            }
            addLinkVerticesAndTriangles(coldetModel, linkInfo, shapes);
            coldetModel->build();
        }
	for(unsigned int i=0;i<9;i++)
	  aColBodyPositionPtr->R[i] = R[i];
	for(unsigned int i=0;i<3;i++)
	  aColBodyPositionPtr->p[i] = p[i];
        linkColdetModels[linkIndex] = coldetModel;
        linkNameToColdetModelMap.insert(std::make_pair(linkInfo.name, coldetModel));
	linkNameToColBodyPosition.insert(std::make_pair(linkInfo.name, aColBodyPositionPtr));

	std::cout << linkInfo.name << " has "
		  << totalNumTriangles << " triangles." << std::endl;
    }
}

void ColdetBody::addLinkPrimitiveInfo(hrp::ColdetModelPtr& coldetModel, 
                                      const double *R, const double *p,
                                      const OpenHRP::ShapeInfo& shapeInfo)
{
    switch(shapeInfo.primitiveType){
    case OpenHRP::SP_BOX:
      coldetModel->setPrimitiveType(hrp::ColdetModel::SP_BOX);
        break;
    case OpenHRP::SP_CYLINDER:
        coldetModel->setPrimitiveType(hrp::ColdetModel::SP_CYLINDER);
        break;
    case OpenHRP::SP_CONE:
        coldetModel->setPrimitiveType(hrp::ColdetModel::SP_CONE);
        break;
    case OpenHRP::SP_SPHERE:
        coldetModel->setPrimitiveType(hrp::ColdetModel::SP_SPHERE);
        break;
    case OpenHRP::SP_PLANE:
        coldetModel->setPrimitiveType(hrp::ColdetModel::SP_PLANE);
        break;
    default:
        break;
    }
    coldetModel->setNumPrimitiveParams(shapeInfo.primitiveParameters.length());
    for (unsigned int i=0; i<shapeInfo.primitiveParameters.length(); i++){
        coldetModel->setPrimitiveParam(i, shapeInfo.primitiveParameters[i]);
    }
    coldetModel->setPrimitivePosition(R, p);
}

void ColdetBody::addLinkVerticesAndTriangles
(hrp::ColdetModelPtr& coldetModel, 
 const OpenHRP::TransformedShapeIndex& tsi, 
 const hrp::Matrix44& Tparent, 
 OpenHRP::ShapeInfoSequence_var& shapes, 
 int& vertexIndex, int& triangleIndex)
{
    short shapeIndex = tsi.shapeIndex;
    const OpenHRP::DblArray12& M = tsi.transformMatrix;;
    hrp::Matrix44 T, Tlocal;
    Tlocal << M[0], M[1], M[2],  M[3],
             M[4], M[5], M[6],  M[7],
             M[8], M[9], M[10], M[11],
             0.0,  0.0,  0.0,   1.0;
    T = Tparent * Tlocal;
    
    const OpenHRP::ShapeInfo& shapeInfo = shapes[shapeIndex];
    int vertexIndexBase = vertexIndex;
    const OpenHRP::FloatSequence& vertices = shapeInfo.vertices;
    const int numVertices = vertices.length() / 3;
    for(int j=0; j < numVertices; ++j){
      hrp::Vector4 v(T * 
		     hrp::Vector4(vertices[j*3], vertices[j*3+1], vertices[j*3+2], 1.0));
        coldetModel->setVertex(vertexIndex++, v[0], v[1], v[2]);
    }

    const OpenHRP::LongSequence& triangles = shapeInfo.triangles;
    const int numTriangles = triangles.length() / 3;
    coldetModel->initNeighbor(numTriangles);
    for(int j=0; j < numTriangles; ++j){
       int t0 = triangles[j*3] + vertexIndexBase;
       int t1 = triangles[j*3+1] + vertexIndexBase;
       int t2 = triangles[j*3+2] + vertexIndexBase;
       coldetModel->setTriangle( triangleIndex, t0, t1, t2);
       coldetModel->setNeighborTriangle(triangleIndex++, t0, t1, t2);
    }
    
}

void ColdetBody::addLinkVerticesAndTriangles
(hrp::ColdetModelPtr& coldetModel, OpenHRP::LinkInfo& linkInfo, OpenHRP::ShapeInfoSequence_var& shapes)
{
    int vertexIndex = 0;
    int triangleIndex = 0;

    const OpenHRP::TransformedShapeIndexSequence& shapeIndices = linkInfo.shapeIndices;

    hrp::Matrix44 E(hrp::Matrix44::Identity());
    for(unsigned int i=0; i < shapeIndices.length(); i++){
        addLinkVerticesAndTriangles(coldetModel, shapeIndices[i], E, shapes,
                                    vertexIndex, triangleIndex);
    }

    hrp::Matrix44 T(hrp::Matrix44::Identity());
    const OpenHRP::SensorInfoSequence& sensors = linkInfo.sensors;
    for (unsigned int i=0; i<sensors.length(); i++){
        const OpenHRP::SensorInfo& sensor = sensors[i]; 
	hrp::calcRodrigues(T, hrp::Vector3(sensor.rotation[0], sensor.rotation[1], 
                                 sensor.rotation[2]), sensor.rotation[3]);
        T(0,3) = sensor.translation[0];
        T(1,3) = sensor.translation[1];
        T(2,3) = sensor.translation[2];
        const OpenHRP::TransformedShapeIndexSequence& shapeIndices = sensor.shapeIndices;
        for (unsigned int j=0; j<shapeIndices.length(); j++){
            addLinkVerticesAndTriangles(coldetModel, shapeIndices[j], T, 
                                        shapes,
                                        vertexIndex, triangleIndex);
        }
    }
}


ColdetBody::ColdetBody(const ColdetBody& org)
{
    linkColdetModels = org.linkColdetModels;
    linkNameToColdetModelMap = org.linkNameToColdetModelMap;
}


void ColdetBody::setLinkPositions(const OpenHRP::LinkPositionSequence& linkPositions)
{
    std::ofstream aof;
    const int srcNumLinks = linkPositions.length();
    const int selfNumLinks = linkColdetModels.size();
    for(int i=0; i < srcNumLinks && i < selfNumLinks; ++i){
      const OpenHRP::LinkPosition& linkPosition = linkPositions[i];
        if(linkColdetModels[i]){
	  linkColdetModels[i]->setPosition(linkPosition.R, linkPosition.p);
	  
	    // aof.open("/tmp/test.dat",std::fstream::app);
	    // aof << linkColdetModels[i]->name() << " " << linkPosition.p[0] << " " 
	    // 	<< linkPosition.p[1] << " " << linkPosition.p[2] << " ";
	    // for(unsigned int j=0;j<9;j++)
	    //   aof << linkPosition.R[j] << " "; 
	    // aof << std::endl;
	    // aof.close();
	  std::string coldetModelName(linkColdetModels[i]->name());
	  StringToColBodyPositionMap::iterator it = 
	    linkNameToColBodyPosition.find(coldetModelName.c_str());
	  
	    if (it!=linkNameToColBodyPosition.end())
	      {
		ColBodyPositionPtr & aColBodyPositionPtr= it->second;
		for(unsigned int i=0;i<3;i++)
		  aColBodyPositionPtr->p[i] = linkPosition.p[i];
		for(unsigned int i=0;i<9;i++)
		  aColBodyPositionPtr->R[i] = linkPosition.R[i];
	      }
        }

    }
}

bool ColdetBody::getLocalizationForLink(const char *aLinkName,
					::OpenHRP::LinkPosition &aLinkPosition)
{
  StringToColBodyPositionMap::iterator it =linkNameToColBodyPosition.find(aLinkName);
  if (it!=linkNameToColBodyPosition.end())
    {
      ColBodyPositionPtr & aColBodyPositionPtr= it->second;
      for(unsigned int i=0;i<3;i++)
	aLinkPosition.p[i] = aColBodyPositionPtr->p[i];
      for(unsigned int i=0;i<9;i++)
	aLinkPosition.R[i] = aColBodyPositionPtr->R[i];
      return true;
    }
  return false;
}
