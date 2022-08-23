/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2022 German Aerospace Center (DLR) and others.
// This program and the accompanying materials are made available under the
// terms of the Eclipse Public License 2.0 which is available at
// https://www.eclipse.org/legal/epl-2.0/
// This Source Code may also be made available under the following Secondary
// Licenses when the conditions for such availability set forth in the Eclipse
// Public License 2.0 are satisfied: GNU General Public License, version 2
// or later which is available at
// https://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
// SPDX-License-Identifier: EPL-2.0 OR GPL-2.0-or-later
/****************************************************************************/
/// @file    GUIOSGPerspectiveChanger.cpp
/// @author  Mirko Barthauer
/// @date    August 2022
///
// Implementation of GUIPerspectiveChanger for OSG 3D views
/****************************************************************************/
#include <config.h>

#include <fxkeys.h>
#include <utils/geom/Boundary.h>
#include <utils/geom/Position.h>
#include <utils/geom/GeomHelper.h>
#include <utils/gui/settings/GUICompleteSchemeStorage.h>
#include "GUIOSGPerspectiveChanger.h"

// ===========================================================================
// method definitions
// ===========================================================================
GUIOSGPerspectiveChanger::GUIOSGPerspectiveChanger(
    GUIOSGView& callBack, const Boundary& viewPort) :
    GUIPerspectiveChanger(callBack, viewPort),
    myOrigWidth(viewPort.getWidth()),
    myOrigHeight(viewPort.getHeight()),
    myRotation(0) {
    myCameraManipulator = callBack.myCameraManipulator;
}


GUIOSGPerspectiveChanger::~GUIOSGPerspectiveChanger() {}


double
GUIOSGPerspectiveChanger::getRotation() const {
    return myRotation;
}


double
GUIOSGPerspectiveChanger::getXPos() const {
    osg::Vec3d lookFrom, lookAt, up;
    myCameraManipulator->getInverseMatrix().getLookAt(lookFrom, lookAt, up);
    return lookFrom.x();
}


double
GUIOSGPerspectiveChanger::getYPos() const {
    osg::Vec3d lookFrom, lookAt, up;
    myCameraManipulator->getInverseMatrix().getLookAt(lookFrom, lookAt, up);
    return lookFrom.y();
}


double
GUIOSGPerspectiveChanger::getZPos() const {
    osg::Vec3d lookFrom, lookAt, up;
    myCameraManipulator->getInverseMatrix().getLookAt(lookFrom, lookAt, up);
    return lookFrom.z();
}


double
GUIOSGPerspectiveChanger::getZoom() const {
    return 100.;
}


double 
GUIOSGPerspectiveChanger::zPos2Zoom(double zPos) const {
    return 100.;
}


double
GUIOSGPerspectiveChanger::zoom2ZPos(double zoom) const {
    // TODO: real implementation
    return 100.;
}


void
GUIOSGPerspectiveChanger::setRotation(double rotation) {
    myRotation = rotation;
}


void 
GUIOSGPerspectiveChanger::centerTo(const Position& pos, double radius, bool applyZoom) {
    // maintain view direction if possible and scale so that the position and the 
    // radius region around it are visible
    osg::Vec3d lookFrom, lookAt, up, dir, orthoDir;
    myCameraManipulator->getInverseMatrix().getLookAt(lookFrom, lookAt, up);
    dir = lookAt - lookFrom;
    // create helper vectors // check if parallel to z
    if (dir * osg::Z_AXIS != 0) {
        orthoDir = -osg::X_AXIS;
    }
    else {
        orthoDir[0] = -dir[1];
        orthoDir[1] = dir[0];
    }
    
    orthoDir.normalize();
    osg::Vec3d center(pos.x(), pos.y(), pos.z());
    osg::Vec3d leftBorder = center + orthoDir * radius;
    osg::Vec3d rightBorder = center - orthoDir * radius;   
    // construct new camera location which respects the fovy
    double fovy, aspectRatio, zNear, zFar;
    dynamic_cast<GUIOSGView&>(myCallback).myViewer->getCamera()->getProjectionMatrixAsPerspective(fovy, aspectRatio, zNear, zFar);
    double halfFovy = DEG2RAD(.5*fovy);
    osg::Vec3d outerFov = dir * cos(halfFovy) + orthoDir * sin(halfFovy);
    osg::Vec3d radiusVec = leftBorder - center;
    int sign = ((outerFov ^ radiusVec) * (outerFov ^ dir) > 0) ? 1 : -1;
    osg::Vec3d camUpdate = center + dir * sign * (outerFov ^ radiusVec).length() / (outerFov ^ dir).length();

    // TODO: still rotates the viewport somehow
    myCameraManipulator->setHomePosition(camUpdate, center, up);
    dynamic_cast<GUIOSGView&>(myCallback).myViewer->home(); 
}


void
GUIOSGPerspectiveChanger::setViewport(double zoom, double xPos, double yPos) {
    setViewportFrom(xPos, yPos, 0.);
}


void
GUIOSGPerspectiveChanger::setViewportFrom(double xPos, double yPos, double zPos) {
    // Keep camera orientation if possible and point it to point to (x,y,0) if possible. 
    // get current camera orientation
    osg::Vec3d lookFrom, lookAt, up, dir;
    myCameraManipulator->getInverseMatrix().getLookAt(lookFrom, lookAt, up);
    dir = lookAt - lookFrom;
    if ((dir.z() > 0. && lookFrom.z() >= 0.) || dir.z() == 0.) {         // create bird view
        lookFrom[0] = xPos;
        lookFrom[1] = yPos;
        lookAt = lookFrom - osg::Vec3d(0., 0., 1.);
    }
    else { // shift current view to reach (x,y,0)
        osg::Vec3d shift;
        // compute the point on the ground which is in line with the camera direction (solve for z=0)
        double factor = -lookFrom.z() / dir.z();
        osg::Vec3d groundTarget = lookFrom + dir * factor;
        shift[0] = xPos - groundTarget.x();
        shift[1] = yPos - groundTarget.y();
        lookFrom += shift;
        lookAt += shift;
    }
    osg::Matrix m;
    m.makeLookAt(lookFrom, lookAt, up);
    myCameraManipulator->setByInverseMatrix(m);
}



void
GUIOSGPerspectiveChanger::changeCanvasSizeLeft(int change) {
    // currently not called from anywhere..?!
}


void
GUIOSGPerspectiveChanger::setViewport(const Boundary& viewPort) {
    // Keep camera orientation if possible and move it to point to (x,y,0). Otherwise create 
    // a bird perspective.
    // Call setViewPort(zoom,x,y) with the center of the given viewport.
    setViewport(100., viewPort.getCenter().x(), viewPort.getCenter().y());
}