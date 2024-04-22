/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2021 German Aerospace Center (DLR) and others.
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
/// @file    Visual.h
/// @author  csy
/// @date    June 2021
///
// Visual identification for junction models in the 2D-world
/****************************************************************************/
#pragma once
#include <config.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>
#include <utils/common/StringTokenizer.h>
#include <utils/common/StringUtils.h>

// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class AngleRef
 * @brief A sample of visual angle w.r.t. velocity.
 */
class AngleRef {
public:
    /// @brief default constructor
    AngleRef() :
        myVelocity(0.0), myAngle(0.0) { }

    /// @brief Parametrised constructor (double input)
    AngleRef(double velocity, double angle) :
        myVelocity(velocity), myAngle(angle) { }
    
    /// @brief Destructor
    ~AngleRef() { }
    
    /// @brief Returns the velocity in km/h
    inline double Velocity() const {
        return myVelocity;
    }
    
    /// @brief Returns the velocity in km/h
    inline double Angle() const {
        return myAngle;
    }

    /// @brief output operator
    friend std::ostream& operator<<(std::ostream& os, const AngleRef& a) {
        os << a.Velocity() << "," << a.Angle();
        return os;
    }

    /// @brief output to string
    inline std::string to_string() const {
        //std::ostringstream os;
        //os << *this;
        //return os.str();
        return std::to_string(this->Velocity()) + "," + std::to_string(this->Angle());
    }

private:
    /// @brief  sample velocity in km/h
    double myVelocity;

    /// @brief  sample angle in degree
    double myAngle;
};

/**
 * @class AngleRefs
 * @brief A sequence of AngleRef.
 */
class AngleRefs : public std::vector<AngleRef> {
public:
    /// @brief default constructor
    AngleRefs() { }

    /// @brief default destructor
    ~AngleRefs() { }

    /// @brief check if the velocity of input data already exists
    bool exists(double velocity) {
        for(size_t i = 0; i < this->size(); i++) {
            if(this->at(i).Velocity() == velocity) {
                return true;
            }
        }
        return false;
    }

    /// @brief add new sample and sort by velocity
    void addSample(const AngleRef& data) {
        if(!exists(data.Velocity())){
            this->push_back(data);
        }
        sort_asc();
    }

    /// @brief compute angle based on the current velocity using interpolation
    double getAngleDEG(double velocityKMH) {
        if(this->empty()) {
            return 360.0;
        }
        if(this->size() == 1) {
            return this->at(0).Angle();
        }
        size_t i = 0;
        for (; i < this->size(); i++) {
            if (this->at(i).Velocity() >= velocityKMH) {
                break;
            }
        }
        if(i == 0) {
            return this->at(0).Angle();
        }
        else if(i == this->size()) {
            return this->at(i-1).Angle();
        }
        else {
            return this->at(i-1).Angle()
            + (this->at(i).Angle() - this->at(i-1).Angle())
            / (this->at(i).Velocity() - this->at(i-1).Velocity())
            * (velocityKMH - this->at(i-1).Velocity());
        }
    }

    /// @brief output to string
    inline std::string to_string() const {
        std::ostringstream os;
        for(size_t i = 0; i < this->size(); i++) {
            if(i == this->size() - 1) {
                os << this->at(i).to_string();
            }
            else {
                os << this->at(i).to_string() << ";";
            }
        }
        return os.str();
    }

    /// @brief check if input string can parse
    static bool can_parse(std::string seq) {
        std::vector<std::string> st0 = StringTokenizer(seq, ";", true).getVector();
        for(size_t i = 0; i < st0.size(); i++) {
            std::vector<std::string> st1 = StringTokenizer(st0[i], ",", true).getVector();
            if(st1.size() == 2) {
                try {
                    StringUtils::toDouble(st1[0]);
                    StringUtils::toDouble(st1[1]);
                } catch (...) {
                    return false;
                }
            } else {
                return false;
            }
        }
        return true;
    }

    /// @brief parse input string to generate vector of refs
    void parse(std::string seq) {
        if(!can_parse(seq) || seq.empty()) return;
        this->clear();
        std::vector<std::string> st0 = StringTokenizer(seq, ";", true).getVector();
        for(size_t i = 0; i < st0.size(); i++) {
            std::vector<std::string> st1 = StringTokenizer(st0[i], ",", true).getVector();
            double vec = StringUtils::toDouble(st1[0]);
            double ang = StringUtils::toDouble(st1[1]);
            this->addSample(AngleRef(vec, ang));
        }
    }

private:
    /// @brief  compare two item by velocity
    static bool compare_asc(AngleRef x1, AngleRef x2) {
        return x1.Velocity() < x2.Velocity();
    }

    /// @brief sort by velocity
    void sort_asc() {
        std::sort(this->begin(),this->end(), compare_asc);
    }
};