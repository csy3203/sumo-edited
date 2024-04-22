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
/// @file    Signal.h
/// @author  csy
/// @date    Sept 2021
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
 * @class ErrProbRef
 * @brief A sample of prediction error probability w.r.t. distance (meter).
 */
class ErrProbRef {
public:
    /// @brief default constructor
    ErrProbRef() :
        myRef(0.0), myErrorProb(0.0) { }

    /// @brief Parametrised constructor (double input)
    ErrProbRef(double distance, double errProb) :
        myRef(distance), myErrorProb(errProb) { }
    
    /// @brief Destructor
    ~ErrProbRef() { }
    
    /// @brief Returns the distance in meter
    inline double Reference() const {
        return myRef;
    }
    
    /// @brief Returns the error probability range from 0 to 1
    inline double ErrorProbability() const {
        return myErrorProb;
    }

    /// @brief output operator
    friend std::ostream& operator<<(std::ostream& os, const ErrProbRef& a) {
        os << a.Reference() << "," << a.ErrorProbability();
        return os;
    }

    /// @brief output to string
    inline std::string to_string() const {
        return std::to_string(this->Reference()) + "," + std::to_string(this->ErrorProbability());
    }

private:
    /// @brief  sample distance in meter
    double myRef;

    /// @brief  sample error probability range from 0 to 1
    double myErrorProb;
};

/**
 * @class ErrProbRefs
 * @brief A sequence of ErrProbRef.
 */
class ErrProbRefs : public std::vector<ErrProbRef> {
public:
    /// @brief default constructor
    ErrProbRefs() { }

    /// @brief default destructor
    ~ErrProbRefs() { }

    /// @brief check if the distance of input data already exists
    bool exists(double distance) {
        for(size_t i = 0; i < this->size(); i++) {
            if(this->at(i).Reference() == distance) {
                return true;
            }
        }
        return false;
    }

    /// @brief add new sample and sort by velocity
    void addSample(const ErrProbRef& data) {
        if(!exists(data.Reference())){
            this->push_back(data);
        }
        sort_asc();
    }

    /// @brief compute angle based on the current velocity using interpolation
    double getErrProb(double ref) {
        if(this->empty()) {
            return 0.0;
        }
        if(this->size() == 1) {
            return this->at(0).ErrorProbability();
        }
        size_t i = 0;
        for (; i < this->size(); i++) {
            if (this->at(i).Reference() >= ref) {
                break;
            }
        }
        if(i == 0) {
            return this->at(0).ErrorProbability();
        }
        else if(i == this->size()) {
            return this->at(i-1).ErrorProbability();
        }
        else {
            return this->at(i-1).ErrorProbability()
            + (this->at(i).ErrorProbability() - this->at(i-1).ErrorProbability())
            / (this->at(i).Reference() - this->at(i-1).Reference())
            * (ref - this->at(i-1).Reference());
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
            double dist = StringUtils::toDouble(st1[0]);
            double errProb = StringUtils::toDouble(st1[1]);
            this->addSample(ErrProbRef(dist, errProb));
        }
    }

private:
    /// @brief  compare two item by velocity
    static bool compare_asc(ErrProbRef x1, ErrProbRef x2) {
        return x1.Reference() < x2.Reference();
    }

    /// @brief sort by velocity
    void sort_asc() {
        std::sort(this->begin(),this->end(), compare_asc);
    }
};