/*
*  zfilterfactory.h
*
*  Created by Ken Self
*  Copyright (c) 2018 Ken Self
*  All rights reserved.
*
*  This source code is distributed under the following "BSD" license
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions are met:
*    Redistributions of source code must retain the above copyright notice,
*     this list of conditions and the following disclaimer.
*    Redistributions in binary form must reproduce the above copyright notice,
*     this list of conditions and the following disclaimer in the
*     documentation and/or other materials provided with the distribution.
*    Neither the name of openphdguiding.org nor the names of its
*     contributors may be used to endorse or promote products derived from
*     this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
*  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
*  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
*  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
*  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
*  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
*  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
*  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
*  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
*  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
*/

#ifndef ZFILTER_FACTORY_H_INCLUDED
#define ZFILTER_FACTORY_H_INCLUDED

#include <complex>
#include <math.h>
#include <vector>
#include <string>
#include <algorithm>
#include <indilogger.h>
#include "stargo.h"

enum FILTER_DESIGN
{
    BESSEL,
    BUTTERWORTH,
    CHEBYCHEV,
};
class StarGoTelescope;
class ZFilterFactory
{

public:
    double gain() { return ::hypot(dc_gain.imag(), dc_gain.real()); };
    double corner() { return 1 / raw_alpha1; };
    FILTER_DESIGN design() { return filt; }
    std::string getname() const;
    int order() { return m_order;  };
    ZFilterFactory(StarGoTelescope* ptr);
    void resetsamples();
    bool rebuild(FILTER_DESIGN f, int o, double p, bool mzt=false );
    double addsample(double input);

private:
    StarGoTelescope* p;
    bool isValid {false};
    const char* getDeviceName(); //{return p->getDeviceName();}
    std::vector<double> m_xv, m_yv;  // Historical values up to m_order
    double m_sumCorr; // Sum of all corrections issued
    std::vector<double> xcoeffs, ycoeffs;
    std::vector<double> rxcoeffs, rycoeffs;

    const double TWOPI = (2.0 * M_PI);
    const double EPS = 1e-10;
    FILTER_DESIGN filt;
    int m_order;
    double raw_alpha1, raw_alpha2;
    bool isMzt;

    std::complex<double> dc_gain, fc_gain, hf_gain;
    double warped_alpha1, warped_alpha2;
    double chripple;
    std::vector<std::complex<double>> bessel_poles;
    std::vector<std::complex<double>> spoles, szeros;
    std::vector<std::complex<double>> zpoles, zzeros;

    void splane();
    void setpole(const std::complex<double>&);
    void prewarp();
    void normalize();
    void zplane();
    std::complex<double> bilinear(const std::complex<double>&);
    void expandpoly();
    void reversecoeffs();
    void expand(const std::vector<std::complex<double>>&, 
                std::vector<std::complex<double>>&);
    void multin(const std::complex<double>&, std::vector<std::complex<double>>&);
    std::complex<double> eval(const std::vector<std::complex<double>>& coeffs,
                              const std::complex<double>& z);
};

inline void ZFilterFactory::setpole(const std::complex<double>& z)
{
    if (z.real() < 0.0)
    {
        spoles.push_back(z);
    }
}

inline std::complex<double> ZFilterFactory::bilinear(const std::complex<double>& pz)
{
    return (2.0 + pz) / (2.0 - pz);
}

inline std::string ZFilterFactory::getname() const
{
    switch (filt)
    {
    case BUTTERWORTH: return "Butterworth";
    case BESSEL: return "Bessel";
    case CHEBYCHEV: return "Chebychev";
    default: return "Unknown filter";
    }
}

inline void ZFilterFactory::reversecoeffs()
{
    rxcoeffs = xcoeffs;
    std::reverse(rxcoeffs.begin(), rxcoeffs.end());
    rycoeffs = ycoeffs;
    std::reverse(rycoeffs.begin(), rycoeffs.end());
}

inline const char* ZFilterFactory::getDeviceName()
{
    return p->getDeviceName();
}

#endif /* ZFILTER_FACTORY_H_INCLUDED */


