/*
*  zfilterfactory.cpp
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

/* Based on mkfilter by A.J. Fisher, University of York    September 1992
https://www-users.cs.york.ac.uk/~fisher/mkfilter/
<fisher@minster.york.ac.uk>
*/

#include <stdio.h>
#include <math.h>
#include <string.h>

#include "zfilterfactory.h"

/*******************************************************************************
 *
 ******************************************************************************/
ZFilterFactory::ZFilterFactory(StarGoTelescope* ptr)
{
    p = ptr;
    bessel_poles = {
    /* table produced by /usr/fisher/bessel --  N.B. only one member of each C.Conj. pair is listed */
        std::complex<double>( -1.00000000000e+00, 0.00000000000e+00),
        std::complex<double>( -1.10160133059e+00, 6.36009824757e-01),
        std::complex<double>( -1.32267579991e+00, 0.00000000000e+00),
        std::complex<double>( -1.04740916101e+00, 9.99264436281e-01),
        std::complex<double>( -1.37006783055e+00, 4.10249717494e-01),
        std::complex<double>( -9.95208764350e-01, 1.25710573945e+00),
        std::complex<double>( -1.50231627145e+00, 0.00000000000e+00),
        std::complex<double>( -1.38087732586e+00, 7.17909587627e-01),
        std::complex<double>( -9.57676548563e-01, 1.47112432073e+00),
        std::complex<double>( -1.57149040362e+00, 3.20896374221e-01),
        std::complex<double>( -1.38185809760e+00, 9.71471890712e-01),
        std::complex<double>( -9.30656522947e-01, 1.66186326894e+00),
        std::complex<double>( -1.68436817927e+00, 0.00000000000e+00),
        std::complex<double>( -1.61203876622e+00, 5.89244506931e-01),
        std::complex<double>( -1.37890321680e+00, 1.19156677780e+00),
        std::complex<double>( -9.09867780623e-01, 1.83645135304e+00),
        std::complex<double>( -1.75740840040e+00, 2.72867575103e-01),
        std::complex<double>( -1.63693941813e+00, 8.22795625139e-01),
        std::complex<double>( -1.37384121764e+00, 1.38835657588e+00),
        std::complex<double>( -8.92869718847e-01, 1.99832584364e+00),
        std::complex<double>( -1.85660050123e+00, 0.00000000000e+00),
        std::complex<double>( -1.80717053496e+00, 5.12383730575e-01),
        std::complex<double>( -1.65239648458e+00, 1.03138956698e+00),
        std::complex<double>( -1.36758830979e+00, 1.56773371224e+00),
        std::complex<double>( -8.78399276161e-01, 2.14980052431e+00),
        std::complex<double>( -1.92761969145e+00, 2.41623471082e-01),
        std::complex<double>( -1.84219624443e+00, 7.27257597722e-01),
        std::complex<double>( -1.66181024140e+00, 1.22110021857e+00),
        std::complex<double>( -1.36069227838e+00, 1.73350574267e+00),
        std::complex<double>( -8.65756901707e-01, 2.29260483098e+00),
    };
}

/*******************************************************************************
 *
 ******************************************************************************/
void ZFilterFactory::resetsamples()
{
    LOG_DEBUG(__FUNCTION__);
    m_xv.clear();
    m_yv.clear();
    m_sumCorr = 0.0;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool ZFilterFactory::rebuild(FILTER_DESIGN f, int o, double p, bool mzt )
{
    LOG_DEBUG(__FUNCTION__);
    bool bError = false;

    if (o <= 0)
    {
        LOGF_ERROR("invalid filter order %d", o);
    }
    if (p < 2.0)
    {
        LOGF_ERROR("invalid corner period multiplier %.4f", p);
    }

    xcoeffs.clear();
    ycoeffs.clear();
    filt = f;
    m_order = o;
    raw_alpha2 = raw_alpha1 = 1.0 / p;
    isMzt = (filt == BESSEL) ? mzt : false; // mzt only applies to BESSEL filter

    spoles.clear();
    szeros.clear();

    splane();
    prewarp();
    normalize();
    zplane();
    expandpoly();
    reversecoeffs();

    resetsamples();

    return bError;
}

/*******************************************************************************
 *
 ******************************************************************************/
double ZFilterFactory::addsample(double input)
{
    LOG_DEBUG(__FUNCTION__);
    double dReturn=0;

//    Digital filter designed by mkfilter/mkshape/gencode   A.J. Fisher
    double l_gain = gain();

// Shift readings and results
    m_xv.insert(m_xv.begin(), (input + m_sumCorr) / l_gain); // Add total guide output to input to get uncorrected waveform
    m_xv.pop_back();
    m_yv.insert(m_yv.begin(), 0.0);
    m_yv.pop_back();

// Calculate filtered value
// FIXME: coeffs are in reverse order compared to samples
// Should reverse them in rebuild
    for (uint64_t i = 0; i < rxcoeffs.size(); i++)
    {
        m_yv.at(0) += m_xv.at(i) * rxcoeffs.at(i);
    }
    for (uint64_t i = 1; i < rycoeffs.size(); i++)
    {
        m_yv.at(0) += m_yv.at(i) * rycoeffs.at(i);
    }
    dReturn = m_yv.at(0) -  m_sumCorr; // Return the difference from the uncorrected waveform
    m_sumCorr += dReturn;

    return dReturn;
}

/*******************************************************************************
 * compute S-plane poles for prototype LP filter
 ******************************************************************************/
void ZFilterFactory::splane()
{
    LOG_DEBUG(__FUNCTION__);

    if (filt == BESSEL)                                 // Bessel filter
    {
        int p = (m_order * m_order) / 4;                // ptr into table
        if (m_order & 1)                                // If order is odd
            setpole(bessel_poles[p++]);
        for (int i = 0; i < m_order / 2; i++)
        {
            setpole(bessel_poles[p]);
            setpole(std::conj(bessel_poles[p]));
            p++;
        }
    }
    if (filt == BUTTERWORTH || filt == CHEBYCHEV)       // Butterworth filter
    {
        for (int i = 0; i < 2 * m_order; i++)
        {
            double theta = (m_order & 1) ? (i * M_PI) / m_order :
                           ((i + 0.5) * M_PI) / m_order;
            setpole(std::polar(1.0,theta));
        }
    }
    if (filt == CHEBYCHEV) // modify for Chebyshev (p. 136 DeFatta et al.)
    {
        if (chripple >= 0.0)
        {
            LOGF_DEBUG(" Chebyshev ripple is %g dB; must be .lt. 0.0", chripple);
            return;
        }
        double rip = pow(10.0, -chripple / 10.0);
        double eps = sqrt(rip - 1.0);
        double y = asinh(1.0 / eps) / (double) m_order;
        for (uint64_t i = 0; i < spoles.size(); i++)
        {
            spoles[i].real(spoles[i].real()*sinh(y));
            spoles[i].imag(spoles[i].imag()*cosh(y));
        }
    }
}

/*******************************************************************************
 * for bilinear transform, perform pre-warp on alpha values
 ******************************************************************************/
void ZFilterFactory::prewarp()
{
    LOG_DEBUG(__FUNCTION__);

    if (isMzt)
    { // Dont prewarp; or use z-transform
         warped_alpha1 = raw_alpha1;
        warped_alpha2 = raw_alpha2;
    }
    else
    {
        warped_alpha1 = tan(M_PI * raw_alpha1) / M_PI;
        warped_alpha2 = tan(M_PI * raw_alpha2) / M_PI;
    }
}

/*******************************************************************************
 * called for trad, not for -Re or -Pi
 ******************************************************************************/
void ZFilterFactory::normalize()
{
    // Only Lp processing included here
    LOG_DEBUG(__FUNCTION__);
    double w1 = TWOPI * warped_alpha1;
    for (uint64_t i = 0; i < spoles.size(); i++)
    {
        spoles[i] = spoles[i] * w1;
    }
    szeros.clear();
}

/*******************************************************************************
 * given S-plane poles & zeros, compute Z-plane poles & zeros
 * using bilinear transform or matched z-transform
 ******************************************************************************/
void ZFilterFactory::zplane()
{
    LOG_DEBUG(__FUNCTION__);

    uint64_t i;
    zpoles.clear();
    zzeros.clear();
    if(!isMzt)
    {
        for (i = 0; i < spoles.size(); i++)
            zpoles.push_back(bilinear(spoles[i]));
        for (i = 0; i < szeros.size(); i++)
            zzeros.push_back(bilinear(szeros[i]));
        while (zzeros.size() < zpoles.size())
            zzeros.push_back(-1.0);
    }
    else
    {
        for (i = 0; i < spoles.size(); i++)
            zpoles.push_back(std::exp(spoles[i]));
        for (i = 0; i < szeros.size(); i++)
            zzeros.push_back(std::exp(szeros[i]));
    }
}

/*******************************************************************************
 * given Z-plane poles & zeros, compute top & bot polynomials in Z,
 * and then recurrence relation
 ******************************************************************************/
void ZFilterFactory::expandpoly()
{
    LOG_DEBUG(__FUNCTION__);

    std::vector<std::complex<double>> topcoeffs, botcoeffs;
    uint64_t i;

    expand(zzeros, topcoeffs);
    expand(zpoles, botcoeffs);

    double theta = TWOPI * 0.5 * (raw_alpha1 + raw_alpha2); /* "jwT" for centre freq. */
    const std::complex<double> z_one(1.0, 0.0);
    const std::complex<double> z_minusone(-1.0, 0.0);
    const std::complex<double> z_theta = std::polar(1.0, theta);

    xcoeffs.clear();
    ycoeffs.clear();
    dc_gain = eval(topcoeffs, z_one) / eval(botcoeffs, z_one);
    fc_gain = eval(topcoeffs, z_theta) / eval(botcoeffs, z_theta);
    hf_gain = eval(topcoeffs, z_minusone) / eval(botcoeffs, z_minusone);
    
    for (i = 0; i < topcoeffs.size(); i++)
    {
        xcoeffs.push_back( +(topcoeffs[i].real() / botcoeffs.back().real()) );
    }
    for (i = 0; i < botcoeffs.size(); i++)
    {
        ycoeffs.push_back( -(botcoeffs[i].real() / botcoeffs.back().real()) );
    }
}

/*******************************************************************************
 * compute product of poles or zeros as a polynomial of z
 ******************************************************************************/
void ZFilterFactory::expand(const std::vector<std::complex<double>>& pz,
    std::vector<std::complex<double>>& coeffs)
{
    LOG_DEBUG(__FUNCTION__);

    uint64_t i;
    coeffs.clear();
    coeffs.push_back(1.0);
    for (i = 0; i < pz.size(); i++)
    {
        coeffs.push_back(0.0);
    }

// coeffs now has 1 more element than pz
    for (i = 0; i < pz.size(); i++)
    {
        multin(pz[i], coeffs);
    }

// check computed coeffs of z^k are all real
    for (i = 0; i < coeffs.size(); i++)
    {
        if (fabs(coeffs[i].imag()) > EPS)
        {
            LOGF_DEBUG("mkfilter: coeff of z^%lu is not real; "
                "poles/zeros are not complex conjugates\n", i);
            // need to fail somehow
        }
    }
}

/*******************************************************************************
 * multiply factor (z-w) into coeffs
 ******************************************************************************/
void ZFilterFactory::multin(const std::complex<double>& w,
    std::vector<std::complex<double>>& coeffs)
{
    LOG_DEBUG(__FUNCTION__);

    std::complex<double> nw = -w;

    for (uint64_t i = coeffs.size()-1; i >= 1; i--)
    {
        coeffs[i] = (nw * coeffs[i]) + coeffs[i - 1];
    }
    coeffs[0] = nw * coeffs[0];
}

/*******************************************************************************
 * evaluate polynomial in z, substituting for z
 ******************************************************************************/
std::complex<double> ZFilterFactory::eval(
    const std::vector<std::complex<double>>& coeffs,
    const std::complex<double>& z)
{
    LOG_DEBUG(__FUNCTION__);

    std::complex<double> sum = std::complex<double>(0.0,0.0);
    for (uint64_t i = coeffs.size(); i >= 1; i--)
        sum = (sum * z) + coeffs[i-1];
    return sum;
}
