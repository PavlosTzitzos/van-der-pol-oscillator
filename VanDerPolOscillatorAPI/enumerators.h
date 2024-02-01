#pragma once

//#ifndef ENUMERATORS_INCLUDE
//#define ENUMERATORS_INCLUDE

#include <iostream>

namespace vdpo
{
    //Algorithm select Enumerator
    enum class algorithm
    {
        FD,    /* Finite Differences */
        SPSA,  /* Simultaneous Perturbation Stochastic Approximation */
        LQR,    /* Linear Quadratic Regulator */
        AC      /* Adaptive Controller */
    };

    enum class theta
    {
        two,
        three
    };
};

//#endif
