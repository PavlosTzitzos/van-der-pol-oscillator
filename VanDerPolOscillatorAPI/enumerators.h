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
        none = 0,
        two = 1,
        three = 2
    };
};

//#endif
