# What you will find inside this solution

## Projects

Report Folder: The report of the initial version.

VanDerPolOscillator(C++11): The initial version (functions only).

VanDerPolOscillatorAPI(C++14): The version with classes (an attempt).

VanDerPolOscillatorTests(C++14): Tests for the API.

## About Tests

You will also find Tests, which you can run. There are some tests that will fail on purpose.

There are three sections in the test project:

1. Section 1 tests the initial project with functions

2. Section 2 tests the second project with classes

3. Section 3 tests both of them to compare their results.

## About Random values , Generators and Distributions

Since rand() function in C++ is a pseudo-random values generator it is not recommended for scientific calculations especially for control systems where the system analysis and systhesis is important. Do not get confused with MATLAB rand() function which is already a uniform distribution of random numbers in range (0,1). To implement the same concept in C++ we declare a 'default_random_engine' object and then we create a distribution of elements in a range we want eg(0.9,1.1) and then we take from the distribution an element randomly selected from the 'default_random_engine'. In this way we can implemnt the Spall requiremnt of Dk to be random with some requirements*. According to Spall the simplest distribution that matches the requirements* is the bernoulli distribution with 0.5 percent probability. Then change the sign if the number is negative or positive.

In the sensitivity analysis the distributions' ranges where selected based on tests. All those parameters cannot have big changes otherwise the control algorithm cannot drive the system to the final state and goes to infinity.

## Conventions

About the names of the parameters. Due to some limitations on naming like how to seperate greek and english 'a' in C/C++ there is a table to clear this.

[insert table]
variable -> actual name

-----------------------

betta -> b greek

gamma -> greek

alpha -> greek

A -> capital A

a -> greek a

p -> english p

hetta -> greek hetta

dtheta -> \Delta \theta


Another convention is that the whole project is named 'plant' although in bibliography the 'plant' is for the system under control. That will be made more clear with the following diagrams of the implementations where the names of the classes are present:

[insert block diagrams]

The algorithm is the control Law.

The System is the Van der Pol Oscillator.

The performance / cost / value block is the controller.

The control law optimizes the controller (or makes the theta parameters the optimal).

The plant contains all the above.

## Libraries

The libraries that where used are:

1. chrono : to access real time and measure the actual time (not the CPU time) elapsed during execution

2. vector : to manipulate the data for plots

3. gnuplot-iostream.h : to plot data without creating from scratch the plot window

4. random : to access the random functions and the bernoulli distribution for the SPSA

5. stdexcept : to handle the exceptions (try-catch blocks)

There was also the Eigen library that could be used for the matrices.

## Classes of API Project

First of all, the classes are 5 :

[insert the image from the class diagram]

The entry point is the plant.h and everything else is included in there.

## Defaults of API Project

The defaults are:

1. Finite Differences Algorithm
2. Two theta parameters and u2() control signal
3. The sensitivity analysis will not run
4. Plots will not show 
5. The system parameters are (k,m,c) = (1,1,1)
6. The starting state space variables values are (x1,x2) = (1,1)
7. The theta parameters initial values are (0,0)

## Enum Class algorithm

To easily refer to and select the available algorithms.

1. FD: Finite Differences

2. SPSA: Simultaneous Perturbation Stochastic Approximation

3. LQR: Linear Quadratic Requlator (Solves the DARE and then uses the K matrix to calculate the cost J)

4. AC: Adaptive Controller (Indirect Model Reference Adaptive Controller)

## Enum Class theta

The theta parameters can be 2 or three or more. Currently only 2 and 3 are implemented.

## Class systemModel

Contains the van der pol oscillator state space equations and the sytem parameters (k,m,c) as well as the number of theta parameters.

The methods are:

1. systemModel(): Default Constructor

2. systemModel(double kk, double mm, double cc, theta numOfTheta): Constructor with k,m,c, number of theta initializers

3. systemModel(double systemParameters[3], theta numOfTheta): same but k,m,c are combined in one vector

4. u2(): calculates the control signal when there are 2 theta parameters

5. u3(): calculates the control signal when there are 3 theta parameters

6. uK(): calculates the control signal when the LQR method is used (LQR does not use theta parameters but the K 1x2 vector)

7. dxCalculate(): calculats the state space values of the x1,x2 derivatives. Selects the appropriate u function.

8. There is a collection of accessors for easy get and set values.

## Class displayData

This is a wrapper of the [gnuplot-iostram.h](https://github.com/dstahlke/gnuplot-iostream/wiki) by Daniel Stahlke.

The data to be ploted are very specific, as well as the number of diagrams. Generally the plots needed are:

1. Performance / Cost / Value vs algorithm iterations.

2. x1 vs x2  (data from the last iteration only of the algorithm).

3. Optional x1 vs time, x2 vs time  (data from the last iteration only of the algorithm).

4. Additionally the theta values could be plot vs the algorithm iterations.

The difference between 'algorithm iterations' and 'time' is that the first term refers to the algorithm eg FD or LQR (calculate theta) and the second refers to the drive of the van der pol oscillator (use the calculated theta).

The methods are:

1. Constructs

2. setVectorId(): Id is a value from 1 to 6 and the arguments of the functions can be a double (will put the element in the last position), double 1xN array (just copies the elements) or double vector (same thing)

3. setXrange() and setYRange(): Custom plot range in the window to appear. Default is \[-1,1\].

4. setPair(): For the special case of (x1,x2) a special setter is used which creates a std::vector with std::pair as elements

5. plotDataId(): Id is a vaule from 1 to 6. Plots the vectors corresponding to the number eg plotData5() plots vectors 1 to 5 in a single xy-plot in a single gnuplot window.

6. plotPair(): after the pair is filled with data then calling this function generates a plot of the vectors 1 and 2. Eg vector1 = [1,5,83,2] and vector2 = [6,2,3,5] the it will plot the points (1,6), (5,2), (83,3), (2,5).

6. exportData(): export data of vectors in a txt file

7. consoleWriteId(): prints on console the data inside the vectorId. Maybe will be deleted since it is not very useful.

## Class ControlAlgorithm

Structure:

1. Base Class

2. FD subclass

3. SPSA subclass

4. LQR subclass

5. AC subclass

Each of the subclasses contains a collection of Accessors for the parameters of the algorithm (Control Law) and four methods:

1. Constructors

2. runALgorithm(): This is the top function

3. sensitivityAnalsis(): something like second level

4. coreFunction(): this is contains the control law, the calls for the plotting functions, cost functions, conditions to stop the execution and some exception handling.

5. costFunction(): this is the performance / cost / value functions and contain the "simulation" of the system, calculates performance or cost, drives the model to final state, checks the conditions to stop and there is some exception handling.

## NOTE: About the exception handling:

If the algorithm fails to drive the system to zero (this is the final state) it might go to infinite (eg x1 very very large). This can result to overflow of the double types.

To prevent this from happening if NaN , inf or numbers greater than a specific number appear throws an exception and stops the execution. If you try to plot the data then the last element that was caught by the try-catch block will be removed. Previously the plot is was mess due to this problem.

To solve this there was another way to change the datatypes to something larger but since the oscillator from the initial state space values and the final can be expressed inside the limits of double then there is no need to change it. Just change the parameters to match the desired behaviour, since that's what this whole project is all about; to make the system behave the way we want.

Base Class
-----------

Contains a Base Class with properties:

1. displayGraph: set this to false and no window will be plotted. Useful if you only want the simulation and not the graphs.

2. x and thetaVar: arrays storing the values during the simulation.

3. simulationIterations: it is the time duration setted for the performance/cost/value to run.

4. iterationsCalculate(): calculates the above value.

5. startTime, stepTime and finalTime: used by the iterationsCalculate() and the performance / cost / value functions.

6. maxRepeats: this is different from simulationIterations. It is the user-defined maximum number of iterations to run. 

There are two simulation parameters: maximum iterations and time parameters. For example:

Start at 0.0, step with 0.5 and stop at 1.0 then this will give 20 simulationIterations.

But for some reason let's say we want to stop at 15 then the maxRepeats will be 15 and it will force the algorithm to stop after the 14th iteration (counting from 0).

7. There are Setters and Getters for the above three values.

8. setSystemModel(arg): the arg is a systemModel object and when this function is called it copies the details of the arg to the protected localModel.

9. There are also two displayData objects plotSSV and plotPerformance and vectors to store data to plot.

FD Subclass
-----------

1. FD(): Default Constructor initializes for two theta parameters and no sensitivity analysis

2. FD(theta numberOfTheta, bool sensitivity): Constructor 

3. runAlgorithm(): if sensitivityAnalysis is true then this funtion calls the sensitivityAnalyzer()

4. finiteDifferences(): the control algorithm that drives the theta parameters

5. performance(): the controller that drives the system for specific theta parameters

6. sensitivityAnalyzer(): iterates the control algorithm parameters h and dtheta. One at a time. The other gets a random value.

7. P: the final Performance of the final iteration

8. There are Accessors for the parameters.

SPSA Subclass
-----------

1. SPSA(): Default Constructor initializes for two theta parameters and no sensitivity analysis

2. SPSA(theta numberOfTheta, bool sensitivity): Constructor 

3. runAlgorithm(): if sensitivityAnalysis is true then this funtion calls the sensitivityAnalyzer()

4. spsa(): the control algorithm that drives the theta parameters

5. performance(): the controller that drives the system for specific theta parameters

6. sensitivityAnalyzer(): iterates the control algorithm parameters b,a,A,gamma,alpha. One at a time. The other gets a random value. The p parameter is the probability which is used in the bernoulli distribution as the SPSA Spall papers suggest and is 0.5 please do not change this value.

7. P: the final Performance of the final iteration

8. There are Accessors for the parameters.

LQR Subclass
-----------

1. LQR(): Default Constructor initializes for two theta parameters and no sensitivity analysis

2. LQR(double Q[4], double R, bool sensitivityAnalysis): Constructor with Q and R initializers

3. runAlgorithm(): if sensitivityAnalysis is true then this funtion calls the sensitivityAnalyzer()

4. riccati(): the control algorithm that drives the theta parameters

5. cost(): the controller that drives the system for specific theta parameters

6. sensitivityAnalyzer(): iterates the control algorithm matrices Q and R. One matrice at a time. The other stays fixed. The difference with the above sensitivity analyzers is that it iterates the q and r around their initial values (q-1 <= q <= q+1 and r-1 <= r <= r+1). 

7. J: the final Performance of the final iteration

8. There are Accessors for the matrices Q and R.

AC Subclass
-----------

1. please fill this after the implementation and tests are finished for this class.


## Class plant

This is the entry point of the whole project. Easily select algorithm, number of theta parameters, set initial x and theta values, system parameters (k,m,c) and optionally run sensititvity analysis.
The available maethods are:

1. plant(): Default Constructor

2. plant(bool sensitivityAnalysis): Default values and runs sensitivity analysis

3. plant(double k, double m, double c, algorithm selectAlgorithm, bool sensitivityAnalysis): Initialize the parameters

4. plant(double systemParameters[3], algorithm selectAlgorithm, bool sensitivityAnalysis): same but in array syntax

5. simulatePlant(): After initialization and / or additional configuration, run this method and this will instatiate and configure all nessecary objects.

6. help(): not yet implemented

7. Contains three setters for x and theta

# Known Errors and Problems

1. Very big numbers (bigger than DOUBLE_MAX) are considered inf or NaN which is unreallistic. Very small numbers like 10e-100 (smaller than the smallest represented with double) are considered zero.

2. The initial version with functions uses almost all of the available stack making everything slow down.
