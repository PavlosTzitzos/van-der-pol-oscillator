# van-der-pol-oscillator

This is a project to get some hands-on experience with basic C++ syntax, OOP.

Do not expect to find good and quality code here. There are many functions and features just to try different implementations of the same functionality.

However it is a good walkthrough from mathematics and some basics of Control Theory to actual code.

The initial version contains only functions.

The API version is an attempt to make a library and one entry point.

The Tests where necessary since there was no easy way to test all this methods.

The HDL version is an attempt to implement this controller this controller into actual hardware using SystemC.

## In this Repository

Report Folder: The report of the initial version.

VanDerPolOscillator(C++11): The initial version (functions only).

VanDerPolOscillatorAPI(C++14): The version with classes (an attempt).

VanDerPolOscillatorTests(C++14): Tests for the API.

VanDerPolOscillatorHDL(): The version of actual hardware using systemC (an attempt).

## ToDo List :
- [ ] Write the report using latex. (check below)
- [ ] make plot of state variables x1 vs x2 using GnuPlot.
- [ ] Implement the Sensitivity Analysis concept.
- [ ] Correct LQR and Adaptive Control algorithms.
- [ ] Make library.
- [ ] Write Unit Tests of the library.
- [ ] Does optimizating the code has any good result? If yes try it.

## Sensitivity Analysis
- [ ] Keep constant all parameters except one (lets say parameter p) and each time give it random values ( p = rand() )
- [ ] a) Vary h
- [ ] b) Vary Delta_t
- [ ] c) controller 1 vs controller 2 ( theta1 = theta2 = theta3 = 0 )
- [ ] d) theta1 , theta2 , theta3 -> 0
- [ ] keep: x1,x2 = 0.1 (generally small values)

# Sources :

- [LaTeX in Visual Studio - Setup](https://guillaumeblanchet.medium.com/using-latex-in-visual-studio-code-on-windows-121032043dad)

- [LaTeX Report Template](https://www.overleaf.com/learn/latex/How_to_Write_a_Thesis_in_LaTeX_(Part_1)%3A_Basic_Structure)

- [Plotting Library - official](http://www.gnuplot.info/)

- [Plotting Library - installation and tutorial](https://www.youtube.com/watch?v=gsLIUtmTs8Q)

- [Best Image Merger](https://www.onlineconverter.com/merge-images)

- [C++ Project Structure](https://medium.com/swlh/c-project-structure-for-cmake-67d60135f6f5)

- [An in depth guide on Overloading with examples - Microsoft Learn](https://learn.microsoft.com/en-us/cpp/cpp/function-overloading?view=msvc-170)

## Reference: C++ Project Structure in VS

1. Header Files (.h | #include "libName.h")

2. Source Files (.cpp)

3. Resource Files (.ico , .rc)

4. External Dependencies (third party libraries | managed by IntelliSense)

5. References (reference other projects/files in the same solution)
    * use case: if tests are in a different project in the same solution, to access the code to be tested references are used

## Reference: C++ Project Structure with CMake

1. include (.h | #include <libName.h> | Public)

2. src (.h , .m , .cpp | #include "libName.h" | Private | libName.cpp)

3. libs (third party libraries)

4. tests (.cpp | libNameTests.cpp)

## Structure of the Plant

Place image here

## SystemC installation on Windows 10 64-bit and Visual Studio 2022

You can follow the guide [here](https://stackoverflow.com/questions/41990606/installing-systemc-for-vs2013).

Or the steps in the [systemc-samples]().

