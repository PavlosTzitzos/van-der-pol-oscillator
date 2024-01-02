# van-der-pol-oscillator

At FD use a constant value between -0.1 &leq; &Delta; &theta; &leq; 0.1.

At SPSA use a constant value but random for dtheta.

## ToDo List :
- [ ] Write the report using latex. (check below)
- [x] Fix code to work for finite differences.
- [x] Make SPSA algorithm work. (SPSA spall)
- [ ] make plot of position in 2d.
- [x] make plot of performance vs time tt.
- [x] Add comments with latex.
- [x] add links in readme for the resources (everything that is not a reference in the report)

## Report Structure:
- [ ] 1) Problem , Methods (Gradient descent: FD , SPSA)
- [ ] 2) Code description
- [ ] 3) For m=c=k=1 what are the perf , init , final
- [ ] 4) Sensitivity Analysis (check below)
- [ ] 5) k , c , m : all possible combinations of the 1,0,-1 using the best of the above
- [ ] 6) the plots of the above combinations
- [ ] 7) References

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

- [Plotting Library - tutorial](https://youtu.be/gsLIUtmTs8Q)

- [SPSA spall](https://www.jhuapl.edu/SPSA/index.html)
