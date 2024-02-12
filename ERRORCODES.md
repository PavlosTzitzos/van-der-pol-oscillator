# Error Codes

Since the errors / NaNs / inf due to computation errors occur frequently, the need to organize the errors is big.

With that in mind the API version contains some codes for each possible error.

# Code List

## 1xx - Errors during calculation

These errors occur mainly due to improper initial values.


101 - Calculated Performance is NaN!

> To solve this you need to adjust the input parameters. Try some default values.

102 - Calculated Performance is infinity!

> To solve this you need to adjust the input parameters. Try some default values.

103 - Calculated x is NaN!

> This means that the system is unstable. You need to adjust the theta values close to the defaults. You can also try making smaller the x0,x1 initial values.

104 - Calculated x is infinity!

> This means that the system is unstable. You need to adjust the theta values close to the defaults. You can also try making smaller the x0,x1 initial values.
