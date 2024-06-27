# Collision Avoidance (COLAV)

## About
The objective of this class is to employ reference a collision avoidance algorithm for obstacle avoidance.

## Dependencies
N/A

## Features
The following methods are avialble for the user in the **sbmpc** class:
- #### sbmpcObj = sbmpc(T, dt, varargin):
  This constructor is used to initialize an instance of the **sbmpc** class.
- #### [chi_c, U_c, chi_m, U_m] = run_sbmpc(self, x, chi_d, U_d, chi_m_last, U_m_last, x_ts):
  This method is used to calculate the modification to the course and speed commands from the guidance controller using SB-MPC algorithm.

## Contact
Dhanika Mahipala: dhanika.mahipala@km.kongsberg.com
Hoang Anh Tran: hoang.a.tran@ntnu.no
