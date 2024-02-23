# {controlClass}

## About

This class provides the control commands for high-level or low-level vessel control.

## Dependencies

For executing the MPC controller, the Casadi tool for MATLAB is required (https://web.casadi.org/).

## Properties

- pid_params: Contains the PID controller gains(K_p, K_i, K_d), psi_d_old: desired heading angle for next iteration, and
 error_old: heading tracking error for next iteration. datatype: struct. 
- mpc_params: Contains MPC gains/weight matrices: Q, R. datatype: struct. 
- Flag_cont: To select the controller type (PID, MPC). datatype: double. 

## Methods

- init_mpc: This function implements the high-level controller.
- LowLevelPIDCtrl: This method implements the low-level PID controller.
- LowLevelMPCCtrl: This method implements the low-level MPC controller.
   
## Contact

Abhishek Dhyani: <A.Dhyani-1@tudelft.nl>
Amirreza Haqshenas Mojaveri: <amirreza.haqshenasmojaveri@kuleuven.be>
