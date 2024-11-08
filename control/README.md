# Control Module

## About

This class provides the control commands for high-level or low-level vessel control.

## Dependencies

For executing the MPC controller, the [Casadi tool for MATLAB](https://web.casadi.org/) is required .

## Properties
 - num_st and num_ct are reserved for the desiner and represents number of states and number of controls in the MPC model, respectively.
- pid_params: Contains the PID controller gains(K_p, K_i, K_d), psi_d_old: desired heading angle for next iteration, and
 error_old: heading tracking error for next iteration. datatype: struct. 
- mpc_params: mpc_params = struct('Ts', sampling time, 'N', Prediction horizon, 'headingGain', Q in the cost function, 'rudderGain', R in the cost function, 'max_iter', maximum iteration of the MPC solver, 'deltaMAX', maximum allowed rudder angle)
- Flag_cont: Reserved to act as a method to select the controller type (PID, MPC). (Just put 0 or 1 the outcome is the same at the moment).

## Methods

- init_mpc: This function implements the high-level controller.
- initial_guess_creator: Providing it with initial states and control values this method will create the initial guess for the MPC solver
- constraintcreator: This method forms the constraint vectors
- LowLevelPIDCtrl: This method implements the low-level PID controller.
- LowLevelMPCCtrl: This method implements the low-level MPC controller.
   
## Contact

Abhishek Dhyani: <A.Dhyani-1@tudelft.nl>
Amirreza Haqshenas Mojaveri: <amirreza.haqshenasmojaveri@kuleuven.be>
