# {modelClass}

## About

This class provides dynamic models as virtual sensor / control reference model based on provided ship dimensions, environment setting and actuating forces from actuatorClass.

## Dependencies

None

## Properties

- ship_dim: Ship dimensions. datatype: structure array.
- env_set: External environment. datatype: structure array.
- dyn_model_params: Parameters in the sensor dynamic model. datatype: structure array.
- ref_model_params: Parameters in the control reference model. datatype: structure array.
- sensor_state: States used in sensor dynamic model. datatype: array (6, 1).
- sensor_state_dot: Output (state_dot) of sensor dynamic model. datatype: array (6, 1).
- sensor_vel_relative: Relative velocity over ground used in sensor dynamic model. datatype: array (3, 1).
- ref_state: States used in control reference model. datatype: array (6, 1).
- ref_state_dot: Output (state_dot) of control reference model. datatype: array (6, 1).

## Methods

- pramsCalculator:
       -- ship_params_calculator: This function calculates model parameters using empirical formulas based on ship dimensions and environment.
       -- ship_params_calculator: This function calculates model parameters using empirical formulas based on ship dimensions.
- Models:
       -- sensor_dynamic_model: This function provides a dynamic model for 3DOF maneuvering motion. It is highly accurate and serves as a virtual sensor.
       -- ctrl_reference_model: This function provides reference model for controllers.

## Contact

Yan-Yun Zhang: <yanyun.zhang@kuleuven.be>
Chengqian Zhang: <chengqian.zhang@chalmers.se>
