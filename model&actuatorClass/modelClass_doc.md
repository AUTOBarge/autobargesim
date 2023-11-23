# {modelClass}

## About

This class provides dynamic models as virtual sensor / control reference model based on provided ship dimensions, environment setting and actuating forces from actuatorClass.

## Dependencies

Datatype dictionary: Require MATLAB R2022b or higher.

## Properties

- ship_dim: Ship dimensions.
- env_set: External environment.
- dyn_model_params: Parameters in the sensor dynamic model.
- ref_model_params: Parameters in the control reference model.
- sensor_state: States used in sensor dynamic model.
- sensor_state_dot: Output (state_dot) of sensor dynamic model.
- ref_state: States used in control reference model.
- ref_state_dot: Output (state_dot) of control reference model.
- tau_act: Actuator forces (from actuatorClass).

## Methods

- pramsCalculator:
       -- ship_params_calculator: This function calculates model parameters using imperical formulas based on ship dimensions.
- Models:
       -- sensor_dynamic_model: This function provides a dynamic model for 3DOF maneuvring motion. It is highly accurate and serves as a virtual sensor.
       -- ctrl_reference_model: This function provides reference model for controllers.

## Contact

Yan-Yun Zhang: <yanyun.zhang@kuleuven.be>
