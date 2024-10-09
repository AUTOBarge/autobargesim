# modelClass

## About

This class provides dynamic models as virtual sensor / control reference model based on provided ship dimensions, environment setting and actuating forces from actuatorClass.

## Dependencies

None

## Properties

- ship_dim: Ship dimensions. datatype: structure array.
- dyn_model_params: Parameters in the sensor dynamic model. datatype: structure array.
- ref_model_params: Parameters in the control reference model. datatype: structure array.
- KTindex: Parameters in the Nomoto model. datatype: structure array.
- sensor_state: States used in sensor dynamic model. datatype: array (6, 1).
- sensor_state_dot: Output (state_dot) of sensor dynamic model. datatype: array (6, 1).
- sensor_vel_relative: sensor_vel_relative: Relative ship velocity over water under ship frame. datatype: array (3, 1).
- ref_state: States used in control reference model. datatype: array (6, 1).
- ref_state_dot: Output (state_dot) of control reference model. datatype: array (6, 1).

## Methods

- pramsCalculator:
  - ship_params_calculator: This function calculates model parameters using empirical formulas based on ship dimensions and environment.
    - Input Arguments:
       env_set (structure array): Environment setting.
       rud_params (structure array): Rudder force model parameters.
    - Output Arguments:
       obj.dyn_model_params (structure array): Parameters in the sensor dynamic model.
       obj.ref_model_params (structure array): Parameters in the control reference model.
       obj.KTindex (structure array): Parameters in the Nomoto model.
- Models:
  - sensor_dynamic_model: This function provides a dynamic model for 3DOF maneuvering motion. It is highly accurate and serves as a virtual sensor.
    - Input Arguments:
       Act: Actuator object.
       env_set (structure array): Environment setting.
    - Output Arguments:
       obj.sensor_state_dot (array (6, 1))
  - ctrl_reference_model: This function provides reference model for controllers (Fossen form with linear damping).
    - Input Arguments:
       Act: Actuator object.
       env_set (structure array): Environment setting.
    - Output Arguments:
       obj.ref_state_dot (array (6, 1))

# actuatorClass

## About

This class provides a force model for actuators.

## Dependencies

None

## Properties

- ship_dim: Ship dimensions. datatype: structure array.
- env_set: External environment. datatype: structure array.
- prop_params: Propeller force model parameters. datatype: structure array.
- rud_params: Rudder force model parameters. datatype: structure array.
- ctrl_actual: Actual control actions (n, delta). datatype: array (1, 2).
- F_P: Propeller force. datatype: array (3, 1).
- F_R: Rudder force. datatype: array (3, 1).
- tau_act: Total actuation force. datatype: array (3, 1).

## Methods

- lowLevelControl:
  - act_response: This function describes the response of the actuators to the control command.
    - Input Arguments:
       ctrl_last (array (1, 2)): Last control action.
       ctrl_command (array (1, 2)): Current control command.
       h (num): Time step.
    - Output Arguments:
       obj.ctrl_actual (array (1, 2)): Actual control actions.
- forceModels:
  - get_prop_force: This function provides a propeller force model.
    - Input Arguments:
       env_set (structure array): Environment setting.
       vel (array (3, 1)): Relative ship velocity over water under ship frame.
    - Output Arguments:
       J_P (num): Propeller advanced ratio.
       K_T (num): Propeller thrust open water characteristic.
       obj.F_P (array (3, 1)): Propeller force matrix.
  - get_rud_force: This function provides a rudder force model.
    - Input Arguments:
       env_set (structure array): Environment setting.
       vel (array): Relative ship velocity over water under ship frame.
       J_P (num): Propeller advanced ratio.
       K_T (num): Propeller thrust open water characteristic.
    - Output Arguments:
       obj.F_R (array (3, 1)): Rudder force matrix.
  - get_act_force: This function combines forces from all actuation devices and produces a total actuation force.
    - Input Arguments:
       None
    - Output Arguments:
       obj.tau_act (array (3, 1)): Actuator force matrix.

## Contact

Yan-Yun Zhang: <yanyun.zhang@kuleuven.be>
Chengqian Zhang: <chengqian.zhang@chalmers.se>
