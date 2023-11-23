# {actuatorClass}

## About

This class provides a force model for actuators.

## Dependencies

Datatype dictionary: Require MATLAB R2022b or higher.

## Properties

- ship_dim: Ship dimensions.
- env_set: External environment.
- prop_params: Propeller force model parameters.
- rud_params: Rudder force model parameters.
- h: Sample time in s.
- ctrl_last: Last step Control actions (n_k, delta_k).
- ctrl_command: Control commands (n_c, delta_c).
- ctrl_actual: Actual control actions (n, delta).
- vel: Ship velocity (u, v, r).
- F_P: Propeller force.
- F_R: Rudder force.
- tau_act: Total actuation force.

## Methods

- lowLevelControl:
       -- act_response: This function describes the response of the actuators to the control command.
- forceModels:
       -- get_prop_force: This function provides a propeller force model.
       -- get_rud_force: This function provides a rudder force model.
       -- get_act_force: This function combines forces from all actuation devices and produces a total actuation force.

## Contact

Yan-Yun Zhang: <yanyun.zhang@kuleuven.be>
