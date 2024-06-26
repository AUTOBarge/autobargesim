# {actuatorClass}

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
       -- act_response: This function describes the response of the actuators to the control command.
- forceModels:
       -- get_prop_force: This function provides a propeller force model.
       -- get_rud_force: This function provides a rudder force model.
       -- get_act_force: This function combines forces from all actuation devices and produces a total actuation force.

## Contact

Yan-Yun Zhang: <yanyun.zhang@kuleuven.be>
Chengqian Zhang: <chengqian.zhang@chalmers.se>
