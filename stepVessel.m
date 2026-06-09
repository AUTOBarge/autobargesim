function [os, xout_row, pout_row, stop_now] = stepVessel(os, ts_sensor_states, i, h, Flag_cont, has_ts, env_set, time)
% This function performs one simulation step for the vessel. 
% It returns the updated vessel struct, the xout and pout rows for this step,
% and a stop_now flag indicating that the vessel has reached or passed its last waypoint.

    psi = os.model.sensor_state(6);
    r = os.model.sensor_state(3);

    os.wp.idx = os.guidance.find_active_wp_segment(os.wp.pos, os.model.sensor_state', os.wp.idx);
    [chi, U] = os.guidance.compute_LOSRef(os.wp.pos, os.wp.speed, os.model.sensor_state', os.wp.idx, 1, os.chi_d_prev, i);
    chi_los = chi;
    U_los = U;

    if has_ts
        own_xy = [os.model.sensor_state(4), os.model.sensor_state(5)];
        d_min_ts = min(vecnorm(ts_sensor_states(:, 4:5) - own_xy, 2, 2));
        react_distance = 8 * os.colav.alg.tuning_param.D_CLOSE_;
        if d_min_ts < react_distance
            effective_stride = 1;
        else
            effective_stride = os.colav.alg.update_stride;
        end
        if mod(i, effective_stride) == 1
            [chi, U, os.colav.param(1), os.colav.param(2)] = os.colav.alg.run_sbmpc(os.model.sensor_state', ...
                chi_los, U_los, os.colav.param(1), os.colav.param(2), ts_sensor_states);
        else
            chi = utils.wrap_angle_to_pmpi(chi_los + os.colav.param(1));
            U = U_los * os.colav.param(2);
        end
    end
    os.chi_d_prev = chi_los;

    if Flag_cont == 2
        r_d = utils.wrap_angle_to_pmpi(chi - psi);
        [ctrl_command_MPC, os.control.param.next_guess, os.control.model] = os.control.model.LowLevelMPCCtrl(vertcat(os.model.sensor_state, os.control.output), chi, r_d, os.control.param.args, os.control.param.next_guess, os.control.param.mpc_nlp);
        ctrl_command = [250; ctrl_command_MPC];
    else
        [ctrl_command, os.control.model] = os.control.model.LowLevelPIDCtrl(chi, r, psi, h);
    end

    os.actuators = os.actuators.act_response(os.control.output, ctrl_command, h);
    os.model = os.model.sensor_dynamic_model(os.actuators, env_set);
    os.model.sensor_state = os.model.sensor_state + os.model.sensor_state_dot * h;

    [xte, psi_er, os.err.xtetot, os.err.psi_er_tot, os.control.model] = os.control.model.XTECalc(os.model.sensor_state, chi, os.wp.pos, os.wp.idx, os.err.xtetot, os.err.psi_er_tot);
    os.control.output = os.actuators.ctrl_actual';

    xout_row = [time, os.model.sensor_state', os.actuators.ctrl_actual, os.model.sensor_state_dot(1:3)'];
    pout_row = [xte, psi_er, os.err.xtetot, os.err.psi_er_tot];

    x_cur = os.model.sensor_state(4);
    y_cur = os.model.sensor_state(5);
    d_threshold = norm([x_cur - os.wp.pos(end, 1), y_cur - os.wp.pos(end, 2)], 2);
    has_passed_end = false;
    n_wps = size(os.wp.pos, 1);
    if n_wps >= 2 && os.wp.idx >= n_wps - 1
        seg_end = os.wp.pos(end, :) - os.wp.pos(end-1, :);
        seg_len = norm(seg_end);
        if seg_len > 0
            proj = dot([x_cur, y_cur] - os.wp.pos(end-1, :), seg_end / seg_len);
            has_passed_end = proj > seg_len;
        end
    end
    stop_now = (d_threshold < 20) || has_passed_end;
end
