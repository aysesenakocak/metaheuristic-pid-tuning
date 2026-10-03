function cost = pid_cost(params)
    % PID_COST Evaluates control performance of candidate PID parameters.
    % Inputs:
    %   params - 1x3 vector containing [Kp, Ki, Kd]
    % Outputs:
    %   cost   - Scalar metric combining ITAE and overshoot penalty

    % Inject candidate parameters into MATLAB base workspace for Simulink
    assignin('base', 'Kp', params(1));
    assignin('base', 'Ki', params(2));
    assignin('base', 'Kd', params(3));

    % Execute the Simulink model silently
    simOut = sim('dc_motor_model_cost', 'StopTime', '3');

    % Extract signals from 'To Workspace' blocks
    e_ts = simOut.get('e');
    y_ts = simOut.get('y');

    t = e_ts.Time;
    e = e_ts.Data;
    y = y_ts.Data;

    % Calculate peak overshoot relative to unit step reference (r = 1)
    Mp = max(0, (max(y) - 1));

    % Calculate Integral of Time-weighted Absolute Error (ITAE)
    ITAE = trapz(t, t .* abs(e));

    % Total weighted cost
    cost = ITAE + 50 * Mp;
end
