

function SH = fitSteinhartHart(R_ohm, T_C)
% R_ohm : vector of resistances in ohms
% T_C   : vector of temperatures in Celsius (same length as R_ohm)
% Returns struct SH with fields A,B,C, RMSE, maxErr_C, etc.

    % --- basic checks
    R_ohm = R_ohm(:);
    T_C   = T_C(:);
    assert(numel(R_ohm)==numel(T_C), 'R and T must be same length');
    assert(all(R_ohm>0), 'Resistances must be > 0');

    % --- convert to Kelvin
    T_K = T_C + 273.15;

    % --- build linear system: y = X * theta
    % y = 1/T_K ;  X = [1, lnR, (lnR)^3] ; theta = [A; B; C]
    lnR = log(R_ohm);
    X = [ones(size(lnR)), lnR, lnR.^3];
    y = 1 ./ T_K;

    % --- least-squares fit
    theta = X \ y;              % A,B,C by ordinary least squares
    A = theta(1); B = theta(2); C = theta(3);

    % --- predictions and errors
    y_hat = X * theta;          % predicted 1/T_K
    T_K_hat = 1 ./ y_hat;       % back to Kelvin
    T_C_hat = T_K_hat - 273.15; % predicted Celsius
    err_C = T_C - T_C_hat;      % error in Celsius

    % quality metrics
    RMSE_C   = sqrt(mean(err_C.^2));
    [maxAbsErr_C, idxMax] = max(abs(err_C));

    % --- package results
    SH.A = A; SH.B = B; SH.C = C;
    SH.RMSE_C = RMSE_C;
    SH.maxAbsErr_C = maxAbsErr_C;
    SH.x_atMaxErr_R = R_ohm(idxMax);
    SH.T_atMaxErr_C = T_C(idxMax);
    SH.err_C = err_C;
    SH.T_C_hat = T_C_hat;

    % --- print summary
    fprintf('Steinhart-Hart fit:\n');
    fprintf('  A = %.10e\n  B = %.10e\n  C = %.10e\n', A,B,C);
    fprintf('  RMSE  = %.4f °C\n', RMSE_C);
    fprintf('  Max |error| = %.4f °C at R = %.3f Ω (T = %.2f °C)\n', ...
            maxAbsErr_C, SH.x_atMaxErr_R, SH.T_atMaxErr_C);

    % --- (optional) quick plots
    figure; 
    subplot(1,2,1); grid on; hold on;
    plot(T_C, R_ohm, 'ko', 'MarkerSize',5);
    T_axis = linspace(min(T_C), max(T_C), 300)';
    R_model = exp(invSH_lnR_from_T(T_axis+273.15, A,B,C)); % model R vs T (optional)
    plot(T_axis, R_model, 'r-', 'LineWidth',1.5);
    set(gca,'YScale','log');
    xlabel('Temperature (°C)'); ylabel('Resistance (Ω)');
    title('R–T data (log scale) + SH curve');

    subplot(1,2,2); grid on;
    plot(T_C, err_C, 'b.-'); yline(0,'k-');
    xlabel('Temperature (°C)'); ylabel('Error (measured - model) [°C]');
    title(sprintf('Residuals; RMSE=%.3f°C, Max=%.3f°C', RMSE_C, maxAbsErr_C));
end

% Helper: for plotting R vs T, invert SH using Newton's method on lnR
function lnR = invSH_lnR_from_T(TK, A,B,C)
% Solve A + B*x + C*x^3 = 1/TK for x = lnR ; Newton iterations
    y = 1 ./ TK;
    % initial guess: Beta approximation around mid-range
    x = log(10000); % starting lnR ~ ln(10kΩ), adjust if you know range
    x = x * ones(size(TK));
    for it = 1:30
        f  = A + B*x + C*x.^3 - y;
        df = B + 3*C*x.^2;
        step = -f ./ df;
        x = x + step;
        if all(abs(step) < 1e-12), break; end
    end
    lnR = x;
end

% Example vectors (replace with your measured data)
 
R =  [68465.8 61307.94 48992.37 40875.68 34612.72 28808.11 23748.3 19761.99 16465.36 13659.66 ...
    11350.53 9768.39 8510.9 7754.8 6630.91 5698.25 5012.6];
  % ohms
T =   [17.5 20.0 25.0 30.0 35.0 40.0 45.0 50.0 55.0 60.0 ...
    65.0 70.0 75.0 80.0 85.0 90.0 95.0];
% °C

SH = fitSteinhartHart(R, T);
% Evaluate temperature for a new resistance:
R_new = 5000;                          % Ω
lnR    = log(R_new);
T_K    = 1 / (SH.A + SH.B*lnR + SH.C*lnR^3);
T_C    = T_K - 273.15