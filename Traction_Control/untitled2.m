[t_out1, s_out1, v_out1, sw1, MVC1] = topp(r, dr, ddr, s_grid, 0.1, 0.05, mean(diff(s_grid)));

    fprintf('Total time : %.4f s\n', t_out1(end));
    fprintf('Switches at s = '); fprintf('%.4f  ', sw1); fprintf('\n');

    figure('Color','w','Position',[80 80 1200 420]);

    subplot(1,3,2); hold on;
    plot(s_out1, v_out1, 'b-', 'LineWidth',2);
    for s_sw = sw
        xline(s_sw, 'r--', 'LineWidth',1.2);
    end
    xlabel('s'); ylabel('v = ds/dt');
    title('Velocity profile  (red = switches, black = MVC)'); grid on;
    hold on
    plot(s_out1, MVC1, 'k', LineWidth=1.5);

    subplot(1,3,3);
    plot(t_out1, s_out1, 'r-', 'LineWidth',2);
    xlabel('t (s)'); ylabel('s');
    title('Time law s(t)'); grid on;

function [t_out, s_out, v_out, switches, MVC, F] = topp(r, dr, ddr, s_grid, v_max, a_max, ds)
% TOPP  Time-Optimal Path Parameterization
%       Lynch & Park "Modern Robotics" (2017), Section 9.4.
%
% Inputs:
%   r      : [N x ndim]  path positions r(s)
%   dr     : [N x ndim]  first  derivative r'(s)
%   ddr    : [N x ndim]  second derivative r''(s)
%   s_grid : [N x 1]     uniform arc-length grid
%   v_max  : scalar      path speed limit (ds/dt <= v_max)
%   a_max  : scalar      Cartesian acceleration limit
%   ds     : scalar      grid spacing
%
% Outputs:
%   t_out, s_out, v_out, switches, MVC, F

    N         = length(s_grid);
    MAX_BSRCH = 80;
    MAX_OUTER = 4*N;

    %% ── 1. Precompute A, B, C ────────────────────────────────────────────
    Ac = zeros(N,1);  Bc = zeros(N,1);  Cc = zeros(N,1);
    for k = 1:N
        T     = dr(k,:);   R = ddr(k,:);
        Ac(k) = dot(T,T);
        Bc(k) = dot(T,R);
        Cc(k) = dot(R,R);
    end

    %% ── 2. MVC ───────────────────────────────────────────────────────────
    % Velocity constraint: |r'| * v <= v_max  =>  v <= v_max / |r'|
    v_vel = v_max ./ sqrt(max(Ac, 1e-12));

    % Acceleration constraint: discriminant of quadratic = 0
    dz    = Ac.*Cc - Bc.^2;
    v_acc = inf(N,1);
    ok    = dz > 1e-12;
    v_acc(ok) = ( Ac(ok) .* a_max^2 ./ dz(ok) ).^0.25;

    MVC = min(v_vel, v_acc);

    % ── Adaptive tolerances scaled to problem ─────────────────────────────
    mvc_med    = max(median(MVC(isfinite(MVC))), 1e-12);
    TOL_touch  = mvc_med * 5e-3;   % MVC-touch window in Step 4
    TOL_cross  = mvc_med * 5e-3;   % arc-crossing window  in Step 5
    TOL_binary = mvc_med * 1e-5;   % binary search stop criterion

    %% ── 3. Build backward braking curve F ───────────────────────────────
    % Integrate s_ddot = L backward from (s=S, v=0)
    % v_{k}^2 = v_{k+1}^2 - 2*L(k+1, v_{k+1})*ds   [backward Euler in v^2]
    F    = zeros(N,1);
    F(N) = 0;
    for k = N-1:-1:1
        v = max(F(k+1), 1e-12);
        [L, U] = compute_ab(k+1, v, Ac, Bc, Cc, a_max);
        v2 = v^2 - 2*L*ds;     % L is negative, so this ADDS to v^2
        if v2 < 0, v2 = 0; end
        F(k) = min(sqrt(v2), MVC(k));
        if U < L                % MVC penetrated: saturate remaining F
            F(1:k) = MVC(1:k);
            break;
        end
    end

    %% ── 4. Initialize (Step 1) ───────────────────────────────────────────
    switches     = [];
    k_cur        = 1;
    v_cur        = 0;
    v_profile    = nan(N,1);
    v_profile(1) = 0;
    outer_iter   = 0;

    %% ════════════════════════════════════════════════════════════════════
    %% MAIN LOOP  Steps 3 → 6
    %% ════════════════════════════════════════════════════════════════════
    while k_cur < N
        outer_iter = outer_iter + 1;
        if outer_iter > MAX_OUTER
            warning('TOPP:maxIter','Exceeded max outer iterations.');
            break;
        end

        k_start = k_cur;
        v_start = v_cur;

        %% ── Step 3: integrate U forward ──────────────────────────────────
        v       = v_start;
        hit_F   = false;
        hit_MVC = false;
        k_lim   = k_start;
        v_lim   = v_start;

        for k = k_start : N-1
            [~, U] = compute_ab(k, v, Ac, Bc, Cc, a_max);
            v2     = v^2 + 2*U*ds;
            if v2 < 0, v2 = 0; end
            v_next = sqrt(v2);

            %-- (b) MVC penetrated => go to Step 4
            if v_next > MVC(k+1) + TOL_touch
                hit_MVC = true;
                k_lim   = k+1;
                v_lim   = MVC(k+1);
                v_profile = fill_arc(v_profile, k_start, k, v_start, ...
                                     true, Ac, Bc, Cc, a_max, MVC, ds);
                break;
            end

            v = v_next;

            %-- (a) Crossed F => U->L switch, problem solved (for this segment)
            if v >= F(k+1) + TOL_cross
                switches(end+1) = s_grid(k+1); %#ok<AGROW>
                v_profile = fill_arc(v_profile, k_start, k+1, v_start, ...
                                     true, Ac, Bc, Cc, a_max, MVC, ds);
                for j = k+1:N
                    v_profile(j) = F(j);
                end
                hit_F = true;
                break;
            end
        end

        if hit_F,   break; end

        if ~hit_MVC
            v_profile = fill_arc(v_profile, k_start, N, v_start, ...
                                 true, Ac, Bc, Cc, a_max, MVC, ds);
            break;
        end

        %% ── Step 4: binary search for tangency at (k_lim, v') ───────────
        % Find v' in [0, v_lim] such that the L-arc from (k_lim, v')
        % touches MVC tangentially without penetrating it.
        %
        % Binary search decisions:
        %   L-arc goes ABOVE MVC  => v' too HIGH  => v_high = v_test
        %   L-arc hits v = 0      => v' too LOW   => v_low  = v_test
        %   L-arc touches MVC     => record best, try higher (v_low=v_test)

        v_high = v_lim;
        v_low  = 0;
        k_tan  = k_lim;       % best tangency point found so far
        v_tan  = 0;

        for iter = 1:MAX_BSRCH
            v_test = (v_high + v_low) / 2;

            [k_touch, v_touch, outcome] = integrate_L_fwd( ...
                k_lim, v_test, MVC, Ac, Bc, Cc, a_max, ds, N, TOL_touch);

            if outcome == 1          % penetrated => too high
                v_high = v_test;
            elseif outcome == -1     % hit zero / no contact => too low
                v_low  = v_test;
            else                     % outcome == 0: touched => good
                k_tan = k_touch;
                v_tan = v_touch;
                v_low = v_test;      % refine upward
            end

            if (v_high - v_low) < TOL_binary
                break;
            end
        end

        % If binary search never found a touch, k_tan stays at k_lim
        % with v_tan = 0.  This means the MVC is a hard wall right here:
        % we must run along the MVC for one step and try again.
        if v_tan < TOL_touch && k_tan == k_lim
            % Slide along MVC for one grid step
            v_profile(k_lim) = MVC(k_lim);
            if k_lim < N
                k_tan = k_lim + 1;
                v_tan = MVC(k_tan);
                v_profile(k_tan) = v_tan;
            else
                break;
            end
        end

        %% ── Step 5: integrate L backward from (k_tan,v_tan) to arc Ai ──
        [k_sw, v_sw] = integrate_L_bwd_to_arc( ...
            k_tan, v_tan, v_profile, k_start, ...
            Ac, Bc, Cc, a_max, ds, TOL_cross);

        % U->L switch at k_sw
        switches(end+1) = s_grid(k_sw); %#ok<AGROW>

        v_profile = fill_arc(v_profile, k_start, k_sw, v_start, ...
                             true, Ac, Bc, Cc, a_max, MVC, ds);
        v_profile(k_sw) = v_sw;

        v_profile = fill_arc(v_profile, k_sw, k_tan, v_sw, ...
                             false, Ac, Bc, Cc, a_max, MVC, ds);
        v_profile(k_tan) = v_tan;

        %% ── Step 6: L->U switch at tangency, resume ─────────────────────
        switches(end+1) = s_grid(k_tan); %#ok<AGROW>

        k_prev = k_cur;
        k_cur  = k_tan;
        v_cur  = v_tan;
        v_profile(k_cur) = v_cur;

        % Safety: if we made no progress, advance by one step
        if k_cur <= k_prev
            k_cur = k_prev + 1;
            if k_cur > N, break; end
            v_cur = min(MVC(k_cur), v_cur);
            v_profile(k_cur) = v_cur;
        end

    end % while

    %% ── Finalize ──────────────────────────────────────────────────────────
    v_profile(N) = 0;
    v_profile    = max(min(v_profile, MVC), 0);
    v_profile(isnan(v_profile)) = 0;

    %% ── Time recovery ─────────────────────────────────────────────────────
    t_out = zeros(N,1);
    for k = 1:N-1
        v_avg = 0.5*(v_profile(k) + v_profile(k+1));
        if v_avg < 1e-12
            % Use constant-acceleration kinematics
            dv = abs(v_profile(k+1) - v_profile(k));
            if dv > 1e-12
                t_out(k+1) = t_out(k) + 2*ds / (v_profile(k) + v_profile(k+1) + 1e-12);
            else
                t_out(k+1) = t_out(k) + sqrt(2*ds / max(a_max, 1e-12));
            end
        else
            t_out(k+1) = t_out(k) + ds / v_avg;
        end
    end

    s_out = s_grid(:);
    v_out = v_profile(:);
end


%% ════════════════════════════════════════════════════════════════════════════
%%  compute_ab
%%  L = alpha = lower acceleration bound (max deceleration, usually negative)
%%  U = beta  = upper acceleration bound (max acceleration, usually positive)
%% ════════════════════════════════════════════════════════════════════════════
function [L, U] = compute_ab(k, v, Ac, Bc, Cc, a_max)
    Ak = Ac(k);  Bk = Bc(k);  Ck = Cc(k);
    disc = Ak*a_max^2 + v^4*(Bk^2 - Ck*Ak);
    if disc < 0, disc = 0; end
    sq = sqrt(disc);
    L  = (-Bk*v^2 - sq) / Ak;   % max deceleration (negative)
    U  = (-Bk*v^2 + sq) / Ak;   % max acceleration  (positive)
end


%% ════════════════════════════════════════════════════════════════════════════
%%  fill_arc
%%  Fills v_profile[k_start..k_end] by integrating U (use_U=true) or L
%% ════════════════════════════════════════════════════════════════════════════
function vp = fill_arc(vp, k_start, k_end, v0, use_U, Ac, Bc, Cc, a_max, MVC, ds)
    v = v0;
    vp(k_start) = v;
    for j = k_start : k_end-1
        [L, U] = compute_ab(j, v, Ac, Bc, Cc, a_max);
        a  = U*use_U + L*(~use_U);
        v2 = v^2 + 2*a*ds;
        if v2 < 0, v2 = 0; end
        v  = min(sqrt(v2), MVC(j+1));
        vp(j+1) = v;
    end
end


%% ════════════════════════════════════════════════════════════════════════════
%%  integrate_L_fwd  — Step 4 helper
%%
%%  Integrates L forward from (k0, v0). Returns:
%%    k_touch, v_touch : best (closest) approach point to MVC
%%    outcome :  1 = penetrated MVC (v_test too high)
%%              -1 = hit zero / reached end without ever touching MVC
%%               0 = touched MVC within TOL_touch (good tangency candidate)
%% ════════════════════════════════════════════════════════════════════════════
function [k_touch, v_touch, outcome] = ...
         integrate_L_fwd(k0, v0, MVC, Ac, Bc, Cc, a_max, ds, N, TOL_touch)

    outcome = -1;
    k_touch = k0;
    v_touch = v0;
    best_gap = inf;

    v = v0;
    for k = k0 : N-1
        [L, ~] = compute_ab(k, v, Ac, Bc, Cc, a_max);
        v2     = v^2 + 2*L*ds;

        if v2 <= 0
            return;          % hit zero => outcome = -1
        end
        v_next = sqrt(v2);

        %-- Penetrated MVC from above
        if v_next > MVC(k+1) + TOL_touch
            outcome = 1;
            return;
        end

        %-- Track best (closest) approach to MVC from below
        gap = MVC(k+1) - v_next;
        if gap >= 0 && gap < best_gap
            best_gap = gap;
            k_touch  = k+1;
            v_touch  = v_next;
        end

        %-- Within touch window: declare tangency found
        if gap >= 0 && gap < TOL_touch
            outcome = 0;
            return;
        end

        v = v_next;
    end
    % Reached end of path: if we got close at some point, accept it
    if best_gap < inf && best_gap < 5*TOL_touch
        outcome = 0;
    end
    % else outcome stays -1
end


%% ════════════════════════════════════════════════════════════════════════════
%%  integrate_L_bwd_to_arc  — Step 5 helper
%%
%%  Integrates L backward from (k_tan, v_tan).
%%  Stops when v_back crosses the already-filled v_profile (the U-arc Ai).
%%  v_{k-1}^2 = v_k^2 - 2*L(k, v_k)*ds   [backward step]
%% ════════════════════════════════════════════════════════════════════════════
function [k_sw, v_sw] = integrate_L_bwd_to_arc( ...
    k_tan, v_tan, v_profile, k_arc_start, Ac, Bc, Cc, a_max, ds, TOL_cross)

    v    = v_tan;
    k_sw = k_arc_start;           % fallback
    v_sw = max(v_profile(k_arc_start), 0);

    for k = k_tan : -1 : k_arc_start+1
        [L, ~] = compute_ab(k, v, Ac, Bc, Cc, a_max);
        v2     = v^2 - 2*L*ds;    % backward: L is negative, so v^2 grows
        if v2 < 0, v2 = 0; end
        v_back = sqrt(v2);

        % Check crossing with forward U-arc stored in v_profile
        vp_prev = v_profile(k-1);
        if ~isnan(vp_prev)
            % Crossing condition: backward arc dropped to or below forward arc
            if v_back <= vp_prev + TOL_cross
                k_sw = k-1;
                v_sw = vp_prev;
                return;
            end
        end
        v = v_back;
    end
end


%% ════════════════════════════════════════════════════════════════════════════
%%  DIAGNOSTIC PLOT — call this to visualise MVC vs F before running TOPP
%%  Usage: topp_diagnose(dr, ddr, s_grid, v_max, a_max)
%% ════════════════════════════════════════════════════════════════════════════
topp_diagnose(dr, ddr, s_grid, 0.1, 0.05)
function topp_diagnose(dr, ddr, s_grid, v_max, a_max)
    % Quick check: is the problem feasible?  Plot MVC and F together.
    ds = mean(diff(s_grid));
    N  = length(s_grid);

    Ac = zeros(N,1); Bc = zeros(N,1); Cc = zeros(N,1);
    for k = 1:N
        T=dr(k,:); R=ddr(k,:);
        Ac(k)=dot(T,T); Bc(k)=dot(T,R); Cc(k)=dot(R,R);
    end
    v_vel = v_max ./ sqrt(max(Ac,1e-12));
    dz    = Ac.*Cc - Bc.^2;
    v_acc = inf(N,1);
    ok    = dz>1e-12;
    v_acc(ok) = (Ac(ok).*a_max^2./dz(ok)).^0.25;
    MVC = min(v_vel, v_acc);

    F = zeros(N,1);
    for k = N-1:-1:1
        v = max(F(k+1),1e-12);
        Ak=Ac(k+1); Bk=Bc(k+1); Ck=Cc(k+1);
        disc = Ak*a_max^2 + v^4*(Bk^2-Ck*Ak);
        if disc<0, disc=0; end
        L = (-Bk*v^2 - sqrt(disc))/Ak;
        v2 = v^2 - 2*L*ds;
        if v2<0, v2=0; end
        F(k) = min(sqrt(v2), MVC(k));
    end

    figure('Color','w','Name','TOPP Diagnostic');
    plot(s_grid, MVC, 'r-',  'LineWidth',2,'DisplayName','MVC'); hold on;
    plot(s_grid, F,   'b--', 'LineWidth',2,'DisplayName','F (braking curve)');
    xlabel('s (arc-length)'); ylabel('v = ds/dt');
    title('MVC vs Braking curve F — feasibility check');
    legend('Location','best'); grid on;

    feasible = all(F <= MVC + 1e-6);
    if feasible
        title('MVC vs F  [FEASIBLE — F lies below MVC everywhere]');
    else
        title('MVC vs F  [WARNING: F exceeds MVC — constraints may be too tight]');
    end

    fprintf('Total time : %.4f s\n', t_out(end));
    fprintf('Switches at s = '); fprintf('%.4f  ', sw); fprintf('\n');

    fprintf('\nDiagnostic summary:\n');
    fprintf('  v_max  = %.4f,  a_max = %.4f\n', v_max, a_max);
    fprintf('  min(MVC) = %.4f\n', min(MVC));
    fprintf('  max(F)   = %.4f\n', max(F));
    fprintf('  Minimum braking distance from v_max: %.4f m\n', ...
            v_max^2 / (2*a_max));
    fprintf('  Path length S = %.4f m\n', s_grid(end));
    if v_max^2/(2*a_max) > s_grid(end)/2
        fprintf('  WARNING: braking distance (%.4f) > half path length (%.4f)\n',...
                v_max^2/(2*a_max), s_grid(end)/2);
        fprintf('  => rover cannot reach v_max and brake in time.\n');
        fprintf('     Try increasing a_max or reducing v_max.\n');
    end
end