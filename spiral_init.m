%% Spiral trajectory generator

a = 9.81/2;
t_ref = [];
vel_refx = [];
vel_refy = [];
vel_refz = [];
pos_refx = [];
pos_refy = [];
pos_refz = [];
vel_oldx = [];
vel_oldy = [];
vel_oldz = [];
psi_ref  = [];
psi_oldx  = [];
psi_old  = 0;
t_end = 0;
noPoints = 100;

delta_z = 3;
ToA = [0, 20];
ii = 2;

t_ref(end+1:end+noPoints) = linspace(t_end,ToA(ii),noPoints);
dt = t_ref(2+noPoints*(ii-2)) - t_ref(1+noPoints*(ii-2));

%% Computing trajectory on z

% delta_z = wpt(ii,3) - wpt(ii-1,3);
if delta_z ~= 0
    
    v_max = [ (a*(ToA(ii)-ToA(ii-1)) + sqrt((a*(ToA(ii)-ToA(ii-1)))^2 - 4*abs(delta_z)*a));...
        (a*(ToA(ii)-ToA(ii-1)) - sqrt((a*(ToA(ii)-ToA(ii-1)))^2 - 4*abs(delta_z)*a)) ]/2;
    v_max = min(v_max)*sign(delta_z);
    delta_t = (ToA(ii)-ToA(ii-1)) - 2*abs(v_max)/a;
    t_star = abs(v_max)/a;
    t_hat = t_star + delta_t;

    % Creating trapezoidal speed profile
    vel_ref_temp(1:noPoints,1) = v_max;
    up = find(t_ref(end-noPoints+1:end) - t_end < t_star);
    down = find(t_ref(end-noPoints+1:end) - t_end > t_hat);
    vel_ref_temp(up) = sign(delta_z)*a*(t_ref(noPoints*(ii-2)+up) - t_end);
    vel_ref_temp(down) = v_max - sign(delta_z)*a*(t_ref(end-noPoints+down) - (t_hat + t_end));
    
    if isempty(pos_refz)
        pos_refz(1) = 0;
    else
        pos_refz(end+1) = pos_refz(end);
    end
    for jj = 2:noPoints
        pos_refz(end+1) = (pos_refz(end)) + ...
            (vel_ref_temp(jj) + vel_ref_temp(jj-1))/2*dt;
    end

else
    vel_ref_temp = zeros(noPoints,1);
    pos_refz(end+1:end+noPoints) = wpt(ii-1,3)*ones(noPoints,1);
end
vel_refz = [ vel_oldz; vel_ref_temp ];
% vel_oldz = vel_refz;
% clear vel_ref_temp

% t_end = t_ref(end);

%% Computing trajectory on x and y

% I give a trapezoidal profile
% v_max = [ (a*(ToA(ii)-ToA(ii-1)) + sqrt((a*(ToA(ii)-ToA(ii-1)))^2 - 4*abs(tot_dist)*a));...
%         (a*(ToA(ii)-ToA(ii-1)) - sqrt((a*(ToA(ii)-ToA(ii-1)))^2 - 4*abs(tot_dist)*a)) ]/2;
% v_max = min(v_max)*sign(tot_dist);
a_xy = 2;
v_max = 0.5;
delta_t = (ToA(ii)-ToA(ii-1)) - 2*abs(v_max)/a_xy;
t_star = abs(v_max)/a_xy;
t_hat = t_star + delta_t;

% Creating trapezoidal speed profile
vel_ref_temp(1:noPoints,1) = v_max;
up = find(t_ref(end-noPoints+1:end) - t_end < t_star);
down = find(t_ref(end-noPoints+1:end) - t_end > t_hat);
vel_ref_temp(up) = sign(1)*a_xy*(t_ref(noPoints*(ii-2)+up) - t_end);
vel_ref_temp(down) = v_max - sign(1)*a_xy*(t_ref(end-noPoints+down) - (t_hat + t_end));
vel_profile_x = vel_ref_temp ./ v_max;
    
vel_refx = cos(t_ref)' .* vel_profile_x;
vel_refy = sin(t_ref)' .* vel_profile_x;

pos_ref = [ zeros(noPoints, 2), pos_refz' ];
vel_ref = [ vel_refx, vel_refy, vel_refz ];
% psi = pi / 2 * ones(noPoints, 1);

%% Computing trajectory for psi

psi = t_ref';
vel_ref = [ vel_profile_x * 0.1, vel_profile_x * 0, vel_refz ];
