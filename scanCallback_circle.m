function scanCallback(src,msg) 
global controlpub
global count
global target
global u
global K_est
global rho_est
global rho
global K
global rho_est_vector
global rho_vector
global u_vector;
global K_vector;
global K_est_vector;

v = 0.2;
u0 = 0;
mu = 5*v;
%rho_vector = zeros(1, total_time * rate);
%kappa_vector = zeros(1, total_time * rate);
kd = 5;
rho_min = 1.2;
K_min = pi/2;
rate =  1000/ (round(toc*1000));
tic;
newMsg = rosmessage('std_msgs/Float32');
values = msg.Data;
count = count + 1;

%%%%%%%%%%%%%%
values = values*1000;
length(values);
i = 1:6:length(values)-3;
x_offset = 57;
y_offset = 430;
z_offset = 180; % Offset is 180 millimeters approx.

min_dists = values(i);
max_dists = values(i+3);
min_x = -values(i+1);
min_y = values(i+2);
max_x = -values(i+4);
max_y = values(i+5);

target_x = -(values(length(values) - 2) + x_offset)/1000;
target_z = (values(length(values)) + z_offset)/1000;

target_data = [target_x, target_z]';
target= [target_x, target_z]';
%norm(target_data)
%{
if norm(target_data) ~= Inf(1)
    target = target_data;
else 
    r_speed = 50/1000;
    dt = 1/15;
    target = [cos(u * dt) sin(u * dt); -sin(u * dt) cos(u * dt)] * target;
    target = target - [0; r_speed] * dt;

        %angle = - atan2(target(1),target(2));
        
        %f = alpha2 * ( 1 - (r0/norm(beacon_est))^2);
        %u =  - h * sin(angle) * cos(angle) + f * sin(angle);  % rotational 
end
%}
%plot(target(1), target(2), 'x')

% plot(rho_vector)
% hold on;
% plot(rho_est_vector);
% plot(K_vector);
% plot(u_vector);
% plot(K_est_vector);
polar(K_est_vector, rho_est_vector)

% Min plot
x = min_x + x_offset;
z = min_dists + z_offset;
% y is 775 mm raised
y = min_y + 430;

mask = (y > 100 & y < 550) & (z < 4000) & ( (x - target(1)*1000).^2 + (z - target(2)*1000).^2) < 300;

valid_x = x(mask)/1000;
valid_z = z(mask)/1000;
valid_y = y(mask);

%scatter3(valid_x,valid_z, valid_y);
hold on;
title('Min distances')
xlabel('X axis (m)')
ylabel('Z axis (m)')
%axis([-3 3 -3 8])
xL = xlim;
yL = ylim;
%line([0 0], yL);  %x-axis
%line(xL, [0 0]);  %y-axis
%line([0, 8000/tan(55*pi/180)], [0 8000])
%line([0, -8000/tan(55*pi/180)], [0 8000])
A = [valid_x valid_z];
epsilon = .13;
MinPts = 10;



r = target;
if target ~=Inf(1)
    rho = norm(r);
    K = (atan2(r(2),r(1)) -pi/2);
end
%rho_vector(1,i) = rho;
%kappa_vector(1,i) = K;
%u = 0;
if norm(target_data) ~= Inf(1) % Beacon is in sight.

        rho_est = rho;
        K_est = K;
        u = u0 - mu/(rho*v);

       % K = pi  -acos(dot(r,x)/norm(r));

        E = rho*(mu + v*sin(K)) - (1/2)*v*u0*rho^2;
        E_d = rho_min*(mu + v*sin(K_min)) - (1/2)*v*u0*rho_min^2;

        u_AD = kd*(E - E_d)*cos(K);

        u = u + u_AD;
        

else % Use estimates.
        fprintf('rho_est = %f, true k = %f, k_est = %f, u = %f, rate = %d, before estimate \n', rho_est , K,K_est, u*180/pi, rate )    

   % K_temp = K_est;
    % K_est = K_est + 1/rate * (-v*u + v/rho_est * sin(K_est));
   % rho_est = rho_est + 1/rate * (-v*cos(K_temp));
        
         rho_temp = rho_est;
        kappa_temp = K_est;
        u_temp = u;
        [rho_est, K_est] = ...
        getNextState(double(rho_temp), double(kappa_temp), (1/rate), v, double(u_temp));

        u = u0 - mu/(rho_est*v);
        
         E = rho_est*(mu + v*sin(K_est)) - (1/2)*v*u0*rho_est^2;
         E_d = rho_min*(mu + v*sin(K_min)) - (1/2)*v*u0*rho_min^2;

        u_AD = kd*(E - E_d)*cos(K_est);

        u = u + u_AD;
   % rho_vector(1,i) = rho_est;
   %     kappa_vector(1,i) = K_est;
end
rho_est_vector = [rho_est_vector rho_est];
rho_vector = [rho_vector rho];

K_est_vector = [K_est_vector K_est];
K_vector = [K_vector K];
u_vector = [u_vector u];
rho_est;
rho;
K_est;
K;
    fprintf('rho_est = %f, true k = %f, k_est = %f, u = %f, rate = %d, after estimate \n', rho_est , K,K_est, u*180/pi, rate )    
    
    
%{
v = 1;
alpha = 1;
r0 = .2;
n = 1;
f = @(r) alpha*(1 - (r0/norm(r)).^2);
rp = [0 0]';
% Specify origin angle
angle = pi/2;

x = [cos(angle) sin(angle)]';
rotation = [0 -1; 1 0];

y =  rotation * x;

r = -target + rp;
u = 0;

goal = target;

[IDX is_noise, C] = DBSCAN(A, epsilon, MinPts);

%PlotClusterinResult(A,IDX);
%scatter3(valid_x,valid_z, valid_y);

% Begin at initial state

%total_time = 100;
%rate = 30; % 30 hz control updates

% C total clusters. Find distance / heading to each and compute control
% terms


    weight_vec = [];
    control_vec = [];
   % pack g matrix

        % Compute goal control
    %u = -n * dot(r/norm(r), x)*dot(r/norm(r), y) ...
    %    + -f(norm(r))*dot(r/norm(r), y);
    u = -n*dot(r/norm(r), y);
    
    control_vec = [u];
        % Compute obstacle control.
        % For weight vector, start with the distance to the goal.
    weight_vec = [1];
        
        %angles = atan2((valid_z - rp(2)),(valid_x - rp(1))) + pi/2 - atan2(x(2), x(1));
       % mask = angles > pi/180 * 55 & angles < pi/180*125;
        %tempx = valid_x(mask);
       % tempz = valid_z(mask);
  %  A = [valid_x valid_z];
   % [IDX is_noise, C] = DBSCAN(A, epsilon, MinPts);
    for k = 1:C
        mask = IDX == k;
        temp = A(mask, :);
        if (mod(count,30) == 1)
            scatter(temp(:,1), temp(:,2));
        end
        min_dist = Inf;
        vec = [0;0];

        for j = 1: length(temp)           
            if (( (temp(j,1) - rp(1) )^2 + (temp(j,2) - rp(2))^2) < min_dist^2) 
                min_dist = sqrt((temp(j,1) - rp(1))^2 + ((temp(j,2) - rp(2))^2));
                vec = [temp(j,1); temp(j,2)];
            end

        end
        r_c = rp - vec;
        weight_vec = [weight_vec (1/(r0-norm(r_c)))^2];
        control_vec = [control_vec  2*n*dot(r_c/norm(r_c), y)];
    end
    
        % Normalize weights
        weight_vec = weight_vec/sum(weight_vec);
        u = dot(weight_vec, control_vec)
        %%%%%%%%%%%%
    %}
    newMsg.Data = u*180/pi; % convert to degrees
    u*180/pi;
    u;
    hold on;
       %{ 
            if (norm(r) < 1)
            newMsg.Data = -1000;
            end
        %}
        send(controlpub,newMsg);
        if (mod(count,30) == 0)
            
            clf(1);
            
        end
    end


