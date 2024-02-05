clear;close all;
%% Set up Robotarium object
% Before starting the algorithm, we need to initialize the Robotarium
% object so that we can communicate with the agents
N = 6;
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true);

%This is how many times the main loop will execute.
iterations = 3000;

%% Experiment constants
% Next, we set up some experiment constants

% Initialize velocity vector for agents.  Each agent expects a 2 x 1
% velocity vector containing the linear and angular velocity, respectively.
dx = zeros(2, N);

% set starting positions of passing through the virtual tube
UAV_PInit(1,:) = [-1.45,0.2];UAV_PInit(2,:)=[-1.25,0.2];
UAV_PInit(3,:) = [-1.35,0];UAV_PInit(4,:)=[-1.15,0];
UAV_PInit(5,:) = [-1.25,-0.2];UAV_PInit(6,:)=[-1.05,-0.2];
x_goal = UAV_PInit';

% flag of task completion
flag = 0; 

%% Retrieve tools for single-integrator -> unicycle mapping
% Let's retrieve some of the tools we'll need.  We would like a
% single-integrator position controller, a single-integrator barrier
% function, and a mapping from single-integrator to unicycle dynamics

position_control = create_si_position_controller();
si_barrier_certificate = create_si_barrier_certificate_with_boundary();
si_to_uni_dyn = create_si_to_uni_dynamics_with_backwards_motion();

%% Begin the experiment
% speed and density planning results
load planning_curly.mat

% virtual tube model
p_left=[];
p_right=[];
for i=1:length(curve_x)
    p_left(i,:)=(p_leader(i,:))+dis(i)*[cos(curve_theta(i)+pi/2),sin(curve_theta(i)+pi/2)];
    p_right(i,:)=(p_leader(i,:))+dis(i)*[cos(curve_theta(i)-pi/2),sin(curve_theta(i)-pi/2)];
end
plot(p_leader(:,1),p_leader(:,2),'k--','Linewidth',2);
plot(p_left(:,1),p_left(:,2),'r','Linewidth',2);
plot(p_right(:,1),p_right(:,2),'r','Linewidth',2);

sum_leader_locate=0;
ave_leader_locate=0;
curve_x_left = curve_x(1,1);
curve_x_right = curve_x(length(curve_x),1);
curve_x_delta = curve_x(2,1) - curve_x(1,1);

for t = 1:iterations
    % Retrieve the most recent poses from the Robotarium.  The time delay is
    % approximately 0.033 seconds
    x = r.get_poses();
    x_position = x(1:2,:);

    %% Algorithm
    % Let's make sure we're close enough the the goals
    if norm(x_goal-x_position,1) < 0.03
        flag = 1-flag;
    end

    if flag == 0
        % Use a single-integrator position controller to drive the agents to
        % the start positions
        dx = position_control(x(1:2, :), x_goal);

        % Ensure the robots don't collide before reaching the start
        % positions of passing through the virtual tube
        dx = si_barrier_certificate(dx, x);

        % Transform the single-integrator dynamics to unicycle dynamics using a
        % diffeomorphism, which can be found in the utilities
        dx = si_to_uni_dyn(dx, x);


        %% Set the velocities of the agents
        % Set velocities of agents 1,...,N
        r.set_velocities(1:N, dx);
        
    elseif flag == 1
        % start passing through the virtual tube

        % parameters setting
        rs = 0.075;
        ra = 0.125;
        vmax = 0.1;
        vmin = 0.03;
        x_narrow = 0.2;
        tunnel_left = p_left;
        tunnel_right = p_right;
        Pcur = x_position';
        
       %% something to calculate before control
       %find the nearest point in p_leader with the i-th agent
       leader_locate = zeros(N,1);
       flight_curve = zeros(N,1);
       for i = 1:N
           dis_leader_min = 10^8;
           for j = 1:length(p_leader(:,1))
               dis_leader = norm(Pcur(i,:)-p_leader(j,:));
               if dis_leader < dis_leader_min
                   leader_locate(i) = j;
                   dis_leader_min = dis_leader;
               end
           end
       end

       % the matched point in p_leader with the initial swarm center
       Init_locate = 0;
       dis_leader_min = 10^8;
       for j = 1:length(p_leader(:,1))
           dis_leader = norm(UAV_PAveInit-p_leader(j,:));
           if dis_leader < dis_leader_min
               Init_locate = j;
               dis_leader_min = dis_leader;
           end
       end
       
       % find the matched point in p_leader with the swarm center 
       for i = 1:N
           sum_leader_locate = sum_leader_locate+leader_locate(i);
       end
       ave_leader_locate =1 /N*sum_leader_locate;
       

       % the length between the initial point of p_leader and the i-th
       % point of p_leader,called lq
       lq = ones(length(curve_x)-1,1);
       lq(1,1) = sqrt((curve_x(2)-curve_x(1))^2+(curve_y(2)-curve_y(1))^2);
       for i = 2:(length(curve_x)-1)
           lq(i,1) = lq(i-1,1)+sqrt((curve_x(i+1)-curve_x(i))^2+(curve_y(i+1)-curve_y(i))^2);
       end
       
       if floor(ave_leader_locate) > (length(curve_x)-1)
           ave_leader_locate = (length(curve_x)-1);
       else
           ave_leader_locate = floor(ave_leader_locate);
       end
        
       %% control command
       nc = zeros(length(tunnel_left(:,1)),2);
       for i = 1:length(tunnel_left(:,1))
           rot = [0 -1;1 0];
           C = rot*(tunnel_right(i,:)-tunnel_left(i,:))';
           nc(i,:) = -(C/norm(C))';
       end
              
       e1 = zeros(N,2);
       V1 = zeros(N,2);
       V2 = zeros(N,2);
       V3 = zeros(N,2);
       V = zeros(N,2);       
       rtksii = zeros(N,1);
       ksiti = zeros(N,2);   
       ra_V2 = zeros(N,1);     
       
       rs1 = rs;
       k1 = 1;
       k2 = 10*vmax;
       k3 = 10*vmax;      
       k_density = 10;

       leader_locate_max = min(max(leader_locate),length(curve_x));
       leader_locate_min = min(leader_locate);
       S = 0;
       for i = leader_locate_min:leader_locate_max
           S = S + curve_x_delta*norm(tunnel_left(i,:)-tunnel_right(i,:));
       end
       k = 0;
       for i = 1:N
           if Pcur(i,1) <= curve_x_right   
               k = k+1;
           end
       end
       density_now = k/S;

       count = 0;
       for i=1:N
            g = floor((Pcur(i,1)+1.5)/0.002*(1/kk))+1;
            gp = g + 10;
            if g > pp
                g = pp;
            end
            if gp > pp 
                gp = pp;
            end

            d_density = density_c(gp);
            d_v = v_c(g);
            ra_density = ra + k_density*(density_now-density_c(gp));
            ra_V2(i,1) = ra;

            if ra_density > ra && Pcur(i,1) < x_narrow
                ra_V2(i,1) = ra_density;
            else
                ra_V2(i,1) = ra;
            end
            
            if ra_V2(i,1) > 2*ra
                ra_V2(i,1) = 2*ra;
            end

           if leader_locate(i) ~= 0
              %% Velocity command component of moving along the virtual tube
               e1(i,:) = nc(leader_locate(i),:);               
               V1(i,:) = -mysat(k1*v_c(g)*e1(i,:),vmax);
              
               %% Velocity command component of keeping within the virtual tube
               dis_curve_min = 10^8;
               %robot in the left part of the tube  
               if norm(Pcur(i,:)-tunnel_left(leader_locate(i),:)) < norm(Pcur(i,:)-tunnel_right(leader_locate(i),:))
                   for j =1 :length(p_leader(:,1))
                       dis_curve = norm(Pcur(i,:)-tunnel_left(j,:));
                       if dis_curve < dis_curve_min
                           %find the nearest point in tunnel_left with the i-th agent
                           flight_curve(i) = j;
                           dis_curve_min = dis_curve;
                       end
                   end
               %robot in the right part of the tube    
               else
                   for j = 1:length(p_leader(:,1))
                       dis_curve = norm(Pcur(i,:)-tunnel_right(j,:));
                       if dis_curve < dis_curve_min
                           %find the nearest point in tunnel_right with the i-th agent
                           flight_curve(i) = -j;
                           dis_curve_min = dis_curve;
                       end
                   end
               end
               
               et = 0;
               mksii = 0.5*(tunnel_left(leader_locate(i),:)+tunnel_right(leader_locate(i),:));
               ksiti(i,:) = Pcur(i,1:2) - mksii;
               rtksii(i) = 0.5*norm(tunnel_left(leader_locate(i),:)-tunnel_right(leader_locate(i),:));              
               %ra_V2 change with density
               ci = k3*dmysigma2(rtksii(i)-norm(ksiti(i,:)),rs1,ra_V2(i,1),et);
               %agent with tunnel_right
               if flight_curve(i) >= 0   
                   if norm(Pcur(i,:)-p_leader(leader_locate(i),:)) <= norm(tunnel_left(flight_curve(i),:)-p_leader(leader_locate(i),:))
                       un = tunnel_left(flight_curve(i),:) - Pcur(i,:);
                   else 
                       un = Pcur(i,:) - tunnel_left(flight_curve(i),:);
                   end
               %agent with tunnel_right
               else  
                   if norm(Pcur(i,:)-p_leader(leader_locate(i),:)) <= norm(tunnel_right(-flight_curve(i),:)-p_leader(leader_locate(i),:))
                       un = tunnel_right(-flight_curve(i),:) - Pcur(i,:);
                   else
                       un = Pcur(i,:) - tunnel_right(-flight_curve(i),:);
                   end
               end
               V3(i,:) = -(un/norm(un))*ci;
               if isnan(V3(i,1)) || isnan(V3(i,2))
                   V3(i,:)=[0,0];
               end
           end
           
          %% Velocity command component of collision avoidance
           for j = 1:N
               if i ~= j
                   em = 0;                   
                   ksimil = (Pcur(i,1:2) - Pcur(j,1:2));
                   bil = k2*dmysigma2(norm(ksimil),2*rs,2*ra_V2(i,1),em);                   
                   V2(i,:) = V2(i,:)+ bil*(ksimil/norm(ksimil));
                   if isnan(V2(i,1)) || isnan(V2(i,2))
                       V2(i,:) = [0,0];
                   end
                   disij = norm(Pcur(i,1:2) - Pcur(j,1:2));
                   if disij < dis
                       dis = disij;
                   end
               end
           end           
           
           %% Final velocity command
           V(i,:) = mysat(V1(i,:) + mysat(V2(i,:)+V3(i,:),norm(V1(i,:))-vmin),vmax);
           
           %% go ouside after passing the tube
           if Pcur(i,1) >= curve_x_right && Pcur(i,2) <= 1.5
               V(i,:) = [0,vmax];
               for m = 1:N
                   if i~=m && Pcur(m,2) > 0.05
                       disim = norm(Pcur(i,1:2) - Pcur(m,1:2));
                       if disim <= 2*rs+0.07
                           V(i,:)=[0,0];
                       end
                   end
               end
           end
           
           %limit agents within the map
           if Pcur(i,2) >= 0.8
               V(i,:)=[0,0];
           end
           
           if isnan(V(i,1)) || isnan(V(i,2))
               V(i,:) = [0,0];
           end

           %% check if the swarm has pass through the tube
            if Pcur(i,1) > curve_x_right - 0.2
                count = count + 1;
            end           

       end

       if count == N
           break;
       end

       % Transform the single-integrator dynamics to unicycle dynamics using a
       % diffeomorphism, which can be found in the utilities
       dx = si_to_uni_dyn(V', x);        

       % Set velocities of agents 1,...,N
       r.set_velocities(1:N, dx);
        
    end
    % Send the previously set velocities to the agents.  This function must be called!
    r.step();
end

% Print out any simulation problems that will produce implementation
%differences and potential submission rejection.
r.debug()
save('ExperimentData.mat')