clear all,close all,clc
%% link length
L1 = 35;
L2 = 40;

%%base coordinates
xbase = 20;
ybase = 0;

%%Obstacle
tht = 0:pi/1000:2*pi;
xobs = 40;
yobs = 60;
robs = 10;
xball = robs * cos(tht)+xobs;
yball = robs * sin(tht)+yobs ;
robs = 11;

%%%initial positions
xinitial = 20;
yinitial = 70;

%%%Goal positions
xgoal = 60;
ygoal = 60;

%Inverse kinematics to find initial angles
q2_initial = acos(((xinitial-xbase)^2+(yinitial-ybase)^2-L1^2-L2^2)/(2*L1*L2));
A = L1+cos(q2_initial)*L2; B = sin(q2_initial)*L2;
q1_initial = atan2(A*(yinitial-ybase)-B*(xinitial-xbase),A*(xinitial-xbase)+...
    B*(yinitial-ybase));
q4_initial = -acos(((xinitial-xbase)^2+(yinitial-ybase)^2-L1^2-L2^2)/(2*L1*L2));
AA = L1+cos(q4_initial)*L2; BB = sin(q4_initial)*L2;
q3_initial = atan2(AA*(yinitial-ybase)-BB*(xinitial-xbase),AA*(xinitial-xbase)+...
    BB*(yinitial-ybase));

%Inýtial end effector position
xEE_initial = xbase + L1*cos(q1_initial)+ L2*cos(q1_initial+q2_initial);
yEE_initial = ybase + L1*sin(q1_initial)+ L2*sin(q1_initial+q2_initial);

%check whether initial point is reachable
distanceToBase_initial = sqrt((xEE_initial-xbase)^2+(yEE_initial-ybase)^2);
if distanceToBase_initial< abs(L1-L2) || distanceToBase_initial> (L1+L2)
    fprintf('You cannot place the end effector(initial position) in the', ...
        'specified location,short \n','error','error')
    return
end
%check whether initial point is within obstacle
distanceToBase_obsinitial = sqrt((xEE_initial-xobs)^2+(yEE_initial-yobs)^2);
if distanceToBase_obsinitial<= robs
    fprintf('End effector is in obstacle (initial position), short \n')
    return
end


%Inverse kinematics to find initial angles
q2_goal = acos(((xgoal-xbase)^2+(ygoal-ybase)^2-L1^2-L2^2)/(2*L1*L2));
A = L1+cos(q2_goal)*L2; B = sin(q2_goal)*L2;
q1_goal = atan2(A*(ygoal-ybase)-B*(xgoal-xbase),A*(xgoal-xbase)+B*(ygoal-ybase));
q4_goal = -acos(((xgoal-xbase)^2+(ygoal-ybase)^2-L1^2-L2^2)/(2*L1*L2));
AA = L1+cos(q4_goal)*L2; BB = sin(q4_goal)*L2;
q3_goal = atan2(AA*(ygoal-ybase)-BB*(xgoal-xbase),AA*(xgoal-xbase)+BB*(ygoal-ybase));
%Inýtial end effector position
xEE_goal = xbase + L1*cos(q1_goal)+ L2*cos(q1_goal+q2_goal);
yEE_goal = ybase + L1*sin(q1_goal)+ L2*sin(q1_goal+q2_goal);
%check whether initial point is reachable
distanceToBase_goal = sqrt((xEE_goal-xbase)^2+(yEE_goal-ybase)^2);
if distanceToBase_goal< abs(L1-L2) || distanceToBase_goal> (L1+L2)
    fprintf('You cannot place the end effector(goal position) in the specified location \n')
    return
end
%check whether goal point is within obstacle
distanceToBase_obsgoal = sqrt((xEE_goal-xobs)^2+(yEE_goal-yobs)^2);
if distanceToBase_obsgoal<= robs
    fprintf('End effector is in obstacle (goal position) \n')
    return
end


%%define grid plane
WaveFront = zeros(361,181);

%%C-space construction
cq1 = [];
cq2 = [];

for q1 = -pi:1*pi/180:pi
    for q2 = -pi:1*pi/180:pi
        
        q1_index = round(q1*180/pi)+181;
        q2_index = round(q2*180/pi)+181;
        x1 = xbase + L1*cos(q1);
        y1 = ybase + L1*sin(q1);
        x2 = xbase + L1*cos(q1) + L2*cos(q1+q2);
        y2 = ybase + L1*sin(q1) + L2*sin(q1+q2);
        
        if (x1 < 0.5 || y1 < 0.5 || x2 < 0.5 || y2 < 0.5)
            cq1(length(cq1)+1) = q1;
            cq2(length(cq2)+1) = q2;
            WaveFront(q2_index,q1_index) = 1;
            
        end
        
        % Obstacle testing for 1st link
        parameter = ((xobs-xbase)*(x1-xbase)+(yobs-ybase)*(y1-ybase))/...
            ((x1-xbase)^2+(y1-ybase)^2);
        
        if (parameter < 1 && parameter > 0)
            x = xbase + parameter*(x1-xbase);
            y = ybase + parameter*(y1-ybase);
            if (sqrt((x-xobs)^2+(y-yobs)^2) <= robs )
                cq1(length(cq1)+1) = q1;
                cq2(length(cq2)+1) = q2;
                WaveFront(q2_index,q1_index) = 1;
            end
        end
        
        % Obstacle testing for 2nd link
        parameter = ((xobs-x1)*(x2-x1)+(yobs-y1)*(y2-y1))/((x2-x1)^2+(y2-y1)^2);
        if (parameter < 1 && parameter > 0)
            x = x1 + parameter*(x2-x1);
            y = y1 + parameter*(y2-y1);
            if (sqrt((x-xobs)^2+(y-yobs)^2) <= robs)
                cq1(length(cq1)+1) = q1;
                cq2(length(cq2)+1) = q2;
                WaveFront(q2_index,q1_index) = 1;
            end
        end
        % Obstacle testing for 1st and 2nd link ends
        if (sqrt((x1-xobs)^2+(y1-yobs)^2) <= robs || sqrt((x2-xobs)^2+...
                (y2-yobs)^2) <= robs)
            cq1(length(cq1)+1) = q1;
            cq2(length(cq2)+1) = q2;
            WaveFront(q2_index,q1_index) = 1;
            
        end
    end
end

figure(1)
plot(cq1*180/pi,cq2*180/pi,'+',q1_initial*180/pi,q2_initial*180/pi,'r+',...
    q1_goal*180/pi,q2_goal*180/pi,'k+',q3_initial*180/pi,q4_initial*180/pi,...
    'ro',q3_goal*180/pi,q4_goal*180/pi,'ko')
axis([-10 180 -180 180]),grid on ,
title('Angles in which End-Effector can reach to desire position without considering boundaries');
xlabel('Theta1 [deg]');
ylabel('Theta2 [deg]');

q11a = q1_initial*180/pi;
q22b = q2_initial*180/pi;
xa = xbase + L1*cosd(q11a);    ya = ybase + L1*sind(q11a);
xb = xbase + L1*cosd(q11a) + L2*cosd(q11a+q22b);    yb = ybase + L1*sind(q11a) ...
    + L2*sind(q11a+q22b);
q11c = q3_initial*180/pi;
q22d = q4_initial*180/pi;
xa1 = xbase + L1*cosd(q11c);    ya1 = ybase + L1*sind(q11c);
xb1 = xbase + L1*cosd(q11c) + L2*cosd(q11c+q22d);    yb1 = ybase + L1*sind(q11c) ...
    + L2*sind(q11c+q22d);

q11a = q1_goal*180/pi;
q22b = q2_goal*180/pi;
xag = xbase + L1*cosd(q11a);    yag = ybase + L1*sind(q11a);
xbg = xbase + L1*cosd(q11a) + L2*cosd(q11a+q22b);    ybg = ybase + L1*sind(q11a) ...
    + L2*sind(q11a+q22b);
q11c = q3_goal*180/pi;
q22d = q4_goal*180/pi;
xa1g = xbase + L1*cosd(q11c);    ya1g = ybase + L1*sind(q11c);
xb1g = xbase + L1*cosd(q11c) + L2*cosd(q11c+q22d);    yb1g = ybase + L1*sind(q11c) ...
    + L2*sind(q11c+q22d);

figure(2)
plot([xbase xa xb],[ybase ya yb],'b-o',[xbase xa1 xb1],[ybase ya1 yb1],'b-o',...
    xball,yball,'k',xinitial,yinitial,'bo',xgoal,ygoal,'ro',[xbase xag xbg],...
    [ybase yag ybg],'g-o',[xbase xa1g xb1g],[ybase ya1g yb1g],'g-o'),grid on
axis([0 100 0 100]) 
title('Possible configurations to reach desired initial and goal positions')
xlabel('Theta1 [deg]');
ylabel('Theta2 [deg]');

%%check which inverse kinematics is not in c-space for initial poisition
distance_c1 = sqrt((cq1*180/pi-q1_initial*180/pi).^2+(cq2*180/pi-q2_initial*180/pi).^2);
distance_c2 = sqrt((cq1*180/pi-q3_initial*180/pi).^2+(cq2*180/pi-q4_initial*180/pi).^2);

if distance_c1 >= 2
    fprintf('Initial angles are q1 = %5f , q2 = %5f \n',q1_initial*180/pi,q2_initial*180/pi)
    
elseif distance_c2 >= 2
    fprintf('Initial angles are q1 = %5f , q2 = %5f \n',q3_initial*180/pi,q4_initial*180/pi)
    q1_initial = q3_initial; q2_initial = q4_initial;
else
    fprintf('Initial position is within c-space \n')
    return
end

%%check which inverse kinematics is not in c-space for goal poisition
distance_c1 = sqrt((cq1*180/pi-q1_goal*180/pi).^2+(cq2*180/pi-q2_goal*180/pi).^2);
distance_c2 = sqrt((cq1*180/pi-q3_goal*180/pi).^2+(cq2*180/pi-q4_goal*180/pi).^2);

if distance_c1 >= 2
    fprintf('Goal angles are q1 = %5f , q2 = %5f \n',q1_goal*180/pi,q2_goal*180/pi)
elseif distance_c2 >= 2
    fprintf('Goal angles are q1 = %5f , q2 = %5f\n',q3_goal*180/pi,q4_goal*180/pi)
    q1_goal = q3_goal; q2_goal = q4_goal;
else
    fprintf('Goal position is within c-space \n ')
    return
end

%%path planning
WaveFront = WaveFront(:,181:361);
q1_goal_index = round(q1_goal*180/pi)+1;
q2_goal_index = round(q2_goal*180/pi)+181;
q1_initial_index = round(q1_initial*180/pi)+1;
q2_initial_index = round(q2_initial*180/pi)+181;

WaveFront(q2_goal_index,q1_goal_index) = 2;

vertical = 361;
horizontal = 181;
%
for q1_index = q2_goal_index:1:361
    for q2_index = q1_goal_index:-1:1
        
        if (WaveFront(q1_index,q2_index) ~= 1 && WaveFront(q1_index,q2_index) ~= 0)
            if (q1_index - 1 >= 1)
                if (WaveFront(q1_index - 1,q2_index) == 0)
                    WaveFront(q1_index - 1,q2_index) = WaveFront(q1_index,q2_index) + 1;
                end
            end
            
            if (q2_index - 1 >= 1)
                if (WaveFront(q1_index,q2_index - 1) == 0)
                    WaveFront(q1_index,q2_index - 1) = WaveFront(q1_index,q2_index) + 1;
                end
            end
            
            if (q1_index + 1 <= vertical)
                if (WaveFront(q1_index + 1,q2_index) == 0)
                    WaveFront(q1_index + 1,q2_index) = WaveFront(q1_index,q2_index) + 1;
                end
            end
            
            if (q2_index + 1 <= horizontal)
                if (WaveFront(q1_index,q2_index + 1) == 0)
                    WaveFront(q1_index,q2_index + 1) = WaveFront(q1_index,q2_index) + 1;
                end
            end
            
        end
    end
end
%
for q1_index = 361:-1:1
    for q2_index = 1:1:181
        
        if (WaveFront(q1_index,q2_index) ~= 1 && WaveFront(q1_index,q2_index) ~= 0)
            if (q1_index - 1 >= 1)
                if (WaveFront(q1_index - 1,q2_index) == 0)
                    WaveFront(q1_index - 1,q2_index) = WaveFront(q1_index,q2_index) + 1;
                end
            end
            
            if (q2_index - 1 >= 1)
                if (WaveFront(q1_index,q2_index - 1) == 0)
                    WaveFront(q1_index,q2_index - 1) = WaveFront(q1_index,q2_index) + 1;
                end
            end
            
            if (q1_index + 1 <= vertical)
                if (WaveFront(q1_index + 1,q2_index) == 0)
                    WaveFront(q1_index + 1,q2_index) = WaveFront(q1_index,q2_index) + 1;
                end
            end
            
            if (q2_index + 1 <= horizontal)
                if (WaveFront(q1_index,q2_index + 1) == 0)
                    WaveFront(q1_index,q2_index + 1) = WaveFront(q1_index,q2_index) + 1;
                end
            end
            
        end
    end
end

figure(3)
meshc(WaveFront),title('Wave Front Planner')
xlabel('Theta1 [deg]');
ylabel('Theta2 [deg]');
zlabel('Wave Front Value')
if (WaveFront(q2_initial_index,q1_initial_index) == 0)
    fprintf('No solution') ,return
end

path = [];
aa = 1;

while (q1_initial_index ~= q1_goal_index || q2_initial_index ~= q2_goal_index)
    WaveFrontcurrent = WaveFront(q2_initial_index,q1_initial_index);
    
    if (q2_initial_index + 1 <= vertical) 
        if (WaveFront(q2_initial_index + 1,q1_initial_index) == WaveFrontcurrent - 1)
            q2_initial_index = q2_initial_index + 1;
            path(aa,:) = [q2_initial_index, q1_initial_index];
            aa = aa + 1;
        end
    end
    
    if (q1_initial_index + 1 <= horizontal) 
        if (WaveFront(q2_initial_index,q1_initial_index + 1) == WaveFrontcurrent - 1)
            q1_initial_index = q1_initial_index + 1;
            path(aa,:) = [q2_initial_index, q1_initial_index];
            aa = aa + 1;
        end
    end
    
    if (q2_initial_index - 1 >= 1)
        if (WaveFront(q2_initial_index - 1,q1_initial_index) == WaveFrontcurrent - 1)
            q2_initial_index = q2_initial_index - 1;
            path(aa,:) = [q2_initial_index, q1_initial_index];
            aa = aa + 1;
        end
    end
    
    if (q1_initial_index - 1 >= 1) 
        if (WaveFront(q2_initial_index,q1_initial_index - 1) == WaveFrontcurrent - 1)
            q1_initial_index = q1_initial_index - 1;
            path(aa,:) = [q2_initial_index, q1_initial_index];
            aa = aa + 1;
        end
    end
    
end



figure(4)
plot(cq1*180/pi,cq2*180/pi,'+',q1_initial*180/pi,q2_initial*180/pi,'ro',...
    q1_goal*180/pi,q2_goal*180/pi,'ko')
axis([-10 180 -200 200]),grid on,hold on,title('Animation in configuration space')
xlabel('Theta1 [deg]');
ylabel('Theta2 [deg]');
hold on

for aa = 1:10:length(path)
    q1 = (path(aa,2)-1)*pi/180;
    q2 = (path(aa,1)-1)*pi/180-pi;
    plot(q1*180/pi,q2*180/pi,'r+',(path(1:10:end,2)-1),(path(1:10:end,1)-180),'yo')
    pause(0.1)
end

for aa = 1:length(path)
    q1 = (path(aa,2)-1);
    q2 = (path(aa,1)-1)-180;
    x1 = xbase + L1*cosd(q1);
    y1 = ybase + L1*sind(q1);
    x2 = xbase + L1*cosd(q1) + L2*cosd(q1+q2);
    y2 = ybase + L1*sind(q1) + L2*sind(q1+q2);
    x(aa)= x2;
    y(aa)= y2;
    figure(5)
    plot([xbase x1 x2],[ybase y1 y2],'b-o',xball,yball,'k',xinitial,yinitial,...
        'bo',xgoal,ygoal,'ro',x,y,'-c')
    grid on
    title('Animation in Task Space')
    xlabel('Theta1 [deg]');
    ylabel('Theta2 [deg]');
    axis([0 100 0 100])
    note = annotation('textarrow',[0.44,0.44],[0.56,0.54],'String',['OBS']);
    text(x2, y2, ['  (', num2str(int8(x2)), ', ', num2str(int8(y2)), ')']);
    
end

