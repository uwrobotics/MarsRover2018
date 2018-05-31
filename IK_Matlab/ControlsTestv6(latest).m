% Controls Test v2
clear all; 
close all;

writerObj = VideoWriter('IK_Sim.avi');
writerObj.FrameRate = 1;

%% Parameters

%Robot Parameters
%Distance between dof and previous dof along axis being rotated about
d2 = 0.03;
lee = 0.1;
d = [0,0,0];

%Distance between dof and previous dof along axis that is rotating

l1 = 0.3;
l2 = 0.3;
l3 = 0.2;

r = [l1,l2,l3];

%Variable rotations
q = zeros(1,3); % state vector
q_ref = zeros(1,length(q)); %commanded new q
q0 = [-1*pi/2,2*pi/3,0];
q = q0;
n = 4;
Q_refpts = zeros(n,length(q)); % Points to interpolate for each DoF

%Offset between reference frame of dof and reference frame of previous dof
a = [0,0,0];

W = DH_solve(q0,a,r,d);

% Plant Parameters
K = [0.00196,0.00196,0.00196]; % Motor constants 
ng = [1/2000,1/2000,1/1500]; % Gear ratio
bm = [0.1,0.1,0.1]; % Viscous friction of motor
bl = [0.01,0.01,0.01]; % Maximum viscuous friction of arm
Im = [0.00002,0.00002,0.00002]; % Inertia of motor
Il = [8*(l1+l2+l3)^2,4.5*(l2+l3)^2,2*(l3)^2]./3; % Maximum inertia of arm
beff = bm + bl.*ng.^2;
Ieff = Im + Il.*ng.^2;

% Controller Parameters
Vmax = [1,0.9,0.7];

T = 1;
dt = 1/1200; % Sampling period
t = 0:dt:T;
Dd = 100; % Proportional constant for how many sample periods over which to perform control loop
DT = Dd*dt;

Kp = [5,5,5];
Kd = [0.45,0.45,0.5];
Ki = [0.01,0.01,0.01];
U = zeros(length(t),length(q));
Wtmp = zeros(length(t),length(q));
W = zeros(length(t),length(q));
Y = zeros(2*length(t)-1,length(q));
Y(1,:) = q0;


A = zeros(4,4,length(t));
O = zeros(4,4,length(t));
A(:,:,1) = DH_solve(q0,a,r,d);
H = zeros(4,4,length(t));
H(:,:,1) = DH_solve(q0(1:2),a,r,d);
Qref = zeros(length(t),length(q)); % Reference DoF angles over time
Q = zeros(length(t),length(q)); % Actual DoF angles over time
Qerr = zeros(length(t),length(q));
Qref(1,:) = q0;
Q(1,:) = q0;
Qerr(1,:) = Q(1,:) - Qref(1,:);

u = zeros(length(t),length(q)); % reference state
x = zeros(length(t),length(q)); % actual state
x_ref = zeros(1,length(q));
vq = zeros(length(t),length(q)); %velocity of each dof at each time step

R = 0*(5*(sin(t) + 0.5*cos(2*t))); % Cylindrical radius of wrist from origin
h = 3*cos(25*t); % Cylindrical height of wrist from origin
b = pi/2; % Cylindrical angle of rotation from forward heading

u(:,1) = R;
u(:,2) = h;
u(:,3) = b;

ee = zeros(length(t),2);
XY = link_solve2d(Qref(1,:),r);
ee(1,:) = XY(length(q)+1,:);

for w=1:length(q)
    if Qref(1,w)>=pi
        Qref(1,w) = -pi+rem(Qref(1,w),pi);
    elseif Qref(1,w)<=-pi
        Qref(1,w) = pi+rem(Qref(1,w),pi);
    end
end


%% Main Loop
for j=2:length(t)
    A(:,:,j) = DH_solve(Qref(j-1,:),a,r,d);
    %disp(A(:,:,j));
    
    H(:,:,j) = DH_solve(Qref(j-1,1:2),a,r,d);
    
    Rcur = sqrt(H(1,4,j)^2 + H(3,4,j)^2);
    hcur = H(2,4,j);
    bcur = atan2((hcur-A(2,4,j)),(Rcur-sqrt(A(1,4,j)^2 + A(3,4,j)^2)));

    x(1) = Rcur;
    x(2) = hcur;
    x(3) = bcur;
    x_ref(j,1) = Rcur + u(j,1)*dt;
    x_ref(j,2) = hcur + u(j,2)*dt;
    x_ref(j,3) = u(j,3);
    
    q_ref(1) = solve_for_theta2(x_ref(j,:), r, Qref(j-1,1));
    q_ref(2) = solve_for_theta3(x_ref(j,:), r, Qref(j-1,2));
    
    vq(j,1:2) = [(q_ref(1)-Qref(j-1,1))/dt,(q_ref(2)-Qref(j-1,2))/dt];
    Qref(j,1:2) = [Qref(j-1,1)+vq(j,1)*dt,Qref(j-1,2)+vq(j,2)*dt];
    q_ref(3) = solve_for_theta4(x_ref(j,3),Qref(j,1:2));
    vq(j,3) = (q_ref(3)-Qref(j-1,3))/dt;
    Qref(j,3) = Qref(j-1,3)+vq(j,3)*dt;
    for w=1:length(q)
        if Qref(j,w)>=pi
            Qref(j,w) = -pi+rem(Qref(j,w),pi);
        elseif Qref(j,w)<=-pi
            Qref(j,w) = pi+rem(Qref(j,w),pi);
        end
    end
    
    %{
    XY = link_solve2d(Qref(j,:),r);
    ee(j,:) = XY(length(q)+1,:);
    figure(6);
    clf;
    hold on;
    grid on;
    axis([-0.8 0.8 -0.8 0.8]);
    plot(XY(1:2,1),XY(1:2,2),'b-o');
    hold on;
    plot(XY(2:3,1),XY(2:3,2),'r-o');
    hold on;
    plot(XY(3:4,1),XY(3:4,2),'g-o');
    hold on;
    scatter(x_ref(j,1),x_ref(j,2),'k')
    if (makemovie) writeVideo(vidObj, getframe(gca)); end
    %}
        
        %if (makemovie) writeVideo(vidObj, getframe(gca)); end
end

for k=2:length(t)
    Qerr(k,:) = Q(k,:) - Qref(k,:);
    Ki_err = zeros(1,length(q));
    for p=1:length(q)
        Ki_err(p) = trapz(Qerr(1:k,p));
    end
    
    U(k,:) = Kp.*Qerr(k,:) + Kd.*(Qerr(k,:)-Qerr(k-1,:)) + Ki.*Ki_err;  
    tq = zeros(length(t),length(q));
    tq(:,1) = t;
    tq(:,2) = t;
    tq(:,3) = t;
    Wtmp = exp(-beff.*tq./Ieff)./Ieff;
    
    %W(k,1) = conv(U(k,1),Wtmp(:,1));
    %W(k,2) = conv(U(k,2),Wtmp(:,2));
    %W(k,3) = conv(U(k,3),Wtmp(:,3));
end

% PID code
B1 = (Ieff +K.*Kd);
A1 = K.*Kd;
A2 = K.*Kp;
A3 = K.*Ki;
B2 = (beff + K./Kp);
B3 = K.*Ki;

C1 = B2./(2.*B1);
C2 = sqrt(B2.^2 - 4.*B1.*B3)./(2.*B1);

D1 = (A1.*(-C1-C2).^2 + A2.*(-C1-C2) + A3)./(-2.*C2);
D2 = (A1.*(-C1+C2).^2 + A2.*(-C1+C2) + A3)./(2.*C2);

T = zeros(length(t),length(q));
for j=1:length(q)
    T(:,j) = (D1(j)*exp(-(C1(j)+C2(j))'*t)+D2(j)*exp(-(C1(j)-C2(j))'*t))';
    Y(:,j) = conv(Qref(:,j),T(:,j));
end
Y = Y(1:length(t),:);

for j=1:length(Y(:,1))
    XY = link_solve2d(Y(j,:),r);
    figure(6);
    clf;
    hold on;
    grid on;
    axis([-0.7 0.9 -0.7 0.7]);
    plot(XY(1:2,1),XY(1:2,2),'b-o');
    hold on;
    plot(XY(2:3,1),XY(2:3,2),'r-o');
    hold on;
    plot(XY(3:4,1),XY(3:4,2),'g-o');
    hold on;
    
    O(:,:,j) = DH_solve(Y(j,:),a,r,d);
    disp(O(:,:,j));
end

figure(1)
plot(t,Qref(:,1),'b');
hold on;
plot(t,Qref(:,2),'r');
hold on;
plot(t,Qref(:,3),'g');
hold off;

figure(2)
plot(t,Y(:,1),'b');
hold on;
plot(t,Y(:,2),'r');
hold on;
plot(t,Y(:,3),'g');
hold off;

figure(3)
scatter(t,-Y(:,1),'b');
hold on;
plot(t,Qref(:,1),'k');
hold on;
scatter(t,-Y(:,2),'r');
hold on;
plot(t,Qref(:,2),'k');
hold on;
scatter(t,-Y(:,3),'g');
hold on;
plot(t,Qref(:,3),'k');
hold off;




%% Closed-loop control



%{
if (makemovie) close(vidObj); end
figure(1);
plot(t,R,'r');
hold on;
plot(t,ee(:,1),'b');
hold off;
figure(2);
plot(t,h,'r');
hold on;
plot(t,ee(:,2),'b');
hold off;
figure(3);
plot(R,h,'b');
%}


%% Functions
function A = DH_solve(q,a,r,d)
    A = eye(4);    
    for i=1:length(q)
        tmp=[cos(q(i))  -cos(a(i))*sin(q(i))   sin(a(i))*sin(q(i))   r(i)*cos(q(i));
        sin(q(i))   cos(a(i))*cos(q(i))  -sin(a(i))*cos(q(i))   r(i)*sin(q(i));
        0              sin(a(i))             cos(a(i))             d(i);
        0         0                     0              1];    
    A = A*tmp;
    end
end

function XYZ = link_solve(q,a,r,d)
    A = eye(4); 
    XYZ = zeros(length(q)+1,3);
    for i=1:length(q)
        tmp=[cos(q(i))  -cos(a(i))*sin(q(i))   sin(a(i))*sin(q(i))   r(i)*cos(q(i));
        sin(q(i))   cos(a(i))*cos(q(i))  -sin(a(i))*cos(q(i))   r(i)*sin(q(i));
        0              sin(a(i))             cos(a(i))             d(i);
        0         0                     0              1];    
    A = A*tmp;
    XYZ(i+1,:) = A(1:3,4)';
    end
end

function XY = link_solve2d(q,r)
    XY = zeros(length(q)+1,2);
    for k=1:length(q)
        qsum = 0;
        for i=1:k
            qsum = qsum + q(i);
        end
        XY(k+1,:) = [XY(k,1) + r(k)*cos(qsum),XY(k,2) + r(k)*sin(qsum)];
    end
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% solve for first joint theta1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function q1 = solve_for_theta1(x)

    q1 = x(3);

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% solve for second joint theta2, two different
% solutions are returned, corresponding
% to elbow up and down solution
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function q2 = solve_for_theta2(x, r, q)
    R = x(1);
    h = x(2);
    xy = sqrt(R^2 + h^2);

    beta = atan2(h, R);
    gamma = (acos((r(1)^2+xy^2-r(2)^2)/(2*xy*r(1))));

    if ~isreal(gamma)
        disp('WARNING:inversekinematic_irb140: the point is not reachable for this configuration, imaginary solutions'); 
        gamma = real(gamma);
    end

    %return two possible solutions
    %elbow up and elbow down
    %the order here is important and is coordinated with the function
    %solve_for_theta3
    q2p(1) = beta - gamma; %elbow up
    q2p(2) = beta + gamma; %elbow down
    
    if (abs(q - q2p(1)) < abs(q - q2p(2)))
        q2 = q2p(1);
    else
        q2 = q2p(2);
    end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% solve for third joint theta3, two different
% solutions are returned, corresponding
% to elbow up and down solution
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function q3 = solve_for_theta3(x, r, q)
    R = x(1);
    h = x(2);
    xy = sqrt(R^2 + h^2);

    eta = (acos((r(1)^2 + r(2)^2 - xy^2)/(2*r(1)*r(2))));

    if ~isreal(eta)
       disp('WARNING:inversekinematic_irb140: the point is not reachable for this configuration, imaginary solutions'); 
       eta = real(eta);
    end

    %return two possible solutions
    %elbow up and elbow down solutions
    %the order here is important
    q3p(1) = pi - eta;
    q3p(2) = eta - pi;
    
    if (abs(q - q3p(1)) < abs(q - q3p(2)))
        q3 = q3p(1);
    else
        q3 = q3p(2);
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% solve for fourth joint theta4
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function q4 = solve_for_theta4(x,q)

    q4 = x-q(1)-q(2);

end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% solve for fifth joint theta5
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function q5 = solve_for_theta5(x)

    q5 = x(5);

end
