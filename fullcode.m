clear all; 
close all;

%construct the object arduino
a=arduino('COM3');
%num of servos
n=7;
s1=servo(a,'D3','MinPulseDuration',500*10^-6,'MaxPulseDuration',2600*10^-6);
s2=servo(a,'D4','MinPulseDuration',500*10^-6,'MaxPulseDuration',2600*10^-6);
s3=servo(a,'D5','MinPulseDuration',500*10^-6,'MaxPulseDuration',2600*10^-6);
s4=servo(a,'D6','MinPulseDuration',500*10^-6,'MaxPulseDuration',2600*10^-6);
s5=servo(a,'D7','MinPulseDuration',500*10^-6,'MaxPulseDuration',2600*10^-6);
s6=servo(a,'D8','MinPulseDuration',500*10^-6,'MaxPulseDuration',2600*10^-6);
s7=servo(a,'D9','MinPulseDuration',500*10^-6,'MaxPulseDuration',2600*10^-6);
serv=[s1;s2;s3;s4;s5;s6;s7];
pause(1);
%% CODE
%Define D-H parameters 
a=[0,0,0,0,0,0,0];
alpha=[-pi/2,pi/2,-pi/2, pi/2,-pi/2,pi/2,0];
d1=0.09;d3=0.1;d5=0.12;d7=0.12;
d=[d1,0,d3,0,d5,0,d7];
%define servo limits
ql=[-90,90;-90,45;-135,45;-120,120;-135,125;-40,130;-135,125];
ql=deg2rad(ql);

%create robot's Links
for i=1:7
    L(i)=Link([0 d(i) a(i) alpha(i)]);
end
%generate the object robot
robot=SerialLink(L);
%Insert servo limitations to the robotic model
robot.qlim=ql;
%name the robot
robot.name="Ros";
q=deg2rad([0 -40 0 -60 0 -40 0]);
%initialization of desired pose
[~,pi,quatd]=qconv(robot,q);
od=quatd.toeul';
eta=quatd.s;
epsilon=quatd.v;
%Algorithm parameters
sdot=1;
dt=5e-4;
steps=5000;
i=1;
K=50*diag([1 1 1]);

%Algorithm for linear path
firstpoint=pi;
secondpoint=firstpoint+[0 -0.05*2 0]';
fourthpoint=firstpoint+[0.04 0 0]';
fifthpoint=fourthpoint+[-0.025 -0.03 0]';
lastpoint=fourthpoint+[0 -0.1 0]';
points=[secondpoint fifthpoint fourthpoint fifthpoint lastpoint]; 
for k=1:length(points)
    pd=points(:,k);
    s=0;
    while(i<steps)
        s=s+dt*sdot;
        [J,pi,quat]=qconv(robot,q(i,:));
        invj=pinv(J);
        pe=pi+(s/(norm(pd-pi))*(pd-pi));
        pedot=(sdot/(norm(pd-pi))*(pd-pi));
    
        %Define the tracking errors
        ep=pe-pi;
        e0=quat.s*epsilon'-eta*quat.v'-skew(epsilon)*quat.v';
        e(:,i+1)=[ep;e0];
        qdot=invj*([pedot;0;0;0]+[K*ep;K*e0]);
        q(i+1,:)=q(i,:)+dt*qdot';
        %checking for singularities
        check=robot.islimit(q(i+1,:));
        if sum(abs(check(:,1)))>=1
            ind=find(check(:,1));
            q(i+1,ind)=q(i,ind);
        end
        
        %checking if we are on the desired position
        if(norm(pd-pi)<0.001)
            break
        end
        i=i+1;
    end
end
for k=1:i+1
    q(k,4)=-q(k,4);
end
maxiter=i;

%use this after having calculated the joint angles q to move the servos
%dt is the Euler integration step you chose (di corresponds to the sampling step - you might have to change it)
di=15;
%convert q from rads to degs
theta=round(rad2deg(q));
%maxiter is the size of euler iterations
angle=zeros(maxiter+1,n);
%normalization to range [0,0.95]
for j=1:n
        angle(:,j)=0.95*((theta(:,j)+140)/(270));
end
for j=1:n
    %update the position of servos
    writePosition(serv(j),angle(1,j));
    %position=readPosition(s(j));
end
pause(6);
%write the normalized angles to each servo
for i=10:di:maxiter+1
    for j=1:n
        %update the position of servos
        writePosition(serv(j),angle(i,j));
        %position=readPosition(s(j));
    end
    pause(1);
end