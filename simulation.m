clear all; 
close all;

%Define D-H parameters 
a=[0,0,0,0,0,0,0];
alpha=[-pi/2,pi/2,-pi/2,pi/2,-pi/2,pi/2,0];
%d1=0.124;d3=0.14;d5=0.082;d7=0.182;
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

%initialization of desired pose (quaternion)
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

%Algorithm for linear path1
firstpoint=pi;
secondpoint=firstpoint+[0 -0.1 0]';
fourthpoint=firstpoint+[0.04 0 0]';
fifthpoint=fourthpoint+[-0.015 -0.04 0]';
lastpoint=secondpoint+[0.05 0 0]';
points=[secondpoint fourthpoint fifthpoint lastpoint];  
for k=1:length(points)
    pd=points(:,k);
    s=0;
    while(i<steps) 
        s=s+dt*sdot;
        [J,pi,quat]=qconv(robot,q(i,:));
        invj=pinv(J);
        pe=pi+s*(pd-pi)/norm(pd-pi);
        pedot=sdot*(pd-pi)/norm(pd-pi);
    
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
i=i+1;
q(i+1,:)=[0 0 0 0 0 0 0];

robot.plot(q,'trail','k-','delay',dt,'lightpos',[1 1 1]);