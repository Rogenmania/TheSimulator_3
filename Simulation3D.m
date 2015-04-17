%a Master file for the three-D simulator
clear all; clc; close all;
%initialization of.. 2 vehicle? actually, yusing teh VO frame, the choice
%is just relative velocity inside the CC. So set the distance and the
%ownship velocity (just keep these contsnat). Then choose a random relative
%velocity (ranging form 0? to double the speed of Vo) and direction inside the cone!, then you can get the
%obstacle velocity (all in 3D?). Assuming a ball separation.

%Initializations -From the VO frame? but imposible for more than 2. Should
%it be a superconflict generator?

%Agent - 0, ownship
PooG = [0; 0; 0]; %seting the origin
VooA = 2; %designated speed to make it interesting
Veeo = 0; %its zero (designated)
%pick the ownship VElv and VAzz
VooElv = -20/180*pi; %on symmteric plane, up 45 degree from the VO base lane
VooAzz = 0/180*pi;
Voo = [VooA*sin(pi/2-VooElv)*cos(VooAzz); 
       VooA*sin(pi/2-VooElv)*sin(VooAzz);
       VooA*cos(pi/2-VooElv)]; %the equation is for inclination, hence the pi/2 -

Dist = 10; %designated distance, time to collision min 5 second
Rsep = 1; %designeated protected sphere
%From here, the CC can be generated
ThetaBVO = asin(Rsep/Dist);
LimElvVO = [-ThetaBVO ThetaBVO];
LimAzzVO = [-ThetaBVO ThetaBVO];
%the CC/VO parameter
RVO = Rsep*(Dist^2-Rsep^2)^0.5/Dist;
DVO = (Dist^2-Rsep^2)/Dist;


%pick a relative velocity
%LETS SET one case first. If it works, then randomize
RelVelA = 0.75*VooA*2; %rand(1)*(Voo*2); %because the max rand is 2 times the Voo
%pick the Elev and Azz? --> set 0.8 and 0.7 as below.
RelVElv = LimElvVO(1)+0.75*range(LimElvVO); %LimElvVO(1)+rand(1)*range(LimElvVO);
RelVAzz = LimAzzVO(1)+0.75*range(LimAzzVO); %LimAzzVO(1)+rand(1)*range(LimAzzVO);
RelVel = [RelVelA*sin(pi/2-RelVElv)*cos(RelVAzz); 
          RelVelA*sin(pi/2-RelVElv)*sin(RelVAzz);
          RelVelA*cos(pi/2-RelVElv)]; %the equation is for inclination, hence the pi/2 -

%which then make the Obstacle velocity:
Vio = Voo-RelVel;

% Plot it first?
[Xunit,Yunit,Zunit] = sphere;
XObs = Xunit*Rsep+Dist; YObs = Yunit*Rsep; ZObs = Zunit*Rsep;
ObsPos = [Dist; 0; 0];
[Xunit2,Yunit2,Zunit2] = cylinder(0:1);
RotMatVO = [cos(pi/2) 0 -sin(pi/2); 0 1 0; sin(pi/2) 0 cos(pi/2)];
XVO = cos(-pi/2)*Xunit2*RVO-sin(-pi/2)*Zunit2*DVO; 
YVO = Yunit2; 
ZVO = sin(-pi/2)*Xunit2*RVO+cos(-pi/2)*Zunit2*DVO; 

%above s just the setup --> testing the cases which difficult to be determined in the bofy frame of reference . The visualization should always be on the ownshp
%frame of reference
%rotation matrix to make all of them on ownship body axis...
R2Bod0 = [cos(VooAzz)*cos(VooElv) sin(VooAzz) cos(VooAzz)*sin(VooElv);
               -sin(VooAzz)*cos(VooElv) cos(VooAzz) -sin(VooAzz)*sin(VooElv);
               -sin(VooElv) 0 cos(VooElv)];
VooB =  R2Bod0*Voo;
UVWo = VooB;
VTPo = [0; 0; 0];
ObsPosB = R2Bod0*ObsPos;
VioB = R2Bod0*Vio;
UVWi = VioB;
VTPi = [0; 0; 0];
RelVelB = R2Bod0*RelVel;
XObsB = R2Bod0(1,1)*XObs+R2Bod0(1,2)*YObs+R2Bod0(1,3)*ZObs; 
YObsB = R2Bod0(2,1)*XObs+R2Bod0(2,2)*YObs+R2Bod0(2,3)*ZObs; 
ZObsB = R2Bod0(3,1)*XObs+R2Bod0(3,2)*YObs+R2Bod0(3,3)*ZObs;
XVOB = R2Bod0(1,1)*XVO+R2Bod0(1,2)*YVO+R2Bod0(1,3)*ZVO; 
YVOB = R2Bod0(2,1)*XVO+R2Bod0(2,2)*YVO+R2Bod0(2,3)*ZVO; 
ZVOB = R2Bod0(3,1)*XVO+R2Bod0(3,2)*YVO+R2Bod0(3,3)*ZVO;

%if you want visualization
InitialVisualization; %three figure, CCframe, CC, VO. NOT A FUNCTION!

%Now move it - move it. 
%just solve the navigation equation in every time step?

TiStCh = 1; %time step when the input is checked?
SimTi = 100; % Simulation Time
CuTi = 0;
Ddn = 0; %Number of Data
Rrn = 0; %Number of Record
while CuTi < SimTi
    %calculate the Navigation Equations --> with current UVW and VTP
    %FOR each vehicle!
    [T,NEPo]=ode45(@(t,y) NavEq(t,UVWo,VTPo),TiStCh,NEPo);
    
    %record all states
    
    %Update Visualization?
    
    %then check VO situation  --> just for the ownship? at first...
    
    
    %Update UVW and VTP? --> actually need a turning rate? = R? or psidot?
    %psidot = Rcosvee/costhet (but for input?)
    
    
    %Update time and datnum
    CuTi = CuTi+TiStCh;
    Ddn = Ddn + 1;
end
%run it, and check it on a constant time step? looks like it
%Check the VO criteria


line(NEPo(:,1),NEPo(:,2),NEPo(:,3))


%As what? a Quadrotor model? input on throttle? actually thats not we want.
%except for very close escape? but we are not talking about that are we?
%but turning rate still make sense? Turnig rate on that avoidance plane?
%actually make sense? for fixed and rotary, can be translated to force and
%moment? actually moment... we try not to change the orientation much? 























