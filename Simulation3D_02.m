%a Master file for the three-D simulator
clear all; clc; close all;
%initialization of.. 2 vehicle? actually, yusing teh VO frame, the choice
%is just relative velocity inside the CC. So set the distance and the
%ownship velocity (just keep these contsnat). Then choose a random relative
%velocity (ranging form 0? to double the speed of Vo) and direction inside the cone!, then you can get the
%obstacle velocity (all in 3D?). Assuming a ball separation.

%%
%Making the world.... making the object==================================
AgentNumber = 2;
tSimTiR = 3; %Recording Alocation
tTiStR = 0.1; %Recording Save
for tii = 1:AgentNumber
    Agent(tii) = UAV(1,1,1,1,0,...
                 [0; 0; 0],[0; 0; 0],[0; 0; 0]);
                
    %Generate Sensor accordingly (Acc,Err,Rang,iData)
    VelSens(tii) = Sensor(1,0,100,[0; 0; 0]);
    AttSens(tii) = Sensor(1,0,100,[0; 0; 0]);
    GPSP(tii) = Sensor(1,0,100,[0; 0; 0]);
    GPSV(tii) = Sensor(1,0,100,[0;0;0]);
    ProxSensP(tii) = Sensor(1,0,40000,[0; 0; 0]);
    ProxSensV(tii) = Sensor(1,0,40000,[0; 0; 0]);
        
    %CASManager?
    load(CASData); %the velocity and distance data of the spheres
    GCS(tii) = GCS001([0; 0; 0],[0; 0; 0],[[0; 0; 0] [0; 0; 0]]);
    CAS(tii) = CAS004([0; 0; 0],[0; 0; 0],[[0; 0; 0] [0; 0; 0]],...
                       1,2,ImpoDist,ImpoVelo); %Last two --> ImpoDist and ImpoVelo
        
    %BlackBox, with simulation properties
    RecGloPos(tii) = FDRecord('XYZ_g',tSimTiR,tTiStR,3);
    
    RecExpVO(tii) = FDRecord('VO',tSimTiR,tTiStR,4); 
end
%========================================================================


%%
%Initializations -========================================================
%From the VO frame? but imposible for more than 2. Should
%it be a superconflict generator? Orjust for two? JUST OFR TWO. FOCUS:
%Reciprocality
%So Initilaization based on the CC framework - ensure conflict

%Agent - 0, ownship
PooG = [0; 0; 0]; %seting the origin ALWAYS
XYZ_g(:,1) = [0;0;0];%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
VooA = 2; %designated 
UVW_b(:,1) = [VooA;0;0];%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
UVW_g(:,1) = UVW_b(:,1);
VTP_g(:,1) = [0;0;0];%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
Veeo = 0; %its zero (designated)
VooElv = -20/180*pi; %on symmteric plane, up 45 degree from the VO base lane
VooAzz = 0/180*pi;
Voo = [VooA*sin(pi/2-VooElv)*cos(VooAzz); 
       VooA*sin(pi/2-VooElv)*sin(VooAzz);
       VooA*cos(pi/2-VooElv)]; %the equation is for inclination, hence the pi/2 -
VeG(:,1) = Voo;
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
VioA = (sum(Vio.^2))^0.5;
UVW_b(:,2) = [VooA;0;0];%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

% Plot it first?

ObsPos = [Dist; 0; 0];

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
XYZ_g(:,2) = ObsPosB;%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
VoiG = R2Bod0*Vio;
UVW_g(:,2) = VoiG;
VioB = R2Bod0*Vio;
UVWi = VioB;
VTPi = [0; acos(VioB(3)/(VioA)); atan2(VioB(2),VioB(1))];
VTP_g(:,2) = VTPi;%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
RelVelB = R2Bod0*RelVel;

%The end of the road
Tifin = 1000; %until it stop. Does not matter
XYZfin_g = XYZ_g + UVW_g*Tifin;%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
%=========================================================================

%%
%if you want scenario visualization======================================
%InitialVisualization; %three figure, CCframe, CC, VO. NOT A FUNCTION!
%=========================================================================

%%
%Puting Init Value into Objects =========================================
for tii = 1:AgentNumber
    Agent(tii).SetInit(XYZ_g(:,tii),UVW_b(:,tii),VTP_g(:,tii))
    %Generate Sensor accordingly (Acc,Err,Rang,iData)
    VelSens(tii).SetInit(UVW_b(:,tii));
    AttSens(tii).SetInit(VTP_g(:,tii));
    GPSP(tii).SetInit(XYZ_g(:,tii));
    %CASManager?
    GCS(tii).SetInit(UVW_b(:,tii),VTP_g(:,tii),[XYZ_g(:,tii) XYZfin_g(:,tii)]);
    CAS(tii).SetInit(UVW_b(:,tii),VTP_g(:,tii),[XYZ_g(:,tii) XYZfin_g(:,tii)],0,0);
    
    RecGloPos(tii).AddRecord(Agent(tii).GloPos)
    %[tii AvoW(tii,1) AvoTy(tii,1)]
end

%========================================================================

%%
%Now move it - move it. 
%just solve the navigation equation in every time step?
ElaTi = 0;
TimeEnd = RecGloPos(1).TimeEnd;
TiSt = RecGloPos(1).TimeStep;
%TiSt = 1;
ttt=300;
dde = 0;
while ElaTi < TimeEnd
    for ii = 1:AgentNumber  %Each Agent Process, still on same phase
        disp([num2str(ElaTi) '  hitungan agent ' num2str(ii) '============'])
        %Sensor Sensing...
        VelSens(ii).Sense(Agent(ii).BodVel)
        AttSens(ii).Sense(Agent(ii).WinAtt)
        GPSP(ii).Sense(Agent(ii).GloPos)
        GPSV(ii).Sense(Agent(ii).GloVel)
        ProxSensP(ii).Clear()
        ProxSensV(ii).Clear()
        for jj = 1:AgentNumber
            if ii ~= jj
                ProxSensP(ii).SenseAdd(Agent(jj).GloPos)
                ProxSensV(ii).SenseAdd(Agent(jj).GloVel)
            end
        end
        
        %CAS Computer receive all sensor data
        CAS(ii).InputSensor(GPSP(ii).MeasureData,GPSV(ii).MeasureData,...
            VelSens(ii).MeasureData, AttSens(ii).MeasureData, ...
            ProxSensP(ii).MeasureData, ProxSensV(ii).MeasureData)
        %GCS Computer receive all sensor data
        GCS(ii).InputSensor(GPSP(ii).MeasureData,GPSV(ii).MeasureData,...
            VelSens(ii).MeasureData, AttSens(ii).MeasureData,[],[])
        
        %Avoidance Computer
        GCS(ii).GCSRun()                                                   %GCS Computer analyzing and deciding
        CAS(ii).ReadGCS(GCS(ii).TGoVel)                                    %CAS Computer read data from GCS and set info for GCS
        CAS(ii).ACASRun()                                                  %CAS Computer analyzing and deciding
        GCS(ii).ReadCAS(CAS(ii).CASFlag,CAS(ii).Decision,CAS(ii).Interupt) %GCS Computer read data and interupt from CAS

        %Put Input on UAV
        %Agent(ii).InputDVel_1(Desicion, Warning)
        Agent(ii).InputDVel_1(GCS(ii).Decision,GCS(ii).CASFlag)
 
    end
    
    %Update and Record states --> only after all Vehicles decide
    for ii = 1:AgentNumber
        %if there are collision, stop immediately?
        if CAS(ii).CASFlag(1) == 3
            %LastPoint = CAS(ii).ObDist(jj)
            dde = 1;
            gugu = ii;
            %ElaTi
            break
        end
        %CHANGE below to world!. Then just update the data inside each agent
        Agent(ii).MoveTimeD_1(RecGloPos(1).TimeStep)
        dde=0;
    end
    ElaTi = ElaTi + TiSt;
end
EndDist = CAS(1).ObDist(1:AgentNumber-1,1);

%%

iguuih
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























