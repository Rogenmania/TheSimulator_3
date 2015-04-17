%to predefine waypoints/Flight Path
%scenario setter



%set also initial position, velocity (design velo), and attitudes
clear all; close all; clc;
load CASData
%ScenarioName = 'FailTryOut';
ZAgNum = 5;
AgentNumber = 5;
Tdens = 1;
abc = AgentNumber/Tdens;
disp(num2str(abc))
%make agents and object first. One for all....
%required set initial function!
tSimTi = 300;
tTiSt = 1;
%make all the agents, sensor, computer and recorder
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
    GCS(tii) = GCS001([0; 0; 0],[0; 0; 0],[[0; 0; 0] [0; 0; 0]]);
    CAS(tii) = CAS004([0; 0; 0],[0; 0; 0],[[0; 0; 0] [0; 0; 0]],...
                      1,2,ImpoDist,ImpoVelo);
    
    
    %BlackBox, with simulation properties
    RecGloPos(tii) = FDRecord('MCI',tSimTi,tTiSt,3);
    
    RecExpVO(tii) = FDRecord('MCI',tSimTi,tTiSt,4);
end
NumMC = 2000000;
%NumMC = factorial(ZAgNum-1)*3000000;
NumCol = zeros(NumMC,1);
ProCol = zeros(NumMC,1);
ProCol2= zeros(NumMC,1);
Collisi = zeros(NumMC,1);
collis = 0;
TraRad = 1111.11;
DecRad = 694.44;
SepRad = 41.67;


DeDens = (pi*DecRad^2)/TraRad^2;

tDsgVelRange = [8 13; 0 0; 0 0]; %Velocity, form 8 m/s to 12 m/s
tIniAttRange = [0 0 ;0 0 ;-180 180]/180*pi; %mugammachi, deg

%tIniPosRange = [-(abc^0.5)*500 (abc^0.5)*500 ;-(abc^0.5)*500 (abc^0.5)*500 ; 0 0 ];  %Position X and Y and Z==> for North-East
tIniPosRange = AgentNumber*0.5*[-TraRad TraRad ;-TraRad TraRad; 0 0 ];  %Position X and Y and Z==> for North-East
%try other scheme, using a more precise variables R and Psi
tIniDisRange = [DecRad TraRad];


%tAvoPoiRange for MC_1 = [1 1];, MC_2 ~ 4 = [0.2 1]
tAvoPoiRange = [0.2 0.99];
%tWithWithOut for MC_4 = [0 1];, other MC = [1 1]
tWithWithOut = [0 1];


%generate randomness
ADsgVel(1,:,:) = tDsgVelRange(1,1)*ones(1,AgentNumber,NumMC)+ rand(1,AgentNumber,NumMC).*(range(tDsgVelRange(1,:)')'*ones(1,AgentNumber,NumMC));
pause(rand(1))
ADsgVel(2,:,:) = tDsgVelRange(2,1)*ones(1,AgentNumber,NumMC)+ rand(1,AgentNumber,NumMC).*(range(tDsgVelRange(2,:)')'*ones(1,AgentNumber,NumMC));
pause(rand(1))
ADsgVel(3,:,:) = tDsgVelRange(3,1)*ones(1,AgentNumber,NumMC)+ rand(1,AgentNumber,NumMC).*(range(tDsgVelRange(3,:)')'*ones(1,AgentNumber,NumMC));
pause(rand(1))
AIniAtt(1,:,:) = tIniAttRange(1,1)*ones(1,AgentNumber,NumMC)+ rand(1,AgentNumber,NumMC).*(range(tIniAttRange(1,:)')'*ones(1,AgentNumber,NumMC));
pause(rand(1))
AIniAtt(2,:,:) = tIniAttRange(2,1)*ones(1,AgentNumber,NumMC)+ rand(1,AgentNumber,NumMC).*(range(tIniAttRange(2,:)')'*ones(1,AgentNumber,NumMC));
pause(rand(1))
AIniAtt(3,:,:) = tIniAttRange(3,1)*ones(1,AgentNumber,NumMC)+ rand(1,AgentNumber,NumMC).*(range(tIniAttRange(3,:)')'*ones(1,AgentNumber,NumMC));
pause(rand(1))
%making the first agent always zero headings,
%AIniAtt(3,1,:) = AIniAtt(3,1,:)*0;

AIniPos(1,:,:) = tIniPosRange(1,1)*ones(1,AgentNumber,NumMC)+ rand(1,AgentNumber,NumMC).*(range(tIniPosRange(1,:)')'*ones(1,AgentNumber,NumMC));
pause(rand(1))
AIniPos(2,:,:) = tIniPosRange(2,1)*ones(1,AgentNumber,NumMC)+ rand(1,AgentNumber,NumMC).*(range(tIniPosRange(2,:)')'*ones(1,AgentNumber,NumMC));
pause(rand(1))
AIniPos(3,:,:) = tIniPosRange(3,1)*ones(1,AgentNumber,NumMC)+ rand(1,AgentNumber,NumMC).*(range(tIniPosRange(3,:)')'*ones(1,AgentNumber,NumMC));
pause(rand(1))


%aaanddd some sophisticated turning scenario???===========================
AAvoPoi(:,:) = tAvoPoiRange(1)*ones(AgentNumber,NumMC)+ rand(AgentNumber,NumMC).*(range(tAvoPoiRange)*ones(AgentNumber,NumMC));

AWitWitOut(:,:) = round(tWithWithOut(1)*ones(AgentNumber,NumMC)+ rand(AgentNumber,NumMC).*(range(tWithWithOut)*ones(AgentNumber,NumMC)));
AWitWitOut = 2*AWitWitOut-1; % for MC04!, right and left randomizations
%ada
%then select appropriate AvoW... Table look up?
load VeloIterate_1
%AAvoW(:,:) = interp1(Dista,RTRsol,(AAvoPoi(:,:)*DecRad)); (do it later)
%=========================================================================

%Throw away situation in which agents are already inside other deconfliction sphere?
%use IniSep?
AIniSep =zeros(AgentNumber,AgentNumber,NumMC);
for ee = 1:AgentNumber
    for ff = ee:AgentNumber
        AIniSep(ee,ff,:) = ((AIniPos(1,ee,:)-AIniPos(1,ff,:)).^2 + (AIniPos(2,ee,:)-AIniPos(2,ff,:)).^2 + + (AIniPos(3,ee,:)-AIniPos(3,ff,:)).^2).^0.5;
    end

end

ggg = 0;
hhh = zeros(1,NumMC);
bbb= 1;

for ddd = 1:NumMC
    ggg = 0;
   for eee = 1:AgentNumber
       for fff = eee:AgentNumber
           if eee ~= fff
               if AIniSep(eee,fff,ddd) < 1.1*DecRad
                   hhh(bbb) = ddd;
                   bbb=bbb+1;
                   ggg = 1;
                   break 
               end
           end
       end
       if ggg == 1;
           break
       end
       ggg = 0;
   end
end

disp('yoi')


ADsgVel(:,:,hhh(1:bbb-1)) = [];
AIniAtt(:,:,hhh(1:bbb-1)) = [];
AIniPos(:,:,hhh(1:bbb-1)) = [];
AIniSep(:,:,hhh(1:bbb-1)) = [];
AAvoPoi(:,hhh(1:bbb-1)) = [];
%AAvoW(:,hhh(1:bbb-1)) = [];
AWitWitOut(:,hhh(1:bbb-1)) = [];

NuSize = size(ADsgVel);
disp(num2str(NuSize));
NumMC = NuSize(3);
SaveNum = 50;
%Saving the first 50 Success
Suc = 1;
SucDsgVel = zeros(3,AgentNumber,SaveNum);
SucIniAtt = zeros(3,AgentNumber,SaveNum);
SucAttDiff = zeros(SaveNum,1);
SucIniPos = zeros(3,AgentNumber,SaveNum);
SucEndDist = zeros(SaveNum,AgentNumber-1);
SucAvoW = zeros(AgentNumber,SaveNum);
SucAvoTy = zeros(AgentNumber,SaveNum);

%Saving the first 50 Failures
Fai = 1;
FailDsgVel = zeros(3,AgentNumber,SaveNum);
FailIniAtt = zeros(3,AgentNumber,SaveNum);
FailAttDiff = zeros(SaveNum,1);
FailIniPos = zeros(3,AgentNumber,SaveNum);
FailEndDist = zeros(SaveNum,AgentNumber-1);
FailAvoW = zeros(AgentNumber,SaveNum);
FailAvoTy = zeros(AgentNumber,SaveNum);

IniSep = zeros(AgentNumber,AgentNumber);%+9999*eye(AgentNumber,AgentNumber);
disp('done random generation')

tic
for mci = 1:1000
    DsgVel(:,:) = ADsgVel(:,:,mci);
    IniAtt(:,:) = AIniAtt(:,:,mci);
    IniPos(:,:) = AIniPos(:,:,mci);
    
    DsgPath = IniPos+...
          [cos(IniAtt(3,:)); sin(IniAtt(3,:)); zeros(1,AgentNumber)].*...
          ([6000;6000;0]*ones(1,AgentNumber))*3;

    WitWitOut = AWitWitOut(:,mci);
    AvoTy = AAvoPoi(:,mci); %divider....
    %for MC_1 ~ 2, AvoW = -0.0873. MC_3~4, use interp1(Dista,RTRsol,(AvoTy*DecRad)).*WitWitOut
    %AvoW = interp1(Dista,RTRsol,(AvoTy*DecRad)).*WitWitOut; 
    AvoW = 0.0873*ones(5,1).*WitWitOut; 
    %MonteCarlo mode
    
    %with those randomness, check collision, with SepRad = 10 meter, 41.67 (escape sphere violated),
    %694.44 (deconf sphere violated),1111.11 (traffics) , 

    %first, for collision
    collidekah = 0;
    for ii = 1:AgentNumber
        for jj = 1:AgentNumber
            if ii ~= jj
                %check collide or not, using Velocity Obstacle
                %define relative velocity
                RelVel = [cos(IniAtt(3,ii)); sin(IniAtt(3,ii))]*DsgVel(1,ii) - ...
                         [cos(IniAtt(3,jj)); sin(IniAtt(3,jj))]*DsgVel(1,jj);
                %define global angle of relative velocity
                AnglRelVel = atan2(RelVel(2),RelVel(1));
                %Define Distant
                IniSep(ii,jj) = ((IniPos(1,ii)-IniPos(1,jj))^2 + (IniPos(2,ii)-IniPos(2,jj))^2)^0.5;
                %Define Distant Angle
                AnglDist = atan2((IniPos(2,ii)-IniPos(2,jj)),(IniPos(1,ii)-IniPos(1,jj)));
                %Define VOBase Angle
                VOBAng = asin(SepRad/IniSep(ii,jj));
                
                
                %Define rotation Matrix (2D) of VOBang, Negative and Positive
                PosRotMat = [cos(VOBAng) -sin(VOBAng); sin(VOBAng) cos(VOBAng)];
                NegRotMat = [cos(-VOBAng) -sin(-VOBAng); sin(-VOBAng) cos(-VOBAng)];
                %define VOcone
                VOVector(1:2,1) = [0;0];
                VOVector(3:4,1) = PosRotMat*(IniPos(1:2,jj)-IniPos(1:2,ii));
                VOVector(5:6,1) = NegRotMat*(IniPos(1:2,jj)-IniPos(1:2,ii));
                
                VOri = VOVector(1:2,1);
                Vect1 = VOVector(3:4,1);
                Vect2 = VOVector(5:6,1);
                %check Inside or out
                CAa = (det([RelVel(1:2),Vect2])-det([VOri,Vect2]))/det([Vect1,Vect2]);
                CBb = -(det([RelVel(1:2),Vect1])-det([VOri,Vect1]))/det([Vect1,Vect2]);
                
                if CAa >= 0 && CBb >= 0 && (CAa+CBb) <= 1 %&& (CAa+CBb) <= 0.00; %if inside
                    collidekah = collidekah + 1;
                else
                    collidekah = collidekah + 0;
                end
            end
            
        end
        
    end
    NumCol(mci) = collidekah;
    if collidekah > 0

        %is it already collide?
        %check initial distance?
        for ii = 1:AgentNumber-1
            for jj = ii+1:AgentNumber
                    if IniSep(ii,jj) <= SepRad
                        collis = collis + 1;
                        ccc = 1;
                        break
                    else
                        ccc = 0;
                    end
            end
            if ccc == 1;
                disp('Hore')
                break;
            end
        end
        
        if ccc == 1;
            collis = collis + 1; %crash initial, count as unsolved collision ??
            Collisi(mci) = -1;
        else
            Collisi(mci) = 1;
            %disp(['Had to do simulation.. :D  :SimNum' num2str(mci)])
            
            %do simulation ?
            %1 set initial configuration the agents etc
            
            
            for tii = 1:AgentNumber
                Agent(tii).SetInit(IniPos(:,tii),DsgVel(:,tii),IniAtt(:,tii))
                %Generate Sensor accordingly (Acc,Err,Rang,iData)
                VelSens(tii).SetInit(DsgVel(:,tii));
                AttSens(tii).SetInit(IniAtt(:,tii));
                GPSP(tii).SetInit(IniPos(:,tii));
                %CASManager?
                GCS(tii).SetInit(DsgVel(:,tii),IniAtt(:,tii),[IniPos(:,tii) DsgPath(:,tii)]);
                CAS(tii).SetInit(DsgVel(:,tii),IniAtt(:,tii),[IniPos(:,tii) DsgPath(:,tii)],AvoW(tii,1),AvoTy(tii,1));
                RecGloPos(tii).AddRecord(Agent(tii).GloPos)
                
                %[tii AvoW(tii,1) AvoTy(tii,1)]
            end
            %Dr = AvoW
            %Er = AvoTy
            %Run Simulation
            UAVAvoidance_2;
            


        end
    else
        Collisi(mci) = 0;
    end
    
    if Collisi(mci) == 2
        if Fai <= SaveNum
            FailDsgVel(:,:,Fai) = DsgVel(:,:);
            FailIniAtt(:,:,Fai) = IniAtt(:,:);
            FailAttDiff(Fai) = (IniAtt(3,1)-IniAtt(3,2))-pi*floor((IniAtt(3,1)-IniAtt(3,2))/pi);
            FailIniPos(:,:,Fai) = IniPos(:,:);
            FailAvoW(:,Fai) = AvoW(:,1);
            FailAvoTy(:,Fai) = AvoTy(:,1);
            FailEndDist(Fai,:) = EndDist;
        end
        Fai = Fai + 1;
%         if Fai == 25
%             break
%         end
    elseif Collisi(mci) == 1 && Suc <=SaveNum
        SucDsgVel(:,:,Suc) = DsgVel(:,:);
        SucIniAtt(:,:,Suc) = IniAtt(:,:);
        SucAttDiff(Suc) = (IniAtt(3,1)-IniAtt(3,2))-pi*floor((IniAtt(3,1)-IniAtt(3,2))/pi);
        SucIniPos(:,:,Suc) = IniPos(:,:);
        SucAvoW(:,Suc) = AvoW(:,1);
        SucAvoTy(:,Suc) = AvoTy(:,1);
        SucEndDist(Suc,:) = EndDist;
        Suc = Suc + 1;
    end
     %ProCol(mci) = collis/mci;

     %just for showing progress
     if rem(mci,5000) == 0
         disp(num2str(mci));
     end

end
toc
WaktuButuh = toc;



%load MC4_5Agents_a
figure(10*ZAgNum)
plot(Collisi)
NCol = 0;
NCol2 = 0;
for ii = 1:NumMC
    NCol = NCol + (Collisi(ii)>1);
    NCol2 = NCol2 + (Collisi(ii)>0);
    ProCol(ii) = NCol/ii;
    ProCol2(ii) = NCol2/ii;
end
figure(20*ZAgNum)
plot(ProCol(1:NumMC),'r'); hold on;
plot(ProCol2(1:NumMC),'k');
% 
% figure(30*ZAgNum)
% plot(abs(FailAttDiff(1:Fai-1)*180/pi),'bd')
%save MC4_5Agents_a








