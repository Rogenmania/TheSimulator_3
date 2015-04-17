%to predefine waypoints/Flight Path
%scenario setter



%set also initial position, velocity (design velo), and attitudes
clear all; close all; clc;
load CASData

load RandomforMC3

tic
for mci = 1:NumMC
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
    AvoW = 0.0873*ones(AgentNumber,1).*WitWitOut; 
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
save MC4_3Agents








