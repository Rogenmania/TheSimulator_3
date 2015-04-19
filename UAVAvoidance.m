

%To Simulate Velocity Obstacle
%Yazdi Ibrahim Jenie
%2012

clear all; close all; 
clc;
for Uum = 1:1
%load scenario
NameFile = ['tryMST13_03_All'];
load([NameFile '.mat'])

%NAgent
%Run simulation===========================================================
tic
ElaTi = 0;
TimeEnd = RecGloPos(1).TimeEnd;
TiSt = RecGloPos(1).TimeStep;
%TiSt = 1;

while ElaTi < 50
    for ii = 1:length(Agent)  %Each Agent Process, still on same phase
        disp(['hitungan agent ' num2str(ii) '============'])
        %Sensor Sensing...
        VelSens(ii).Sense(Agent(ii).BodVel)
        AttSens(ii).Sense(Agent(ii).WinAtt)
        GPSP(ii).Sense(Agent(ii).GloPos)
        GPSV(ii).Sense(Agent(ii).GloVel)
        
        ProxSensP(ii).Clear()
        ProxSensV(ii).Clear()
        for jj = 1:length(Agent)
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
        
        %GCS Computer analyzing and deciding
        GCS(ii).GCSRun()
        %ACAS Computer read data from GCS and set info for GCS
        CAS(ii).ReadGCS(GCS(ii).TGoVel)
        %CAS Computer analyzing and deciding
        CAS(ii).ACASRun() 
        %GCS Computer read data and interupt from CAS
        GCS(ii).ReadCAS(CAS(ii).CASFlag,CAS(ii).Decision,CAS(ii).Interupt)

        
        %Put Input on UAV
        %Agent(ii).InputDVel_1(Desicion, Warning)
        Agent(ii).InputDVel_1(GCS(ii).Decision,GCS(ii).CASFlag)
 
    end
    %RECORDING!!!!
    %Update and Record states
    %G = length(Agent)
    MatVOall = zeros(52*12,1);
    for ii = 1:length(Agent)
        D= GCS(ii).MatE2B;
        E= GCS(ii).DriftAvo;
        RecGloPos(ii).AddRecord(Agent(ii).GloPos)
        RecGloVel(ii).AddRecord(Agent(ii).GloVel)
        RecBodVel(ii).AddRecord([Agent(ii).BodVel ; GCS(ii).TGoVel])        
        for jj = 1:length(Agent)-1            
            MatVOall(1+(jj-1)*12:11+1+(jj-1)*12,1) = CAS(ii).Vector(:,jj);
            MatVOall(51+(jj-1)*12:1+51+(jj-1)*12,1) = CAS(ii).ObPosBod(1:2,jj);
%             MatVOall(3+(jj-1)*6:4+(jj-1)*6,1) = CAS(ii).Vector(:,jj);
%             MatVOall(5+(jj-1)*6:6+(jj-1)*6,1) = CAS(ii).Vector(:,jj);
        end
        RecVO(ii).AddRecord(MatVOall)
%        RecOLim(ii).AddRecord([CAS(ii).LimData(1:2,1);CAS(ii).LimData(1:2,2);CAS(ii).LimData(1:2,3)])
        RecExpVO(ii).AddRecord([CAS(ii).Bababa; GCS(ii).CASFlag(1) ; CAS(ii).RoW(1); CAS(ii).Interupt]);
        
        if CAS(ii).CASFlag(1) == 3
            disp('heheheheeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee')
            A = [ii CAS(ii).Bababa]
            %pause(0.5)

        end
            


        %Updating
        Agent(ii).MoveTimeD_1(RecGloPos(1).TimeStep)
    end

    ElaTi = ElaTi + TiSt;
end
CAS(1).AvoW
CAS(2).AvoW
pause(5)
toc

save Record RecGloPos RecGloVel RecBodVel RecVO RecExpVO CAS


Visualization

end
