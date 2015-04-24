classdef CAS004< Computer
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        %First Input
        Strategy = 1 %Preimplemented strategy
        Multiple
        Priorities
        Rule = 1
        CASFlag = 0
        VOFlag = 0
        
        SepRad
        VEType
        VEDensity
        VEPoint
        LimType
        LimPoint
        OwnCat = 1;
        ObsCats
        
        CASDistDat = zeros(3,6,6)
        CASVeloDat
        AvoTy
        %Predictor
        
        %OnVO
        VOStat
        VOtype 
        VONumber
        VOOrigin
        VectorNear
        VectorFar
        Vector = zeros(50,50)
        DivVector = zeros(50,50)
        VOBasAngLim = 0;
        VOBasAng = zeros(50,1)
        VOBasAngFF = zeros(50,1)
        VOLuDa = zeros(2,50);
        DivLuDa
        
        ObPosVO
        ObVelVO 
        ObAngVO
        
        ObPosGloFF
        ObPosVOFF
        
        VRelBoVO
        VRelTGoVO
        VelBoVO 
        VelTGoVO 
        
        RoW = ones(1,51);
        Bababa = 0;
        CASVelDes
        AvoW
        Interupt = 0;
        Cate
        
        %Proximity Sensor Input
        ObsNum   %[NEW]
        ImObsNum %Number of imminent object [NEW]
        ObVelAbs = zeros(50,1)
        ObDist = zeros(50,1)
        ObDistFF  = zeros(50,1)
        ObPosBod = zeros(3,50)
        ObPosBodFF = zeros(3,50)
        ObAngBod = zeros(50,1)
        ObVelBod = zeros(3,50)
        ObVelABod = zeros(3,50)
        RelVelBod 
        RelVelABod  %[NEW]
        RelVelAbs   %[NEW]
        RelVelBodTG
    end
    
    methods
        function MAC = CAS004(DBoVel,Attitude,FliPath,VOFlag,Cate,ImpoDist,ImpoVelo)
            MAC = MAC@Computer(DBoVel,Attitude,FliPath);
            MAC.VOFlag = VOFlag;
            MAC.Cate = Cate;
            %Generating Important Distance data
            MAC.CASDistDat = ImpoDist;
            MAC.CASVeloDat = ImpoVelo;


        end
        
        function ACASRun(MAC)
            %whole computations
           ObsStateCalc(MAC); %obstacles states!! also the number of detected obstacle
           CheckDistance(MAC); %first Check the Distance. further, VO not calculated. ObsCats  and  CASFlag
           
           WarningSys(MAC); %triger warning, but basically here, it doesn;t do anything fancy, based on CASFlag
           
           VelocityObstacle(MAC); %Check threats, well this make the ACAS calculating all the time....
           %This results MacVoLuDa (whether or not (both) rel vel inside VO
           %     and also RoW (wether or not have Right of Way)
           
           %then, with CASFlag, MacVoLuDa and RoW
           %according to CASFlag, do nothing, deconf, or aggresive
           switch MAC.CASFlag(1)
               case 1    % warning sphere
                   %what?
               case {2,3}    % deconf sphere, and for temporary the avoidance Sphere
                   %according to MacVoLuDa and RoW?
                   if sum(MAC.VOLuDa(1,:).*MAC.RoW(2:end)) > 0 && ...
                      sum(MAC.VOLuDa(2,:)) > 0 % C and TG -RelVel, even just one
                       MAC.Bababa = 1;
                       %make computer read the ACAS
                       MAC.Interupt = 1;
                       %turn to right
                       DeConfAvoid(MAC);
                   elseif sum(MAC.VOLuDa(1,:).*MAC.RoW(2:end)) == 0 && ...
                          sum(MAC.VOLuDa(2,:)) > 0 %TG- Relvel, even just one
                      MAC.Bababa = 2;
                      %still read ACAS
                       MAC.Interupt = 1;
                      %Keep velocity
                      KeepVelo(MAC);
                   elseif sum(MAC.VOLuDa(1,:).*MAC.RoW(2:end)) > 0 && ...
                          sum(MAC.VOLuDa(2,:)) == 0 %C- Relvel, even just one
                      %go to TG?, give it to normal control?
                      MAC.Bababa = 3;
                      MAC.Interupt = 1;
                      KeepVelo(MAC);
                   elseif sum(MAC.VOLuDa(1,:).*MAC.RoW(2:end)) == 0 && ...
                          sum(MAC.VOLuDa(2,:)) == 0 && ...
                          sum(MAC.DivLuDa(1,:)) > 0 %still, keep speed
                      MAC.Bababa = 4;
                      MAC.Interupt = 1;
                      KeepVelo(MAC);
                   elseif sum(MAC.VOLuDa(1,:).*MAC.RoW(2:end)) == 0 && ...
                          sum(MAC.VOLuDa(2,:)) == 0 && ...
                          sum(MAC.DivLuDa(1,:)) == 0 %none
                      MAC.Bababa = 40;
                      MAC.Interupt = 0;
                       
                   end
               %case 3    % escape sphere
                   
               otherwise  %do nothing sphere

           end

           
        end
        function ObsStateCalc(MAC)
            %Distance, Velocity and Position of Obstacle, in OwnAircraft
            %Body Axis
            SizeOb = size(MAC.ObPosGlo); MAC.ObsNum = SizeOb(2);
            for ii = 1:MAC.ObsNum
                MAC.ObDist(ii) = sqrt(sum((MAC.ObPosGlo(:,ii)-MAC.PosGlo).^2));
                MAC.ObVelAbs(ii) = sqrt(sum((MAC.ObVelGlo(:,ii)).^2));
                MAC.ObPosBod(:,ii) = MAC.MatE2B*(MAC.ObPosGlo(:,ii)-MAC.PosGlo);
                MAC.ObAngBod(:,ii) = Vect2Angls(MAC.ObPosBod(:,ii)); %angles
                MAC.ObVelBod(:,ii) = MAC.MatE2B*MAC.ObVelGlo(:,ii);
                MAC.ObVelABod(:,ii) = Vect2Angls(MAC.ObVelBod(:,ii)); %angles
                
                %Calculate Relative Values, usefull for 3DVO inclusion determination
                MAC.RelVelBod(:,ii) =  MAC.VelBo - MAC.ObVelBod(:,ii); 
                MAC.RelVelABod(:,ii) = Vect2Angls(MAC.RelVelBod(:,ii)); %angles
                MAC.RelVelAbs(ii) =  (sum(MAC.RelVelABod(:,ii).^2))^0.5;
            end                         
        end
        
        function CheckDistance(MAC)
            %use the MAC.ObDist properties --> Obstacles Distances
            %check categories? how about manned?
            %use MAC.ObVelAbs?
            for ii = 1:MAC.NumObs(2) %also with itself
                %MAC.ObsCats(ii) = MAC.CatFall(MAC.ObVelAbs(ii));
                if MAC.ObVelAbs(ii) > MAC.CASVeloDat(5)
                    MAC.ObsCats(ii) = 6;
                    MAC.SepRad(ii) = MAC.CASDistDat(1,MAC.Cate,6);
                elseif MAC.ObVelAbs(ii) > MAC.CASVeloDat(4)
                    MAC.ObsCats(ii) = 5;
                    MAC.SepRad(ii) = MAC.CASDistDat(1,MAC.Cate,5);
                elseif MAC.ObVelAbs(ii) > MAC.CASVeloDat(3)
                    MAC.ObsCats(ii) = 4;
                    MAC.SepRad(ii) = MAC.CASDistDat(1,MAC.Cate,4);
                elseif MAC.ObVelAbs(ii) > MAC.CASVeloDat(2)
                    MAC.ObsCats(ii) = 3;
                    MAC.SepRad(ii) = MAC.CASDistDat(1,MAC.Cate,3);
                elseif MAC.ObVelAbs(ii) > MAC.CASVeloDat(1);
                    MAC.ObsCats(ii) = 2;
                    MAC.SepRad(ii) = MAC.CASDistDat(1,MAC.Cate,2);
                else
                    MAC.ObsCats(ii) = 1;
                    MAC.SepRad(ii) = MAC.CASDistDat(1,MAC.Cate,1);
                end
                
                if MAC.ObDist(ii) < MAC.CASDistDat(3,MAC.Cate,MAC.ObsCats(ii))
                    MAC.CASFlag(ii+1) = 3;  %inside escape sphere
                elseif MAC.ObDist(ii) < (MAC.CASDistDat(2,MAC.Cate,MAC.ObsCats(ii))*MAC.AvoTy);
                    MAC.CASFlag(ii+1) = 2;  %inside deconf sphere
                elseif MAC.ObDist(ii) < MAC.CASDistDat(1,MAC.Cate,MAC.ObsCats(ii))
                    MAC.CASFlag(ii+1) = 1;  %inside warning sphere
                else
                    MAC.CASFlag(ii+1) = 0;  %do nothing - too far
                end
                
                %if  -- one more if to determined when to avoid. Can be a
                %range?
                
                %above is good, but we can actually fixed it using two
                %digit flag. One just for the zones, and one for whether it
                %is less than he avoidance distance AvoTy. AvoTy can be a
                %real number, not a fraction then... Also, it chould be for each
                %obstacle. Also, flag when its colliding
            end
            %below is the summary?
            MAC.CASFlag(1) = max(MAC.CASFlag(2:MAC.NumObs(2)+1));
        end     
        function WarningSys(MAC)
        %function send nessesary Warning
        %if CASFlag 1-2-3
        OutD3 = MAC.CASFlag;
        
        if MAC.CASFlag ~= 0
            %warn according to CASFlag
            %at this point, only change UAV properties, which means, let UAV do the work.. :) 
            
        end
        
        end
        
        function VelocityObstacle(MAC)
            SizeOb = size(MAC.ObPosGlo);
            MAC.VONumber = SizeOb(2); 
            MAC.VOBasAng(1:MAC.VONumber) = asin((1.1*MAC.SepRad')./MAC.ObDist(1:MAC.VONumber));
            
            %looking 1 step (second) forward?
            
            MAC.VOBasAngFF(1:MAC.VONumber) = asin((1.1*MAC.SepRad')./MAC.ObDistFF(1:MAC.VONumber));
            %MAC.VOBasAng = MAC.VOBasAngFF;
            %MAC.ObPosBod = MAC.ObPosBodFF;
            

            for ii = 1:MAC.VONumber
                MAC.VOOrigin(:,ii) =MAC.ObVelBod(1:2,ii);
                MAC.Vector(1:2,ii) = MAC.ObVelBod(1:2,ii);
                %Define VO! theres no near or far now.... 
                MAC.Vector(3:4,ii) = MAC.RotMat(MAC.VOBasAng(ii),2)*(100*MAC.ObPosBod(1:2,ii)+MAC.ObVelBod(1:2,ii));
                MAC.Vector(5:6,ii) = MAC.RotMat(-MAC.VOBasAng(ii),2)*(100*MAC.ObPosBod(1:2,ii)+MAC.ObVelBod(1:2,ii));
                
                %Define VO for one step forward, need new origin?
                MAC.Vector(7:8,ii) = [0;0];%MAC.RotMat(MAC.VOBasAngFF(ii),2)*(100*MAC.ObPosBodFF(1:2,ii)+MAC.ObVelBod(1:2,ii));
                MAC.Vector(9:10,ii) = [0;0];%MAC.RotMat(-MAC.VOBasAngFF(ii),2)*(100*MAC.ObPosBodFF(1:2,ii)+MAC.ObVelBod(1:2,ii));
                
                %check if the relatives velocity is inside VO or not
                MAC.VOLuDa(1,ii) = MAC.IX2D(MAC.VelBo(1:2),...
                    [MAC.Vector(3:4,ii),MAC.Vector(1:2,ii), MAC.Vector(5:6,ii)],1,1);
                MAC.VOLuDa(2,ii) = MAC.IX2D(MAC.TGoVel(1:2),...
                    [MAC.Vector(3:4,ii),MAC.Vector(1:2,ii), MAC.Vector(5:6,ii)],1,1); 
                
                %The Divergent Area
                %asume there exist a divergen area
                %the divergen area is made from two triangle, 
                MAC.DivVector(1:2,ii) = MAC.ObVelBod(1:2,ii);
                MAC.DivVector(3:4,ii) = MAC.ObVelBod(1:2,ii)-MAC.ObPosBod(1:2,ii);
                MAC.DivVector(5:6,ii) = MAC.ObVelBod(1:2,ii)+[-MAC.ObPosBod(2,ii);MAC.ObPosBod(1,ii)];
                MAC.DivVector(7:8,ii) = MAC.ObVelBod(1:2,ii)-MAC.ObPosBod(1:2,ii);
                MAC.DivVector(9:10,ii) = MAC.ObVelBod(1:2,ii)+[MAC.ObPosBod(2,ii);-MAC.ObPosBod(1,ii)];
                
                MAC.DivLuDa(1,ii) = 1-(MAC.IX2D(MAC.TGoVel(1:2),...
                    [MAC.DivVector(7:8,ii),MAC.DivVector(1:2,ii), MAC.DivVector(9:10,ii)],1,1)+...
                                    MAC.IX2D(MAC.TGoVel(1:2),...
                    [MAC.DivVector(3:4,ii),MAC.DivVector(1:2,ii), MAC.DivVector(5:6,ii)],1,1)); 
                %now for the rule, find out wether it is 
                %in this CAS004, there are no rule!
                MAC.RoW(ii+1) = 1;
                MAC.RoW(1) = max(MAC.RoW(2:MAC.NumObs(2)+1));
            end
        %so on or off???
        %Now is up to the interupt/restore function!
        end     
        
        function VelocityObstacle2(MAC)
        %function CheckVO ==> create VO
        %Transform all nessesary vector to VObase axis
        %check relative velocity in/outside VO
        %Check have RoW or not
        %check if relatives velocity is inside VO
       
        SizeOb = size(MAC.ObPosGlo);
        MAC.VONumber = SizeOb(2); 
        MAC.RoW = ones(1,MAC.VONumber);
        MAC.VOLuDa = zeros(2,MAC.VONumber);
        MAC.VOBasAng = asin((MAC.SepRad)./MAC.ObDist);
        
        for ii = 1:MAC.VONumber
            MAC.VOOrigin(:,ii) =[0;0];
            MAC.Vector(1:2,ii) = [0;0];
            
            %Everything in VOBaseAxis
            MAC.ObPosVO(:,ii) = MAC.RotMat(-(MAC.ObAngBod(ii)),2)*MAC.ObPosBod(1:2,ii);
            %also rotate obstacle speed
            MAC.ObVelVO(:,ii) = MAC.RotMat(-(MAC.ObAngBod(ii)),2)*MAC.ObVelBod(1:2,ii);
            %Own Velocity in VObase axis
            MAC.VelBoVO(:,ii) = MAC.RotMat(-(MAC.ObAngBod(ii)),2)*MAC.VelBo(1:2);
            %calculated the angle
            MAC.ObAngVO(:,ii) = dot(MAC.VelBoVO(1:2,ii),MAC.ObVelVO(:,ii))/((sum((MAC.VelBoVO(1:2,ii)+MAC.ObVelVO(:,ii)).^2))^0.5);
            %TGoal Velocity in VObase axis
            MAC.VelTGoVO(:,ii) = MAC.RotMat(-(MAC.ObAngBod(ii)),2)*MAC.TGoVel(1:2);
            %Calculated Relatives Velocity on VObase axis
            MAC.VRelBoVO(:,ii) = MAC.RotMat(-(MAC.ObAngBod(ii)),2)*MAC.RelVelBod(1:2,ii);
            MAC.VRelTGoVO(:,ii) = MAC.RotMat(-(MAC.ObAngBod(ii)),2)*MAC.RelVelBodTG(1:2,ii);
                        
            %Define VO! theres no near or far now.... 
            MAC.Vector(3:4,ii) = MAC.RotMat(MAC.VOBasAng(ii),2)*MAC.ObPosVO(1:2,ii);
            MAC.Vector(5:6,ii) = MAC.RotMat(-MAC.VOBasAng(ii),2)*MAC.ObPosVO(1:2,ii);
            
            %check if the relatives velocity is inside VO or not
            MAC.VOLuDa(1,ii) = MAC.IX2D(MAC.VRelBoVO(1:2,ii),...
                [MAC.Vector(3:4,ii),MAC.Vector(1:2,ii), MAC.Vector(5:6,ii)],1,1);
            MAC.VOLuDa(2,ii) = MAC.IX2D(MAC.VRelTGoVO(1:2,ii),...
                [MAC.Vector(3:4,ii),MAC.Vector(1:2,ii), MAC.Vector(5:6,ii)],1,1);               
            
            %Check the RoW rules
            if MAC.ObVelVO(2,ii) < 0               %if ObVelVO heading to the left 
               % MAC.RoW(ii) = 0; 

                if MAC.ObAngVO(:,ii) > cos(45/180*pi)  %if it is not head on ==> Have RoW
                    MAC.RoW(ii+1) = 0; 
                else                             %if it is head on? ==> should move
                    MAC.RoW(ii+1) = 1; 
                end
            else                                 %if ObVelAneh heading to the right ==> Should Move
                MAC.RoW(ii+1) = 1; 
            end


        end
        MAC.RoW(1) = max(MAC.RoW(2:MAC.NumObs(2)+1));

        %so on or off???
        %Now is up to the interupt/restore function!
        end     

        function KeepVelo(MAC)
            MAC.Decision = [0;0;0; 0;0;0];
        end
        function DeConfAvoid(MAC)
            %DECision is trans vel and rot Vel??
            %or trans acc and rot vel?
            %or trans vel and new heading? --> new heading actually new pos
            %thinking control, the input then velocity and HeadinG!
            %actually, thinking of avoidance plane, should be the angle -->
            %roll, then heading... and the heading is determined? while the
            %avoplane is not. but, if we using closed loop, than should be
            %heaing angle afetr roll. nad not speed? or speed constant.
            %Controller on Speed, Roll, and Heading?
            MAC.Decision = [MAC.DMaVel; 0; 0; ... %because it is body axis...
                            0;0;0];
        end
        function ReadGCS(MAC,TGoVel)
            MAC.TGoVel = TGoVel;
        end
        
        function SetInit(MAC,DBoVel,Attitude,FliPath,AvoW,AvoTy)
            MAC.AvoW = AvoW;
            MAC.DBoVel = DBoVel;
            MAC.DMaVel = (sum(DBoVel.^2))^0.5;
            MAC.TGoVel = DBoVel;
            MAC.FliPath  = FliPath;
            MAC.AttGlo = Attitude;
            
            %initiate type of avoidance?
            MAC.AvoTy = AvoTy;
        end
        
    end
end





































