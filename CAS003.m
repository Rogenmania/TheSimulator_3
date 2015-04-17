classdef CAS003 < Computer
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
        
        ObVelAbs = zeros(10,1)
        ObDist = zeros(10,1)
        ObDistFF  = zeros(10,1)
        ObPosBod = zeros(3,10)
        ObPosBodFF = zeros(3,10)
        ObAngBod = zeros(10,1)
        ObVelBod = zeros(3,10)
        RelVelBod
        RelVelABod
        RelVelBodTG
    end
    
    methods
        function MAC = CAS003(DBoVel,Heading,FliPath,VOFlag,Cate,ImpoDist,ImpoVelo)
            MAC = MAC@Computer(DBoVel,Heading,FliPath);
            MAC.VOFlag = VOFlag;
            MAC.Cate = Cate;
            %Generating Important Distance data
            MAC.CASDistDat = ImpoDist;
            MAC.CASVeloDat = ImpoVelo;


        end
        
        function ACASRun(MAC)
            %whole computations
           SensorCalc(MAC);
           CheckDistance(MAC); %first Check the Distance
           %This results ObsCats (Obstacle Categories based on Velocity)
           %     and also CASFlag (3-2-1 or 0, from escape to do nothing)
           WarningSys(MAC); %warning system, but basically here, it doesn;t do anything fancy, based on CASFlag
           VelocityObstacle(MAC); %Check threats, well this make the ACAS calculating all the time....
           %This results MacVoLuDa (whether or not (both) rel vel inside VO
           %     and also RoW (wether or not have Right of Way)
           
           %then, with CASFlag, MacVoLuDa and RoW
           %according to CASFlag, do nothing, deconf, or aggresive
           switch MAC.CASFlag(1)
               case 1    % warning sphere
                   %what?
                   MAC.Interupt = 0;
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
                   MAC.Interupt = 0;

           end

           
        end
        function SensorCalc(MAC)
        %Distance, Velocity and Position of Obstacle, in OwnAircraft
            %Body Axis
            SizeOb = size(MAC.ObPosGlo);
            %for looking one step forward?
            MAC.ObPosGloFF = MAC.ObPosGlo + MAC.ObVelGlo;
            
            
            for ii = 1:SizeOb(2)
                MAC.ObDist(ii) = sqrt(sum((MAC.ObPosGlo(:,ii)-MAC.PosGlo).^2));
                MAC.ObVelAbs(ii) = sqrt(sum((MAC.ObVelGlo(:,ii)).^2));
                MAC.ObPosBod(:,ii) = MAC.MatE2B*(MAC.ObPosGlo(:,ii)-MAC.PosGlo);
                MAC.ObAngBod(ii) = atan2((MAC.ObPosBod(2,ii)),(MAC.ObPosBod(1,ii)));
                MAC.ObVelBod(:,ii) = MAC.MatE2B*MAC.ObVelGlo(:,ii);
                
                MAC.ObDistFF(ii) = sqrt(sum(((MAC.ObPosGlo(:,ii)+MAC.ObVelGlo(:,ii))...
                                             -(MAC.PosGlo+MAC.VelGlo)).^2));
                MAC.ObPosBodFF(:,ii) = MAC.MatE2B*((MAC.ObPosGlo(:,ii)+MAC.ObVelGlo(:,ii))...
                                              -(MAC.PosGlo+MAC.VelGlo));          
            end
            %Calculate Relative Values
            %Relative Velocity Magnitudes
            NumObj = size(MAC.ObVelBod);
            MAC.RelVelBod =  MAC.VelBo*ones(1,NumObj(2)) - MAC.ObVelBod;
            MAC.RelVelBodTG = MAC.TGoVel*ones(1,NumObj(2)) - MAC.ObVelBod;
            MAC.RelVelABod = [atan2(MAC.RelVelBod(2,:),MAC.RelVelBod(1,:));
                              atan2(MAC.RelVelBodTG(2,:),MAC.RelVelBodTG(1,:))];
             %Define Divergen Area?can only go to ToGoal if To Goal is
             %inside the divergen area?
             %Line of Obstacle Velocity Vector
               %[0,0];[MAC.ObVelBod(1:2,ii)];
             %and the opposite
               %[0,0];[-MAC.ObVelBod(1:2,ii)];
             %check obstacle position
               %[0,0)];[MAC.ObPosBod(1:2,ii)]
             %so the ToGoal need not to be inside the area, separated by
             %Obstacle velocity vector, where the obstacle exist.
             %Togoal Angle from Position vector need to be bigger than
             %positive angle ofobstacle velocity vec tor positive or smaller than the
             %negative part or simply, if teh angle always positive, are
             %bigger than either two of them..             
             
             %So, abs(angle) between ObsPos to ObsVel?
%              ObPosVelAng1 = dot(MAC.ObPosBod(1:2,:),MAC.ObVelBod(1:2,:))./(MAC.ObDist.*MAC.ObVelAbs);
%              ObPosVelAng2 = dot(MAC.ObPosBod(1:2,:),-MAC.ObVelBod(1:2,:))./(MAC.ObDist.*MAC.ObVelAbs);
                          
        end
        function CheckDistance(MAC)
            %use the MAC.ObDist properties --> Obstacles Distances
            %check categories? how about manned?
            %use MAC.ObVelAbs?
            for ii = 1:MAC.NumObs(2)
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
                
                if MAC.ObDist(ii) < (MAC.CASDistDat(1,MAC.Cate,MAC.ObsCats(ii))*MAC.AvoTy)
                    MAC.CASFlag(ii+1) = 3;  %escape sphere
                elseif MAC.ObDist(ii) < (MAC.CASDistDat(2,MAC.Cate,MAC.ObsCats(ii))*MAC.AvoTy);
                    MAC.CASFlag(ii+1) = 2;  %deconf sphere
                elseif MAC.ObDist(ii) < (MAC.CASDistDat(3,MAC.Cate,MAC.ObsCats(ii))*MAC.AvoTy)
                    MAC.CASFlag(ii+1) = 1;  %warning sphere
                else
                    MAC.CASFlag(ii+1) = 0;  %do nothing
                end
            end
            MAC.CASFlag(1) = max(MAC.CASFlag(2:MAC.NumObs(2)+1));
        end     
        function WarningSys(MAC)
        %function send nessesary Warning
        %if CASFlag 1-2-3
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
                %Converge-Left, Converge Right, Head on or Take Over
                %first find angle of Obstacle speed form the Vehicle
                %WindAxis
                MAC.ObAngVO(:,ii) = atan2(MAC.ObVelBod(2,ii),MAC.ObVelBod(1,ii));
                
                
                if MAC.ObAngVO(:,ii) >= pi/4 && MAC.ObAngVO(:,ii) <= pi*3/4         %its ConvergeRight, should move
                    MAC.RoW(ii+1) = 1;
                elseif MAC.ObAngVO(:,ii) > pi*3/4 || MAC.ObAngVO(:,ii) < -pi*3/4     %its HeadOn, should move
                    MAC.RoW(ii+1) = 1;
                elseif MAC.ObAngVO(:,ii) < pi/4 && MAC.ObAngVO(:,ii) > -pi/4          %its On the same path, 
                    %check whether is in front or behind, by
                    %obstacle position angle MAC.ObAngBod(ii)
%                     if MAC.ObAngBod(ii) <= pi/2 && MAC.ObAngBod(ii) >= -pi/2           %Its Behind, should move
%                         MAC.RoW(ii+1) = 1;
%                     elseif MAC.ObAngBod(ii) > pi/2 || MAC.ObAngBod(ii) < -pi/2         %its on fornt, stay
%                         MAC.RoW(ii+1) = 0;
%                     end
                    %use velocity magnitude?
                    if MAC.ObVelAbs(ii) < MAC.DMaVel         %Its Behind, should move
                        MAC.RoW(ii+1) = 1;
                    elseif MAC.ObVelAbs(ii) > MAC.DMaVel     %its on front, stay
                        MAC.RoW(ii+1) = 0;
                    else % in an unfortunate even when both have the same speed?
                        if MAC.ObAngVO(:,ii) >= 0           %its ConvergeRight, should move
                            MAC.RoW(ii+1) = 1;
                        else
                            MAC.RoW(ii+1) = 0;
                        end
                    end
                elseif MAC.ObAngVO(:,ii) <= -pi/4 && MAC.ObAngVO(:,ii) >= -pi*3/4        %its ConvergeLeft, no need to move
                    MAC.RoW(ii+1) = 0;
                end
                %E = MAC.NumObs
                %F = MAC.RoW
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
            %Avoid Maneuver, turn to the right, if we consider aircraft
            %dynamics,the decision should be on the edge of VO?, or, just
            %turn decision? or according to max turn (not include n the
            %dynamics?)
            %make velocity vector from the AvoW.. change of velocity per
            %second?
            
            %TurnRight Matrix
            %MatAvo = [cos(MAC.AvoW) -sin(MAC.AvoW);sin(MAC.AvoW) cos(MAC.AvoW)];
            %NewVect = MatAvo*
            MAC.Decision = [-MAC.DMaVel*(sin(MAC.AvoW))^2; MAC.DMaVel*sin(MAC.AvoW)*cos(MAC.AvoW); 0; ...
                            0;0;0];
        end
        function ReadGCS(MAC,TGoVel)
            MAC.TGoVel = TGoVel(1:3,1);
        end
        
        function SetInit(MAC,DBoVel,Heading,FliPath,AvoW,AvoTy)
            MAC.AvoW = AvoW;
            MAC.DBoVel = DBoVel;
            MAC.DMaVel = (sum(DBoVel.^2))^0.5;
            MAC.TGoVel = DBoVel;
            MAC.FliPath  = FliPath;
            MAC.HeaGlo = Heading(3);
            
            %initiate type of avoidance?
            MAC.AvoTy = AvoTy;
        end
        
    end
end
