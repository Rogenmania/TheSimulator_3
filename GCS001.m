classdef GCS001 < Computer
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
        %Predictor
        
        %OnVO
        VOStat
        VOtype 
        VONumber
        VOOrigin
        VectorNear
        VectorFar
        Vector
        VOBasAngLim = 0;
        VOBasAng
        VOLuDa
        
        ObPosVO
        ObVelVO 
        ObAngVO
        
        VRelBoVO
        VRelTGoVO
        
        RoW
        
        CASVelDes
        
        
    end
    
    methods
        function MAC = GCS001(DBoVel,Heading,FliPath)
            MAC = MAC@Computer(DBoVel,Heading,FliPath);
            
            %load waypoints from matfile? textfile? ==> scenario make it
            %flight path and design velocity?
            

        end
        
        function GCSRun(MAC)
           %whole computations
           %GCS meant to control aircraft to follow the predefined
           %waypoints (FliPath)
           TGVelo(MAC);
           %Keep Velo
           
           %Turn to ToGoal
           
        end
        function TGVelo(MAC)
            %use current position and destination, create flight path,
            %define TGVel...

            %Distance from initial/setup flight path
            MAC.DriftAvo = MAC.Point2Vect(MAC.FliPath,MAC.PosGlo,3); %the result is global!
            %Velocity directly to goal (EndWP/ End of Path? / special
            %algorithm for drift nullification?
            ToGoal = MAC.FliPath(:,2)-MAC.PosGlo; %  but it isstill global orientation
            ToPath = [MAC.DriftAvo(2); MAC.DriftAvo(3); MAC.DriftAvo(4)]-MAC.PosGlo; %  but it isstill global orientation
            DistToGoal = (sum((ToGoal).^2))^0.5;
                        
            %The TGoVel need to head to goal when it is on the path, but to go as soon to the pat when it is not on the path.
            Fact = 0.001; %meaning point 100 meters ahead?
            if DistToGoal < 0.0001 && MAC.DriftAvo(1) < 0.0001
               MAC.TGoVel = MAC.VelGlo;
            else
               MAC.TGoVel = MAC.DMaVel*(Fact*ToGoal+ToPath)/((sum((Fact*ToGoal+ToPath).^2))^0.5);
            end
            Aaa = MAC.DriftAvo(1);
            Bbb = (MAC.TGoVel)';
            Ccc = ToPath';
            
            %if the change of vel is only velocity based, then we are
            %finish. if it need to be angle and avoidan plane...
            %TheAvoplane is the palne that consisted current Vel vector and
            %the TGoVel vector. What we are searching is the dihedral of
            %this wit the body, or simply the angle in body axis. First off
            %all, we need to find the plane of two vectors --> cross
            %product?
            
            %change the roll until a desired angle, then turn. --> to
            %support the theory, just keep it as yawing? or just turn...
            %Actually, rolling will be just a matrix, while turning and
            %turn rate, will be the rate..... THE ROLL is just a part of
            %the matrix, conducted.. first? first
            %for the GCS, teh avoidance plane is simply the plane where
            %TGoal and vector 100 lies. the normal of this vectors:
            %BUT in Body axis.. 
            NAvoPlTGoVel = cross(MAC.MatE2B*MAC.TGoVel,[1 0 0]); %--> already in body axis, the nromal also in body,
            
            %above does not work when its the same. on Y and Z.
            %the Z axis --? [0 0 1]; so the roll angle:
            NorLeng = sum((NAvoPlTGoVel).^2)^0.5;
            if NorLeng == 0
                AVoPl = 0; %dont move... actualy, any plane will do, but we just keep it as dont move
            else
                AVoPl = acos((dot(NAvoPlTGoVel,[0 0 1]))/((sum((NAvoPlTGoVel).^2))^0.5)); %remember unit vector
                %since there are no pos/neg, use the sign of the Y in TGOVel
                AVoPl = sign(NAvoPlTGoVel(2))*AVoPl;
            end
            
            if AVoPl > pi/2 %just to limit, but seems not possible
                AVoPl = AVoPl-pi;
            elseif AVoPl < -pi/2 % also not possible. the result will always positive
                AVoPl = AVoPl+pi;
            end
            %above result is always the smallest angle. need to be cautious
            %about the direction of the next turning.
            RollCom = AVoPl;
            PitchCom = 0; %Current Pitch
            BodTGoVel = MAC.MatE2B*MAC.TGoVel;
            YawCom = acos(dot([1 0 0],BodTGoVel')/...
                         ((sum(BodTGoVel.^2))^0.5));
            %also always positive. How to decide the turning direction? to
            %the driection of TGoVel of course.
            %YawCom = -sign(BodTGoVel(2))*YawCom;
            
%             if RollCom > pi/2  %just to limit
%                 RollCom = RollCom-pi;
%             elseif RollCom < -pi/2
%                 RollCom = RollCom+pi;
%             end
            Rol =RollCom*57.3;
            Pit =PitchCom*57.3;
            Yaw =YawCom*57.3;
            
            MAC.Decision(:,1) = [MAC.TGoVel-MAC.VelGlo; ...
                                 RollCom; PitchCom; YawCom];
            Aaa = MAC.Decision.*[1;1;1;57.3;57.3;57.3]
            %Ro = csvread('DecRec.txt');
            %csvwrite('DecRec.txt',[Ro ; Aaa'])

            %MAC.Decision(:,1) = [0;0;0; 0;0;0];
        end
        
        function ReadCAS(MAC,CASFlag,CASDesicion,Interupt)
            %GCS(ii).ReadCAS(CAS(ii).CASFlag,CAS(ii).Decision,CAS(ii).InteruptN)
            
            MAC.CASFlag = CASFlag; %for warning input

            if Interupt == 1
                MAC.Decision = CASDesicion;
            end
            
            
            
        end
        
        function SetInit(MAC,DBoVel,Heading,FliPath)
            MAC.DBoVel = DBoVel;
            MAC.DMaVel = (sum(DBoVel.^2))^0.5;
            MAC.TGoVel = DBoVel;
            MAC.FliPath  = FliPath;
            MAC.HeaGlo = Heading(3);
        end
    end
end































