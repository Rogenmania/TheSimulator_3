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
            if DistToGoal ==0
                ToGoal = [0;0;0];
            else
                ToGoal = ToGoal/((sum((ToGoal).^2))^0.5); %Normalized, just direction
            end
            if MAC.DriftAvo(1) == 0
                ToPath=ToPath;
            else
                ToPath = ToPath/(MAC.DriftAvo(1)); %Normalized, just direction
            end
            %The TGoVel need to head to goal when it is on the path, but to go as soon to the pat when it is not on the path.
            MAC.TGoVel = MAC.MatE2B*(MAC.DMaVel*(ToGoal+ToPath)); %this should work
            %if the change of vel is only velocity based, then we are
            %finish. if it need to be angle and avoidan plane...
            %TheAvoplane is the palne that consisted current Vel vector and
            %the TGoVel vector. What we are searching is the dihedral of
            %this wit the body, or simply the angle in body axis. First off
            %all, we need to find the plane of two vectors --> cross
            %product?
            NAvoPlTGoVel = cross(MAC.TGoVel,[1 0 0]); %--> already in body axis, the nromal also in body,
            %above does not work when its the same. on Y and Z.
            %the Z axis --? [0 0 1]; so the dihedral
            AVoPl = acos((dot(NAvoPlTGoVel,[1/(2)^0.5 0 1/(2)^0.5]))/((sum((NAvoPlTGoVel).^2))^0.5)); %remember unit vector
            %the heading? if.. pitch or head? --> at 45 degree?
            if AVoPl > pi/2 %just to limit
                AVoPl = AVoPl-pi;
            elseif AVoPl < -pi/2
                AVoPl = AVoPl+pi;
            end
            Aa = NAvoPlTGoVel
            Bb = AVoPl
            Cc = dot(NAvoPlTGoVel,[1/(2)^0.5 0 1/(2)^0.5])
            
            if AVoPl >= -pi/4 && AVoPl <= pi/4 %
                %then go change heading
                RollCom = AVoPl;
                PitchCom = 0; %Current Pitch
                %only need the X and Y of TGOvel
                YawCom = acos(dot([MAC.TGoVel(1) 0 MAC.TGoVel(3)],[MAC.TGoVel(1) MAC.TGoVel(2) MAC.TGoVel(3)])/...
                         ((MAC.TGoVel(1)^2+MAC.TGoVel(3)^2)^0.5*(MAC.TGoVel(1)^2+MAC.TGoVel(2)^2+MAC.TGoVel(3)^2)^0.5));
            else
                RollCom = AVoPl-pi/2;
                %go change pitch
                YawCom = 0; %current Yaw
                PitchCom = acos(dot([0 MAC.TGoVel(2) MAC.TGoVel(3)],[MAC.TGoVel(1) MAC.TGoVel(2) MAC.TGoVel(3)])/...
                           ((MAC.TGoVel(2)^2+MAC.TGoVel(3)^2)^0.5*(MAC.TGoVel(1)^2+MAC.TGoVel(2)^2+MAC.TGoVel(3)^2)^0.5));
            end
            
            if RollCom > pi/2  %just to limit
                RollCom = RollCom-pi;
            elseif RollCom < -pi/2
                RollCom = RollCom+pi;
            end
            
            MAC.Decision(:,1) = [MAC.TGoVel-MAC.VelBo; ...
                                 RollCom; PitchCom; YawCom];
                             
                             
            
            sdgsdev
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































