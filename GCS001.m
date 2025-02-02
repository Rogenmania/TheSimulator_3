classdef GCS001 < Computer
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        %First Input
        CASFlag = 0
        TGTurnR8
        TGvTP
    end
    
    methods
        function MAC = GCS001(DBoVel,Attitude,FliPath)
            MAC = MAC@Computer(DBoVel,Attitude,FliPath);

        end
        function GCSRun(MAC)
            ToGoal = MAC.FliPathBod(:,2); %  On Body Axis
            DistToGoal = (sum((ToGoal).^2))^0.5;
            
            MAC.DriftAvo = MAC.Point2Vect(MAC.FliPathBod,[0;0;0],3); %the result is body!
            ToPath = [MAC.DriftAvo(2); MAC.DriftAvo(3); MAC.DriftAvo(4)]; %  but it isstill global orientation
                                   
            %The TGoVel need to head to goal when it is on the path, but also to go as soon to the path when it is not on the path.
            %meaning more to eliminate drift then going to goal? --> MAC.TGvTP
            if DistToGoal < 0.001 || MAC.DriftAvo(1) < 0.001
               MAC.TGoVel = MAC.VelBo;
            else
               MAC.TGoVel = MAC.DMaVel*(MAC.TGvTP*ToGoal+ToPath)/((sum((MAC.TGvTP*ToGoal+ToPath).^2))^0.5);
            end
            %change the TGoVel to global axis, B2E
            
            MAC.TGoVel = MAC.MatB2E*MAC.TGoVel;
            TGoAng = MAC.Vect2Angls(MAC.TGoVel);
            
            MAC.Decision(:,1) = [MAC.TGoVel; TGoAng];
        end
        function GCSRun_x(MAC)
            %use current position and destination, create flight path,
            %define TGVel... --> from goal (toGoal) and drift from path (ToPath)
            %Distance from initial/setup flight path
            ToGoal = MAC.FliPathBod(:,2); %  On Body Axis
            DistToGoal = (sum((ToGoal).^2))^0.5;
            
            MAC.DriftAvo = MAC.Point2Vect(MAC.FliPathBod,[0;0;0],3); %the result is body!
            ToPath = [MAC.DriftAvo(2); MAC.DriftAvo(3); MAC.DriftAvo(4)]; %  but it isstill global orientation
                                   
            %The TGoVel need to head to goal when it is on the path, but also to go as soon to the path when it is not on the path.
            %meaning more to eliminate drift then going to goal? --> MAC.TGvTP
            if DistToGoal < 0.00001 || MAC.DriftAvo(1) < 0.00001
               MAC.TGoVel = MAC.VelBo;
            else
               MAC.TGoVel = MAC.DMaVel*(MAC.TGvTP*ToGoal+ToPath)/((sum((MAC.TGvTP*ToGoal+ToPath).^2))^0.5);
            end
            
            %TheAvoplane is the plane that consisted current Vel [1; 0; 0] and
            %the TGoVel vector. --> cross product is the normal of this plane
            NAvoPlTGoVel = cross(MAC.TGoVel,[1; 0; 0]); %--> already in body axis, the nromal also in body,
            %above does not work when its the same. on Y and Z.
            %so to anticipates:
            NorLeng = sum((NAvoPlTGoVel).^2)^0.5;
            if NorLeng == 0
                AVoPl = 0; %dont move... actualy, any plane will do, but we just keep it as dont move
            else
                %angle with the Z axis [0 0 1];
                AVoPl = acos((dot(NAvoPlTGoVel,[0; 0; 1]))/(NorLeng)); %remember unit vector
                % the angle will always be the smallest. Hence, another step is
                % required to differentiate e.g. 45 deg and -45 deg (currently both read as 45)
                if abs(AVoPl) < 0.0001
                   AVoPl = 0;
                end
                
                AVoPl = sign(NAvoPlTGoVel(2))*AVoPl;
            end
            
            while AVoPl > pi/2 || AVoPl < -pi/2
                if AVoPl > pi/2 %just to limit, 
                    AVoPl = AVoPl-pi;
                elseif AVoPl < -pi/2 
                    AVoPl = AVoPl+pi;
                end
            end

            RollCom = AVoPl;
            PitchCom = 0; %
            YawCom = acos(dot(MAC.TGoVel,[1; 0; 0])/((sum(MAC.TGoVel.^2))^0.5)); %scalar!
            if abs(YawCom) < 0.0001
               YawCom = 0;
            end
            YawCom = -sign(MAC.TGoVel(2))*YawCom; %the direction of yawing --> neg is mistery
            
            MAC.Decision(:,1) = [0; 0; 0; RollCom; PitchCom; YawCom];
            %Aaa = MAC.Decision.*[1;1;1;57.3;57.3;57.3];
            %MAC.Decision(:,1) = [0;0;0; 0;0;0];
        end
        
        function ReadCAS(MAC,CASFlag,CASDesicion,Interupt)
            %GCS(ii).ReadCAS(CAS(ii).CASFlag,CAS(ii).Decision,CAS(ii).InteruptN)
            MAC.CASFlag = CASFlag; %for warning input
            if Interupt >= 1
                MAC.Decision = CASDesicion;
            end
            
        end
        
        function SetInit(MAC,DBoVel,Attitude,FliPath,NTurnRate,GoalvPath)
            MAC.DBoVel = DBoVel;
            MAC.DMaVel = (sum(DBoVel.^2))^0.5;
            MAC.TGoVel = DBoVel;
            MAC.FliPath  = FliPath;
            MAC.AttGlo = Attitude;
                        
            MAC.TGTurnR8 = NTurnRate;
            MAC.TGvTP = GoalvPath;

        end
    end
end































