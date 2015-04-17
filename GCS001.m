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
            MAC.DriftAvo = MAC.Point2Vect(MAC.FliPath,MAC.PosGlo,3);
            
            %Velocity directly to goal (EndWP/ End of Path? / special
            %algorithm for drift nullification?
            ToGoal = MAC.FliPath(:,2)-MAC.PosGlo;
            MAC.TGoVel = MAC.MatE2B*(MAC.DMaVel*(ToGoal/((sum(ToGoal.^2))^0.5)));
            %Ddd = MAC.DriftAvo
            ToPath = ToGoal+[MAC.DriftAvo(2);MAC.DriftAvo(3);0];
            %ToPath = (MAC.FliPath(:,1)+(MAC.FliPath(:,2)-MAC.FliPath(:,1))/1.5)-MAC.PosGlo;
            
            MAC.TGoVel = MAC.MatE2B*(MAC.DMaVel*(ToPath/((sum(ToPath.^2))^0.5)));

            MAC.Decision(:,1) = [MAC.TGoVel-MAC.VelBo; 0;0;0];
            
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































