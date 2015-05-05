classdef FDRecord < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Name = 'untitled'
        Time = 0
        TimeStep
        TimeEnd
        Data
        SizeData
        Data2
        Elapsed = 1
        Number = 1
        
    end
    
    methods
        function FDR = FDRecord(SaveFileName,SimTi,TiSt,Dim)
            FDR.Name = SaveFileName;
            FDR.Data = zeros(Dim, round(SimTi/TiSt));
            size(FDR.Data);
            FDR.SizeData = size(FDR.Data);
            FDR.Data2 = zeros(Dim,50);
            FDR.TimeStep = TiSt;
            FDR.Time = zeros(1,round(SimTi/TiSt));
            FDR.TimeEnd = SimTi;
        end
        function AddRecord(FDR,Datanya)
            size(FDR.Data);
            FDR.Data(:,FDR.Elapsed) = Datanya; 
            FDR.Time(:,FDR.Elapsed+1) = FDR.Time(:,FDR.Elapsed) + FDR.TimeStep;
            FDR.Elapsed = FDR.Elapsed + 1;
        end
        function AddRecord2(FDR,Datanya)
            FDR.Data2(:,FDR.Number) = Datanya;
            FDR.Number = FDR.Number + 1;
        end
    end
    
end

