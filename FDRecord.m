classdef FDRecord < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Name = 'untitled'
        Time = 0
        TimeStep
        TimeEnd
        Data
        Elapsed = 1
    end
    
    methods
        function FDR = FDRecord(SaveFileName,SimTi,TiSt,Dim)
            FDR.Name = SaveFileName;
            FDR.Data = zeros(Dim,SimTi/TiSt);
            FDR.TimeStep = TiSt;
            FDR.Time = zeros(1,SimTi/TiSt);
            FDR.TimeEnd = SimTi;
        end
        function AddRecord(FDR,Datanya)
            FDR.Data(:,FDR.Elapsed) = Datanya;
            FDR.Time(:,FDR.Elapsed+1) = FDR.Time(:,FDR.Elapsed) + FDR.TimeStep;
            FDR.Elapsed = FDR.Elapsed + 1;
        end
    end
    
end

