classdef Sensor < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Accuracy = 1;
        Error = 0;
        Range
        TrueData = [0;0;0];
        MeasureData = 0
    end
    
    methods
        function US = Sensor(Acc,Err,Rang,iData)
            US.Accuracy = Acc;
            US.Error = Err;
            US.Range = Rang;
            US.TrueData = iData;
            US.MeasureData = US.TrueData;
        end
        function Clear(US)
           US.TrueData = [];
           US.MeasureData = []; 
        end
        function Sense(US,NewData)
            US.TrueData = NewData;
            US.MeasureData = US.TrueData;
        end
        function SenseAdd(US,NewData)
            US.TrueData = [US.TrueData NewData];
            US.MeasureData = US.TrueData;
        end
        function SetInit(US,iData)
            US.TrueData = iData;
            US.MeasureData = US.TrueData;
        end
    end
    
end

