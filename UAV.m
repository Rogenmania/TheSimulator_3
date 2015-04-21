classdef UAV < handle
    %UAV for Collision Avoidance System simulation - 2D
    %In two dimension!
    
    properties
        Mass
        MAC
        Span
        WArea
        SepRad          %Radius of Separation
        BodAcc = [0;0;0]
        BodVel = [0;0;0]         %Velocity on Body Axis
        BodPQR = [0;0;0]
        WinAtt = [0;0;0]
        BodEul = [0;0;0]
        GloPos          %Global Position
        GloVel          %Global Velocity
        GloAtt         
        GloAtti
        InputDV
        InputDA
        TrWarn = 0;
    end
    
    methods
        function UAVC = UAV(MASS,MAC,SP,WA,SR,iGPOS,iBVEL,iGATT)
            UAVC.Mass = MASS;
            UAVC.MAC = MAC;
            UAVC.Span = SP;
            UAVC.WArea = WA;
            UAVC.SepRad = SR;          %Radius of Separation
            UAVC.GloPos = iGPOS;         %Global Position
            UAVC.GloAtti = iGATT;
            UAVC.GloAtt = iGATT;
            MatB2E = [cos(UAVC.GloAtt(3)) -sin(UAVC.GloAtt(3)) 0; 
                      sin(UAVC.GloAtt(3))  cos(UAVC.GloAtt(3)) 0;
                      0                    0                   1];
            UAVC.GloVel = MatB2E*iBVEL;
            UAVC.BodVel = iBVEL;
        end
        %function update velocity, only specific, add warning status
        function InputD(UAVC,DVEL,Warn) %Only for 2D
            UAVC.InputDV = DVEL(1:3);
            UAVC.InputDA = DVEL(4:6);
            %read the warnings
            UAVC.TrWarn = Warn;
        end
        %function state after 1 time step, only specific
        function MoveTimeD_1(UAVC,TiSt)
            
            NewVel = UAVC.BodVel + UAVC.InputDV(1:3);
            UAVC.WinAtt = [0;0;0];
            %DDd = UAVC.GloAtti 
            DelRot = atan2(NewVel(2),NewVel(1));
            UAVC.GloAtt = UAVC.GloAtt + [0; 0; DelRot];
            MatDelRot = [cos(DelRot) -sin(DelRot); 
                         sin(DelRot)  cos(DelRot)];
            UAVC.GloVel = [MatDelRot*UAVC.GloVel(1:2); 0];
            UAVC.GloPos = UAVC.GloPos + UAVC.GloVel*TiSt;

            
        end
        function MoveTimeD_3(UAVC,TiSt)%A threeD movement, but linear
            %first roll?
            GRol = 1; %maybe this need to be big. or should it be instanteneous? 
            GPit = 1;
            GYaw = 1;
            RolRate = 30/180*pi; %deg/sec
            PitRate = 30/180*pi;
            YawRate = 30/180*pi;
            RolTi = UAVC.InputDA(1)*GRol*RolRate*TiSt;
            PitTi = UAVC.InputDA(2)*GPit*PitRate*TiSt; 
            YawTi = UAVC.InputDA(3)*GYaw*YawRate*TiSt; 
            
            Rot3D = [cos(PitTi)*cos(YawTi) cos(PitTi)*sin(YawTi) -sin(PitTi);
                    -cos(RolTi)*sin(YawTi)+sin(RolTi)*sin(PitTi)*cos(YawTi) cos(RolTi)*cos(YawTi)+sin(RolTi)*sin(PitTi)*sin(YawTi) sin(RolTi)*cos(PitTi);
                     sin(RolTi)*sin(YawTi)+cos(RolTi)*sin(PitTi)*cos(YawTi) -sin(RolTi)*cos(YawTi)+cos(RolTi)*sin(PitTi)*sin(YawTi) cos(RolTi)*cos(PitTi)];
            UAVC.GloVel= Rot3D*UAVC.GloVel;
            UAVC.GloPos = UAVC.GloPos + UAVC.GloVel*TiSt;
            UAVC.GloAtt = UAVC.GloAtt + [RolTi; PitTi; YawTi];
            
            %reach or put these input to zero by turning? So InoputDA -
            %will control the turn in three axis, but how abot trolling
            %first?
            
            %[T,NEPo]=ode45(@(t,y) NavEq(t,UVWo,VTPo),TiStCh,NEPo);
            
            
        end
        
        function SetInit(UAVC,iGPOS,iBVEL,iGATT)
            UAVC.GloPos = iGPOS;         %Global Position
            UAVC.GloAtt = iGATT;
            MatB2E = [cos(UAVC.GloAtt(3)) -sin(UAVC.GloAtt(3)) 0; 
                      sin(UAVC.GloAtt(3))  cos(UAVC.GloAtt(3)) 0;
                      0                    0                   1];
            UAVC.GloVel = MatB2E*iBVEL;
            UAVC.BodVel = iBVEL;
        end
        
    end
    
end















