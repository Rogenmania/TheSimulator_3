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
            RotMatP = [cos(-UAVC.GloAtt(3)) sin(-UAVC.GloAtt(3)) 0; -sin(-UAVC.GloAtt(3)) cos(-UAVC.GloAtt(3)) 0 ; 0 0 1]; %3D turning to heading
            RotMatT = [cos(-UAVC.GloAtt(2)) 0 -sin(-UAVC.GloAtt(2)); 0 1 0; sin(-UAVC.GloAtt(2)) 0 cos(-UAVC.GloAtt(2))];
            RotMatV = [1 0 0; 0 cos(-UAVC.GloAtt(1)) sin(-UAVC.GloAtt(1)); 0 -sin(-UAVC.GloAtt(1)) cos(-UAVC.GloAtt(1))];
            MatB2E = RotMatP*RotMatT*RotMatV;
            UAVC.GloVel = MatB2E*iBVEL;
            UAVC.BodVel = iBVEL;
        end
        %function update velocity, only specific, add warning status
        function InputD(UAVC,DVEL,Warn) 
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
        
        function MoveTimeD_3(UAVC,TiSt)
            %RotMatP = [cos(-UAVC.GloAtt(3)) sin(-UAVC.GloAtt(3)) 0; -sin(-UAVC.GloAtt(3)) cos(-UAVC.GloAtt(3)) 0 ; 0 0 1]; %3D turning to heading
            %RotMatT = [cos(-UAVC.GloAtt(2)) 0 -sin(-UAVC.GloAtt(2)); 0 1 0; sin(-UAVC.GloAtt(2)) 0 cos(-UAVC.GloAtt(2))];
            %RotMatV = [1 0 0; 0 cos(-UAVC.GloAtt(1)) sin(-UAVC.GloAtt(1)); 0 -sin(-UAVC.GloAtt(1)) cos(-UAVC.GloAtt(1))];
            %MatB2E = RotMatP*RotMatT*RotMatV;
            %UAVC.BodVel = UAVC.InputDV;
            UAVC.GloVel = UAVC.InputDV;
            UAVC.GloAtt = [0;
                          -atan2(UAVC.GloVel(3),((UAVC.GloVel(2)^2+UAVC.GloVel(1)^2)^0.5)); 
                           atan2(UAVC.GloVel(2),UAVC.GloVel(1))];
            
            UAVC.GloPos = UAVC.GloPos + UAVC.GloVel*TiSt;
        end
        
        function MoveTimeD_3_x(UAVC,TiSt)%A threeD movement, but linear
            %first roll?
            RotMatP = [cos(-UAVC.GloAtt(3)) sin(-UAVC.GloAtt(3)) 0; -sin(-UAVC.GloAtt(3)) cos(-UAVC.GloAtt(3)) 0 ; 0 0 1]; %3D turning to heading
            RotMatT = [cos(-UAVC.GloAtt(2)) 0 -sin(-UAVC.GloAtt(2)); 0 1 0; sin(-UAVC.GloAtt(2)) 0 cos(-UAVC.GloAtt(2))];
            RotMatV = [1 0 0; 0 cos(-UAVC.GloAtt(1)) sin(-UAVC.GloAtt(1)); 0 -sin(-UAVC.GloAtt(1)) cos(-UAVC.GloAtt(1))];
            MatB2E = RotMatP*RotMatT*RotMatV;
           
            %turn roll in instant, yaw slowly. Vel diff is not used, but
            %consequences
            %turning rtae gain = 1, depend on the input diff
             %UAVC.GloAtt = UAVC.GloAtt + UAVC.InputDA.*[0;1;TiSt];
             Tt = 1;
             %turn the vel as consequences
             RMatP = [cos(UAVC.InputDA(3)*Tt) sin(UAVC.InputDA(3)*Tt) 0;
                   -sin(UAVC.InputDA(3)*Tt) cos(UAVC.InputDA(3)*Tt) 0; 
                   0 0 1];
             RMatT = [cos(UAVC.InputDA(2)) 0 -sin(UAVC.InputDA(2));
                      0 1 0;
                      sin(UAVC.InputDA(2)) 0 cos(UAVC.InputDA(2))];  
             RMatV = [1 0 0;
                      0  cos(UAVC.InputDA(1)) sin(UAVC.InputDA(1));
                      0 -sin(UAVC.InputDA(1)) cos(UAVC.InputDA(1))];
             eGloVel = MatB2E*RMatV*RMatT*RMatP*UAVC.BodVel;
             deGloVel = eGloVel-UAVC.GloVel;
             eGloAng = [0;
                       -atan2(eGloVel(3),((eGloVel(2)^2+eGloVel(1)^2)^0.5)); 
                        atan2(eGloVel(2),eGloVel(1))];
              
                    
             deGloAng = eGloAng-UAVC.GloAtt;
             
             UAVC.GloVel= UAVC.GloVel + deGloVel;
             UAVC.GloAtt = UAVC.GloAtt + [UAVC.InputDA(1)-UAVC.GloAtt(1); deGloAng(2); deGloAng(3)]; 
             
             for ii = 1:3
                 while UAVC.GloAtt(ii) > pi || UAVC.GloAtt(ii) < -pi
                     if UAVC.GloAtt(ii) > pi
                         UAVC.GloAtt(ii) = UAVC.GloAtt(ii)-2*pi;
                     elseif UAVC.GloAtt(ii) < -pi
                         UAVC.GloAtt(ii) = UAVC.GloAtt(ii)+2*pi;
                     end
                 end
             end
             
             
             %Aaa = UAVC.GloAtt*57.3

            
            
            %UAVC.GloVel= UAVC.GloVel+UAVC.InputDV;
            UAVC.GloPos = UAVC.GloPos + UAVC.GloVel*TiSt;
            %UAVC.GloAtt = UAVC.GloAtt + UAVC.InputDA;
            
            %reach or put these input to zero by turning? So InoputDA -
            %will control the turn in three axis, but how abot trolling
            %first?
            
            %[T,NEPo]=ode45(@(t,y) NavEq(t,UVWo,VTPo),TiStCh,NEPo);
            
            
        end
        function SetInit(UAVC,iGPOS,iBVEL,iGATT)
            UAVC.GloPos = iGPOS;         %Global Position
            UAVC.GloAtt = iGATT;
            RotMatP = [cos(-iGATT(3)) sin(-iGATT(3)) 0; -sin(-iGATT(3)) cos(-iGATT(3)) 0 ; 0 0 1]; %3D turning to heading
            RotMatT = [cos(-iGATT(2)) 0 -sin(-iGATT(2)); 0 1 0; sin(-iGATT(2)) 0 cos(-iGATT(2))];
            RotMatV = [1 0 0; 0 cos(-iGATT(1)) sin(-iGATT(1)); 0 -sin(-iGATT(1)) cos(-iGATT(1))];
            MatB2E = RotMatP*RotMatT*RotMatV;
            UAVC.GloVel = MatB2E*iBVEL;
            UAVC.BodVel = iBVEL;
        end
        
    end
    
end















