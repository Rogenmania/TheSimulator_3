classdef Computer < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        %First Input / Setting 
        DBoVel
        DMaVel
        FliPath
        FliPathAng
        AbsType
        AbsPoin
        AvoPath = [0 50]
        
        %Calculated State
        OnOff
        TGoVel
        TPaVel
        DriftAvo
        
        %BoEndWP
        MatE2B
        MatP2E
        MatB2E
        MatW2B
        %Output for Dynamic, 2 calculation?
        Decision      %Decision Delta V
        %Manager Function
        WUse = 1;
        %Obstacle sensor
        ObPosGlo
        ObVelGlo
        NumObs

        %Own Sensor Input
        VelBo = [0;0;0]
        AttWi
        PosGlo
        VelGlo
        HeaGlo
    end
    
    methods
        function MAC = Computer(DBoVel,Heading,FliPath)
                      %input
            MAC.DBoVel = DBoVel;
            MAC.DMaVel = (sum(DBoVel.^2))^0.5;
            MAC.TGoVel = DBoVel;
            MAC.FliPath  = FliPath;
            MAC.HeaGlo = Heading;
            
               
        end
        function InputSensor(MAC,SensGPos,SensGVel,SensVel,SensAtt,SensObPos,SensObVel)
            %Own State
            MAC.PosGlo = SensGPos;%(X,Y,Z), should be GPS output
            MAC.VelGlo = SensGVel; %maybe should be indirect
            %MAC.HeaGlo = atan2(SensGVel(2),SensGVel(1));
            MAC.VelBo = SensVel;%Vxb, Vyb, Vzb
            MAC.AttWi = SensAtt;
            
            %Obstacle, with additional, flight path limit!
            MAC.ObPosGlo = SensObPos; % ]
            MAC.ObVelGlo = SensObVel; % ]
            MAC.NumObs = size(MAC.ObPosGlo);
            MAC.CalcSensor()
       
        end
        function CalcSensor(MAC)
            %Calculate Heading 3D, from ObVelGlo ?
            
            MAC.HeaGlo = MAC.Vect2Angls(MAC.VelGlo);
            
            %Global to Body Rotational Matrix 
            %earth to Body, just turn backwards... 
            %we need this for the obstacles. or other vector like ToGoal
            MAC.MatE2B = MAC.RotMat(MAC.HeaGlo,3); 
            %Path to Body 
            
            %Path to Global
            MAC.FliPathAng = MAC.Vect2Angls(MAC.FliPath(:,2)-MAC.FliPath(:,1));
            MAC.MatP2E = MAC.RotMat(MAC.FliPathAng,3);

        end
        
        %function on Guidance selection?
        %Make WUse only changeable by CAS...
        
        
      end
    methods(Static)
        function [RelData AbsData] = MakeEnvelope(VEPoint,VEDensity,VEType,DBoVel)
            switch VEType
                case 1 %'square' 
                    %if square, the VEP is square sides semi-length
                    Xmin = -VEPoint;
                    Xmax = VEPoint;
                    Ymin = -VEPoint;
                    Ymax = VEPoint;  
                    Xx = Xmin:VEDensity:Xmax;
                    Yy = Ymin:VEDensity:Ymax;
                    JumX = length(Xx);
                    JumY = length(Yy);
                    for ii = 1:JumX
                        for jj = 1:JumY
                            RelData(:,(ii-1)*JumY+jj) = [Xx(ii);Yy(jj)];
                        end
                    end      
                case 2 %'fan'    
                    %if fan, The VEP is the Vel- Vel+, and AngleMin AngleMax
                    Xmin = cos(max(VEPoint(3:4)))*(VEPoint(1)+DBoVel(1));
                    Xmax = VEPoint(2)+DBoVel(1);
                    Ymax = sin(VEPoint(4))*(VEPoint(2)+DBoVel(1));
                    Ymin = sin(VEPoint(3))*(VEPoint(2)+DBoVel(1));
                    Xx = Xmin:VEDensity:Xmax;
                    Yy = Ymin:VEDensity:Ymax;
                    JumX = length(Xx);
                    JumY = length(Yy);
                    aa = 1;
                    bb = 1;
                    for ii = 1:JumX
                        for jj = 1:JumY
                            %[ii jj MAC.VEPoint+[MAC.DBoVel(1) MAC.DBoVel(1) 0 0]]
                            InOut = Computer.IX2D(([Xx(ii);Yy(jj)]),VEPoint+[DBoVel(1) DBoVel(1) 0 0],2,1);
                            if InOut ~= 1
                                continue
                            end
                            RelData(:,(aa-1)*JumY+bb) = [Xx(ii)-DBoVel(1);Yy(jj)];
                            bb = bb+1;    
                        end
                        aa = aa+1;
                    end
                case 3 %'circle' 
                    %if circle, The VEP is the Radius
                    Xmin = -VEPoint(1);
                    Xmax = VEPoint(1);
                    Ymin = -VEPoint(1);
                    Ymax = VEPoint(1);
                    Xx = Xmin:VEDensity:Xmax;
                    Yy = Ymin:VEDensity:Ymax;
                    JumX = length(Xx);
                    JumY = length(Yy);
                    aa = 1;
                    bb = 1;
                    for ii = 1:JumX
                        for jj = 1:JumY
                            InOut = Computer.IX2D([Xx(ii);Yy(jj)],[DBoVel [VEPoint; 0]],3,1);
                            if InOut ~= 1
                                continue
                            end
                            RelData(:,(aa-1)*JumY+bb) = [Xx(ii);Yy(jj)];
                            bb = bb+1;    
                        end
                        aa = aa+1;
                    end
                otherwise     
                    %if polygon, The VEP is points encircle it, with Vel
                    %point as origin, BodyAxis
                    Xmin = min(VEPoint(1,:));
                    Xmax = max(VEPoint(1,:));
                    Ymin = min(VEPoint(2,:));
                    Ymax = max(VEPoint(2,:));
                    Xx = Xmin:VEDensity:Xmax;
                    Yy = Ymin:VEDensity:Ymax;
                    JumX = length(Xx);
                    JumY = length(Yy);
                    aa = 1;
                    bb = 1;
                    for ii = 1:JumX
                        for jj = 1:JumY
                            InOut = Computer.IX2D(([Xx(ii);Yy(jj)]-DBoVel),(VEPoint),4,1);
                            if InOut ~= 1
                                continue
                            end
                            RelData(:,(aa-1)*JumY+bb) = [Xx(ii);Yy(jj)];                            
                            bb = bb+1;    
                        end
                        aa = aa+1;
                    end                     
            end
            %Dd = size(MAC.XYData)
            AbsData = RelData+[DBoVel(1) 0;0 DBoVel(2)]*ones(size(RelData));
            
        end
        function LuDa = IX2D(TestPoint,Points,Type,Behave)
            %check if the end of vel vector of OwnAircraft fall in an area
            %use points of the cone --> usefull maybe in 3D
            Numpo = size(Points);
            NumTe = size(TestPoint);
            LuDa = zeros(1,NumTe(2));
            
            for jj = 1:NumTe(2)
                switch Type
                    case 1   % triangle
                        VOri  = Points(:,2)  ;     %origin of VO01
                        Vect1 = Points(:,1)-VOri;   %vector1 from CPo01
                        Vect2 = Points(:,3)-VOri;   %vector2 from CPo01
                        
                        CAa = (det([TestPoint(:,jj),Vect2])-det([VOri,Vect2]))/det([Vect1,Vect2]);
                        CBb = -(det([TestPoint(:,jj),Vect1])-det([VOri,Vect1]))/det([Vect1,Vect2]);
                        
                        if Behave < 0
                            if CAa > 0 && CBb > 0 %&& (CAa+CBb) <= 1 %|| (CAa+CBb) <= 0.00; %if inside
                                LuDa(jj) = 1;
                            else
                                LuDa(jj) = 0;
                            end
                        else
                            if CAa >= 0 && CBb >= 0 %&& (CAa+CBb) <= 1 %|| (CAa+CBb) <= 0.00; %if inside
                                LuDa(jj) = 1;
                            else
                                LuDa(jj) = 0;
                            end
                        end
                        
                        
                    case 2 % Fan!
                        %if fan, The VEP is the Vel- Vel+, and AngleMin AngleMax
                        %Limit Magnitude
                        %Limit Angle
                        Magn = (sum(TestPoint(:,jj).^2))^0.5;
                        Grad = TestPoint(2,jj)/TestPoint(1,jj);
                        if Behave < 0
                            if Magn > Points(1) && Magn < Points(2) && ...
                                    Grad > tan(Points(3)) && Grad < tan(Points(4))
                                LuDa(jj) = 1;
                            else
                                LuDa(jj) = 0;
                            end
                        else
                            if Magn >= Points(1) && Magn <= Points(2) && ...                                
                                    Grad >= tan(Points(3)) && Grad <= tan(Points(4))
                            LuDa(jj) = 1;
                            else
                                LuDa(jj) = 0;
                            end
                            
                        end
                        
                        
                    case 3  %a Circle!
                        Cenc = Points(:,1);
                        Radc = Points(1,2);
                        
                        RadPo = (sum((TestPoint(:,jj)-Cenc).^2))^0.5;
                        if Behave < 0
                            if RadPo < Radc
                                LuDa(jj) = 1;
                            else
                                LuDa(jj) = 0;
                            end
                        else
                            if RadPo <= Radc
                                LuDa(jj) = 1;
                            else
                                LuDa(jj) = 0;
                            end
                        end
                        
                    case 4   %Polygon
                        %close the loop
                        Points(:,Numpo(2)+1) = Points(:,1);
                        Points(:,Numpo(2)+2) = Points(:,2);
                        
                        for ii = 2:Numpo(2)+1
                            VOri  = Points(:,ii);          %origin of VO01
                            Vect1 = Points(:,ii-1)-VOri;   %vector1 from CPo01
                            Vect2 = Points(:,ii+1)-VOri;   %vector2 from CPo01
                            
                            CAa = (det([TestPoint(:,jj),Vect2])-det([VOri,Vect2]))/det([Vect1,Vect2]);
                            CBb = -(det([TestPoint(:,jj),Vect1])-det([VOri,Vect1]))/det([Vect1,Vect2]);
                            
                            if Behave < 0
                                if CAa > 0 && CBb > 0 %&& (CAa+CBb) <= 1 %|| (CAa+CBb) <= 0.00; %if inside
                                    LuDa(jj) = 1;
                                else
                                    LuDa(jj) = 0;
                                    break
                                end
                            else
                                if CAa >= 0 && CBb >= 0 %&& (CAa+CBb) <= 1 %|| (CAa+CBb) <= 0.00; %if inside
                                    LuDa(jj) = 1;
                                else
                                    LuDa(jj) = 0;
                                    break
                                end
                            end
                            
                        end
                        
                        
                        
                        
                end
            end
            
        end
        function RoMa = RotMat(AngleNya,Dimension)
            switch Dimension
                case 2
                    RoMa = [cos(AngleNya) -sin(AngleNya); sin(AngleNya) cos(AngleNya)];
                case 31
                    RoMa = [cos(AngleNya) -sin(AngleNya) 0; sin(AngleNya) cos(AngleNya) 0; 0 0 1];
                case 3 %true 3 dimenstional rotational matrix - fdrom Stevens and Lewis
                    RolTi = AngleNya(1); PitTi = AngleNya(2); YawTi = AngleNya(3);
                    RoMa = [cos(PitTi)*cos(YawTi) cos(PitTi)*sin(YawTi) -sin(PitTi);
                           -cos(RolTi)*sin(YawTi)+sin(RolTi)*sin(PitTi)*cos(YawTi) cos(RolTi)*cos(YawTi)+sin(RolTi)*sin(PitTi)*sin(YawTi) sin(RolTi)*cos(PitTi);
                            sin(RolTi)*sin(YawTi)+cos(RolTi)*sin(PitTi)*cos(YawTi) -sin(RolTi)*cos(YawTi)+cos(RolTi)*sin(PitTi)*sin(YawTi) cos(RolTi)*cos(PitTi)];
                otherwise
                    RoMa = [cos(AngleNya) -sin(AngleNya); sin(AngleNya) cos(AngleNya)];
            end
        end
        function DisT = Point2Vect(Path,Pos,Dim)
            
            if Dim ==2
                %Poin1 = [0;0];
                Poin2 = Path(:,2)-Path(:,1);
                Poin3 = Pos-Path(:,1);
                %
                Aax = Poin2(2)/Poin2(1); %gradien?
                %ax+by+c = 0,
                Aox = -Poin2(1)/Poin2(2); %gradien?
                if isinf(Aox)
                   Co = Poin3(1);
                   Xxi = Co;
                   Yyi = Xxi*Aax;
                else
                    Boy = -1;
                    Co = -Aox*Poin3(1)-Boy*Poin3(2);
                    Xxi = -Co/(Aox - Aax);
                    Yyi = Xxi*Aax;                     
                end
                XYo = [Xxi;Yyi;0]-Poin3;
                DisT = ([sum(XYo.^2)^0.5 XYo(1) XYo(2)]);
            elseif Dim == 3
                %from http://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html
                %Poin1 = [0;0];
                X2_X1 = Path(:,2)-Path(:,1);
                X1_X0 = Path(:,1)-Pos;
                
                Dis = ((sum((cross(X2_X1,X1_X0)).^2))^0.5)/((sum(X2_X1).^2)^0.5);
                %eq above will error if ... what happen if the dist = 0,
                %with the point?
                Tt = -(dot(X1_X0,X2_X1))/(((sum(X2_X1).^2)^0.5)^2);
                %the Tt can be used to detect whether the goal is
                %accomplished or not!
                XYZo =Path(:,1)+(X2_X1)*Tt;
                DisT = ([Dis XYZo']);
            end
            
        end
        function Angl = Vect2Angls(Vect)
            %without-roll!
            Angl = [0; -atan2(Vect(3),((Vect(2)^2+Vect(1)^2)^0.5)); atan2(Vect(2),Vect(1))];
        end
            
    end
    
end
































