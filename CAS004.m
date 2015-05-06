classdef CAS004< Computer
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        %First Input
        Strategy = 1 %Preimplemented strategy
        Multiple
        Priorities
        Rule = 1
        CASFlag = zeros(4,51)
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
        AvoTy
        %Predictor
        
        %OnVO
        VOStat
        VOtype 
        VONumber
        VOOrigin
        VectorNear
        VectorFar
        Vector = zeros(50,50)
        DivVector = zeros(50,50)
        VOBasAngLim = 0;
        VOBasAng = zeros(50,1)
        VOBasAngFF = zeros(50,1)
        VOLuDa = zeros(2,50);
        DivLuDa
        
        ObPosVO
        ObVelVO 
        ObAngVO
        
        ObPosGloFF
        ObPosVOFF
        
        VRelBoVO
        VRelTGoVO
        VelBoVO 
        VelTGoVO 
        
        RoW = ones(1,51);
        Bababa = 0;
        CASVelDes
        AvoW
        Interupt = 0;
        Cate
        
        %Proximity Sensor Input
        ImObsNum %Number of imminent object [NEW]
        ObVelAbs = zeros(50,1)
        ObDist = zeros(50,1)
        ObDistFF  = zeros(50,1)
        ObPosBod = zeros(3,50)
        ObPosBodFF = zeros(3,50)
        ObAngBod = zeros(3,50)
        ObVelBod = zeros(3,50)
        ObVelABod = zeros(3,50)
        RelVelBod 
        RelVelABod  %[NEW]
        RelVelAbs   %[NEW]
        RelVelBodTG
        
        VORad = zeros(50,1)
        VODis = zeros(50,1)
        VOBAng = zeros(50,1)
        VOVRel = zeros(3,50)
        VOAp = zeros(3,50)
        TiVOBp  %^the size of this need to be det in initial
        VOBp = zeros(3,37,50) %^the size of this need to be det in initial
        VOpVee 
        VOPv
        VOpNum
        VOpInt
        VOpNumInt
        VOpIntUn
        VOpIntNumUn
        VOpEscOp
        
        DecMode
        DecFiAvoPl
        DecAVoPl
    end
    
    methods
        function MAC = CAS004(DBoVel,Attitude,FliPath,VOFlag,Cate,ImpoDist,ImpoVelo)
            MAC = MAC@Computer(DBoVel,Attitude,FliPath);
            MAC.VOFlag = VOFlag;
            MAC.Cate = Cate;
            %Generating Important Distance data
            MAC.CASDistDat = ImpoDist;
            MAC.CASVeloDat = ImpoVelo;


        end
        
        function ACASRun(MAC)
            %whole computations
           ObsStateCalc(MAC); %obstacles states!! also the number of detected obstacle
           CheckDistance(MAC); %first Check the Distance. further, VO not calculated. ObsCats  and  CASFlag
                 %actually dont need to calculate VOP if is not imminent, or if
                 %it is not inside VO. but we just try to calculate all.
                 %Only not avoid
           VelocityObstacle3D(MAC); %Check threats, well this make the ACAS calculating all the time....      
           %VOAvoPlaneTest(MAC);
           VOAvoPlane(MAC);
           
           MAC.CASFlag(4,2:MAC.NumObs+1) = MAC.CASFlag(1,2:MAC.NumObs+1).*MAC.CASFlag(3,2:MAC.NumObs+1); %this is imminnent and inclusing
           MAC.CASFlag(4,1) = sum(MAC.CASFlag(4,2:MAC.NumObs+1));
           AngAvoPl = pi/2;
           VelAvo = MAC.VelGlo;
           AngAvoBod = 0;
           AngEsc = 0;
           SelAvoPl = 1;

           switch MAC.DecMode
               case 0
                   MAC.Decision(:,1) = [MAC.VelGlo; [AngAvoPl; 0; 0]];
               case 1 %default, go to closest CW (as in RoW)
                   %MAC.CASFlag % imminence and inclusion. also which zone, but probably not used here.
                   %MAC.TiVOBp  % points on BVO
                   %MAC.VOpVee  % considered avoidance planes
                   
                   %MAC.VOPv(4,kk,oo,ii) %no 4, angle of points of VOp
                   %MAC.VOpNum(oo,ii) %number of VOp points
                   %MAC.VOpInt(mm,oo,ii) %index of intersecting point of VOp
                   %MAC.VOpNumInt(oo,ii) %number of intersecting point
                   %MAC.VOpIntUn(1:12,jj,oo)
                   
                   %finding the right avoidance plane --> the one with the
                   %smallest angle to the right of escape (4), with smallest dcc biggest but lower than Vo (3), ?
                   %hence, the smallest positive (4). actually, only one..
                   %what if multiplication?
                   for oo = 1:length(MAC.VOpVee)
                       for ii = 1:MAC.VOpIntNumUn(oo)
                           if MAC.VOpIntUn(4,ii,oo) >= 0
                               if MAC.VOpIntUn(4,ii,oo) < AngAvoPl
                                   VelAvo = MAC.MatB2E*[MAC.VOpIntUn(7,ii,oo); MAC.VOpIntUn(8,ii,oo); MAC.VOpIntUn(9,ii,oo)];
                                   AngAvoPl = MAC.VOpIntUn(4,ii,oo);
                                   SelAvoPl = MAC.VOpVee(oo);
                               end
                           end
                       end
                   end
                   
                   %MAC.Decision(:,1) = [VelAvo; [AngAvoPl; 0; 0]]; % its already global velocity
                   %then dont change the speed nor the AvoPlane before it
                   %become not imminent.
                   
               case 2 %, go to closest CCW
                   
               case 3 %go to the closest, either CW/CCW
                   
               case 4 %go to the closest acc or dcc
                   
               case 5 %go to the closest off all (need to define energy)
                   
               otherwise
                   
           end
           
           %interupron --> calculation canbe done after interuption to be
           %more efficient. But for the research, better calculate all time
           if MAC.CASFlag(1,1)  >= 1 && MAC.CASFlag(3,1) >= 1 %Imminent and inside VO (DIV?)
              MAC.Interupt = 2; 
              MAC.Decision(:,1) = [VelAvo; [AngAvoPl; SelAvoPl; AngAvoBod]];
           elseif MAC.CASFlag(1,1)  >= 1 %&& MAC.CASFlag(3,1) ~= 1
              MAC.Interupt = 1;
              MAC.Decision(:,1) = [MAC.VelGlo; [AngAvoPl; SelAvoPl; AngAvoBod]]; %stay on VelGlo...
           else
              MAC.Interupt = 0; 

           end

        end
        function ObsStateCalc(MAC)
            %Distance, Velocity and Position of Obstacle, in OwnAircraft
            %Body Axis
            for ii = 1:MAC.NumObs
                MAC.ObDist(ii) = sqrt(sum((MAC.ObPosGlo(:,ii)-MAC.PosGlo).^2));
                MAC.ObVelAbs(ii) = sqrt(sum((MAC.ObVelGlo(:,ii)).^2));
                MAC.ObPosBod(:,ii) = MAC.MatE2B*(MAC.ObPosGlo(:,ii)-MAC.PosGlo);
                MAC.ObAngBod(:,ii) = MAC.Vect2Angls(MAC.ObPosBod(:,ii)); %angles
                MAC.ObVelBod(:,ii) = MAC.MatE2B*MAC.ObVelGlo(:,ii);
                MAC.ObVelABod(:,ii) = MAC.Vect2Angls(MAC.ObVelBod(:,ii)); %angles
                
                %Calculate Relative Values, usefull for 3DVO inclusion determination
                MAC.RelVelBod(:,ii) =  MAC.VelBo - MAC.ObVelBod(:,ii); 
                MAC.RelVelABod(:,ii) = MAC.Vect2Angls(MAC.RelVelBod(:,ii)); %angles
                MAC.RelVelAbs(ii) =  (sum(MAC.RelVelABod(:,ii).^2))^0.5;
            end                         
        end
        
        function CheckDistance(MAC)
            %use the MAC.ObDist properties --> Obstacles Distances
            %check categories? 
            
            for ii = 1:MAC.NumObs
                if MAC.ObVelAbs(ii) > MAC.CASVeloDat(5)
                    MAC.ObsCats(ii) = 6;
                elseif MAC.ObVelAbs(ii) > MAC.CASVeloDat(4)
                    MAC.ObsCats(ii) = 5;
                elseif MAC.ObVelAbs(ii) > MAC.CASVeloDat(3)
                    MAC.ObsCats(ii) = 4;
                elseif MAC.ObVelAbs(ii) > MAC.CASVeloDat(2)
                    MAC.ObsCats(ii) = 3;
                elseif MAC.ObVelAbs(ii) > MAC.CASVeloDat(1);
                    MAC.ObsCats(ii) = 2;
                else
                    MAC.ObsCats(ii) = 1;
                end
                
                for jj = 1:4 %the four radii - TW, Dec, Esc, Pz
                    MAC.SepRad(jj,ii) = MAC.CASDistDat(jj,MAC.Cate,MAC.ObsCats(ii));
                    %MAC.SepRad(jj,ii) = 2;
                    
                end
                
                if MAC.ObDist(ii) < MAC.AvoTy
                    MAC.CASFlag(1,ii+1) = 1;  %Imminent
                else
                    MAC.CASFlag(1,ii+1) = 0;
                end
                
                if MAC.ObDist(ii) < MAC.CASDistDat(4,MAC.Cate,MAC.ObsCats(ii))
                    MAC.CASFlag(2,ii+1) = 4;  %collideded
                elseif MAC.ObDist(ii) < MAC.CASDistDat(3,MAC.Cate,MAC.ObsCats(ii))
                    MAC.CASFlag(2,ii+1) = 3;  %inside escape sphere
                elseif MAC.ObDist(ii) < (MAC.CASDistDat(2,MAC.Cate,MAC.ObsCats(ii))*MAC.AvoTy);
                    MAC.CASFlag(2,ii+1) = 2;  %inside deconf sphere
                elseif MAC.ObDist(ii) < MAC.CASDistDat(1,MAC.Cate,MAC.ObsCats(ii))
                    MAC.CASFlag(2,ii+1) = 1;  %inside warning sphere
                else
                    MAC.CASFlag(2,ii+1) = 0;  %do nothing - too far
                end
            end
            %below is the summary?
            MAC.CASFlag(1,1) = sum(MAC.CASFlag(1,2:MAC.NumObs+1)); %imminence, and the number of imminence obs
            MAC.CASFlag(2,1) = max(MAC.CASFlag(2,2:MAC.NumObs+1)); %which zone (always the closest)
        end     
        
        function VelocityObstacle3D(MAC)
            %lets follow the paper
            %The VO-cone properties

            %MAC.VOBasAng(1:MAC.VONumber) = asin((1.1*MAC.SepRad')./MAC.ObDist(1:MAC.VONumber));
             for ii = 1:MAC.NumObs
                %lets follow the paper
                %The VO-cone properties
                MAC.VORad(ii) = MAC.SepRad(4,ii)*(MAC.ObDist(ii)^2-MAC.SepRad(4,ii)^2)^0.5/MAC.ObDist(ii);
                MAC.VODis(ii) = (MAC.ObDist(ii)^2-MAC.SepRad(4,ii)^2)/MAC.ObDist(ii); %fourth SepRad, since we are working in EscapeSphr
                if abs(MAC.VODis(ii)) > 0
                    MAC.VOBAng(ii) = atan2(MAC.VORad(ii),MAC.VODis(ii));
                else
                    MAC.VOBAng(ii) = sign(MAC.VORad(ii))*pi/2;
                end
                
                %forget the obstacle if its is too close.. --> collide
                if MAC.ObDist(ii) < MAC.SepRad(4,ii)
                    break;
                end
                
                %turning the RelVels to negate the ObsAng
                MAC.VOVRel(:,ii) = MAC.RotMat(MAC.ObAngBod(:,ii),3)*MAC.RelVelBod(:,ii);
                %determining the inclusion
                if MAC.VOVRel(1,ii) > 0 && ...
                  ((MAC.VOVRel(2,ii)^2+MAC.VOVRel(3,ii)^2)^0.5)/(MAC.VOVRel(1,ii)) < (MAC.VORad(ii))/(MAC.VODis(ii))
                  %just use the third CASFlag
                  MAC.CASFlag(3,ii+1) = 1;
                else
                  MAC.CASFlag(3,ii+1) = 0;
                    
                end

                %Now for each obstacle, we are going to define the A and B
                MAC.VOAp(:,ii) =  MAC.ObVelBod(:,ii);
                %MAC.TiVOBp = 0:2*pi/36:2*pi;
                for jj = 1:length(MAC.TiVOBp); %this is the t of the circle
                    Rota = MAC.RotMat(MAC.ObAngBod(:,ii),-3);
                    MAC.VOBp(:,jj,ii) = Rota*[MAC.VODis(ii);MAC.VORad(ii)*cos(MAC.TiVOBp(jj));MAC.VORad(ii)*sin(MAC.TiVOBp(jj))] + MAC.VOAp(:,ii);
                end
                
                
                chch = 1;
                if chch > 0
                    figure(15)
                    VOA = MAC.VOAp;
                    VOB = MAC.VOBp;
                    plot3([VOA(1,ii) VOB(1,1:round(end/4),ii) VOA(1,ii) VOB(1,round(end/4):round(end/2),ii) VOA(1,ii) VOB(1,round(end/2):round(3*end/4),ii) VOA(1,ii) VOB(1,round(3*end/4):end,ii)],...
                          [VOA(2,ii) VOB(2,1:round(end/4),ii) VOA(2,ii) VOB(2,round(end/4):round(end/2),ii) VOA(2,ii) VOB(2,round(end/2):round(3*end/4),ii) VOA(2,ii) VOB(2,round(3*end/4):end,ii)],...
                          [VOA(3,ii) VOB(3,1:round(end/4),ii) VOA(3,ii) VOB(3,round(end/4):round(end/2),ii) VOA(3,ii) VOB(3,round(end/2):round(3*end/4),ii) VOA(3,ii) VOB(3,round(3*end/4):end,ii)]); grid on; axis equal;
                    
                    hold on;
                    
                end
             end
             stopppp
             MAC.CASFlag(3,1) = sum(MAC.CASFlag(3,2:MAC.NumObs+1)); %and the number of inclusion
        end
        
        function VOAvoPlane(MAC)
            %actually only need to run if it is included in the VO3D and imminent....
            %it need to combined all points from all obstacles.
            for ii = 1:MAC.NumObs
                %We need access to VOAp and VOBp, and other VO properties
                VOBpVee = zeros(3,length(MAC.TiVOBp));
                for oo = 1:length(MAC.VOpVee)
                    RotVirRoll =[1 0 0; %actually can be calc in beginnig (init)
                                 0 cos(MAC.VOpVee(oo)) sin(MAC.VOpVee(oo));
                                 0 -sin(MAC.VOpVee(oo)) cos(MAC.VOpVee(oo))];
                    VOApVee = RotVirRoll*MAC.VOAp(:,ii);
                    
                    for pp = 1:length(MAC.TiVOBp)
                        VOBpVee(:,pp) =RotVirRoll*MAC.VOBp(:,pp,ii);
                        
                        if isreal(VOBpVee(:,pp)) == 0 
                           sdgvsdv
                        end
                    end

                    %for every point (t), jj. But remember, not all t result in
                    %a valid point in VOp.
                    %and to anticipate degenerate cases
                    %1. if VOApVee is on the AvoPlane --> VOApVee(3) = 0;--> a triangle?
                    %2. if --> a line? actually would mean Vo is not on VO, or is on the egde
                    %3. if --> a point?actually would mean Vo is not on VO,or at the origin of vO 
                    kk = 1;
                    
                    if abs(VOApVee(3)) > 0.00001 %not degenerate case
                        for jj = 1:length(MAC.TiVOBp)
                            TiVOGl =-VOApVee(3)/(VOBpVee(3,jj)-VOApVee(3)); %genetrix line time
                            if isreal(TiVOGl) == 0
                                Thetimeimaginer
                            end
                            if  TiVOGl >= 0 && TiVOGl <= 1 %eliminating the hyperbolic or point beyond dvo, if z=0 is inside the line
                                MAC.VOPv(1,kk,oo,ii) = (VOBpVee(1,jj)-VOApVee(1))*TiVOGl+VOApVee(1);
                                MAC.VOPv(2,kk,oo,ii) = (VOBpVee(2,jj)-VOApVee(2))*TiVOGl+VOApVee(2);
                                MAC.VOPv(3,kk,oo,ii) = (MAC.VOPv(1,kk,oo,ii)^2+MAC.VOPv(2,kk,oo,ii)^2)^0.5; %Polar magnitude
                                if abs(MAC.VOPv(1,kk,oo,ii)) > 0
                                    MAC.VOPv(4,kk,oo,ii) = atan2(MAC.VOPv(2,kk,oo,ii),MAC.VOPv(1,kk,oo,ii)); %Polar angle
                                else
                                    MAC.VOPv(4,kk,oo,ii) = sign(MAC.VOPv(2,kk,oo,ii))*pi/2;
                                end
                                kk = kk+1;

                            end
                            
                        end
                        
                        if kk > 1
                            for jj = kk:length(MAC.TiVOBp)
                                MAC.VOPv(1:4,jj,oo,ii) = MAC.VOPv(1:4,kk-1,oo,ii);
                            end
                        end
                        MAC.VOpNum(oo,ii) = kk-1;
                        
                    else %degenerate case --> triangle
                        %first point is the VOApVee it self
                        MAC.VOPv(1,1,oo,ii) = VOApVee(1);
                        MAC.VOPv(2,1,oo,ii) = VOApVee(2);
                        
                        %what the end (half and half +1) point?
                        %the intersection of the Bvo with the AVOplane.
                        %same way with deciding the escape route... first
                        %finding a consecutive segment that went through z
                        %= 0
                        if mod(length(MAC.TiVOBp),2) == 0 %even numbe rf point
                            kk = length(MAC.TiVOBp)/2;
                            HalfPo = length(MAC.TiVOBp)/2;
                        else
                            kk = (length(MAC.TiVOBp)-1)/2;
                            HalfPo = (length(MAC.TiVOBp)-1)/2;
                        end

                        for jj = 1:length(MAC.TiVOBp)
                            if jj == length(MAC.TiVOBp) %the end point
                               jjj = 1; 
                            else
                               jjj = jj+1;
                            end

                            if VOBpVee(3,jj)*VOBpVee(3,jjj) < 0
                                TiVOGl = VOBpVee(3,jjj)/(VOBpVee(3,jj)-VOBpVee(3,jjj));
                                MAC.VOPv(1,kk,oo,ii) = -(VOBpVee(1,jj)-VOBpVee(1,jjj))*TiVOGl+VOBpVee(1,jjj);
                                MAC.VOPv(2,kk,oo,ii) = -(VOBpVee(2,jj)-VOBpVee(2,jjj))*TiVOGl+VOBpVee(2,jjj);
                                kk = kk+1;
                            elseif VOBpVee(3,jj) == 0
                                MAC.VOPv(1,kk,oo,ii) = VOBpVee(1,jj);
                                MAC.VOPv(2,kk,oo,ii) = VOBpVee(2,jj);
                                kk = kk+1;
                            end
                        end

                        %iyhgiu
                        %^above should always result 2 point!25,26 (convex)
                        %NOW put 25 (half length(MAC.TiVOBp) point evenly? on 
                        %each side --> or more packed on x smaller than
                        %2*Vo?
                        for ll = 2:(HalfPo-1)
                            MAC.VOPv(1,ll,oo,ii) = (MAC.VOPv(1,HalfPo,oo,ii)-MAC.VOPv(1,1,oo,ii))*(ll-1)/(HalfPo-1)+MAC.VOPv(1,1,oo,ii);
                            MAC.VOPv(2,ll,oo,ii) = (MAC.VOPv(2,HalfPo,oo,ii)-MAC.VOPv(2,1,oo,ii))*(ll-1)/(HalfPo-1)+MAC.VOPv(2,1,oo,ii);

                            MAC.VOPv(1,ll+HalfPo,oo,ii) = (MAC.VOPv(1,1,oo,ii)-MAC.VOPv(1,(HalfPo+1),oo,ii))*(ll-1)/(HalfPo-1)+MAC.VOPv(1,(HalfPo+1),oo,ii);
                            MAC.VOPv(2,ll+HalfPo,oo,ii) = (MAC.VOPv(2,1,oo,ii)-MAC.VOPv(2,(HalfPo+1),oo,ii))*(ll-1)/(HalfPo-1)+MAC.VOPv(2,(HalfPo+1),oo,ii);

                        end
                        MAC.VOpNum(oo,ii) = HalfPo*2-1;
                        %completing no 3 and 4
                        for pp = 1:HalfPo*2-1
                            MAC.VOPv(3,pp,oo,ii) = (MAC.VOPv(1,pp,oo,ii)^2+MAC.VOPv(2,pp,oo,ii)^2)^0.5; %Polar magnitude
                            if abs(MAC.VOPv(1,pp,oo,ii)) > 0
                                aaa  = MAC.VOPv(2,pp,oo,ii)/MAC.VOPv(1,pp,oo,ii);
                                MAC.VOPv(4,pp,oo,ii) = atan2(MAC.VOPv(2,pp,oo,ii),MAC.VOPv(1,pp,oo,ii)); %Polar angle
                            else
                                MAC.VOPv(4,pp,oo,ii) = sign(MAC.VOPv(2,pp,oo,ii))*pi/2;
                            end
                        end
                        
                        
                        %=================================================
                        chck = 0;
                        if chck > 0
                            figure(5)
                            plot(MAC.VOPv(1,1:MAC.VOpNum(oo,ii),oo,ii),MAC.VOPv(2,1:MAC.VOpNum(oo,ii),oo,ii)); hold on;
                            plot(MAC.VOPv(1,1,oo,ii),MAC.VOPv(2,1,oo,ii),'or','MarkerSize',10)
                            plot(MAC.VOPv(1,HalfPo,oo,ii),MAC.VOPv(2,HalfPo,oo,ii),'ob','MarkerSize',10)
                            plot(MAC.VOPv(1,HalfPo+1,oo,ii),MAC.VOPv(2,HalfPo+1,oo,ii),'og','MarkerSize',10)
                            plot(MAC.VOPv(1,MAC.VOpNum(oo,ii),oo,ii),MAC.VOPv(2,HalfPo*2-1,oo,ii),'om','MarkerSize',10)  
                        end
                        %================================================
                        
    
                    end
                    
                    %escaping options
                    mm = 1;
                    for ll = 1:MAC.VOpNum(oo,ii)                
                        %a search for two points closest to
                        %intersection (R ~ Vo). Two ways, pos and neg The
                        %search is for all point to take account every
                        %cases of intersection point. Two point is recorded
                        %if (VOp(t)-Vo)*(VOp(t+1)-Vo) < 0 only one is
                        %negative
                        if ll == MAC.VOpNum(oo,ii)
                            lll = 1;
                        else
                            lll = ll+1;
                        end
                        
                        if (MAC.VOPv(3,ll,oo,ii)-MAC.VelBo(1))*(MAC.VOPv(3,lll,oo,ii)-MAC.VelBo(1)) <= 0
                            MAC.VOpInt(mm,oo,ii) = ll; %recorded ll is the index of escape route of VOPv
                            MAC.VOpInt(mm+1,oo,ii) = lll;
                            MAC.VOpEscOp(ll,oo,ii) = 1;
                            mm = mm+2;
                        else
                            MAC.VOpEscOp(ll,oo,ii) = 0;
                        end
                    end
                    MAC.VOpNumInt(oo,ii) = mm-1;
                end
            end
            %should eliminate points inside... how? --> on each AvoPlane oo
            %with multiple obstacle ii test ii point with ii+1 polygon, and
            %the ii+1 point with ii polygon., for each oo
            for oo = 1:length(MAC.VOpVee)
                nn = 1;
                for ii = 1:MAC.NumObs
                    for mm = 1:MAC.VOpNumInt(oo,ii) %with the list of esc point
                        forget = 0;
                        for jj = 1:MAC.NumObs %trying the point with jj polygon
                            if ii ~= jj
                                %test ii point to jj polygon % angle should be
                                %enough. test the closest angle with the point
                                %(two closest) if existed, count. if not,
                                %should be outside!)
                                InOut = 0;
                                for kk = 1:MAC.VOpNum(oo,jj)
                                    if kk == MAC.VOpNum(oo,jj) %last point
                                        kkk = 1;
                                    else
                                        kkk = kk+1;
                                    end
                                    if (MAC.VOPv(2,kk,oo,jj)-MAC.VOPv(2,MAC.VOpInt(mm,oo,ii),oo,ii))*...
                                            (MAC.VOPv(2,kkk,oo,jj)-MAC.VOPv(2,MAC.VOpInt(mm,oo,ii),oo,ii)) <= 0 
                                        %eq => (y1-y)/(y1-y2) = (x1-x)/(x1-x2)
                                        XInt=MAC.VOPv(1,kk,oo,jj)-...
                                            (MAC.VOPv(1,kk,oo,jj)-MAC.VOPv(1,kkk,oo,jj))*...
                                            (MAC.VOPv(2,kk,oo,jj)-MAC.VOPv(2,MAC.VOpInt(mm,oo,ii),oo,ii))/(MAC.VOPv(2,kk,oo,jj)-MAC.VOPv(2,kkk,oo,jj));
                                        if XInt <= MAC.VOPv(1,MAC.VOpInt(mm,oo,ii),oo,ii)
                                            InOut = InOut+1;
                                        end
                                    end 
                                end
                                if mod(InOut,2) ~= 0 %odd = inside (directly) forget about it, no need to test the point with other jj
                                    forget = 1;
                                    break; %break the jj, stopping jj loop
                                     %number of escape route in every avoplane, regardless the obstacle
                                end
                            end
                        end
                        if forget == 0 % not included in any of jj polygon
                           MAC.VOpIntUn(1,nn,oo) = MAC.VOPv(1,MAC.VOpInt(mm,oo,ii),oo,ii);
                           MAC.VOpIntUn(2,nn,oo) = MAC.VOPv(2,MAC.VOpInt(mm,oo,ii),oo,ii);
                           MAC.VOpIntUn(3,nn,oo) = MAC.VOPv(3,MAC.VOpInt(mm,oo,ii),oo,ii);
                           MAC.VOpIntUn(4,nn,oo) = MAC.VOPv(4,MAC.VOpInt(mm,oo,ii),oo,ii);
                           MAC.VOpIntUn(5,nn,oo) = cos(MAC.VOpIntUn(4,nn,oo))*MAC.VelBo(1);
                           MAC.VOpIntUn(6,nn,oo) = sin(MAC.VOpIntUn(4,nn,oo))*MAC.VelBo(1);
                           %reverse the rolltation?
%                            RotVirRoll =[1 0 0; 
%                                         0 cos(MAC.VOpVee(oo)) sin(MAC.VOpVee(oo));
%                                         0 -sin(MAC.VOpVee(oo)) cos(MAC.VOpVee(oo))];
                           MAC.VOpIntUn(7,nn,oo) = MAC.VOpIntUn(1,nn,oo)+0*MAC.VOpIntUn(2,nn,oo)+0;
                           MAC.VOpIntUn(8,nn,oo) = 0*MAC.VOpIntUn(1,nn,oo)+cos(-MAC.VOpVee(oo))*MAC.VOpIntUn(2,nn,oo)+0;
                           MAC.VOpIntUn(9,nn,oo) = 0*MAC.VOpIntUn(1,nn,oo)-sin(-MAC.VOpVee(oo))*MAC.VOpIntUn(2,nn,oo)+0;
                           %or if we are using the VelBo magnitude
                           MAC.VOpIntUn(10,nn,oo) = MAC.VOpIntUn(5,nn,oo)+0*MAC.VOpIntUn(6,nn,oo)+0;
                           MAC.VOpIntUn(11,nn,oo) = 0*MAC.VOpIntUn(5,nn,oo)+cos(-MAC.VOpVee(oo))*MAC.VOpIntUn(6,nn,oo)+0;
                           MAC.VOpIntUn(12,nn,oo) = 0*MAC.VOpIntUn(5,nn,oo)-sin(-MAC.VOpVee(oo))*MAC.VOpIntUn(6,nn,oo)+0;
                           
                           MAC.VOpEscOp(MAC.VOpInt(mm,oo,ii),oo,ii) = 2; % 2 index for global sol entire oo
                           nn = nn+1;
                           
                        end
                        %then try another mm of ii
                    end
                end
                MAC.VOpIntNumUn(oo) = nn-1;
            end
            
            
            chch =0; 
            if chch > 0 
                %figure(20)
                coco = ['b';'m';'c';'k'];
                for oo = 1:12
                    figure(20+oo)
                    for ii = 1:MAC.NumObs
                        plot(MAC.VOPv(1,1:MAC.VOpNum(oo,ii),oo,ii),MAC.VOPv(2,1:MAC.VOpNum(oo,ii),oo,ii),['-d' coco(ii)]); grid on; axis equal; hold on;
                        for nn = 1:MAC.VOpNumInt(oo,ii)
                            plot(MAC.VOPv(1,MAC.VOpInt(nn,oo,ii),oo,ii),MAC.VOPv(2,MAC.VOpInt(nn,oo,ii),oo,ii),'*r')
                        end
                        xcirc = 0:0.01:MAC.VelBo(1);
                        ycirc = (MAC.VelBo(1)^2-xcirc.^2).^0.5;
                        plot(xcirc,ycirc,'r'); plot(xcirc,-ycirc,'r');
                        plot(MAC.VOPv(1,1,oo,ii),MAC.VOPv(2,1,oo,ii),'dr','MarkerFaceColor','b');
                        %[min(MAC.VOPv(1,:,oo,ii)) abs(max(MAC.VOPv(1,:,oo,ii))) 2*min(MAC.VOPv(2,:,oo,ii)) 2*max(MAC.VOPv(2,:,oo,ii))]
                        %axis([min(MAC.VOPv(1,:,oo,ii)) abs(max(MAC.VOPv(1,:,oo,ii))) 2*min(MAC.VOPv(2,:,oo,ii)) abs(2*max(MAC.VOPv(2,:,oo,ii)))])
                        axis([0 3 -1 1])
                        title(['AvoPlane : ' num2str(round(MAC.VOpVee(oo)*57.3))  '^o '  num2str(MAC.VOpIntNumUn(oo))])
                    
                    end
                    plot(MAC.VOpIntUn(1,1:MAC.VOpIntNumUn(oo),oo),MAC.VOpIntUn(2,1:MAC.VOpIntNumUn(oo),oo),'ok','MarkerSize',10)
                    for jj = 1:MAC.VOpIntNumUn(oo)
                        plot([0 MAC.VOpIntUn(1,jj,oo)],[0 MAC.VOpIntUn(2,jj,oo)],'--g','linewidth',1)
                    end
                end
                stopppp
                
            end
            
        end
        
        function VelocityObstacle(MAC)
            SizeOb = size(MAC.ObPosGlo);
            MAC.VONumber = SizeOb(2); 
            MAC.VOBasAng(1:MAC.VONumber) = asin((1.1*MAC.SepRad')./MAC.ObDist(1:MAC.VONumber));
            
            %looking 1 step (second) forward?
            
            MAC.VOBasAngFF(1:MAC.VONumber) = asin((1.1*MAC.SepRad')./MAC.ObDistFF(1:MAC.VONumber));
            %MAC.VOBasAng = MAC.VOBasAngFF;
            %MAC.ObPosBod = MAC.ObPosBodFF;
            

            for ii = 1:MAC.VONumber
                MAC.VOOrigin(:,ii) =MAC.ObVelBod(1:2,ii);
                MAC.Vector(1:2,ii) = MAC.ObVelBod(1:2,ii);
                %Define VO! theres no near or far now.... 
                MAC.Vector(3:4,ii) = MAC.RotMat(MAC.VOBasAng(ii),2)*(100*MAC.ObPosBod(1:2,ii)+MAC.ObVelBod(1:2,ii));
                MAC.Vector(5:6,ii) = MAC.RotMat(-MAC.VOBasAng(ii),2)*(100*MAC.ObPosBod(1:2,ii)+MAC.ObVelBod(1:2,ii));
                
                %Define VO for one step forward, need new origin?
                MAC.Vector(7:8,ii) = [0;0];%MAC.RotMat(MAC.VOBasAngFF(ii),2)*(100*MAC.ObPosBodFF(1:2,ii)+MAC.ObVelBod(1:2,ii));
                MAC.Vector(9:10,ii) = [0;0];%MAC.RotMat(-MAC.VOBasAngFF(ii),2)*(100*MAC.ObPosBodFF(1:2,ii)+MAC.ObVelBod(1:2,ii));
                
                %check if the relatives velocity is inside VO or not
                MAC.VOLuDa(1,ii) = MAC.IX2D(MAC.VelBo(1:2),...
                    [MAC.Vector(3:4,ii),MAC.Vector(1:2,ii), MAC.Vector(5:6,ii)],1,1);
                MAC.VOLuDa(2,ii) = MAC.IX2D(MAC.TGoVel(1:2),...
                    [MAC.Vector(3:4,ii),MAC.Vector(1:2,ii), MAC.Vector(5:6,ii)],1,1); 
                
                %The Divergent Area
                %asume there exist a divergen area
                %the divergen area is made from two triangle, 
                MAC.DivVector(1:2,ii) = MAC.ObVelBod(1:2,ii);
                MAC.DivVector(3:4,ii) = MAC.ObVelBod(1:2,ii)-MAC.ObPosBod(1:2,ii);
                MAC.DivVector(5:6,ii) = MAC.ObVelBod(1:2,ii)+[-MAC.ObPosBod(2,ii);MAC.ObPosBod(1,ii)];
                MAC.DivVector(7:8,ii) = MAC.ObVelBod(1:2,ii)-MAC.ObPosBod(1:2,ii);
                MAC.DivVector(9:10,ii) = MAC.ObVelBod(1:2,ii)+[MAC.ObPosBod(2,ii);-MAC.ObPosBod(1,ii)];
                
                MAC.DivLuDa(1,ii) = 1-(MAC.IX2D(MAC.TGoVel(1:2),...
                    [MAC.DivVector(7:8,ii),MAC.DivVector(1:2,ii), MAC.DivVector(9:10,ii)],1,1)+...
                                    MAC.IX2D(MAC.TGoVel(1:2),...
                    [MAC.DivVector(3:4,ii),MAC.DivVector(1:2,ii), MAC.DivVector(5:6,ii)],1,1)); 
                %now for the rule, find out wether it is 
                %in this CAS004, there are no rule!
                MAC.RoW(ii+1) = 1;
                MAC.RoW(1) = max(MAC.RoW(2:MAC.NumObs(2)+1));
            end
        %so on or off???
        %Now is up to the interupt/restore function!
        end     
        
        function VelocityObstacle2(MAC)
        %function CheckVO ==> create VO
        %Transform all nessesary vector to VObase axis
        %check relative velocity in/outside VO
        %Check have RoW or not
        %check if relatives velocity is inside VO
       
        SizeOb = size(MAC.ObPosGlo);
        MAC.VONumber = SizeOb(2); 
        MAC.RoW = ones(1,MAC.VONumber);
        MAC.VOLuDa = zeros(2,MAC.VONumber);
        MAC.VOBasAng = asin((MAC.SepRad)./MAC.ObDist);
        
        for ii = 1:MAC.VONumber
            MAC.VOOrigin(:,ii) =[0;0];
            MAC.Vector(1:2,ii) = [0;0];
            
            %Everything in VOBaseAxis
            MAC.ObPosVO(:,ii) = MAC.RotMat(-(MAC.ObAngBod(ii)),2)*MAC.ObPosBod(1:2,ii);
            %also rotate obstacle speed
            MAC.ObVelVO(:,ii) = MAC.RotMat(-(MAC.ObAngBod(ii)),2)*MAC.ObVelBod(1:2,ii);
            %Own Velocity in VObase axis
            MAC.VelBoVO(:,ii) = MAC.RotMat(-(MAC.ObAngBod(ii)),2)*MAC.VelBo(1:2);
            %calculated the angle
            MAC.ObAngVO(:,ii) = dot(MAC.VelBoVO(1:2,ii),MAC.ObVelVO(:,ii))/((sum((MAC.VelBoVO(1:2,ii)+MAC.ObVelVO(:,ii)).^2))^0.5);
            %TGoal Velocity in VObase axis
            MAC.VelTGoVO(:,ii) = MAC.RotMat(-(MAC.ObAngBod(ii)),2)*MAC.TGoVel(1:2);
            %Calculated Relatives Velocity on VObase axis
            MAC.VRelBoVO(:,ii) = MAC.RotMat(-(MAC.ObAngBod(ii)),2)*MAC.RelVelBod(1:2,ii);
            MAC.VRelTGoVO(:,ii) = MAC.RotMat(-(MAC.ObAngBod(ii)),2)*MAC.RelVelBodTG(1:2,ii);
                        
            %Define VO! theres no near or far now.... 
            MAC.Vector(3:4,ii) = MAC.RotMat(MAC.VOBasAng(ii),2)*MAC.ObPosVO(1:2,ii);
            MAC.Vector(5:6,ii) = MAC.RotMat(-MAC.VOBasAng(ii),2)*MAC.ObPosVO(1:2,ii);
            
            %check if the relatives velocity is inside VO or not
            MAC.VOLuDa(1,ii) = MAC.IX2D(MAC.VRelBoVO(1:2,ii),...
                [MAC.Vector(3:4,ii),MAC.Vector(1:2,ii), MAC.Vector(5:6,ii)],1,1);
            MAC.VOLuDa(2,ii) = MAC.IX2D(MAC.VRelTGoVO(1:2,ii),...
                [MAC.Vector(3:4,ii),MAC.Vector(1:2,ii), MAC.Vector(5:6,ii)],1,1);               
            
            %Check the RoW rules
            if MAC.ObVelVO(2,ii) < 0               %if ObVelVO heading to the left 
               % MAC.RoW(ii) = 0; 

                if MAC.ObAngVO(:,ii) > cos(45/180*pi)  %if it is not head on ==> Have RoW
                    MAC.RoW(ii+1) = 0; 
                else                             %if it is head on? ==> should move
                    MAC.RoW(ii+1) = 1; 
                end
            else                                 %if ObVelAneh heading to the right ==> Should Move
                MAC.RoW(ii+1) = 1; 
            end


        end
        MAC.RoW(1) = max(MAC.RoW(2:MAC.NumObs(2)+1));

        %so on or off???
        %Now is up to the interupt/restore function!
        end     

        function KeepVelo(MAC)
            MAC.Decision = [0;0;0; 0;0;0];
        end
        function DeConfAvoid(MAC)
            
            MAC.Decision = [MAC.DMaVel; 0; 0; ... %because it is body axis...
                            0;0;0];
        end
        function ReadGCS(MAC,TGoVel)
            MAC.TGoVel = TGoVel;
        end
        
        function SetInit(MAC,DBoVel,Attitude,FliPath,AvoW,AvoTy,VOpPo,VOpVee,DecMode)
            
            MAC.DBoVel = DBoVel;
            MAC.DMaVel = (sum(DBoVel.^2))^0.5;
            MAC.TGoVel = DBoVel;
            MAC.FliPath  = FliPath;
            MAC.AttGlo = Attitude;
            
            %initiate type of avoidance?
            MAC.AvoW = AvoW;
            MAC.AvoTy = AvoTy;
            MAC.TiVOBp = VOpPo;
            MAC.VOpVee = VOpVee;
            MAC.DecMode = DecMode;
        end
        
    end
end





































