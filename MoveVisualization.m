close all;
load('Record.mat')

%simplifying the names ===================================================
for ii = 1:AgentNumber
    for jj = 1:length(RecUVW_g(1).Data)-1
        for kk = 1:3
            %some(agnetNo,xyz,time)
            XYZ_g(ii,kk,jj) = RecXYZ_g(ii).Data(kk,jj);
            UVW_g(ii,kk,jj) = RecUVW_g(ii).Data(kk,jj);
            VTP_g(ii,kk,jj) = RecVTP_g(ii).Data(kk,jj);
            
            CInteru(ii,jj) = RecODist(ii).Data(2,jj);
            CDecis(ii,jj) = RecODist(ii).Data(3,jj);
            for kk = 1:AgentNumber-1
                ODist(ii,kk,jj) = RecODist(ii).Data(kk,jj);
                CImm(ii,kk,jj) = RecOFlag(ii).Data(1+kk,jj); %Flag 1
                CInc(ii,kk,jj) = RecOFlag(ii).Data(103+kk,jj); %Flag 3
                CDIV(ii,kk,jj) = RecOFlag(ii).Data(154+kk,jj); %Flag 4
            end
        end
    end
    XYZ_start(:,ii) = RecXYZ_g(ii).Data2(:,2);
    XYZ_goal(:,ii) = RecXYZ_g(ii).Data2(:,1);
    
end

DatNum = length(RecUVW_g(1).Data)-1;
VTP_g(2,:,1)
UVW_g(:,:,1)
XYZ_g(2,:,1)

[Xunit,Yunit,Zunit] = sphere;
XAge = Xunit*Rsep*0.5; YAge = Yunit*Rsep*0.5; ZAge = Zunit*Rsep*0.5; %0.5 for half radius
[Xunit2,Yunit2,Zunit2] = cylinder(0:1); %use this for heading
RotMatVO = [cos(pi/2) 0 -sin(pi/2); 0 1 0; sin(pi/2) 0 cos(pi/2)]; % too turn it on the VTP right direction first
HeadConeLength = 2*Rsep*0.5; %little outside
HeadConeBase = Rsep*0.5*0.6; %Stay inside
XVO = cos(pi/2)*Xunit2*HeadConeBase-sin(pi/2)*(Zunit2-1)*HeadConeLength+1.2*0.5*Rsep; %easer not with matrix?
YVO = Yunit2*HeadConeBase; 
ZVO = sin(pi/2)*Xunit2*HeadConeBase+cos(pi/2)*(Zunit2-1)*HeadConeLength;

figure(10) %just to make sure..
hold on; grid on; axis equal;
set(gca,'ZDir','reverse','YDir','reverse','CameraPosition',[-62 101 -35],'CameraViewAngle',9,...
        'View',[-35.4711 15.846])
axis([-10 10 -10 10 -10 10])
ColSet = ['b'; 'r'; 'g'; 'm';'b'; 'r'; 'g'; 'm']; 

ForTrckX = zeros(1,DatNum); 
ForTrckY = zeros(1,DatNum);
ForTrckZ = zeros(1,DatNum);

for tii = 1:AgentNumber
    TrackLine(tii) = line(XYZ_g(tii,1,1), XYZ_g(tii,2,1), XYZ_g(tii,3,1),'LineStyle','--','Color',ColSet(tii),'linewidth',1.5);
    
    vAge(tii) = surf(XAge+XYZ_g(tii,1,1), YAge+XYZ_g(tii,2,1), ZAge+XYZ_g(tii,3,1),...  %the XYZ_g is just for initial
                'FaceColor','k','FaceAlpha',0.2,'EdgeColor',ColSet(tii),'EdgeAlpha',0.5); %making agent sphere
    hold on;
    RotMatP = [cos(-VTP_g(tii,3,1)) sin(-VTP_g(tii,3,1)) 0; -sin(-VTP_g(tii,3,1)) cos(-VTP_g(tii,3,1)) 0 ; 0 0 1]; %3D turning to heading
    RotMatT = [cos(-VTP_g(tii,2,1)) 0 -sin(-VTP_g(tii,2,1)); 0 1 0; sin(-VTP_g(tii,2,1)) 0 cos(-VTP_g(tii,2,1))];
    RotMatV = [1 0 0; 0 cos(-VTP_g(tii,1,1)) sin(-VTP_g(tii,1,1)); 0 -sin(-VTP_g(tii,1,1)) cos(-VTP_g(tii,1,1))];
    RotMat3D = RotMatP*RotMatT*RotMatV;
    %RotMat3D = RotMatP;
    %rotate the heading
    XVO1 = XVO*RotMat3D(1,1)+YVO*RotMat3D(1,2)+ZVO*RotMat3D(1,3);
    YVO1 = XVO*RotMat3D(2,1)+YVO*RotMat3D(2,2)+ZVO*RotMat3D(2,3);
    ZVO1 = XVO*RotMat3D(3,1)+YVO*RotMat3D(3,2)+ZVO*RotMat3D(3,3);
    vAgeHead(tii)=surf(XVO1+XYZ_g(tii,1,1), YVO1+XYZ_g(tii,2,1), ZVO1+XYZ_g(tii,3,1),... %making heading indicator
                 'FaceColor','b','FaceAlpha',0.2,'EdgeColor','k','EdgeAlpha',0.2);
             
    InitDir(tii) = line([XYZ_start(1,tii) XYZ_goal(1,tii)],...
                        [XYZ_start(2,tii) XYZ_goal(2,tii)],...
                        [XYZ_start(3,tii) XYZ_goal(3,tii)]);
    InitDir2(tii) = line([XYZ_g(tii,1,1) XYZ_g(tii,1,1)+10*UVW_g(tii,1,1)],...
                        [XYZ_g(tii,2,1) XYZ_g(tii,2,1)+10*UVW_g(tii,2,1)],...
                        [XYZ_g(tii,3,1) XYZ_g(tii,3,1)+10*UVW_g(tii,3,1)],'color','r');
end
disp(num2str(VTP_g(2,:,1)*57.3))
%ilkyhgky
%run it for sim time
for sii = 1:DatNum
    for tii = 1:AgentNumber
        set(vAge(tii),'XData',XAge+XYZ_g(tii,1,sii), ...
                      'YData',YAge+XYZ_g(tii,2,sii), ...
                      'ZData',ZAge+XYZ_g(tii,3,sii));
        %Rot mat calculation is done every time step
        RotMatP = [cos(-VTP_g(tii,3,sii)) sin(-VTP_g(tii,3,sii)) 0; -sin(-VTP_g(tii,3,sii)) cos(-VTP_g(tii,3,sii)) 0 ; 0 0 1]; %3D turning to heading
        RotMatT = [cos(-VTP_g(tii,2,sii)) 0 -sin(-VTP_g(tii,2,sii)); 0 1 0; sin(-VTP_g(tii,2,sii)) 0 cos(-VTP_g(tii,2,sii))];
        RotMatV = [1 0 0; 0 cos(-VTP_g(tii,1,sii)) sin(-VTP_g(tii,1,sii)); 0 -sin(-VTP_g(tii,1,sii)) cos(-VTP_g(tii,1,sii))];
        RotMat3D = RotMatP*RotMatT*RotMatV;
        %rotate the heading
        XVO1 = XVO*RotMat3D(1,1)+YVO*RotMat3D(1,2)+ZVO*RotMat3D(1,3);
        YVO1 = XVO*RotMat3D(2,1)+YVO*RotMat3D(2,2)+ZVO*RotMat3D(2,3);
        ZVO1 = XVO*RotMat3D(3,1)+YVO*RotMat3D(3,2)+ZVO*RotMat3D(3,3);
        set(vAgeHead(tii),'XData',XVO1+XYZ_g(tii,1,sii), ...
                          'YData',YVO1+XYZ_g(tii,2,sii), ...
                          'ZData',ZVO1+XYZ_g(tii,3,sii));
                      
        %ForTrckX(1,DatNum-sii) = XYZ_g(tii,1,sii);
        %ForTrckY(1,DatNum-sii) = XYZ_g(tii,1,sii);
        %ForTrckZ(1,DatNum-sii) = XYZ_g(tii,1,sii);
        set(TrackLine(tii),'XData',[get(TrackLine(tii),'XData'), XYZ_g(tii,1,sii)], ...
                           'YData',[get(TrackLine(tii),'YData'), XYZ_g(tii,2,sii)], ...
                           'ZData',[get(TrackLine(tii),'ZData'), XYZ_g(tii,3,sii)]);
                      
        
                       
    end
    disp(num2str(VTP_g(1,:,sii)*57.3))
    Rola(1,sii) = VTP_g(1,1,sii)*57.3;
    Rola(2,sii) = VTP_g(1,2,sii)*57.3;
    Rola(3,sii) = VTP_g(1,3,sii)*57.3;
    %Rola(4,sii) = CInteru(1,sii);
    %Rola(5,sii) = CDecis(1,sii)*57.3;
    
   
    
    Rolu(1,sii) = UVW_g(1,1,sii);
    Rolu(2,sii) = UVW_g(1,2,sii);
    Rolu(3,sii) = UVW_g(1,3,sii);
    %Rolu(4,sii) = CInteru(1,sii);
    %Rolu(5,sii) = CDecis(1,sii)*57.3;
    %disp(num2str(XYZ_g(2,:,sii)*57.3))
    %disp(num2str(UVW_g(2,:,sii)*57.3))
    
    for sjj = 1:AgentNumber
        %Interu(sjj,sii) = CInteru(sjj,sii);
        UVWpl(3*(sjj-1)+1,sii) = UVW_g(sjj,1,sii);
        UVWpl(3*(sjj-1)+2,sii) = UVW_g(sjj,2,sii);
        UVWpl(3*(sjj-1)+3,sii) = UVW_g(sjj,3,sii);
        
        VTPpl(3*(sjj-1)+1,sii) = VTP_g(sjj,1,sii);
        VTPpl(3*(sjj-1)+2,sii) = VTP_g(sjj,2,sii);
        VTPpl(3*(sjj-1)+3,sii) = VTP_g(sjj,3,sii);
        
        
        for skk =1:AgentNumber-1         
            Dista((sjj-1)*(AgentNumber-1)+skk,sii) = ODist(sjj,skk,sii);
            Immin((sjj-1)*(AgentNumber-1)+skk,sii) = CImm(sjj,skk,sii);                
            Inclu((sjj-1)*(AgentNumber-1)+skk,sii) = CInc(sjj,skk,sii);
            DIV((sjj-1)*(AgentNumber-1)+skk,sii) = CDIV(sjj,skk,sii);
        end
    end
    pause(0.1)

end
figure(13)
plot(Rola(1,:),'b'); hold on;
plot(Rola(2,:),'r');
plot(Rola(3,:),'g');

grid on;

figure(14)
plot(Rolu(1,:),'b'); hold on;
plot(Rolu(2,:),'r');
plot(Rolu(3,:),'g');
grid on;



ColPlo ='brgmkbrgmk';
for aa = 1:AgentNumber
    figure(14)
    subplot(AgentNumber,1,aa)
    plot(UVWpl(3*(aa-1)+1:3*aa,:)');grid on; hold on;
    if aa == 1;    title('Global Velocity (UVW_g)'); end
    
    figure(15)
    subplot(AgentNumber,1,aa)
    plot(VTPpl(3*(aa-1)+1:3*aa,:)');grid on; hold on;
    if aa == 1;    title('Global Attitude (VTP_g)'); end
    
    for bb = 1:AgentNumber-1
        figure(16)
        subplot(AgentNumber,1,aa)
        plot(Dista((aa-1)*(AgentNumber-1)+bb,:)',ColPlo(bb)); grid on; hold on;
        if aa == 1 && bb==1;    title('Distance Between'); end
        axis([0 80 0 1])
        
        figure(17)
        subplot(AgentNumber,1,aa)
        plot(Immin((aa-1)*(AgentNumber-1)+bb,:)',ColPlo(bb)); grid on; hold on;
        if aa == 1 && bb==1;    title('Imminent Flag'); end
        
        figure(18)
        subplot(AgentNumber,1,aa)
        plot(Inclu((aa-1)*(AgentNumber-1)+bb,:)',ColPlo(bb)); grid on; hold on;
        if aa == 1 && bb==1;    title('VOIncluded Flag'); end
        
        figure(19)
        subplot(AgentNumber,1,aa)
        plot(DIV((aa-1)*(AgentNumber-1)+bb,:)',ColPlo(bb)); grid on; hold on;
        if aa == 1 && bb==1;    title('DIVincluded Flag'); end
        

    end
end 
   

%========================================================================