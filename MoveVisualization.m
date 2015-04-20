
load('Record.mat')

%simplifying the names ===================================================
for ii = 1:AgentNumber
    for jj = 1:length(RecUVW_g(1).Data)-1
        for kk = 1:3
            %some(agnetNo,xyz,time)
            XYZ_g(ii,kk,jj) = RecXYZ_g(ii).Data(kk,jj);
            UVW_g(ii,kk,jj) = RecUVW_g(ii).Data(kk,jj);
            VTP_g(ii,kk,jj) = RecVTP_g(ii).Data(kk,jj);
        end
    end
end
VTP_g(2,:,1)
UVW_g(:,:,1)
XYZ_g(2,:,1)

[Xunit,Yunit,Zunit] = sphere;
XAge = Xunit*Rsep*0.5; YAge = Yunit*Rsep*0.5; ZAge = Zunit*Rsep*0.5; %0.5 for half radius
[Xunit2,Yunit2,Zunit2] = cylinder(0:1); %use this for heading
RotMatVO = [cos(pi/2) 0 -sin(pi/2); 0 1 0; sin(pi/2) 0 cos(pi/2)]; % too turn it on the VTP right direction first
HeadConeLength = 1.5*Rsep*0.5; %little outside
HeadConeBase = Rsep*0.5*0.5; %Stay inside
XVO = cos(pi/2)*Xunit2*HeadConeBase-sin(pi/2)*(Zunit2-1)*HeadConeLength; %easer not with matrix?
YVO = Yunit2*HeadConeBase; 
ZVO = sin(pi/2)*Xunit2*HeadConeBase+cos(pi/2)*(Zunit2-1)*HeadConeLength;

figure(10) %just to make sure..
hold on; grid on; axis equal;
set(gca,'ZDir','reverse','CameraPosition',[-22 -66 -30],'CameraViewAngle',9)

ColSet = ['b'; 'r'; 'g'; 'm']; 
for tii = 1:AgentNumber
    vAge(tii) = surf(XAge+XYZ_g(tii,1,1), YAge+XYZ_g(tii,2,1), ZAge+XYZ_g(tii,3,1),...  %the XYZ_g is just for initial
                'FaceColor','k','FaceAlpha',0.2,'EdgeColor',ColSet(tii),'EdgeAlpha',0.5); %making agent sphere
    
    RotMatP = [cos(VTP_g(tii,1,1)) sin(VTP_g(tii,1,1)) 0; -sin(VTP_g(tii,1,1)) cos(VTP_g(tii,1,1)) 0 ; 0 0 1]; %3D turning to heading
    RotMatT = [cos(VTP_g(tii,2,1)) 0 -sin(VTP_g(tii,2,1)); 0 1 0; sin(VTP_g(tii,2,1)) 0 cos(VTP_g(tii,2,1))];
    RotMatV = [1 0 0; 0 cos(VTP_g(tii,2,1)) sin(VTP_g(tii,2,1)); 0 -sin(VTP_g(tii,2,1)) cos(VTP_g(tii,2,1))];
    RotMat3D = RotMatV*RotMatT*RotMatP;
    %rotate the heading
    XVO1 = XVO*RotMat3D(1,1)+YVO*RotMat3D(1,2)+ZVO*RotMat3D(1,3);
    YVO1 = XVO*RotMat3D(2,1)+YVO*RotMat3D(2,2)+ZVO*RotMat3D(2,3);
    ZVO1 = XVO*RotMat3D(3,1)+YVO*RotMat3D(3,2)+ZVO*RotMat3D(3,3);
    vAgeHead(tii)=surf(XVO1+XYZ_g(tii,1,1), YVO1+XYZ_g(tii,2,1), ZVO1+XYZ_g(tii,3,1),... %making heading indicator
                 'FaceColor','k','FaceAlpha',0.2,'EdgeColor','c','EdgeAlpha',0.5);
end


%run it for sim time
for sii = 1:length(RecUVW_g(1).Data)-1
    for tii = 1:AgentNumber
        set(vAge(tii),'XData',XAge+XYZ_g(tii,1,sii), ...
                      'YData',YAge+XYZ_g(tii,2,sii), ...
                      'ZData',ZAge+XYZ_g(tii,3,sii));
        pause(0.2)
    end
end



dfbd
RotMatVO = [cos(pi/2) 0 -sin(pi/2); 0 1 0; sin(pi/2) 0 cos(pi/2)];
XVO = cos(-pi/2)*Xunit2*RVO-sin(-pi/2)*Zunit2*DVO; 
YVO = Yunit2; 
ZVO = sin(-pi/2)*Xunit2*RVO+cos(-pi/2)*Zunit2*DVO;

XObsB = R2Bod0(1,1)*XObs+R2Bod0(1,2)*YObs+R2Bod0(1,3)*ZObs; 
YObsB = R2Bod0(2,1)*XObs+R2Bod0(2,2)*YObs+R2Bod0(2,3)*ZObs; 
ZObsB = R2Bod0(3,1)*XObs+R2Bod0(3,2)*YObs+R2Bod0(3,3)*ZObs;
XVOB = R2Bod0(1,1)*XVO+R2Bod0(1,2)*YVO+R2Bod0(1,3)*ZVO; 
YVOB = R2Bod0(2,1)*XVO+R2Bod0(2,2)*YVO+R2Bod0(2,3)*ZVO; 
ZVOB = R2Bod0(3,1)*XVO+R2Bod0(3,2)*YVO+R2Bod0(3,3)*ZVO;
%VISUALIZATION  =======================================================

vObsV = surf(XObsB+VioB(1), YObsB+VioB(2), ZObsB+VioB(3),'FaceColor','k','FaceAlpha',0.2,'EdgeColor','c','EdgeAlpha',0.5);
vDistV =line([0 ObsPosB(1)],[0 ObsPosB(2)],[0 ObsPosB(3)],'linewidth',1,'linestyle','-.','color','k'); 
vVooV = line([0 VooB(1)],[0 VooB(2)],[0 VooB(3)],'linewidth',1,'Marker','d','color','g');
vVioV = line([ObsPosB(1) VioB(1)+ObsPosB(1)],[ObsPosB(2) VioB(2)+ObsPosB(2)],[ObsPosB(3) VioB(3)+ObsPosB(3)],'linewidth',1,'Marker','d','color','r');
vVio2V = line([0 VioB(1)],[0 VioB(2)],[0 VioB(3)],'linewidth',1,'Marker','d','color','r');
%vRelVV = line([0 RelVelB(1)],[0 RelVelB(2)],[0 RelVelB(3)],'linewidth',2,'Marker','d');
vVO1V = surf(XVOB+VioB(1), YVOB+VioB(2), ZVOB+VioB(3),'FaceColor','k','FaceAlpha',0.2,'EdgeColor','c','EdgeAlpha',0.5);

%========================================================================