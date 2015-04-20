[Xunit,Yunit,Zunit] = sphere;
XObs = Xunit*Rsep+Dist; YObs = Yunit*Rsep; ZObs = Zunit*Rsep;

[Xunit2,Yunit2,Zunit2] = cylinder(0:1);
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
figure(1)
set(gca,'ZDir','reverse');
vObs1 = surf(XObs, YObs, ZObs,'FaceColor','k','FaceAlpha',0.2,'EdgeColor','c','EdgeAlpha',0.5);
vDist =line([0 ObsPos(1)],[0 ObsPos(2)],[0 ObsPos(3)],'linewidth',1,'linestyle','-.','color','k'); 
vVoo = line([0 Voo(1)],[0 Voo(2)],[0 Voo(3)],'linewidth',1,'Marker','d');
vVio = line([Dist Vio(1)+Dist],[0 Vio(2)],[0 Vio(3)],'linewidth',1,'Marker','d','color','r');
vVio2 = line([Voo(1) -Vio(1)+Voo(1)],[Voo(2) -Vio(2)+Voo(2)],[Voo(3) -Vio(3)+Voo(3)],'linewidth',1,'Marker','d','color','r');
vRelV = line([0 RelVel(1)],[0 RelVel(2)],[0 RelVel(3)],'linewidth',2,'Marker','d');
hold on; axis equal; axis([0 Dist+Rsep -2*Rsep 2*Rsep -2*Rsep 2*Rsep]);
vVO1 = surf(XVO, YVO, ZVO,'FaceColor','k','FaceAlpha',0.2,'EdgeColor','c','EdgeAlpha',0.5);

figure(2)
set(gca,'ZDir','reverse');
hold on; grid on; axis equal
vObsB = surf(XObsB, YObsB, ZObsB,'FaceColor','k','FaceAlpha',0.2,'EdgeColor','c','EdgeAlpha',0.5);
vDistB =line([0 ObsPosB(1)],[0 ObsPosB(2)],[0 ObsPosB(3)],'linewidth',1,'linestyle','-.','color','k'); 
vVooB = line([0 VooB(1)],[0 VooB(2)],[0 VooB(3)],'linewidth',1,'Marker','d','color','g');
vVioB = line([ObsPosB(1) VioB(1)+ObsPosB(1)],[ObsPosB(2) VioB(2)+ObsPosB(2)],[ObsPosB(3) VioB(3)+ObsPosB(3)],'linewidth',1,'Marker','d','color','r');
vVio2B = line([VooB(1) -VioB(1)+VooB(1)],[VooB(2) -VioB(2)+VooB(2)],[VooB(3) -VioB(3)+VooB(3)],'linewidth',1,'Marker','d','color','r');
vRelVB = line([0 RelVelB(1)],[0 RelVelB(2)],[0 RelVelB(3)],'linewidth',2,'Marker','d');
vVO1B = surf(XVOB, YVOB, ZVOB,'FaceColor','k','FaceAlpha',0.2,'EdgeColor','c','EdgeAlpha',0.5);

figure(3)
set(gca,'ZDir','reverse');
hold on; grid on; axis equal
vObsV = surf(XObsB+VioB(1), YObsB+VioB(2), ZObsB+VioB(3),'FaceColor','k','FaceAlpha',0.2,'EdgeColor','c','EdgeAlpha',0.5);
vDistV =line([0 ObsPosB(1)],[0 ObsPosB(2)],[0 ObsPosB(3)],'linewidth',1,'linestyle','-.','color','k'); 
vVooV = line([0 VooB(1)],[0 VooB(2)],[0 VooB(3)],'linewidth',1,'Marker','d','color','g');
vVioV = line([ObsPosB(1) VioB(1)+ObsPosB(1)],[ObsPosB(2) VioB(2)+ObsPosB(2)],[ObsPosB(3) VioB(3)+ObsPosB(3)],'linewidth',1,'Marker','d','color','r');
vVio2V = line([0 VioB(1)],[0 VioB(2)],[0 VioB(3)],'linewidth',1,'Marker','d','color','r');
%vRelVV = line([0 RelVelB(1)],[0 RelVelB(2)],[0 RelVelB(3)],'linewidth',2,'Marker','d');
vVO1V = surf(XVOB+VioB(1), YVOB+VioB(2), ZVOB+VioB(3),'FaceColor','k','FaceAlpha',0.2,'EdgeColor','c','EdgeAlpha',0.5);

%========================================================================