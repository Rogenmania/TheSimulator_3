
%the Cube scenario --> 8 agent
iVo = 2;
iDist = 10;
AgentNumber = 8;

XYZ_g = [-1 1 1 -1 -1 1 1 -1;
          1 1 -1 -1 1 1 -1 -1;
          1 1 1 1 -1 -1 -1 -1]*iDist/3*3^0.5;
      
UVW_g = [1 -1 -1 1 1 -1 -1 1;
         -1 -1 1 1 -1 -1 1 1;
         -1 -1 -1 -1 1 1 1 1]*iVo/3*3^0.5;
     
UVW_b = [iVo iVo iVo iVo iVo iVo iVo iVo;
         0 0 0 0 0 0 0 0; 
         0 0 0 0 0 0 0 0];
     
VTP_g = zeros(3,8);
for ii = 1:8
    VTP_g(:,ii) = [0; 
        -atan2(UVW_g(3,ii),((UVW_g(2,ii)^2+UVW_g(1,ii)^2)^0.5)); 
        atan2(UVW_g(2,ii),UVW_g(1,ii))];
end

XYZsta_g = XYZ_g - UVW_g*Tifin/1000;
XYZfin_g = XYZ_g + UVW_g*Tifin;

DecMode = ones(AgentNumber,1)*2;
ADist = ones(AgentNumber,1)*10;




