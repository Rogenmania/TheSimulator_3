clear all;
close all;
load('RecordVO.mat')
VOpPoiX = zeros(AgentNumber,AgentNumber-1,...
                length(VOpVee),length(VOpPo),RecVOpVe(1).Elapsed-1);
VOpPoiY = VOpPoiX;
VOpPoiN = VOpPoiX; 
VOpPoiS = VOpPoiX;
    
for ii = 1:AgentNumber
    nn = 1;
    oo = 1;
    for jj = 1:AgentNumber-1
        for kk = 1:length(VOpVee)
            for ll = 1:length(VOpPo)
                for mm = 1:RecVOpVe(1).Elapsed-1
                    %some(agnetNo,xyz,time)
                    aa = size(RecVOpVe(ii).Data(nn,mm));
                    VOpPoiX(ii,jj,kk,ll,mm) = RecVOpVe(ii).Data(nn,mm);
                    VOpPoiY(ii,jj,kk,ll,mm) = RecVOpVe(ii).Data(nn+1,mm);
                    VOpPoiN(ii,jj,kk,ll,mm) = RecVOpVe(ii).Data(nn+2,mm);
                    VOpPoiS(ii,jj,kk,ll,mm) = RecVOpVe(ii).Data(nn+3,mm);
                end
                nn = nn+4;
            end
        end

    end
end

for mm = 1:RecVOpVe(1).Elapsed-1
    for ii = 1:AgentNumber
        oo = 1;
        for kk = 1:length(VOpVee)
            VOpPoiA(ii,kk,1,mm) = RecVOpVe2(ii).Data(oo,mm);
            VOpPoiA(ii,kk,2,mm) = RecVOpVe2(ii).Data(oo+1,mm);
            oo = oo+2;
            for jj = 1:AgentNumber-1
                VOpPoiVi(ii,jj,kk,1,mm) = RecVOpVe2(ii).Data(oo,mm);
                VOpPoiVi(ii,jj,kk,2,mm) = RecVOpVe2(ii).Data(oo+1,mm); 
                oo = oo+2;
            end 
        end
    end
end


%for first agent, make 2 x 2 subplot
Poi = zeros(2,length(VOpPo));
Col = 'brmgk';
age = 2;
figure(30); 
for jj = 1:AgentNumber-1
    for kk = 1:12
        subplot(4,3,kk); grid on;
        
        Poi(1,:) = VOpPoiX(age,jj,kk,:,1);
        Poi(2,:) = VOpPoiY(age,jj,kk,:,1);
        PoiTG(1) = VOpPoiA(age,kk,1,1);
        PoiTG(2) = VOpPoiA(age,kk,2,1);
        VOpPol(jj,kk) = line(Poi(1,:),Poi(2,:),'Color',Col(jj));
        VOpTGPol(jj,kk) = line([0 PoiTG(1)],[0 PoiTG(2)],'Color',[0 0.5 0],'Marker','d');
        axis([-2 3 -1 1]); hold on;
    end
end

%move it?
for mm = 2:RecVOpVe(1).Elapsed-1
    for jj = 1:AgentNumber-1
        for kk = 1:12
            PoiX(1,:) = VOpPoiX(age,jj,kk,:,mm);
            PoiX(2,:) = VOpPoiY(age,jj,kk,:,mm);
            set(VOpPol(jj,kk),'XData',PoiX(1,:),'YData',PoiX(2,:))    
            PoiTG(1) = VOpPoiA(age,kk,1,mm);
            PoiTG(2) = VOpPoiA(age,kk,2,mm);
            set(VOpTGPol(jj,kk),'XData',[0 PoiTG(1)],'YData',[0 PoiTG(2)])  
        end
    end
    pause(0.5)
end

