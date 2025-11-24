% RSS改正方法，基于状态量对FFT结果进行改正，不能加汉宁窗
% NED系下改正
% 2025.11.24

function dRSSs = RSS_corr(t,fft_result,common_t,p,LED,A,M)
T=1;hz=200;%目前仅允许FFT窗口1s、状态的采样率200hz，后续改成任意hz
dt=t(2)-t(1); % RSS采样间隔
[nled,~]=size(LED);
[tlength,~]=size(fft_result);
dRSSs=zeros(floor(tlength),nled);
start_ind= find(t > common_t(1), 1, 'first');
figure;
for i=1:nled
    for j=1:tlength
        % 高速载体改正
        dP1=0;dP2=0;dP3=0;dP4=0;
        if(t(j)<=common_t(1)+0.5 || t(j)>=common_t(end)-0.5) % 状态量时间段外
            continue;
        end
        % 积分
        for k=1:hz
            ind=(j-start_ind)*hz*dt+k-hz/2;
            r=p(ind,1:3);
            cnb=Euler2Dcm(p(ind,4),p(ind,5),p(ind,6));
            n_pd=cnb*[0;0;-1];
            n_led=[0;0;-1];
            D=LED(i,:)-r;
            if(ind==1)
                vel=[0,0,0];
                w=[0,0,0];
            else
                vel=(p(ind,1:3)-p(ind-1,1:3))*hz;
                w=(p(ind,4:6)-p(ind-1,4:6))*hz;
            end
            coef1=-cross(D,n_pd)/dot(D,n_pd);
            coef2=-n_pd/dot(n_pd,D)-M(i)*n_led/dot(n_led,D)+(3+M(i))*D'/norm(D)^2;
            coef3=-A(i)*cross(D,n_pd)*dot(n_led,D)^M(i)/norm(D)^(3+M(i));
            coef4=-A(i)*n_pd*dot(n_led,D)^M(i)/norm(D)^(3+M(i))...
                -A(i)*M(i)*n_led*dot(n_pd,D)*dot(n_led,D)^(M(i)-1)/norm(D)^(3+M(i))...
                +A(i)*(3+M(i))*dot(n_pd,D)*dot(n_led,D)^M(i)*D'/norm(D)^(5+M(i));
            if(dot(D,n_pd)/norm(D)<0.01 || dot(D,n_led)/norm(D)<0.01)
                continue;
            end
            
            % RSS可选：fft_results(ind,i)，RSSs(ind,i)，fft_result(j,i)
            if(k<=hz/2)
                % dP1=dP1+(k/hz)*RSSs(ind,i)*dot(coef1,cnb*w')/hz/T;
                % dP2=dP2+(k/hz)*RSSs(ind,i)*dot(coef2,vel)/hz/T;
                dP1=dP1+(k/hz)*dot(coef3,cnb*w')/hz/T;
                dP2=dP2+(k/hz)*dot(coef4,vel)/hz/T;
            else
                % dP3=dP3-((hz-k)/hz)*RSSs(ind,i)*dot(coef1,cnb*w')/hz/T;
                % dP4=dP4-((hz-k)/hz)*RSSs(ind,i)*dot(coef2,vel)/hz/T;
                dP3=dP3-((hz-k)/hz)*dot(coef3,cnb*w')/hz/T;
                dP4=dP4-((hz-k)/hz)*dot(coef4,vel)/hz/T;
            end
        end
        dRSSs(j,i)=dP1+dP2+dP3+dP4;
    end
    subplot(nled,1,i);
    plot(t,dRSSs(:,i));
    ylabel(['LED',num2str(i)]);
end
xlabel('Time (s)');