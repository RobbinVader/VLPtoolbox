% 根据原始VLP数据计算高频RSS，没有加汉明窗

function [t,fft_result]=VLP_preprocess_highfreq(vlp,nLED,fhz,fs,dt,fre)

Ntotal=length(vlp);

% FFT parameter
N=dt*fs; %采集点个数
num_point=floor(Ntotal/N)*fre;
fft_result=zeros(num_point,nLED);
n=0:N-1;
t=(1:num_point)*dt/fre;

for k=0:(num_point-1)
    %傅里叶变换
    if(k<fre/2)
        s=vlp(1:N);
    elseif(N+(k-fre/2)*N/fre>=Ntotal)
        s=vlp(Ntotal-N+1:Ntotal);
    else
        s=vlp(1+(k-fre/2)*N/fre:N+(k-fre/2)*N/fre);
    end
    F=fft(s,N);
    mag=abs(F);
    mag=mag*2/N;
    f=n*fs/N;
    for i=1:nLED
        fft_result(k+1,i) = mag(int32((fhz(i))*N/fs)+1)/1.27;
    end
end