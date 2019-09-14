function y=min_rate_of_mmWave(N,d,r,e,t) 
% 计算V2V链路正常通信的最小的传输速率
% N 数据包平均长度  6400
% d 数据包的最长延时  50ms=0.05s
% r 数据包的到达率  0.01Packets/ms=10^-5
% e 最小允许中断率  0.05
% t 对准时延
d=d-t;
k=r*d/(1-exp(r*d));
x=k*e*exp(k);
z=lambertw(-1,x);
y=-N/d*(z-k);
    
end