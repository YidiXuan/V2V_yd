function y=min_rate_of_cell(N,d,r,e) 
% 计算V2V链路正常通信的最小的传输速率
% N 数据包平均长度  N= 6400
% d 数据包的最长延时 d=50 ms = 50/1000=0.05s 
% r 数据包的到达率  r=0.01 packets/slot= 0.01 packets/ms = 10^-5 s
% e 最小允许中断率  e=0.05
% min_rate_of_cell(6400,0.05,10^-5,0.05)=6.07215e^5
k=r*d/(1-exp(r*d));
x=k*e*exp(k);
z=lambertw(-1,x);
y=-N/d*(z-k);
    
end