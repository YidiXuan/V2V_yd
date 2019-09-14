function y=min_rate_of_cell(N,d,r,e) 
% ����V2V��·����ͨ�ŵ���С�Ĵ�������
% N ���ݰ�ƽ������  N= 6400
% d ���ݰ������ʱ d=50 ms = 50/1000=0.05s 
% r ���ݰ��ĵ�����  r=0.01 packets/slot= 0.01 packets/ms = 10^-5 s
% e ��С�����ж���  e=0.05
% min_rate_of_cell(6400,0.05,10^-5,0.05)=6.07215e^5
k=r*d/(1-exp(r*d));
x=k*e*exp(k);
z=lambertw(-1,x);
y=-N/d*(z-k);
    
end