function y=min_rate_of_mmWave(N,d,r,e,t) 
% ����V2V��·����ͨ�ŵ���С�Ĵ�������
% N ���ݰ�ƽ������  6400
% d ���ݰ������ʱ  50ms=0.05s
% r ���ݰ��ĵ�����  0.01Packets/ms=10^-5
% e ��С�����ж���  0.05
% t ��׼ʱ��
d=d-t;
k=r*d/(1-exp(r*d));
x=k*e*exp(k);
z=lambertw(-1,x);
y=-N/d*(z-k);
    
end