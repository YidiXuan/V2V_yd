function [Power_list_of_d2d] = Power_control(r,cue_csi,d2d_csi,pc,n)
%��D2D�û����й��ʿ���
%rΪƽ��sinr��������
%cue_sinrΪcue��d2d����·״̬��Ϣ������
%d2d_csiΪD2D�û�֮�����·״̬��Ϣ��ά����
cue_csi = cue_csi';

Power_list_of_d2d = r / d2d_csi * (pc * cue_csi + n);
end

