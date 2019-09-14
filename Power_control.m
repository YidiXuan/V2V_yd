function [Power_list_of_d2d] = Power_control(r,cue_csi,d2d_csi,pc,n)
%对D2D用户进行功率控制
%r为平均sinr限制条件
%cue_sinr为cue对d2d的链路状态信息列向量
%d2d_csi为D2D用户之间的链路状态信息二维数组
cue_csi = cue_csi';

Power_list_of_d2d = r / d2d_csi * (pc * cue_csi + n);
end

