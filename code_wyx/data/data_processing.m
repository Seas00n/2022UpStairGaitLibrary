clear all;clc;close all;
load data_wyx.mat
% ax ay az wx wy wz anglex angley anglez (in degree)
figure(1)
% auto_gait_division基于（大腿IMU）z轴加速度划分步态
% input：data,num_of_step,acc_threshold,near_threshold
%       data：采集到的数据
%       num_of_steps: 提取前多少步的数据
%       acc_threshold: z轴加速度*20大于多少时判断为触地(建议使用足跟IMU进行相位分割)
%       near_threshold:
%       大于分割阈值的z轴加速度峰值往往集中出现，峰值相邻小于该参数的将被判断为处于统一cluster，
%       在cluster内取最大值作为实际的相位分割点
% output: 
%       idx: 触地对应的时间戳的index
idx1=auto_gait_division(data1,8,11.9,30);
idx1=idx1(2:end-1);
figure(2)
idx2=auto_gait_division(data2,8,13.6,40);
idx2=idx2(2:end-1);
figure(3)
idx3=auto_gait_division(data3,8,11.5,40);
idx3=idx3(2:end-1);
figure(4)
idx4=auto_gait_division(data4,8,13.7,40);
idx4=idx4(2:end-1);
figure(5)
idx5=auto_gait_division(data5,8,10.9,40);
idx5=idx5(2:end-1);
figure(6)
% gait_interp: 步内插值处理函数,每一步被插值为101个时间帧（0-100）
% input: 
%   data:采集数据
%   idx:步态分割点序号
%   steps: 取前多少步
% output: 
%   angle:矩阵，101*steps
angle = gait_interp(data1,idx1,5);
angle = [angle,gait_interp(data2,idx2,5)];
angle = [angle,gait_interp(data3,idx3,5)];
angle = [angle,gait_interp(data4,idx4,5)];
angle = [angle,gait_interp(data5,idx5,5)];
% shadedErrorBar: （'—'）好用的曲线绘制函数（参考网上相关代码）
angle_center=shadedErrorBar(0:100,angle',{@mean,@std},'lineProps',{'b','Linewidth',2});
hold on
plot(0:100,angle');
% change_range: 调整支撑相和摆动相的相位长度，便于更好地估计相位
% input: angle, idx_stance_end, idx_swing_end
% 简单的内部插值，不讲了（'—'）
angle_center = change_range(angle_center,63,92);
figure(7)
[stance_end_threshold,stance_end_idx]= min(angle_center(1:70));
[swing_end_threshold,swing_end_idx]=max(angle_center(stance_end_idx:end));
% 拟合相位时需要初始值，这里使用之前已有的超参数
popt_list = readNPY('popt_list.npy');
phase_vec = zeros(1,101);
idx_list = [1,stance_end_idx,swing_end_idx+stance_end_idx-1,101];
popt_list_new = zeros(3,4);
% 使用sigmoid函数拟合超参数(建议仔细阅读)
for i =1:3
    x=idx_list(i):idx_list(i+1)-1;
    [popt_list_new(i,:),phase_vec(x)] = fit_phase_of_monotonous_vec(x',angle_center(x)',popt_list(i,:));
end
phase_vec(end)=100;
idx_list = idx_list-1;
figure(8)
plot(1:size(phase_vec,2),phase_vec,'LineWidth',2);
hold on
% 基于Kalman滤波的相位平滑(建议仔细阅读)
phase_vec = phase_filter(phase_vec);
plot(1:size(phase_vec,2),phase_vec,'LineWidth',2);
xlim([0,100])
function angle=gait_interp(data,idx,n)
    angle = zeros(101,n);
    q_mean = mean(data(1:50,7));
    for i=1:n
        q = data(idx(i):idx(i+1),7);
        x = 1:size(q,1);
        xx = linspace(1,size(q,1),101)';
        angle(:,i) = spline(x,q,xx);
    end
    angle = angle-q_mean;
end



function idx = auto_gait_division(data,num_of_step,acc_threshold,near_threshold)
    plot(1:size(data,1),data(:,7))
    hold on
    plot(1:size(data,1),20*data(:,3))
    idx_pre = find(data(:,3)*20>acc_threshold);
    scatter(idx_pre,20*data(idx_pre,3),'cyan','filled')
    if size(idx_pre)==num_of_step+1
        idx = idx_pre;
    else
        custer = zeros(near_threshold,num_of_step);
        acc_custer = zeros(near_threshold,num_of_step);
        p=1;p_=1;
        idx_new = zeros(num_of_step,1);
        for i=1:num_of_step
            custer(1,i) = idx_pre(p);
            acc_custer(1,i) = 20*data(idx_pre(p),3);
            if p<size(idx_pre,1)
                while idx_pre(p+p_)-idx_pre(p)<near_threshold
                    custer(p_+1,i) = idx_pre(p+p_);
                    acc_custer(p_+1,i) = 20*data(idx_pre(p+p_),3);
                    p_ = p_+1;
                    if p+p_>size(idx_pre,1)
                        break
                    end
                end
            end
            p = p+p_;
            p_ = 1;
            [~,p_max] = max(acc_custer(:,i));
            idx_new(i) = custer(p_max,i); 
            scatter(idx_new(i),20*data(idx_new(i),3),'magenta','filled')
        end
        idx = idx_new;
    end
end
function [popt,phase] = fit_phase_of_monotonous_vec(x,y,beta0)
    sig = @(popt,x)popt(1)./(1+exp(-popt(3)*(x-popt(2))))+popt(4);
    sig_inv=@(popt,y)popt(2)-log(popt(1)./(y-popt(4))-1)/popt(3);
    plot(x-1,sig(beta0,x-1),'blue','linewidth',1,'LineStyle','-')
    hold on
    plot(x-1,y,'red','linewidth',2,'LineStyle','-');
    opt=statset('MaxIter',300);
    if y(1)==y(2)
        y(1)=y(1)+0.05;
    end
    popt = nlinfit(x-1,y,sig,beta0,opt);
    [popt,~]=fmincon(@(beta)new_target_fun(beta,y,x-1),popt);
    plot(x-1,sig(popt,x-1),'*','color',[0,0.4,0.4],'LineWidth',0.5);
    phase = abs(sig_inv(popt,y));
    phase(phase>100) = 100;
    grid on
end
function error=new_target_fun(popt,y,x)
    phase = popt(2)-log(popt(1)./(y-popt(4))-1)/popt(3);
    for i=2:size(phase,1)
        if phase(i)<phase(i-1)
            phase(i) = phase(i-1);
        end
    end
    delta = abs(phase-x);
    delta_max = max(delta);
    error = 0.01*sum(delta'*delta)+1*delta_max/(x(end)-x(1));
    error = abs(error);
end
function phase_filtered = phase_filter(phase)
    phase_filtered = zeros(2,size(phase,1));
    phase_filtered(:,1) = [0;0];
    P=0.5*[0.5,0;0,0.1];
    for i=2:size(phase,2)
        phase_measurement = [phase(i);phase(i)-phase_filtered(i-1)];
        if phase_filtered(2,i-1)>0.8
            phase_filtered(2,i-1) = 0.5;
        end
        phase_filtered(:,i)=Phase_Kalman_Filter([phase_filtered(:,i-1)],phase_measurement,P);
    end
   phase_filtered = phase_filtered(1,:);
   phase_filtered(1) = 0;
   phase_filtered(phase_filtered>100) = 100;
end
function angle = change_range(angle,new_stance_idx,new_swing_idx)
    [~,stance_end_idx]= min(angle(1,:));
    [~,swing_end_idx] = max(angle(1,stance_end_idx+1:end));
    xx=linspace(1,stance_end_idx,new_stance_idx);
    angle1 = spline(1:stance_end_idx,angle(:,1:stance_end_idx),xx);
    xx=linspace(stance_end_idx,swing_end_idx+stance_end_idx,new_swing_idx-new_stance_idx+1);
    angle2 = spline(stance_end_idx:swing_end_idx+stance_end_idx,angle(:,stance_end_idx:swing_end_idx+stance_end_idx),xx);
    xx=linspace(swing_end_idx+stance_end_idx,101,101-size(angle1,2)-size(angle2,2));
    angle3 = spline(swing_end_idx+stance_end_idx:101,angle(:,stance_end_idx+swing_end_idx:101),xx);
    angle=[angle1,angle2,angle3];
end