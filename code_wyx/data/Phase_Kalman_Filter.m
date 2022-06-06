function [x_e_k2,P]=Phase_Kalman_Filter(x_e_k1,x_real_k2,P)
    % x_e_k1 上一时刻状态估计 [phase_last_term;phase_d_last_term]
    % x_reak_k2 当前时刻观测量   [phase_obs;phase_d]
    R=[0.5,0;0,5];
    Q=[0,0;0,1e-4];
    A=[1,1;0,1];
    H=[1,0;0,1];
    % Prediction
    x_e_k2=A*x_e_k1;% 模型预测值
    P=A*P*A'+Q;
    % Update
    K=0;
    W=0;
    x_hat = x_e_k2;
    for i=1:2
        K=P*H'/(H*P*H'+R);
        W=K*(x_real_k2-x_e_k2-H*(x_hat-x_e_k2));
        x_e_k2=x_hat+W;
        P=([1,0;0,1]-K*H)*P;
    end
end