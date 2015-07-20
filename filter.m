%PID Control with Gradual approaching input value
clear all;
close all;


kf_Xd=0;kf_Xs=0;
kf_Y=0;kf_Ys=0;
kf_P=0;kf_Ps=0.0001;
kf_K=0;
kf_R=0.88;
kf_Q=0.22;


for k=1:1:10
    rin(k)=9;
    kf_Ys = kf_Xs;
    kf_Y = rin(k);
    
    kf_K = kf_Ps/(kf_Ps + kf_R);
    kf_P = kf_Ps - kf_K*kf_Ps;
    kf_Xd = kf_Xs + kf_K*(kf_Y - kf_Ys);
    
    kf_Xs = kf_Xd;
    kf_Ps = kf_P + kf_Q;
    yout(k)=kf_Xd;
end

figure(1);
plot(yout,'r');

