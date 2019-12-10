clear all;
close all;
clc;
%生成两种不同频率的正弦信号频率分别为150KHz、140KHz。采样率为750KHz，采样点数为512个点，信噪比为20dB
%Step1:----------输入观测数据x0,x1,...,xN -------------------
f0 = 150e3;             %定义信号的频率
f1 = 140e3;
fs = 5*f0;               %定义采样率
points = 512;           %采样点数
signal_points = 512;   %信号的长度
SNR = 20;                %信噪比
 
nTs = (0:points-1)/fs;
Delta_f = (0:points-1)*fs/points;
 
signal = sin(2*pi*f0*nTs)+sin(2*pi*f1*nTs);
signal(signal_points:points) = 0;
signal_noise = signal + 10^(-SNR/20)*randn(1,points);


%Step2:-------选定窗宽L=128,步长p=8,分组数K=113----------------
M = 32;                                %每段的点数
p= M/16;                                %步长
Overlap = M-p;                          %重叠点数
L = (points - Overlap)/(M-Overlap);     %段数(分组数)
Welch_Spectrum = zeros(1,points);       %初始化

%Step3:--------------Welch方法计算过程-------------------------
for i=1:L
    temp_signal = zeros(1,points);                                 %清零
    start_point = (i-1)*(M-Overlap);
    temp_signal(1:M) = signal_noise((start_point+1):(start_point+M));  %截取第i段信号
    fft_temp_signal = fft(temp_signal);                                %做傅里叶变换
    squar_fft_temp_signal = fft_temp_signal.*conj(fft_temp_signal);    %求当前段的功率谱
    Welch_Spectrum = Welch_Spectrum + squar_fft_temp_signal;           %累加
end
max_Welch_Spectrum = max(Welch_Spectrum);
Welch_Spectrum = Welch_Spectrum/max_Welch_Spectrum;           %归一化
log10_Welch_Spectrum = 10*log10(Welch_Spectrum);              %求对数
Delta_f3 = (0:points-1)*fs/points;

%Step4:--------------------画图--------------------------------
figure;
plot(Delta_f3(1:points/2)/1000,log10_Welch_Spectrum(1:points/2));
xlabel('KHz');
ylabel('归一化功率谱P(k)/dB');
string = ['段数L=',num2str(L),',每段点数M=',num2str(M),',重叠点数=',...
          num2str(Overlap)];
title(string);
saveas(gcf,'D:\研究生课程资料\信号学\Welch3.jpg')