clear all;
close all;
clc;
%�������ֲ�ͬƵ�ʵ������ź�Ƶ�ʷֱ�Ϊ150KHz��140KHz��������Ϊ750KHz����������Ϊ512���㣬�����Ϊ20dB
%Step1:----------����۲�����x0,x1,...,xN -------------------
f0 = 150e3;             %�����źŵ�Ƶ��
f1 = 140e3;
fs = 5*f0;               %���������
points = 512;           %��������
signal_points = 512;   %�źŵĳ���
SNR = 20;                %�����
 
nTs = (0:points-1)/fs;
Delta_f = (0:points-1)*fs/points;
 
signal = sin(2*pi*f0*nTs)+sin(2*pi*f1*nTs);
signal(signal_points:points) = 0;
signal_noise = signal + 10^(-SNR/20)*randn(1,points);


%Step2:-------ѡ������L=128,����p=8,������K=113----------------
M = 32;                                %ÿ�εĵ���
p= M/16;                                %����
Overlap = M-p;                          %�ص�����
L = (points - Overlap)/(M-Overlap);     %����(������)
Welch_Spectrum = zeros(1,points);       %��ʼ��

%Step3:--------------Welch�����������-------------------------
for i=1:L
    temp_signal = zeros(1,points);                                 %����
    start_point = (i-1)*(M-Overlap);
    temp_signal(1:M) = signal_noise((start_point+1):(start_point+M));  %��ȡ��i���ź�
    fft_temp_signal = fft(temp_signal);                                %������Ҷ�任
    squar_fft_temp_signal = fft_temp_signal.*conj(fft_temp_signal);    %��ǰ�εĹ�����
    Welch_Spectrum = Welch_Spectrum + squar_fft_temp_signal;           %�ۼ�
end
max_Welch_Spectrum = max(Welch_Spectrum);
Welch_Spectrum = Welch_Spectrum/max_Welch_Spectrum;           %��һ��
log10_Welch_Spectrum = 10*log10(Welch_Spectrum);              %�����
Delta_f3 = (0:points-1)*fs/points;

%Step4:--------------------��ͼ--------------------------------
figure;
plot(Delta_f3(1:points/2)/1000,log10_Welch_Spectrum(1:points/2));
xlabel('KHz');
ylabel('��һ��������P(k)/dB');
string = ['����L=',num2str(L),',ÿ�ε���M=',num2str(M),',�ص�����=',...
          num2str(Overlap)];
title(string);
saveas(gcf,'D:\�о����γ�����\�ź�ѧ\Welch3.jpg')