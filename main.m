p_tepe = 2.0e+6;
lambda = 0.03;
g = 35.0;
sigma = 5;
te = 290.0;
b = 2.0e+6 ;
nf = 2.0;
l=2.0;
menzil = linspace(10e3,200e3,200);
snr = radar_denk(p_tepe, lamda, g, sigma, te, b, nf, l, menzil);
figure;
menzilkm = menzil ./ 1000;
plot(menzilkm,snr,'k')
grid
xlabel('Tespit Menzili(km) - km');
ylabel('SNR - dB');
