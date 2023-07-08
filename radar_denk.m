function [snr_cikis] = radar_denk(p_tepe,lambda,g,sigma,te,b,nf,l,menzil)
p_tepe_db=10*log10(p_tepe);
lambda_kare_db = 10*log10(lambda^2);
sigma_db=10*log10(sigma);
dort_pi_kup_db = 10*log10((4.0 * pi)^3);
k_db=10*log10(1.38e-23);
t_db=10*log10(te);
b_db=10*log10(b);
menzil_db=10*log10(menzil.^4);
pay=p_tepe_db + 2*g + lambda_kare_db + sigma_db;
payda = dort_pi_kup_db + k_db + t_db + b_db + nf + l + menzil_db;
snr_cikis = pay - payda;
return

