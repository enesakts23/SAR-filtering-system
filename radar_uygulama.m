
function radar_uygulama
uzunluk = input('Uzunluk :');
if isempty(uzunluk)
uzunluk = (400000) ;
end
genislik= input('Genişlik :');
if isempty(genislik)
genislik = (30000) ;
end
frekans= input('Frekans :');
if isempty(frekans)
frekans = (24) ;
end
frekans=frekans*10^9;
eps = 0.000001;
dalgaboyu = 3.0e+8 /frekans;
ka = 2. * pi * uzunluk / dalgaboyu;
teta = 0.05:0.1:85;
tetaradyan = (pi/180.) .* teta;
dikey1 = cos(ka .*sin(tetaradyan)) - 1i .* sin(ka .*sin(tetaradyan)) ./sin(tetaradyan);
dikey2 = exp(1i * ka - (pi /4)) / (sqrt(2 * pi) *(ka)^1.5);
dikey3 = (1. + sin(tetaradyan)) .* exp(-1i * ka .* sin(tetaradyan)) ./ (1. - sin(tetaradyan)).^2;
dikey4 = (1. - sin(tetaradyan)) .* exp(1i * ka .* sin(tetaradyan)) ./ (1. + sin(tetaradyan)).^2;
dikey5 = 1. - (exp(1i * 2. * ka - (pi / 2)) / (8. * pi * (ka)^3));
yatay1 = cos(ka .*sin(tetaradyan)) + 1i .* sin(ka .*sin(tetaradyan)) ./ sin(tetaradyan);
yatay2 = 4. * exp(1i * ka + (pi / 4.)) / (sqrt(2 * pi * ka));
yatay3 = exp(-1i * ka .* sin(tetaradyan)) ./ (1. - sin(tetaradyan));
yatay4 = exp(1i * ka * sin(tetaradyan)) ./ (1. + sin(tetaradyan));
yatay5 = 1. - (exp(1i *2. * ka + (pi / 2.)) / 2. * pi * ka);
rcsdikey = (genislik^2 / pi) .* (abs(dikey1 - dikey2 .*((1. ./ cos(tetaradyan)) +.25 .* dikey2 .* (dikey3 + dikey4)) .* (dikey5).^-1)).^2 + eps;
rcsyatay = (genislik^2 / pi) .* (abs(yatay1 - yatay2 .*((1. ./ cos(tetaradyan)) -.25 .* yatay2 .* (yatay3 + yatay4)) .* (yatay5).^-1)).^2 + eps;
rcsdikeydb = 10. .*log10(rcsdikey);
rcsyataydb = 10. .*log10(rcsyatay);
figure(1);
plot (teta, rcsdikeydb,'k','linewidth',1.5);
grid;
title (('Dikey Polarma'));
ylabel ('RCS -desibel');
xlabel ('Yön Acisi - derece');
figure(2);
plot (teta, rcsyataydb,'k','linewidth',1.5);
grid;
title (('Yatay Polarma'));
ylabel ('RCS -desibel');
xlabel ('Yön Acisi - derece');

