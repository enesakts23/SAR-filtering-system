
r= input('Yarıçap : ');
if isempty(r)
r = (1.0) ;
end
h= input('Yükseklik : ');
if isempty(h)
h = [10.0] ;
end
frekans= input('Frekans : ');
if isempty(frekans)
frekans = [3] ;
end
frekans=frekans*10^9;
eps =0.00001;
a = pi/180;
dalgaboyu = 3.0e+8 /frekans;
b = 0;
for teta = 0.0:.1:89.9
b = b +1;
tetaradyan = teta * a;
rcs(b) = (dalgaboyu * r * sin(tetaradyan) / (8. * pi * (cos(tetaradyan))^2))+ eps;
end
tetaradyan = pi/2;
b = b +1;
rcs(b) = (2. * pi * h^2 * r / dalgaboyu )+ eps;
for teta = 90.1:.1:180
b = b + 1;
tetaradyan = teta * a;
rcs(b) = ( dalgaboyu * r * sin(tetaradyan) / (8. * pi * (cos(tetaradyan))^2))+ eps;
end
uzunluk= 180/(b-1);
cizimacisi = 0:uzunluk:180;
plot(cizimacisi,10*log10(rcs),'k','linewidth',1.5);
grid;
xlabel ('Yon acisi, Teta [Derece]');
ylabel ('RCS - desibel');
title (' Silindir');
