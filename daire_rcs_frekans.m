r= input('Yarıçap : ');
if isempty(r)
r = (10000.0) ;
end
frekans= input('Frekans : ');
if isempty(frekans)
frekans = (3) ;
end
frekans=frekans*10^9;
eps = 0.000001;
dalgaboyu = 3.e+8 / frekans;
a = 0;
for aci= 0.:.1:180
a = a +1;
aciradyan = (pi /180.) * aci;
if (aciradyan == 0 | aciradyan == pi)
rcs(a) = (4.0 * pi^3 * r^4 / dalgaboyu^2) + eps;
else
    rcs(a) = ((dalgaboyu * r) / (8. * pi * sin(aciradyan) * (tan(aciradyan))^2)) + eps;
end
end
rcsdesibel = 10 * log10(rcs);
aci2 = 0:.1:180;
plot(aci2,rcsdesibel,'k','linewidth',1.5);
grid;
xlabel ('aci - derece');
ylabel ('RCS - desibel');
title (['RCS Daire ']);

