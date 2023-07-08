fc=8e8;
To=10e-6;
N=200;
td=2e-6;
t=0:To/(10*N-1):To;
tt=t*1e6;
s=5*ones(1,N);
s(10*N)=0;
sr=s;
M=round(td/To*(10*N));
ss=circshift(sr.',M);
figure;
h=area(tt,sr);
set(h,'FaceColor',[.5 .5 .5])
set(gca,'FontName','Arial','FontSize',12,'FontWeight','Bold');
title('İletilen sinyal');
xlabel('Time [\mus]')
axis([min(tt),max(tt)-30 30]);
figure; %figure 1 de 4.1 e 4.2 yede figure 2 de 
h=area(tt,ss);
set(h,'FaceColor',[.5 .5 .5])
set(gca,'FontName','Arial','FontSize',12,'FontWeight','Bold');
title('Gürültüsüz Alınan Sinyal ');
xlabel('Time [\mus]')
%axis([min(tt)max(tt)-30 30]); %düzenle%
n=5*randn(1,10*N);
figure;
h=area(tt,n);
set(h,'FaceColor',[.5 .5 .5])
set(gca,'FontName','Arial','FontSize',12,'FontWeight','Bold');
xlabel('Time[\mus]'),
title('Gürültülü Sinyal');
axis([min(tt),max(tt)-30 30]);
x=ss+n;
figure;
h=area(tt,x);
set(h,'FaceColor',[.5 .5 .5])
set(gca,'FontName','Arial','FontSize',12,'FontWeight','Bold');
xlabel('Time[\mus]'),
title('Gürültülü Alınan Sinyal');
axis([min(tt),max(tt)-30 30]);
X=fft(x)/N;
S=conj(fft(sr)/N);
H=S;
Y=X.*H;
y=ifft(Y);
figure;
h=area(tt,real(y));
set(h,'FaceColor',[.5 .5 .5])
set(gca,'FontName','Arial','FontSize',12,'FontWeight','Bold');
xlabel('Time[\mus]'),
axis([min(tt),max(tt)-.1 2]);
title('Eşleşen Filtre Çıktısı');







