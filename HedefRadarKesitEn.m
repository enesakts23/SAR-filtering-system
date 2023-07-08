c = 3e8;
fc = 3e8;
pedestrian = (phased.RadarTarget('MeanRCS',1,'PropagationSpeed',c,...
    'OperatingFrequency',fc));
x = 1;
ped_echo = (pedestrian(x));
[cylrcs,az,el] = rcscylinder(1,1,10,c,fc);
helperTargetRCSPatternPlot(az,el,cylrcs);
title('RCS Pattern of Cylinder');
plot(el,pow2db(cylrcs));
grid; axis tight; ylim([-30 30]);
xlabel('Elevation Angles (degrees)');
ylabel('RCS (dBsm)');
title('RCS Pattern for Cylinder');
cylindricalTarget = (phased.BackscatterRadarTarget('PropagationSpeed',c,...
    'OperatingFrequency',fc,'AzimuthAngles',az,'ElevationAngles',el,...
    'RCSPattern',cylrcs));
x = [1 1 1];            % 3 unit signals
ang = [0 30 30;0 0 30]; % 3 directions
cyl_echo = (cylindricalTarget(x,ang));
scatpos = [-0.5 -0.5 0.5 0.5;0.5 -0.5 0.5 -0.5;0 0 0 0];
naz = numel(az);
nel = numel(el);
extrcs = zeros(nel,naz);
for m = 1:nel
    sv = steervec(scatpos,[az;el(m)*ones(1,naz)]);
    % sv is squared due to round trip in a monostatic scenario
    extrcs(m,:) = abs(sqrt(cylrcs(m,:)).*sum(sv.^2)).^2;
end
helperTargetRCSPatternPlot(az,el,extrcs);
title('RCS Pattern of Extended Target with 4 Scatterers');
extendedTarget = phased.BackscatterRadarTarget('PropagationSpeed',c,...
    'OperatingFrequency',fc,'AzimuthAngles',az,'ElevationAngles',el,...
    'RCSPattern',extrcs);

ext_echo = (extendedTarget(x,ang));
sweepaz = -90:90; % Azimuthal sweep across target
sweepel = 0;
[elg,azg] = meshgrid(sweepel,sweepaz);
sweepang = [azg(:)';elg(:)'];
x = ones(1,size(sweepang,2)); % unit signals

release(extendedTarget);
extNarrowbandSweep = extendedTarget(x,sweepang);

clf;
plot(sweepaz,pow2db(extNarrowbandSweep));
grid on; axis tight;
xlabel('Azimuth Angles (degrees)');
ylabel('RCS (dBsm)');
title(['RCS Pattern at 0^o Elevation ',...
    'for Extended Target with 4 Scatterers']);
bw = 0.10*fc; % Bandwidth is greater-than 5% of center frequency
fs = 2*bw;
modelFreq = (-80e6:1e6:80e6)+fc;
[modelCylRCS,modelAz,modelEl] = helperCylinderRCSPattern(c,modelFreq);
nf = numel(modelFreq);
naz = numel(modelAz);
nel = numel(modelEl);
modelExtRCS = zeros(nel,naz,nf);
for k = 1:nf
    for m = 1:nel
        pos = scatpos*modelFreq(k)/fc;
        sv = steervec(pos,[modelAz;modelEl(m)*ones(1,naz)]);
        % sv is squared due to round trip in a monostatic scenario
        modelExtRCS(m,:,k) = abs(sqrt(modelCylRCS(m,:,k)).*sum(sv.^2)).^2;
    end
end
widebandExtendedTarget = phased.WidebandBackscatterRadarTarget(...
    'PropagationSpeed',c,'OperatingFrequency',fc,'SampleRate',fs,...
    'AzimuthAngles',modelAz,'ElevationAngles',modelEl,...
    'FrequencyVector',modelFreq,'RCSPattern',modelExtRCS);
extWidebandSweep = widebandExtendedTarget(x,sweepang);

hold on;
plot(sweepaz,pow2db(extWidebandSweep));
hold off;
legend('Narrowband','Wideband');
cylindricalTargetSwerling1 = ...
    phased.BackscatterRadarTarget('PropagationSpeed',c,...
    'OperatingFrequency',fc,'AzimuthAngles',az,'ElevationAngles',el,...
    'RCSPattern',cylrcs,'Model','Swerling1');
N = 10000;
tgt_echo = zeros(1,N);
x = 1;
for m = 1:N
    tgt_echo(m) = cylindricalTargetSwerling1(x,[0;0],true);
end
p_echo = tgt_echo.^2; % Reflected power
p_n = cyl_echo(1)^2;

helperTargetRCSReturnHistogramPlot(p_echo,p_n)

