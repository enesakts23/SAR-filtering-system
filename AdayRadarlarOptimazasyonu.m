dtedfile = "n39_w106_3arc_v2.dt1";
attribution = "SRTM 3 arc-second resolution. Data available from the U.S. Geological Survey.";
addCustomTerrain("southboulder",dtedfile, ...
"Attribution",attribution)
viewer = siteviewer("Terrain","southboulder");
names = "Radar site" + (1:5);
rdrlats = [39.6055 39.6481 39.7015 39.7469 39.8856];
rdrlons = [-105.1602 -105.1378 -105.1772 -105.2000 -105.2181];

% Create transmitter sites associated with radars
rdrtxs = txsite("Name",names, ...
    "AntennaHeight",10, ...
    "Latitude",rdrlats, ...
    "Longitude",rdrlons);

% Create receiver sites associated with radars
rdrrxs = rxsite("Name",names, ...
    "AntennaHeight",10, ...
    "Latitude",rdrlats, ...
    "Longitude",rdrlons);

% Show just the radar transmitter sites
show(rdrtxs);
pd = 0.9;         % Probability of detection
pfa = 1e-6;       % Probability of false alarm
maxrange = 35000; % Maximum unambiguous range (m)
rangeres = 5;     % Required range resolution (m)
tgtrcs = .1;      % Required target radar cross section (m^2)
numpulses = 10;
snrthreshold = albersheim(pd, pfa, numpulses); % Unit: dB
disp(snrthreshold);
fc = 10e9;    % Transmitter frequency: 10 GHz
antgain = 38; % Antenna gain: 38 dB
c = physconst('LightSpeed');
lambda = c/fc;
pulsebw = c/(2*rangeres);
pulsewidth = 1/pulsebw;
Ptx = radareqpow(lambda,maxrange,snrthreshold,pulsewidth,...
    'RCS',tgtrcs,'Gain',antgain);
disp(Ptx)
% Define region of interest
latlims = [39.5 40];
lonlims = [-105.6 -105.1];

% Define grid of target locations in region of interest
tgtlatv = linspace(latlims(1),latlims(2),50);
tgtlonv = linspace(lonlims(1),lonlims(2),50);
[tgtlons,tgtlats] = meshgrid(tgtlonv,tgtlatv);
tgtlons = tgtlons(:);
tgtlats = tgtlats(:);
% Create temporary array of sites corresponding to target locations and query terrain
Z = elevation(txsite("Latitude",tgtlats,"Longitude",tgtlons));
[Zmin, Zmax] = bounds(Z);
Zmean = mean(Z);

disp("Ground elevation (meters): Min    Max    Mean" + newline + ...
    "                           " + round(Zmin) + "   " + round(Zmax) + "   " + round(Zmean))
tgtalt = 500;
viewer.Name = "Radar Coverage Region of Interest";
regionData = propagationData(tgtlats,tgtlons,'Area',ones(size(tgtlats)));
contour(regionData,'ShowLegend',false,'Colors','green','Levels',0)
% Create a terrain propagation model, using TIREM or Longley-Rice
tiremloc = tiremSetup;
if ~isempty(tiremloc)
    pm = propagationModel('tirem');
else
    pm = propagationModel('longley-rice');
end

% Compute additional path loss due to terrain and return distances between radars and targets 
[L, ds] = helperPathlossOverTerrain(pm, rdrtxs, rdrrxs, tgtlats, tgtlons, tgtalt);
% Compute SNR for all radars and targets
numtgts = numel(tgtlats);
numrdrs = numel(rdrtxs);
rawsnr = zeros(numtgts,numrdrs);
for tgtind = 1:numtgts
    for rdrind = 1:numrdrs
        rawsnr(tgtind,rdrind) = radareqsnr(lambda,ds(tgtind,rdrind),Ptx,pulsewidth, ...
            'Gain',antgain,'RCS',tgtrcs,'Loss',L(tgtind,rdrind));
    end
end
bestsitenums = helperOptimizeRadarSites(rawsnr, snrthreshold);
snr = max(rawsnr(:,bestsitenums),[],2);
% Show selected radar sites using red markers
viewer.Name = "Radar Coverage";
clearMap(viewer)
show(rdrtxs(bestsitenums))

% Plot radar coverage
rdrData = propagationData(tgtlats,tgtlons,"SNR",snr);
legendTitle = "SNR" + newline + "(dB)";
contour(rdrData, ...
   "Levels",snrthreshold, ...
   "Colors","green", ...
   "LegendTitle",legendTitle)
% Calculate SNR thresholds corresponding to different number of pulses
numpulses = 1:10;
snrthresholds = zeros(1,numel(numpulses));
for k = 1:numel(numpulses)
    snrthresholds(k) = albersheim(pd, pfa, numpulses(k));
end

% Plot SNR thresholds vs number of pulses to integrate
plot(numpulses,snrthresholds,'-*')
title("SNR at Radar Receiver Required for Detection")
xlabel("Number of pulses to integrate")
ylabel("SNR (dB)")
grid on;
% Show best sites
viewer.Name = "Radar Coverage for Multiple SNR Thresholds";
show(rdrtxs(bestsitenums))

colors = jet(4);
colors(4,:) = [0 1 0];
contour(rdrData, ...
   "Levels",snrthresholds([1 2 5 10]), ...
   "Colors",colors, ...
   "LegendTitle",legendTitle)
% Target altitude above ground (m)
tgtalt = 250;
[L, ds] = helperPathlossOverTerrain(pm, rdrtxs, rdrrxs, tgtlats, tgtlons, tgtalt);

% Compute SNR for all radars and targets
numrdrs = numel(rdrtxs);
rawsnr = zeros(numtgts,numrdrs);
for tgtind = 1:numtgts
    for rdrind = 1:numrdrs
        rawsnr(tgtind,rdrind) = radareqsnr(lambda,ds(tgtind,rdrind),Ptx,pulsewidth, ...
            'Gain',antgain,'RCS',tgtrcs,'Loss',L(tgtind,rdrind));
    end
end

% Select best combination of 3 radar sites
bestsitenums = helperOptimizeRadarSites(rawsnr, snrthreshold);
snr = max(rawsnr(:,bestsitenums),[],2);

% Show best sites
viewer.Name = "Radar Coverage";
clearMap(viewer);
show(rdrtxs(bestsitenums))

% Plot radar coverage
rdrData = propagationData(tgtlats,tgtlons,"SNR",snr);
contour(rdrData, ...
   "Levels",snrthreshold, ...
   "Colors","green", ...
   "LegendTitle",legendTitle)
% Show best sites
viewer.Name = "Radar Coverage for Multiple SNR Thresholds";
show(rdrtxs(bestsitenums))

contour(rdrData, ...
   "Levels",snrthresholds([1 2 5 10]), ...
   "Colors",colors, ...
   "LegendTitle",legendTitle)
close(viewer)
removeCustomTerrain("southboulder")