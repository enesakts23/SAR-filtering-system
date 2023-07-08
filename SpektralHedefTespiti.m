hypercube = hypercube('paviaU.hdr');
hypercube;
rgbImg = colorize(hypercube,'Method','rgb','ContrastStretching',true);
figure
imshow(rgbImg)
title('RGB Image')
fileroot = matlabshared.supportpkg.getSupportPackageRoot();
addpath(fullfile(fileroot,'toolbox','images','supportpackages','hyperspectral','hyperdata','ECOSTRESSSpectraFiles'));
lib = readEcostressSig("manmade.roofingmaterial.metal.solid.all.0692uuucop.jhu.becknic.spectrum.txt");
lib;
wavelength = lib.Wavelength;
reflectance = lib.Reflectance;
plot(wavelength,reflectance,'LineWidth',2)
axis tight
xlabel('Wavelength (\mum)')
ylabel('Reflectance (%)')
title('Reference Spectra')
scoreMap = spectralMatch(lib,hypercube);
figure('Position',[0 0 500 600])
imagesc(scoreMap)
colormap parula
colorbar
title('Score Map')
figure
imhist(scoreMap);
title('Histogram Plot of Score Map');
xlabel('Score Map Values')
ylabel('Number of occurrences');
maxthreshold = 0.25;
thresholdedImg = scoreMap <= maxthreshold;
overlaidImg = imoverlay(rgbImg,thresholdedImg,'green');
fig = figure('Position',[0 0 900 500]);
axes1 = axes('Parent',fig,'Position',[0.04 0.11 0.4 0.82]);
imagesc(thresholdedImg,'Parent',axes1);
colormap([0 0 0;1 1 1]);
title('Detected Target Region')
axis off
axes2 = axes('Parent',fig,'Position',[0.47 0.11 0.4 0.82]);
imagesc(overlaidImg,'Parent',axes2)
axis off
title('Overlaid Detection Results')
load('paviauRoofingGT.mat');
err = immse(im2double(paviauRoofingGT), im2double(thresholdedImg));
fprintf('\n The mean squared error is %0.4f\n', err)
fig = figure('Position',[0 0 900 500]);
axes1 = axes('Parent',fig,'Position',[0.04 0.11 0.4 0.82]);
imagesc(thresholdedImg,'Parent',axes1);
colormap([0 0 0;1 1 1]);
title('Result Obtained')
axis off
axes2 = axes('Parent',fig,'Position',[0.47 0.11 0.4 0.82]);
imagesc(paviauRoofingGT,'Parent',axes2)
colormap([0 0 0;1 1 1]);
axis off
title('Ground Truth')
