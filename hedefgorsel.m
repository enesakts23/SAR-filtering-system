v = 250;
deltaPhi = 360/60;
sensormotion = phased.Platform(...
    'InitialPosition',[0;-3000;500],...
    'VelocitySource','Input port',...
    'InitialVelocity',[0;v;0]);
tgtmotion = phased.Platform('InitialPosition',[0;0;0],...
    'Velocity',[30;0;0]);
tau = 0.5;
sceneview = phased.ScenarioViewer('ShowBeam','None');
for m = 1:tau:60
    [sensorpos,sensorvel] = sensormotion(tau,...
        v*[cosd(m*deltaPhi);sind(m*deltaPhi);0]);
    [tgtpos,tgtvel] = tgtmotion(tau);
    sceneview(sensorpos,sensorvel,tgtpos,tgtvel);
    drawnow;
end
sensormotion = phased.Platform(...
    'InitialPosition',[0 0;-3000 500;500 1], ...
    'VelocitySource','Input port', ...
    'InitialVelocity',[0 100;v 0;0 0], ...
    'OrientationAxesOutputPort', true);
tgtmotion = phased.Platform(...
    'InitialPosition',[0 2000.66 3532.63;0 0 500;0 500 500],...
    'Velocity',[30 120 -120; 0 0 -20; 0 0 60],...
    'OrientationAxesOutputPort', true);
sceneview = phased.ScenarioViewer('BeamRange',[3e3 3e3],...
    'BeamWidth',[5 5], ...
    'ShowBeam', 'All', ...
    'CameraPerspective', 'Custom', ...
    'CameraPosition', [-15453.85 -19716.96 13539], ...
    'CameraOrientation', [-47 -27 0]', ...
    'CameraViewAngle', 11.28, ...
    'OrientationInputPort', true, ...
    'UpdateRate',1/tau);
for m = 1:60
    [sensorpos,sensorvel,sensoraxis] = sensormotion(tau,...
        [v*[cosd(m*deltaPhi);sind(m*deltaPhi);0] [100; 0; 0]]);
    [tgtpos,tgtvel,tgtaxis] = tgtmotion(tau);
    % Radar 1 tracks Target 1
    [lclrng, lclang] = rangeangle(tgtpos(:,1),sensorpos(:,1),...
        sensoraxis(:,:,1));
    % Update beam direction
    sceneview.BeamSteering = [lclang [0;0]];
    sceneview(sensorpos,sensorvel,sensoraxis,tgtpos,tgtvel,tgtaxis);
    drawnow;
end
load BasicMonostaticRadarExampleData.mat
fc = radiator.OperatingFrequency;
fs = waveform.SampleRate;
c = radiator.PropagationSpeed;

sensormotion = phased.Platform(...
    'InitialPosition',[0; 0; 10],...
    'Velocity',[0; 0; 0]);

target = phased.RadarTarget(...
    'MeanRCS',[1.6 2.2 1.05],...
    'OperatingFrequency',fc);
tgtmotion = phased.Platform(...
    'InitialPosition',[2000.66 3532.63 3845.04; 0 0 0;10 10 10], ...
    'Velocity',[120 -120 0; 0 0 0; 0 0 0]);

channel = phased.FreeSpace(...
    'SampleRate',fs,...
    'TwoWayPropagation',true,...
    'OperatingFrequency',fc);
matchingcoeff = getMatchedFilter(waveform);
matchingdelay = size(matchingcoeff,1)-1;
matchedfilter = phased.MatchedFilter(...
    'Coefficients',matchingcoeff,...
    'GainOutputPort',true);

prf = waveform.PRF;
fast_time_grid = unigrid(0,1/fs,1/prf,'[)');
rangeGates = c*fast_time_grid/2;

lambda = c/fc;
max_range = 5000;
tvg = phased.TimeVaryingGain(...
    'RangeLoss',2*fspl(rangeGates,lambda),...
    'ReferenceLoss',2*fspl(max_range,lambda));

num_pulse_int = 10;
r_update = 20;

sceneview = phased.ScenarioViewer('UpdateRate',r_update,...
    'Title','Monostatic Radar');

rtiscope = phased.IntensityScope('Name','Menzil-Zaman Yoğunluk Kapsamı',...
    'XLabel','Range (m)', ...
    'XResolution',c/(2*fs), ...
    'XOffset',-(matchingdelay-1)*c/(2*fs), ...
    'TimeResolution',1/r_update,'TimeSpan',5,'IntensityUnits','dB');

nfft = 128;
df = prf/nfft;
dtiscope = phased.IntensityScope(...
    'Name','Doppler-Zaman Yoğunluğu Kapsamı',...
    'XLabel','Velocity (m/sec)', ...
    'XResolution',dop2speed(df,lambda)/2, ...
    'XOffset', dop2speed(-prf/2,lambda)/2, ...
    'TimeResolution',1/r_update,'TimeSpan',5,'IntensityUnits','dB');
% Pre-allocate array for improved processing speed
rxpulses = zeros(numel(rangeGates),num_pulse_int);

for k = 1:100
    for m = 1:num_pulse_int
        % Update sensor and target positions
        [sensorpos,sensorvel] = sensormotion(1/prf);
        [tgtpos,tgtvel] = tgtmotion(1/prf);

        % Calculate the target angles as seen by the sensor
        [~,tgtang] = rangeangle(tgtpos,sensorpos);

        % Simulate propagation of pulse in direction of targets
        pulse = waveform();
        [txsig,txstatus] = transmitter(pulse);
        txsig = radiator(txsig,tgtang);
        txsig = channel(txsig,sensorpos,tgtpos,sensorvel,tgtvel);

        % Reflect pulse off of targets
        tgtsig = target(txsig);

        % Receive target returns at sensor
        rxsig = collector(tgtsig,tgtang);
        rxpulses(:,m) = receiver(rxsig,~(txstatus>0));
    end

    rxpulses = matchedfilter(rxpulses);

    % Correct for matched filter delay
    rxpulses = buffer(...
        rxpulses(matchingdelay+1:end),...
        size(rxpulses,1));

    rxpulses = tvg(rxpulses);

    rx_int = pulsint(rxpulses,'noncoherent');

    % display RTI
    rtiscope(rx_int);

    % display DTI
    rx_dop = mean(fftshift(...
        abs(fft(rxpulses,nfft,2)),2));
    dtiscope(rx_dop.');

    % display scene
    sceneview(sensorpos,sensorvel,tgtpos,tgtvel);

    % perform next detection when next update is needed
    sensormotion(1/r_update);
    tgtmotion(1/r_update);
end

hide(dtiscope);
hide(rtiscope);
hide(rtiscope);
show(dtiscope);

