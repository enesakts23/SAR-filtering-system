tasiyicifrekans = 2e9;                 % Radar carrier frequency (Hz)
c = 3e8;                 % Propagation speed (m/s)
lambda = c/tasiyicifrekans;                % Radar wavelength (m)

maxmenzil = 100e3;               % Maximum range (m)
minmenzil = 2e3;                 % Minimum range (m)
bw     = 1e6;
fs     = 1.5*bw;
prf    = 1/range2time(maxmenzil,c);
dcycle = 0.1;

wav = phased.LinearFMWaveform('SampleRate', fs, ...
    'DurationSpecification', 'Duty cycle', 'DutyCycle', dcycle, ...
    'PRF', prf, 'SweepBandwidth', bw);
rngres = bw2range(bw,c);
arraysz   = 50;
ant       = phased.URA('Size',arraysz,'ElementSpacing',lambda/2);
ant.Element.BackBaffled = true;

arraystv  = phased.SteeringVector('SensorArray',ant,'PropagationSpeed',c);
radiator  = phased.Radiator('OperatingFrequency',tasiyicifrekans, ...
    'PropagationSpeed', c, 'Sensor',ant, 'WeightsInputPort', true);
collector = phased.Collector('OperatingFrequency',tasiyicifrekans, ...
    'PropagationSpeed', c, 'Sensor',ant);

beamw = rad2deg(lambda/(arraysz*lambda/2));
tespitolasiligi      = 0.9;                     
yanlisalarmolasiligi     = 1e-6;                    
snr_min = albersheim(tespitolasiligi, yanlisalarmolasiligi, 1);
ampgain = 20;
tgtrcs  = 1;
ant_snrgain = pow2db(arraysz^2);

ppower  = radareqpow(lambda,maxmenzil,snr_min,wav.PulseWidth,...
    'RCS',tgtrcs,'Gain',ampgain+ant_snrgain);

tx = phased.Transmitter('PeakPower',ppower,'Gain',ampgain,'InUseOutputPort',true);
rx = phased.ReceiverPreamp('Gain',ampgain,'NoiseFigure',7,'EnableInputPort',true);
% Eşleşen Filtreler
mfcoeff = getMatchedFilter(wav);
mf      = phased.MatchedFilter('Coefficients',mfcoeff,'GainOutputPort', true);

tgrid   = unigrid(0,1/fs,1/prf,'[)');
rgates  = c*tgrid/2;
rngloss = 2*fspl(rgates,lambda);
refloss = 2*fspl(maxmenzil,lambda);
tvg     = phased.TimeVaryingGain('RangeLoss',rngloss,'ReferenceLoss',refloss);

% teklidarbe
monfeed = phased.MonopulseFeed('SensorArray',ant,'PropagationSpeed',c,...
    'OperatingFrequency',tasiyicifrekans,'SquintAngle',1);
monest  = getMonopulseEstimator(monfeed);
tracker = trackerGNN('FilterInitializationFcn',@initMPARGNN,...
    'ConfirmationThreshold',[2 3], 'DeletionThreshold',5,...
    'HasDetectableTrackIDsInput',true,'AssignmentThreshold',100,...
    'MaxNumTracks',2,'MaxNumSensors',1);
mfradar.Tx      = tx;
mfradar.Rx      = rx;
mfradar.TxAnt   = radiator;
mfradar.RxAnt   = collector;
mfradar.Wav     = wav;
mfradar.RxFeed  = monfeed;
mfradar.MF      = mf;
mfradar.TVG     = tvg;
mfradar.DOA     = monest;
mfradar.STV     = arraystv;
mfradar.Tracker = tracker;
mfradar.IsTrackerInitialized = false;
% hedef tanımla burada örnek olacak lokasyonlar verilmiştie 2 tane hedef
% yerleştirildi rastgele bir şekilde
tgtpos = [29875 49637; 0 4225; 0 0];
tgtvel = [-100 120; 0 100; 0 0];

ntgt = size(tgtpos,2);
tgtmotion = phased.Platform('InitialPosition',tgtpos,'Velocity',tgtvel);
target = phased.RadarTarget('MeanRCS',tgtrcs*ones(1,ntgt),'OperatingFrequency',tasiyicifrekans);
channel = phased.FreeSpace('SampleRate',fs,'TwoWayPropagation',true,'OperatingFrequency',tasiyicifrekans);
env.Target       = target;
env.TargetMotion = tgtmotion;
env.Channel      = channel;
scanregion   = [-30, 30, 0, 20];
azscanspan   = diff(scanregion(1:2));
numazscan    = ceil(azscanspan/beamw);
azscanangles = linspace(scanregion(1),scanregion(2),numazscan);
elscanspan   = diff(scanregion(3:4));
numelscan    = ceil(elscanspan/beamw);
elscanangles = linspace(scanregion(3),scanregion(4),numelscan);
[elscangrid,azscangrid] = meshgrid(elscanangles,azscanangles);
scanangles   = [azscangrid(:) elscangrid(:)].';
sceneplot = helperMPARTaskPlot('initialize',scanangles,azscanangles,maxmenzil,beamw,tgtpos);
searchq = struct('JobType','Search','BeamDirection',num2cell(scanangles,1),...
    'Priority',1000,'WaveformIndex',1);
current_search_idx = 1;
disp(searchq(current_search_idx))
trackq(10) = struct('JobType',[],'BeamDirection',[],'Priority',3000,'WaveformIndex',[],...
    'Time',[],'Range',[],'TrackID',[]);
num_trackq_items = 0;
disp(trackq(1))
jobq.SearchQueue  = searchq;
jobq.SearchIndex  = current_search_idx;
jobq.TrackQueue   = trackq;
jobq.NumTrackJobs = num_trackq_items;
rng(2018);
current_time = 0;
Npulses      = 10;
numdwells    = 200;
dwelltime    = 0.01;

jobload.num_search_job = zeros(1,numdwells);
jobload.num_track_job  = zeros(1,numdwells);
for dwell_idx = 1:14
    
    [current_job,jobq]       = getCurrentJob(jobq,current_time);

    [xsum,xdaz,xdel,mfradar] = generateEcho(mfradar,env,current_job);

    [detection,mfradar]      = generateDetection(xsum,xdaz,xdel,mfradar,current_job,current_time);

    % iz kuyruğu güncelleme ve göstermek için gerekli kodlar
    [jobq,allTracks,mfradar] = updateTrackAndJob(detection,jobq,mfradar,current_job,current_time,dwelltime);

    % burada gerekli radar görünümünü güncelleştirme için matlab mathworks
    % üzeinde hazır bulunan kaynak kodları kullanıldı
    helperMPARTaskPlot('update',sceneplot,current_job,maxmenzil,beamw,tgtpos,allTracks,detection.Measurement);

    tgtpos = env.TargetMotion(dwelltime-Npulses/mfradar.Wav.PRF);
    current_time = current_time+dwelltime;

    if strcmp(current_job.JobType,'Search')
        jobload.num_search_job(dwell_idx) = 1;
    else
        jobload.num_track_job(dwell_idx)  = 1;
    end

end
[mfradar,env,jobq,jobload,current_time,tgtpos] = MPARSimRun(...
    mfradar,env,jobq,jobload,current_time,dwelltime,sceneplot,maxmenzil,beamw,tgtpos,15,15);
[mfradar,env,jobq,jobload,current_time,tgtpos] = MPARSimRun(...
    mfradar,env,jobq,jobload,current_time,dwelltime,sceneplot,maxmenzil,beamw,tgtpos,16,25);
[mfradar,env,jobq,jobload,current_time,tgtpos] = MPARSimRun(...
    mfradar,env,jobq,jobload,current_time,dwelltime,sceneplot,maxmenzil,beamw,tgtpos,26,numdwells);
L = 10;
searchpercent = sum(buffer(jobload.num_search_job,L,L-1,'nodelay'))/L;
trackpercent  = sum(buffer(jobload.num_track_job,L,L-1,'nodelay'))/L;
figure;
plot((1:numel(searchpercent))*L*dwelltime,[searchpercent(:) trackpercent(:)]);
xlabel('Time (s)');
ylabel('Job Percentage');
title('Resource Distribution between Search and Track');
legend('Search','Track','Location','best');
grid on;
