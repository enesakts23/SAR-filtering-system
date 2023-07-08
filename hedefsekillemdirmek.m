% Save current random generator state
s = rng;
% Set random seed for predictable results
rng(2020);
% Create scenario
scene = trackingScenario('IsEarthCentered',true,'UpdateRate',1);
load('flightwaypoints.mat')
flightRoute = geoTrajectory(lla,timeofarrival);
airplane = platform(scene,'Trajectory',flightRoute);
onboardINS = insSensor('PositionAccuracy',100,'VelocityAccuracy',10,...
    "RandomStream","mt19937ar with seed");
airplane.PoseEstimator = onboardINS;
load('737rcs.mat');
airplane.Signatures{1} = boeing737rcs;
% Model an ARSR-4 radar
updaterate = 1/12;
fov = [360;30.001];
elAccuracy = atan2d(0.9,463); % 900m accuracy @ max range
elBiasFraction = 0.1;

arsr4 = monostaticRadarSensor(1,'UpdateRate',updaterate,...
    'FieldOfView',fov,...,
    'HasElevation',true,...
    'ScanMode','Mechanical',...
    'MechanicalScanLimits',[0 360; -30 0],...
    'HasINS',true,...
    'HasRangeRate',true,...
    'HasFalseAlarms',false,...
    'ReferenceRange',463000,...
    'ReferenceRCS',0,...
    'AzimuthResolution',1.4,...
    'AzimuthBiasFraction',0.176/1.4,...
    'ElevationResolution',elAccuracy/elBiasFraction,...
    'ElevationBiasFraction',elBiasFraction,...
    'RangeResolution', 323,...
    'RangeBiasFraction',116/323,... Accuracy / Resolution
    'RangeRateResolution',100,...
    'DetectionCoordinates','Scenario');

% Add ARSR-4 radars at each ARTCC site
radarsitesLLA = [41.4228  -88.0583  0;...
    40.6989  -89.8258  0;...
    39.2219  -95.2461  0];

for i=1:3
    radar = clone(arsr4);
    radar.SensorIndex = i;
    platform(scene,'Position',radarsitesLLA(i,:),...
        'Signatures',rcsSignature('Pattern',-50),'Sensors',{radar});
end
tracker = trackerGNN('FilterInitializationFcn',@initfilter,...
    'ConfirmationThreshold',[3 5],...
    'DeletionThreshold',[5 5],...
    'AssignmentThreshold',[1000 Inf]);
gl = helperGlobeViewer;
setCamera(gl,[28.9176  -95.3388  5.8e5],[0 -30 10]);

% Show radar sites
plotPlatform(gl,scene.Platforms(2:end),'d');
% Show radar coverage
covcon = coverageConfig(scene);
plotCoverage(gl,covcon);
% Show flight route
plotTrajectory(gl,airplane);
% Take a snapshot
snap(gl);
useADSB = false;
snapTimes = [300 1600];

% Declare loop variables
detBuffer = {};
tLLA = [];
tHeading = [];
tSpeed = [];
tTime = [];
images = {};

% Set updata rates in seconds
trackupdatetime = 12;
adsbupdatetime  = 1;
arsrupdatetime  = 12;

while advance(scene)
    time = scene.SimulationTime;
    
    % Update position of airplane on the globe
    plotTarget(gl,'airplane',airplane,'^','full');
    
    % Generate radar detections at the defined rate
    if mod(time,arsrupdatetime) == 0 
        % Generate synthetic radar detections
        dets = detect(scene);
        dets = removeBelowGround(dets);
        detBuffer = [detBuffer; dets]; %#ok<AGROW>
    end
    
    % Generate ADS-B detections at the defined rate
    if useADSB && mod(time,adsbupdatetime) == 0
        adsb = adsbDetect(time,airplane);
        detBuffer = [detBuffer; adsb]; %#ok<AGROW>
    end
    
    % Fuse detections in tracker and update tracks on the globe
    if mod(time,trackupdatetime) == 0 && time > 0
        % Update detections on the globe
        plotDetection(gl,detBuffer);
        % Tracker needs detections for first call i.e.
        cond = ~isempty(detBuffer) || ~isLocked(tracker);
        if cond
            tracks = tracker(detBuffer,time);
            if ~isempty(tracks)
                % Record the estimated airplane data
                [tLLA, tSpeed, tHeading, tTime] = ...
                    helperNavigationData(tracks,tLLA, tSpeed, tHeading, tTime);
            end
            plotTrack(gl,tracks);
            detBuffer = {};
        end
    end
    
    % Move camera and take snapshots
    images = moveCamera(gl,time,snapTimes,images);
end
for i=1:numel(images)
    figure
    imshow(images{i});
end
useADSB = true;
snapTimes = [200 1244];
[trackLLA2, trackSpeed2, trackHeading2, trackTime2, images] = ...
    simulateScene(gl,scene,tracker,useADSB,snapTimes);

% Reset random seed
rng(s);
for i=1:numel(images)
    figure
    imshow(images{i});
end
% Query truth at the recorded timestamps for both runs
[trueLLA, ~, trueVel] = lookupPose(flightRoute,tTime);
[trueLLAadsb, ~, trueVeladsb] = lookupPose(flightRoute,trackTime2);

figure
tiledlayout(3,1);
nexttile
hold on
plot(tTime/60,trueLLA(:,3),'LineWidth',2,'LineStyle','--','DisplayName','Truth');
plot(tTime/60,tLLA(:,3),'DisplayName','Surveillance radar only');
plot(trackTime2/60,trackLLA2(:,3),'DisplayName','Surveillance radar + ADS-B');
legend('Location','northeastoutside')
xlabel('Time (minute)');
ylabel('Altitude (meter)')
title('Altitude')

nexttile
hold on
plot(tTime/60,vecnorm(trueVel(:,(1:2)),2,2),'LineWidth',2,'LineStyle','--');
plot(tTime/60,tSpeed);
plot(trackTime2/60,trackSpeed2);
xlabel('Time (minute)');
ylabel('Speed (m/s)')
title('Groundspeed')

nexttile
hold on
plot(tTime/60,atan2d(trueVel(:,2),trueVel(:,1)),'LineWidth',2,'LineStyle','--');
plot(tTime/60,tHeading);
plot(trackTime2/60,trackHeading2);
xlabel('Time (minute)');
ylabel('Heading (degree)')
title('Heading')
function [tLLA,tSpeed,tHeading,tTime,images] = simulateScene(gl,scene,tracker,useADSB,snapTimes)
% Reset the scenario, globe, and seed
rng(2020);
restart(scene);
airplane = scene.Platforms{1};
release(tracker);
clear(gl);
setCamera(gl,[39.4563 -90.1187 2.356e6]);

% Display initial state of the scene
covcon = coverageConfig(scene);
plotPlatform(gl,scene.Platforms(2:end),'d');
plotCoverage(gl,covcon);

% Declare loop variables
detBuffer = {};
tLLA = [];
tHeading = [];
tSpeed = [];
tTime = [];
images = {};

% Set update rates in seconds
if useADSB
    trackupdatetime = 1;
else
    trackupdatetime = 12;
end
adsbupdatetime  = 1;
arsrupdatetime  = 12;

while advance(scene)
    time = scene.SimulationTime;
    
    % Update position of airplane on the globe
    plotTarget(gl,'airplane',airplane,'^','full');
    
    % Generate radar detections at the defined rate
    if mod(time,arsrupdatetime) == 0 && time > 0
        % Generate synthetic radar detections
        dets = detect(scene);
        dets = removeBelowGround(dets);
        detBuffer = [detBuffer; dets]; %#ok<AGROW>
    end
    
    % Generate ADS-B detections at the defined rate
    if useADSB && mod(time,adsbupdatetime) == 0
        adsb = adsbDetect(time,airplane);
        detBuffer = [detBuffer; adsb]; %#ok<AGROW>
    end
    
    % Update detections on the globe
    plotDetection(gl,detBuffer);
    
    % Fuse detections in tracker and update tracks on the globe
    if mod(time,trackupdatetime) == 0 && time > 0
        % Tracker needs detections for first call
        cond = ~(isempty(detBuffer) && ~isLocked(tracker));
        if cond
            tracks = tracker(detBuffer,time);
            if ~isempty(tracks)
                % Record the estimated airplane data
                [tLLA, tSpeed, tHeading, tTime] = ...
                    helperNavigationData(tracks,tLLA, tSpeed, tHeading, tTime);
            end
            plotTrack(gl,tracks);
            detBuffer = {};
        end
    end
    
    % Move camera and take snapshots
    images = moveCamera(gl,time,snapTimes,images);
    
end
end
function detection = adsbDetect(time,airplane,~)
%adsbDetect generate ADS-B detections

estimpose = pose(airplane,'CoordinateSystem','Cartesian');
meas = [estimpose.Position(:) ; estimpose.Velocity(:)];
measnoise = diag([100 100 100 10 10 10].^2);

mparams = struct('Frame','Rectangular',...
    'OriginPositin',zeros(3,1),...
    'OriginVelocity',zeros(3,1),...
    'Orientation',eye(3),... ADS-B does not report orientation
    'IsParentToChild', 1,...
    'HasAzimuth', 1,...
    'HasElevation', 1,...
    'HasRange', 1, ...
    'HasVelocity', 1);

attr ={ struct('TargetIndex',airplane.PlatformID,'SNR',10)};

detection = {objectDetection(time, meas, 'SensorIndex',20,...
    'MeasurementNoise',measnoise,...
    'MeasurementParameters',mparams,...
    'ObjectAttributes',attr)};
end
function filter = initfilter(detection)
filter = initcvekf(detection);
filter.StateCovariance = 4*filter.StateCovariance; % initcvekf uses measurement noise as the default
filter.ProcessNoise = 0.02*eye(3);
end
function detsout = removeBelowGround(detsin)
n = numel(detsin);
keep = zeros(1,n,'logical');
for i=1:n
    meas = detsin{i}.Measurement(1:3)';
    lla = fusion.internal.frames.ecef2lla(meas);
    if lla(3)>0
        keep(i) = true;
    else
        keep(i) = false;
    end
end
detsout = detsin(keep);
end
function images = moveCamera(gl, time, snapTimes,images)

if time == 120
    setCamera(gl,[37, -97.935, 4.328e4],[0 -23 30]);
end

if time == 1244
    setCamera(gl,[38.8693 -95.6109 1.214e4],[0 -26.8 38.7]);
end

if time == 1445
    setCamera(gl,[37.995, -94.60 6.87e4],[0 -15 2]);
end

if time == 2100
    setCamera(gl, [38.9747 -94.6385 1.3e5], [0 -20 55]);
end

if time == 3000
    setCamera(gl,[41.636 -87.011 6.33e4],[0 -25 270]);
end

% Snaps
if any(time == snapTimes)
    img = snap(gl);
    images = [ images, {img}];
end
end


