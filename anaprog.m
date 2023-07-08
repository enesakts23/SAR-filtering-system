stopTime = 10;
v = 60;
scenario = trackingScenario;
scenario.StopTime = stopTime;
scenario.UpdateRate = 0;
p1 = platform(scenario);
p1.Trajectory = waypointTrajectory([30 20 0; 30 .8*v*stopTime 0], [0 stopTime]);
p2 = platform(scenario);
p2.Trajectory = waypointTrajectory([0 0 0; 0 v*stopTime 0], [0 stopTime]);
p3 = platform(scenario);
p3.Trajectory = waypointTrajectory([-30 -20 0; -30 1.2*v*stopTime 0], [0 stopTime]);
pRadar = platform(scenario);
pRadar.Trajectory = kinematicTrajectory('Position', [-v*stopTime 0.5*v*stopTime 0]);
radar = monostaticRadarSensor(1, 'No scanning', 'UpdateRate', 5, ...
    'MountingAngles', [0 0 0], 'AzimuthResolution', 1, ...
    'FieldOfView', [100 1], 'HasINS', true, 'DetectionCoordinates', 'Scenario', ...
    'MaxUnambiguousRange', 1000);
pRadar.Sensors = radar;
fig = figure;
ax = axes(fig);
tp = theaterPlot('Parent', ax, 'XLimits', [-11*v 100], 'YLimits', [-50 15*v], 'ZLimits', [-100 100]);
rp = platformPlotter(tp, 'DisplayName', 'Radar', 'Marker', 'd');
pp = platformPlotter(tp, 'DisplayName', 'Platforms');
dp = detectionPlotter(tp, 'DisplayName', 'Detections');
trp = trackPlotter(tp, 'DisplayName', 'Tracks', 'ConnectHistory', 'on', 'ColorizeHistory', 'on');
covp = coveragePlotter(tp, 'DisplayName', 'Radar Coverage', 'Alpha', [0.1 0]);
tracker = trackerGNN;
tgm = trackGOSPAMetric("Distance","posabserr");
gospa = zeros(1,51); % number of timesteps is 51
i = 0;

% Define the random number generator seed for repeatable results
s = rng(2019); 
while advance(scenario)
    % Get detections
    dets = detect(scenario);
    
    % Update the tracker
    if isLocked(tracker) || ~isempty(dets)
        [tracks, ~, ~, info] = tracker(dets, scenario.SimulationTime);
    end
    
    % Evaluate GOSPA
    i = i + 1;
    truth = platformPoses(scenario);
    gospa(i) = tgm(tracks, truth);
    
    % Update the display
    updateDisplay(rp, pp, dp, trp, covp, scenario, dets, tracks);
end
rng(s)
figure
plot(gospa)
title('Genişletilmiş radar metriği ve zaman adımı')

%Atama Eşiği
release(tracker);
tracker.AssignmentThreshold = 50;
rerunScenario(scenario, tracker, tgm, tp);
release(tracker);
tracker.AssignmentThreshold = [50 2000];
rerunScenario(scenario, tracker, tgm, tp);
v = 200;
p1.Trajectory = waypointTrajectory([30 0 0; 30 0.8*v*stopTime 0], [0 stopTime]);
p2.Trajectory = waypointTrajectory([0 0 0; 0 v*stopTime 0], [0 stopTime]);
p3.Trajectory = waypointTrajectory([-30 0 0; -30 1.2*v*stopTime 0], [0 stopTime]);
pRadar.Trajectory = kinematicTrajectory('Position', [-v*stopTime 0.5*v*stopTime 0]);
tp.XLimits = [-100-v*stopTime 300];
tp.YLimits = [-100 100+v*1.2*stopTime];
release(radar);
radar.MaxUnambiguousRange = 3000;
rerunScenario(scenario, tracker, tgm, tp);
release(tracker)
tracker.FilterInitializationFcn = @initFastCVEKF;
rerunScenario(scenario, tracker, tgm, tp);
release(radar);
radar.FalseAlarmRate = 2.5e-4;
tp.XLimits = [-2100 300];
tp.YLimits = [-100  3100];
tp.ZLimits = [-1000 1000];
rerunScenario(scenario, tracker, tgm, tp);
release(tracker)
tracker.DeletionThreshold = [2 3];
rerunScenario(scenario, tracker, tgm, tp);
disp(tracker.ConfirmationThreshold);
release(tracker)
tracker.ConfirmationThreshold = [3 4];
rerunScenario(scenario, tracker, tgm, tp);
release(tracker)
tracker.MaxNumTracks = 200;
tracker.ConfirmationThreshold = [5 6];

release(radar)
radar.FalseAlarmRate = 1e-3;
rerunScenario(scenario, tracker, tgm, tp);
jpda = helperGNN2JPDA(tracker);
jpda.ClutterDensity = 1e-3;
rerunScenario(scenario, tracker, tgm, tp);
function updateDisplay(rp, pp, dp, trp, covp, scenario, dets, tracks)
    % Plot the platform positions
    poses = platformPoses(scenario);
    pos = reshape([poses(1:3).Position], 3, [])'; % Only the platforms
    plotPlatform(pp, pos);
    radarPos = poses(4).Position; % Only the radar
    plotPlatform(rp, radarPos)
    
    % Plot the detection positions
    if ~isempty(dets)
        ds = [dets{:}];
        dPos = reshape([ds.Measurement], 3, [])';
    else
        dPos = zeros(0,3);
    end
    plotDetection(dp, dPos);
    
    % Plot the tracks
    tPos = getTrackPositions(tracks, [1 0 0 0 0 0; 0 0 1 0 0 0; 0 0 0 0 1 0]);
    tIDs = string([tracks.TrackID]);
    plotTrack(trp, tPos, tIDs);
    
    % Plot the coverage
    covcon = coverageConfig(scenario);
    plotCoverage(covp, covcon);
end
function info = rerunScenario(scenario, tracker, tgm, tp)
    % Reset the objects after the previous run
    restart(scenario);
    reset(tracker);
    reset(tgm);
    rp = findPlotter(tp, 'DisplayName', 'Radar');
    pp = findPlotter(tp, 'DisplayName', 'Platforms');
    trp = findPlotter(tp, 'DisplayName', 'Tracks');
    dp = findPlotter(tp, 'DisplayName', 'Detections');
    covp = findPlotter(tp, 'DisplayName', 'Radar Coverage');
    clearPlotterData(tp);
    
    gospa = zeros(1,51); % number of timesteps is 51
    i = 0;
    s = rng(2019); 
    while advance(scenario)
        % Get detections
        dets = detect(scenario);

        % Update the tracker
        if isLocked(tracker) || ~isempty(dets)
            [tracks, ~, ~, info] = tracker(dets, scenario.SimulationTime); 
        end
        
        % Evaluate GOSPA
        i = i + 1;
        truth = platformPoses(scenario);
        gospa(i) = tgm(tracks, truth);

        % Update the display
        updateDisplay(rp, pp, dp, trp, covp, scenario, dets, tracks);
    end
    rng(s)
    figure
    plot(gospa(gospa>0))
    title('Genişletilmiş radar metriği ve zaman adımı')
end
function ekf = initFastCVEKF(detection)
    ekf = initcvekf(detection);
    initialCovariance = diag(ekf.StateCovariance);
    initialCovariance([2,4,6]) = 300^2; % Increase the speed covariance
    ekf.StateCovariance = diag(initialCovariance);
end
function jpda = helperGNN2JPDA(gnn)
    jpda = trackerJPDA(...
        'MaxNumTracks', gnn.MaxNumTracks, ...
        'AssignmentThreshold', gnn.AssignmentThreshold, ...
        'FilterInitializationFcn', gnn.FilterInitializationFcn, ...
        'MaxNumSensors', gnn.MaxNumSensors, ...
        'ConfirmationThreshold', gnn.ConfirmationThreshold, ...
        'DeletionThreshold', gnn.DeletionThreshold, ...
        'HasCostMatrixInput', gnn.HasCostMatrixInput, ...
        'HasDetectableTrackIDsInput', gnn.HasDetectableTrackIDsInput);

    if strcmpi(gnn.TrackLogic, 'History')
        jpda.TrackLogic = 'History';
    else
        jpda.TrackLogic = 'Integrated';
        jpda.DetectionProbability = gnn.DetectionProbability;
        jpda.ClutterDensity = gnn.FalseAlarmRate / gnn.Volume;
    end
end

