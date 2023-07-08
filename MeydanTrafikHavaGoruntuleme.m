% Create tracking scenario
scenario = trackingScenario;

% Add a stationary platform to model the ATC tower
tower = platform(scenario);
rpm = 12.5;
fov = [1.4;10];
scanrate = rpm*360/60;  % deg/s
updaterate = scanrate/fov(1); % Hz

radar = monostaticRadarSensor(1, 'Rotator', ...
    'UpdateRate', updaterate, ...           % Hz
    'FieldOfView', fov, ...                 % [az;el] deg
    'MaxMechanicalScanRate', scanrate, ...  % deg/sec
    'AzimuthResolution', fov(1), ...        % deg
    'ReferenceRange', 111e3, ...            % m
    'ReferenceRCS', 0, ...                  % dBsm
    'RangeResolution', 135, ...             % m
    'HasINS', true, ...
    'DetectionCoordinates', 'Scenario');

% Mount radar at the top of the tower
radar.MountingLocation = [0 0 -15];
tower.Sensors = radar;
% Enable elevation scanning
radar.HasElevation = true;

% Set mechanical elevation scan to begin at 2 degrees above the horizon
elFov = fov(2);
tilt = 2; % deg
radar.MechanicalScanLimits(2,:) = [-fov(2) 0]-tilt; % deg
radar.FieldOfView(2) = elFov+1e-3;
% Duration of scenario
sceneDuration = 60; % s

% Inbound airliner
ht = 3e3;
spd = 900*1e3/3600; % m/s
wp = [-5e3 -40e3 -ht;-5e3 -40e3+spd*sceneDuration -ht];
traj = waypointTrajectory('Waypoints',wp,'TimeOfArrival',[0 sceneDuration]);
platform(scenario,'Trajectory', traj);

% Outbound airliner
ht = 4e3;
spd = 700*1e3/3600; % m/s
wp = [20e3 10e3 -ht;20e3+spd*sceneDuration 10e3 -ht];
traj = waypointTrajectory('Waypoints',wp,'TimeOfArrival',[0 sceneDuration]);
platform(scenario,'Trajectory', traj);

% Tangential airliner
ht = 4e3;
spd = 300*1e3/3600; % m/s
wp = [-20e3 -spd*sceneDuration/2 -ht;-20e3 spd*sceneDuration/2 -ht];
traj = waypointTrajectory('Waypoints',wp,'TimeOfArrival',[0 sceneDuration]);
platform(scenario,'Trajectory', traj);
tracker = trackerGNN( ...
    'Assignment', 'Auction', ...
    'AssignmentThreshold',50, ...
    'FilterInitializationFcn',@initFilter);
origin = [42.366978, -71.022362, 50];
mapViewer = helperATCMap('ReferenceLocation',origin);
setCamera(mapViewer, origin + [0 0 1e5]);
showScenario(mapViewer,scenario);
snap(mapViewer);
% Set simulation to advance at the update rate of the radar
scenario.UpdateRate = radar.UpdateRate;

% Create a buffer to collect the detections from a full scan of the radar
scanBuffer = {};

% Initialize the track array
tracks = [];

% Save visualization snapshots for each scan
allsnaps = {};
scanCount = 0;

% Set random seed for repeatable results
s = rng;
rng(2020)

while advance(scenario)
    
    % Update airliner positions
    plotTarget(mapViewer, scenario.Platforms([2 3 4]));
    
    % Generate detections on targets in the radar's current field of view
    [dets,config] = detect(scenario);
    scanBuffer = [scanBuffer;dets]; %#ok<AGROW>
    % Plot beam and detections
    plotCoverage(mapViewer,coverageConfig(scenario))
    plotDetection(mapViewer,scanBuffer);
    
    
    % Update tracks when a 360 degree scan is complete
    simTime = scenario.SimulationTime;
    isScanDone = config.IsScanDone;
    if isScanDone
        scanCount = scanCount+1;
        % Update tracker
        [tracks,~,~,info] = tracker(scanBuffer,simTime);
        % Clear scan buffer for next scan
        scanBuffer = {};
    elseif isLocked(tracker)
        % Predict tracks to the current simulation time
        tracks = predictTracksToTime(tracker,'confirmed',simTime);
    end
    
    % Update map and take snapshots
    allsnaps = snapPlotTrack(mapViewer,tracks,isScanDone, scanCount, allsnaps);

end
allsnaps = [allsnaps, {snap(mapViewer)}];
figure
imshow(allsnaps{1});
figure
imshow(allsnaps{2});
figure
imshow(allsnaps{3});
figure
imshow(allsnaps{4});
figure
imshow(allsnaps{5});
figure
imshow(allsnaps{6});
truthTrackTable = tabulateData(scenario, tracks) %#ok<NOPTS>
% Reposition and orient the camera to show the 3-D nature of the map
camPosition = origin + [0.367, 0.495, 1.5e4];
camOrientation = [0, -17, 235]; %Looking south west, 17 degrees below the horizon
setCamera(mapViewer, camPosition, camOrientation);
snap(mapViewer);
