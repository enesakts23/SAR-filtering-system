% Set the random seed for repeatable results.
rng(2050);

% Create a tracking scenario to manage the movement of platforms.
scene = trackingScenario;

% Set the duration of the scenario to 30 seconds and the update rate of the
% scenario to 1 Hz.
scene.StopTime = 30;    % sec
scene.UpdateRate = 1;   % Hz
% Set the minimum number of sensor-emitter pairs required by the spherical
% intersection algorithm for a better localization of the targets in
% 3-dimensional space.
numSensors = 4;
emitterIdx = 1; % Emitter is added as first platform in scene
% Define an emitter.
emitter = radarEmitter(emitterIdx,'No scanning', ...
    'FieldOfView',[360 180], ...    % [az el] deg
    'CenterFrequency', 300e6, ...   % Hz
    'Bandwidth', 30e6, ...          % Hz
    'WaveformType', 1);             % Use 1 for an LFM-like waveform

% Mount emitter on a platform.
thisPlat = platform(scene,'Trajectory',kinematicTrajectory('Position',[0 0 0],...
    'Velocity',[0 0 0]));
thisPlat.Emitters = {emitter};
% Define some random trajectories for the four bistatic radar sensors.
% Circularly distributed with some variance.
r = 2000;                                        % Range, m
theta = linspace(0,pi,numSensors);               % Theta, rad
xSen = r*cos(theta) + 100*randn(1,numSensors);   % m
ySen = r*sin(theta) + 100*randn(1,numSensors);   % m

% To observe the z of the targets, the sensors must have some elevation
% with respect to each other and emitter.
zSen = -1000*rand(1,numSensors);     % m

% Define a bistatic radar sensor.
sensor = radarSensor(1,'No scanning', ...
    'FieldOfView',[360 180], ...    % [az el] deg
    'DetectionMode','Bistatic', ... % Bistatic detection mode
    'CenterFrequency', 300e6, ...   % Hz
    'Bandwidth', 30e6, ...          % Hz
    'WaveformTypes', 1,...          % Use 1 for an LFM-like waveform
    'HasINS',true,...               % Has INS to enable tracking in scenario
    'AzimuthResolution',360,...     % Does not measure azimuth and has a single resolution cell
    'HasElevation',true,...         % Enable elevation to set elevation resolution
    'ElevationResolution',180);     % Single elevation resolution cell

% Mount bistatic radar sensors on platforms.
for iD = 1:numSensors
    % Create a platform with the trajectory. The sensing platforms are
    % considered stationary, but can be provided with a velocity.
    thisPlat = platform(scene,...
        'Trajectory',kinematicTrajectory('Position',[xSen(iD) ySen(iD) zSen(iD)],...
        'Velocity',[0 0 0]));
    % Clone the bistatic radar sensor and mount to platforms.
    thisPlat.Sensors = {clone(sensor)};
    % Provide the correct sensor index.
    thisPlat.Sensors{1}.SensorIndex = iD;
end
% Add one target here using the platform method of scenario. Specify the
% trajectory using a kinematicTrajectory with random position and constant
% velocity.
platform(scene,'Trajectory',kinematicTrajectory(...
                            'Position', [2000*randn 100*randn -1000],...
                            'Velocity',[10*randn 10*randn 5*randn]));

% Initialize the display for visualization.
theaterDisplay = helperBistaticRangeFusionDisplay(scene,...
    'XLim',[-3 3],'YLim', [-3 3],'ZLim',[-2.5 0],...
    'GIF',''); % records new GIF if name specified
view(3);
% Create a fused detection to represent the triangulated position and
% visualize the position as a 3-D fused position detection.
measParam = struct('Frame','Rectangular',...
    'HasAzimuth',true,'HasElevation',true,...
    'HasRange',true,'HasVelocity',false,...
    'OriginPosition',[0;0;0],...% Fused position is in scenario frame
    'OriginVelocity',[0;0;0],...% Scenario frame has zero velocity
    'Orientation',eye(3),...    % Scenario frame has no rotation.
    'IsParentToChild',false);   % Specify if rotation is specified in parent frame

% Represent the fused detection using objectDetection. It has a 3-D
% position and covariance.
fusedDetection = {objectDetection(0,zeros(3,1),'MeasurementNoise',eye(3),...
    'MeasurementParameters',measParam)};

% Change view
view(-125,9);

% Run scenario.
while advance(scene)
    % Get current simulation time.
    time = scene.SimulationTime;

    % Get bistatic range detections from 1 target.
    detections = detectBistaticTargetRange(scene,time,emitterIdx,true);

    % Triangulate detections to estimate target position.
    [position, covariance] = helperBistaticRangeFusion(detections);

    % Update the fused detection.
    fusedDetection{1}.Measurement = position;
    fusedDetection{1}.MeasurementNoise = covariance;

    % Update the display.
    theaterDisplay([detections;fusedDetection]);
end

% Write new GIF if requested
writeGIF(theaterDisplay);
% Restart the scenario and add remaining targets.
restart(scene);

% Number of targets added here.
numTargets = 4;

% randomly distributed targets.
r = abs(2000*randn(1,numTargets));                % Random ranges (m)
theta = linspace(0,numTargets*pi/4,numTargets);   % Angular position (rad)
xTgt = r.*cos(theta) + 100*randn(1,numTargets);   % x position (m)
yTgt = -r.*sin(theta) + 100*randn(1,numTargets);  % y position (m)

% Targets above ground.
zTgt = -1000*ones(1,numTargets); % z position (m)

for iD = 1:numTargets
    thisPlat = platform(scene,...
        'Trajectory',kinematicTrajectory('Position',[xTgt(iD) yTgt(iD) zTgt(iD)],...
        'Velocity',[10*randn 10*randn 5*randn]));
end

% Update platforms variable.
platforms = scene.Platforms;

% Reset the display.
release(theaterDisplay);

% Turn off plotting for bistatic ellipse for all targets.
theaterDisplay.PlotBistaticEllipse = false;

% No recording
theaterDisplay.GIF = '';

% Call once to plot new trajectories.
theaterDisplay();

% Scenario display with trajectories.
showScenario(theaterDisplay);
snapnow;
showGrabs(theaterDisplay,[]);
% Define a static detection fuser.
fuser = staticDetectionFuser( ...
    'MeasurementFusionFcn','helperBistaticRangeFusion', ...
    'UseParallel',true, ...                         % Do parallel processing
    'MaxNumSensors',numSensors, ...                 % Number of bistatic radar sensors
    'Volume',sensor.RangeResolution, ...            % Volumes of the sensors' detection bin
    'MeasurementFormat','custom', ...               % Define custom fused measurement as bistatic cannot not reported by cvmeas
    'MeasurementFcn','helperBistaticMeas',...       % Set measurement function for reporting bistatic measurement
    'DetectionProbability',0.99 ...                 % Probability of detecting the target
    );

% Define a GNN tracker.
tracker = trackerGNN('AssignmentThreshold',100);
while advance(scene)
    % Get current simulation time.
    time = scene.SimulationTime;

    % Get bistatic range detections
    detections = detectBistaticTargetRange(scene,time,emitterIdx);

    % Fuse bistatic detections into one structure.
    [superDets, info] = fuser(detections);

    % Track fused bistatic detections using the GNN tracker.
    confTracks = tracker(superDets,scene.SimulationTime);

    % Update display with current platform positions and tracks.
    theaterDisplay([superDets(:);detections(:)],confTracks);
end
showGrabs(theaterDisplay,1);
showGrabs(theaterDisplay,2,false);
function [pos,cov] = helperBistaticRangeFusion(detections)
% This function is for example purposes only and may be removed in a future
% release.
% This function returns the estimated position and covariance of the target
% given the bistatic detections generated from it.

%   Copyright 2019 The MathWorks, Inc.

% Do a coarse gating, as a minimum of 3 measurements are required for
% finding a solution.
if numel(detections) < 3
    pos = 1e10*ones(3,1);
    cov = 2*eye(3);
else
    % Retrieve info from measurements
    ranges = zeros(numel(detections),1);
    receiverLocations = zeros(3,numel(detections));
    emitterLocation = detections{1}.MeasurementParameters.EmitterPosition;
    for i = 1:numel(detections)
        rLoc =  detections{i}.MeasurementParameters(2).OriginPosition;
        receiverLocations(:,i) = rLoc;
        
        % The spherical intersection method assumes that measurment is 
        % Remit + Rrecv. Bistatic measurement is defined as Remit + Rrecv - Rb. 
        % Add the Rb to the actual measurement
        L = norm(emitterLocation(:) - rLoc(:));
        ranges(i) = detections{i}.Measurement + L;
    end
    pos = helperSphericalIntersection(ranges,receiverLocations,emitterLocation);
    
    % Covariance is calculated only when required. This helps saving
    % computation during cost calculation for static fusion, where only
    % position is required.
    if nargout > 1
        cov = linearFusionFcn(pos,detections);
    end
end

end

%% linear fusion function for measurement noise
function measCov = linearFusionFcn(pos,thisDetections)
% Linear noise fusion function. It requires measJacobian to use linear
% transformation. 
% Use a constant velocity state to calculate jacobians.
estState = zeros(6,1);
estState(1:2:end) = pos;
n = numel(thisDetections);
totalJacobian = zeros(n,3);
totalCovariance = zeros(n,n);
for i = 1:numel(thisDetections)
    H = cvmeasjac(estState,thisDetections{i}.MeasurementParameters);
    totalJacobian(i,:) = H(1,1:2:end);
    totalCovariance(i,i) = thisDetections{i}.MeasurementNoise;
end
toInvertJacobian = totalJacobian'/(totalCovariance)*totalJacobian;
I = eye(3);
% 2-D to 3-D conversion with 0 jacobian wrt z.
if toInvertJacobian(3,3) == 0
    toInvertJacobian(3,3) = 1;
end
measCov = I/toInvertJacobian;
% Return true positive definite.
measCov = (measCov + measCov')/2;
measCov(~isfinite(measCov)) = 1000; % Some big number for inf and nan
end
function detections = detectBistaticTargetRange(scene,time,emitterIdx,removeFalseAlarms)
    % Get platforms from scenario.
    platforms = scene.Platforms;

    % A flag to indicate if false alarms should be removed from detections.
    if nargin == 3
        removeFalseAlarms = false;
    end

    % Distinguish between receivers and targets to remove detections from
    % the receiver. It is assumed that these detections can be removed from
    % the batch using prior information.
    isReceiver = cellfun(@(x)~isempty(x.Sensors),scene.Platforms);
    allIDs = cellfun(@(x)x.PlatformID,scene.Platforms);
    receiverIDs = allIDs(isReceiver);

    % Generate RF emissions
    emitPlatform = platforms{emitterIdx};
    txEmiss = emit(emitPlatform, time);

    % Propagate the emissions and reflect these emissions from platforms.
    reflSigs = radarChannel(txEmiss, platforms,'HasOcclusion',false);

    % Generate detections from the bistatic radar sensor.
    detections = {};
    numPlat = numel(platforms);
    for iPlat = 1:numPlat
        thisPlatform = platforms{iPlat};

        % Receive the emissions, calculate interference losses, and
        % generate bistatic detections.
        thisDet = detect(thisPlatform, reflSigs, time);

        % Remove the detections that are the bistatic receivers. Only the
        % detections from the target platforms will be fused and tracked.
        detectedTargetIDs = cellfun(@(x)x.ObjectAttributes{1}.TargetIndex,thisDet);
        toRemove = ismember(detectedTargetIDs, receiverIDs) | removeFalseAlarms*(detectedTargetIDs<=0);
        thisDet = thisDet(~toRemove);

        % Add this platform's detections to the detections array.
        detections = [detections; thisDet]; %#ok<AGROW>
    end

    % Determine emitter position and velocity for this simulation time.
    emitterPosition = emitPlatform.Trajectory.Position(:);

    % Update detections structure to indicate that only bistatic range measurements are retained.
    for iD = 1:numel(detections)
        detections{iD}.Measurement = detections{iD}.Measurement(end);  %#ok<AGROW> % Range measurement
        detections{iD}.MeasurementNoise = detections{iD}.MeasurementNoise(end,end); %#ok<AGROW> % Measurement noise for range
        detections{iD}.MeasurementParameters(1).HasAzimuth = false; %#ok<AGROW> % Update measurement parameters to indicate that azimuth no longer exists
        detections{iD}.MeasurementParameters(1).HasElevation = false; %#ok<AGROW> % Update measurement parameters to indicate that elevation no longer exists
        detections{iD}.MeasurementParameters(1).EmitterPosition = emitterPosition; %#ok<AGROW> % Add emitter position
    end
end