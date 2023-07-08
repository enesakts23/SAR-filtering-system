[detectionBuffer,truthLog,theaterDisplay] = helperGenerateStaticFusionScenarioData;
showScenario(theaterDisplay);
showGrabs(theaterDisplay,[]);
% Number of sensors
numSensors = 3;

% Create a detection fuser using triangulateLOS function as the
% MeasurementFusionFcn and specify parameters of sensors.
fuser = staticDetectionFuser('MeasurementFusionFcn',@triangulateLOS,...
    'MaxNumSensors',numSensors,...
    'UseParallel',true,...
    'FalseAlarmRate',1e-3,...
    'Volume',0.0716,...
    'DetectionProbability',0.99);

% Tracking using a GNN tracker
tracker = trackerGNN('AssignmentThreshold',50,...
    'ConfirmationThreshold',[3 5],'DeletionThreshold',[4 5]);

% Use assignment and error metrics to compute accuracy.
trackingMetrics = trackAssignmentMetrics('DistanceFunctionFormat','custom',...
    'AssignmentDistanceFcn',@trueAssignment,'DivergenceDistanceFcn',@trueAssignment);
errorMetrics = trackErrorMetrics;
% Measurement noise
measNoise = 0.01;

time = 0;
dT = 1; % 1 Hz update rate of scenario.

% Loop through detections and track targets
for iter = 1:numel(detectionBuffer)
    % Truth information
    time = time + dT;
    sensorPlatPoses = truthLog{iter}(1:numSensors);
    targetPlatPoses = truthLog{iter}(6:end);
    groundTruth = [sensorPlatPoses;targetPlatPoses];

    % Generate noisy detections using recorded detections
    thisBuffer = detectionBuffer{iter};
    availableDetections = vertcat(thisBuffer{1:numSensors});
    noiseDetections = addNoise(availableDetections,measNoise);

    % Fuse noisy detections using fuser
    fusedDetections = fuser(noiseDetections);

    % Run a tracker on fused detections
    confTracks = tracker(fusedDetections,time);

    % Update track and assignment metrics
    trackingMetrics(confTracks,targetPlatPoses);
    [trackIDs,truthIDs] = currentAssignment(trackingMetrics);
    errorMetrics(confTracks,trackIDs,targetPlatPoses,truthIDs);

    % Update theater display
    detsToPlot = [noiseDetections(:);fusedDetections(:)];
    theaterDisplay(confTracks,detsToPlot,groundTruth);
end
axes(theaterDisplay.TheaterPlot.Parent);
ylim([0 1.5]);
assignmentTable = trackMetricsTable(trackingMetrics);
assignmentTable(:,{'TrackID','AssignedTruthID','TotalLength','FalseTrackStatus'})
disp(cumulativeTrackMetrics(errorMetrics));
numSensors = 3;
measNoise = 2; %standard deviation of sqrt(2) degrees
[trackingMetrics,errorMetrics] = helperRunStaticFusionSimulation(detectionBuffer,truthLog,numSensors,measNoise,theaterDisplay,false);
axes(theaterDisplay.TheaterPlot.Parent);
ylim([0 1.5]);
assignmentTable = trackMetricsTable(trackingMetrics);
assignmentTable(:,{'TrackID','AssignedTruthID','TotalLength','FalseTrackStatus'})
disp(cumulativeTruthMetrics(errorMetrics));
type('mexFuser');
% Get a sample detection from the stored buffer
sampleDetection = detectionBuffer{1}{1}{1};

% Use the coder.typeof function to allow variable-size inputs for
% detections.
maxNumDets = 500;
inputDets = coder.typeof({sampleDetection},[maxNumDets,1],[1 0]);

h = msgbox({'Generating code for function. This may take a few minutes...';...
    'This message box will close when done.'},'Codegen Message');

% Use the codegen command to generate code by specifying input arguments
% via example by using the |-args| option.
codegen mexFuser -args {inputDets};

close(h);
testDetections = addNoise(vertcat(detectionBuffer{1}{1:5}),1);
tic;mexFuser(testDetections);t_ML = toc;
tic;mexFuser_mex(testDetections);t_Mex = toc;
disp(['MATLAB Code Execution time = ',num2str(t_ML)]);
disp(['MEX Code Execution time = ',num2str(t_Mex)]);
measNoise = 2; % Same noise as 3 sensors
numSensors = 5;
[trackingMetrics,errorMetrics] = helperRunStaticFusionSimulation(detectionBuffer,truthLog,numSensors,measNoise,theaterDisplay,true);
axes(theaterDisplay.TheaterPlot.Parent);
ylim([0 1.5]);
assignmentTable = trackMetricsTable(trackingMetrics);
assignmentTable(:,{'TrackID','AssignedTruthID','TotalLength','FalseTrackStatus'})
disp(cumulativeTruthMetrics(errorMetrics))
