load('MultiplatformScenarioRecording.mat');
tracker = trackerGNN('FilterInitializationFcn', @initFilter, ...
        'AssignmentThreshold', 50, 'DeletionThreshold', 3, ...
        'HasDetectableTrackIDsInput', true);
    trackUpdateRate = 0.5;   % Update the tracker every 2 seconds

% Create a display to show the true, measured, and tracked positions of the
% detected targets and platforms.
theaterDisplay = helperMultiPlatFusionDisplay(recording,'PlotAssignmentMetrics', true);

% Construct an object to analyze assignment and error metrics
tam = trackAssignmentMetrics('DistanceFunctionFormat','custom',...
    'AssignmentDistanceFcn',@truthAssignmentDistance,...
    'DivergenceDistanceFcn',@truthAssignmentDistance);

% Initialize buffers
detBuffer = {};
sensorConfigBuffer = [];
allTracks = [];
detectableTrackIDs = uint32([]);
assignmentTable = [];

% Initialize next tracker update time
nextTrackUpdateTime = 2;

while ~isDone(recording)
    % Read the next record of the recording.
    [time, truePoses, covcon, dets, senscon, sensPlatIDs] = read(recording);
    
    % Buffer all detections and sensor configurations until the next tracker update
    detBuffer = [detBuffer ; dets]; %#ok<AGROW>
    sensorConfigBuffer = [sensorConfigBuffer ; senscon']; %#ok<AGROW>
        
    % Follow the trackUpdateRate to update the tracker
    if time >= nextTrackUpdateTime || isDone(recording)
        
        if isempty(detBuffer)
            lastDetectionTime = time;
        else
            lastDetectionTime = detBuffer{end}.Time;
        end
        
        if isLocked(tracker)
            % Collect list of tracks which fell within at least one radar's field
            % of view since the last tracker update
            predictedtracks = predictTracksToTime(tracker, 'all', lastDetectionTime);
            detectableTrackIDs = detectableTracks(allTracks, predictedtracks, sensorConfigBuffer);
        end
        
        % Update tracker. Only run track logic on tracks that fell within at
        % least one radar's field of view since the last tracker update
        [confirmedTracks, ~, allTracks] = tracker(detBuffer, lastDetectionTime, detectableTrackIDs);
        
        % Analyze and retrieve the current track-to-truth assignment metrics
        tam(confirmedTracks, truePoses);
        
        % Store assignment metrics in a table
        currentAssignmentTable = trackMetricsTable(tam);
        rowTimes = seconds(time*ones(size(currentAssignmentTable,1),1));
        assignmentTable = cat(1,assignmentTable,table2timetable(currentAssignmentTable,'RowTimes',rowTimes));
        
        % Update display with detections, coverages, and tracks
        theaterDisplay(detBuffer, covcon, confirmedTracks, assignmentTable, truePoses, sensPlatIDs);
        
        % Empty buffers
        detBuffer = {};
        sensorConfigBuffer = [];
        
        % Update next track update time
        nextTrackUpdateTime = nextTrackUpdateTime + 1/trackUpdateRate;
    end
    
end
endTime = assignmentTable.Time(end);
assignmentTable(endTime,{'TrackID','AssignedTruthID','TotalLength','DivergenceCount','RedundancyCount','RedundancyLength'})
maxCondNum = 300;
figure;
helperPlotLongRangeCorrection(maxCondNum)
[confirmedTracks,correctedAssignmentTable,ctheaterDisplay] = ...
    runMultiPlatFusionSim(recording,tracker,@longRangeCorrection);
endTime = correctedAssignmentTable.Time(end);
correctedAssignmentTable(endTime,{'TrackID','AssignedTruthID','TotalLength','DivergenceCount','RedundancyCount','RedundancyLength'})
allDetections = vertcat(recording.RecordedData.Detections);
ctheaterDisplay(allDetections,covcon,confirmedTracks,correctedAssignmentTable,truePoses, sensPlatIDs) 
axes(ctheaterDisplay.TheaterPlot.Parent)
legend('off')
xlim([-1000 5000]); ylim([-41000 -36000]); zlim([-5000 0]);
view([-90 90])
axis square
title('Jet Executing Horizontal Turn')
view([-60 25])
xlim([-25000 -9000]); ylim([-31000 -19000]); zlim([-9000 -2000]);
view([-45 10])
title('Crossing Airliners')
xlim([-10000 10000]); ylim([-0 10000]); zlim([-12000 -5000]);
view([-15 10])
title('Airborne Radar Platforms')
