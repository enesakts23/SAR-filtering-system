
numTracks = 20;
gate = 45;
vol = 1e9;
beta = 1e-14;
pd = 0.8;
far = 1e-6;
 
tracker = trackerJPDA(...
    'FilterInitializationFcn',@initCVFilter,...
    'MaxNumTracks', numTracks, ...
    'MaxNumSensors', 1, ...
    'AssignmentThreshold',gate, ...
    'TrackLogic','Integrated',...
    'DetectionProbability', pd, ...
    'ClutterDensity', far/vol, ...
    'NewTargetDensity', beta,...
    'TimeTolerance',0.05);
Define a track plotter


trackp = trackPlotter(tp,'DisplayName','Tracks','ConnectHistory','on','ColorizeHistory','on');
 Define a buffer for detections before the loop


detBuffer = {};
 Update the tracker within the loop


detBuffer = [detBuffer; dets]; 
if configs.IsScanDone
    tracks = tracker(detBuffer,scenario.SimulationTime);
    pos = getTrackPositions(tracks,[1 0 0 0 0 0; 0 0 1 0 0 0 ; 0 0 0 0 1 0]);
    labels = string([tracks.TrackID]);
    detBuffer = {};
end
Update track plotter


if configs.IsScanDone
    trackp.plotTrack(pos,labels);
end