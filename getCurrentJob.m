function [currentjob,jobq] = getCurrentJob(jobq,current_time)

searchq   = jobq.SearchQueue;
trackq    = jobq.TrackQueue;
searchidx = jobq.SearchIndex;
num_trackq_items = jobq.NumTrackJobs;

% Update search queue index
searchqidx = mod(searchidx-1,numel(searchq))+1;

% Find the track job that is due and has the highest priority
readyidx = find([trackq(1:num_trackq_items).Time]<=current_time);
[~,maxpidx] = max([trackq(readyidx).Priority]);
taskqidx = readyidx(maxpidx);

% If the track job found has a higher priority, use that as the current job
% and increase the next search job priority since it gets postponed.
% Otherwise, the next search job due is the current job.
if ~isempty(taskqidx) && trackq(taskqidx).Priority >= searchq(searchqidx).Priority
    currentjob = trackq(taskqidx);
    for m = taskqidx+1:num_trackq_items
        trackq(m-1) = trackq(m);
    end
    num_trackq_items = num_trackq_items-1;
    searchq(searchqidx).Priority = searchq(searchqidx).Priority+100;
else
    currentjob = searchq(searchqidx);
    searchidx = searchqidx+1;

end

jobq.SearchQueue  = searchq;
jobq.SearchIndex  = searchidx;
jobq.TrackQueue   = trackq;
jobq.NumTrackJobs = num_trackq_items;
function [xrsint,xrdazint,xrdelint,mfradar] = generateEcho(mfradar,env,current_job)

% Radar position
radarpos = [0;0;0];
radarvel = [0;0;0];

% Number of pulses and operating frequency
Npulses = 10;
fc = mfradar.TxAnt.OperatingFrequency;

for m = 1:Npulses
    % Waveform
    x = mfradar.Wav();
    
    % Update target motion
    [tgtpos,tgtvel] = env.TargetMotion(1/mfradar.Wav.PRF);
    [~,tgtang]      = rangeangle(tgtpos);
    
    % Transmit
    [xt,inuseflag]  = mfradar.Tx(x);
    w  = mfradar.STV(fc,current_job.BeamDirection);
    xt = mfradar.TxAnt(xt,tgtang,conj(w));
    
    % Propagation
    xp = env.Channel(xt,radarpos,tgtpos,radarvel,tgtvel);
    xp = env.Target(xp);
    
    % Receive and monopulse
    xr = mfradar.RxAnt(xp,tgtang);
    [xrs,xrdaz,xrdel] = mfradar.RxFeed(xr,current_job.BeamDirection);
    
    % Pulse integration
    if m == 1
        xrsint   = mfradar.Rx(xrs,~(inuseflag>0));
        xrdazint = mfradar.Rx(xrdaz,~(inuseflag>0));
        xrdelint = mfradar.Rx(xrdel,~(inuseflag>0));
    else
        xrsint   = xrsint+mfradar.Rx(xrs,~(inuseflag>0));
        xrdazint = xrdazint+mfradar.Rx(xrdaz,~(inuseflag>0));
        xrdelint = xrdelint+mfradar.Rx(xrdel,~(inuseflag>0));
    end
end
function [detection,mfradar] = generateDetection(xrsint,xrdazint,xrdelint,mfradar,current_job,current_time)

% Compute detection threshold
nbw         = mfradar.Rx.SampleRate/(mfradar.Wav.SampleRate/mfradar.Wav.SweepBandwidth);
npower      = noisepow(nbw,mfradar.Rx.NoiseFigure,mfradar.Rx.ReferenceTemperature);
pfa         = 1e-6;
threshold   = npower * db2pow(npwgnthresh(pfa,1,'noncoherent'));
arraysz     = mfradar.TxAnt.Sensor.Size(1);
ant_snrgain = pow2db(arraysz^2);
mfcoeff     = getMatchedFilter(mfradar.Wav);
mfgain      = pow2db(norm(mfcoeff)^2);
threshold   = threshold * db2pow(mfgain+2*ant_snrgain); 
threshold   = sqrt(threshold);    

tgrid   = unigrid(0,1/mfradar.Wav.SampleRate,1/mfradar.Wav.PRF,'[)');
rgates  = mfradar.TxAnt.PropagationSpeed*tgrid/2;

% Matched filtering and time varying gain
xrsmf = mfradar.TVG(mfradar.MF(xrsint));

% Detection in range and angle estimation via monopulse
if any(abs(xrsmf)>threshold)
    [~,tgtidx] = findpeaks(abs(xrsmf),'MinPeakHeight',threshold,...
        'Sortstr','Descend','NPeaks',1);
    rng_est = rgates(tgtidx-(numel(mfcoeff)-1));
    ang_est = mfradar.DOA(xrsint(tgtidx-1),xrdazint(tgtidx-1),xrdelint(tgtidx-1),current_job.BeamDirection);
    % Form the detection object.  
    measNoise = diag([0.1, 0.1, 150].^2);           % Measurement noise matrix
    detection = objectDetection(current_time,...
        [ang_est(1);ang_est(2);rng_est], 'MeasurementNoise', measNoise,...
        'MeasurementParameters',struct('Frame','spherical', 'HasVelocity', false));  
else
    detection = objectDetection.empty;
end

if current_time < 0.3 || strcmp(current_job.JobType,'Track')
    fprintf('\n%f sec:\t%s\t[%f %f]',current_time,current_job.JobType,current_job.BeamDirection(1),...
        current_job.BeamDirection(2));
end
function [jobq,allTracks,mfradar] = updateTrackAndJob(detection,jobq,mfradar,current_job,current_time,dwellinterval)

trackq           = jobq.TrackQueue;
num_trackq_items = jobq.NumTrackJobs;

% Execute current job
switch current_job.JobType
    case 'Search'
        % For search job, if there is a detection, establish tentative
        % track and schedule a confirmation job
        if ~isempty(detection)
            ang_est = detection.Measurement(1:2);
            rng_est = detection.Measurement(3);
            if ~mfradar.IsTrackerInitialized
                [~,~,allTracks] = mfradar.Tracker(detection,current_time,uint32([]));
                mfradar.IsTrackerInitialized = true;
            else
                [~,~,allTracks] = mfradar.Tracker(detection,current_time,uint32([]));
            end
            num_trackq_items = num_trackq_items+1;
            trackq(num_trackq_items) = struct('JobType','Confirm','Priority',2000,...
                'BeamDirection',ang_est,'WaveformIndex',1,'Time',current_time+dwellinterval,...
                'Range',rng_est,'TrackID',allTracks(~[allTracks.IsConfirmed]).TrackID);
            if current_time < 0.3 || strcmp(current_job.JobType,'Track')
                fprintf('\tTarget detected at %f m',rng_est);
            end
        else
            allTracks = [];
        end

    case 'Confirm'
        % For confirm job, if the detection is confirmed, establish a track
        % and create a track job corresponding to the revisit time
        if ~isempty(detection)
            trackid = current_job.TrackID;
            [~,~,allTracks] = mfradar.Tracker(detection,current_time,trackid);

            rng_est = detection.Measurement(3);
            if rng_est >= 50e3
                updateinterval = 0.5;
            else
                updateinterval = 0.1;
            end
            revisit_time = current_time+updateinterval;
            predictedTrack = predictTracksToTime(mfradar.Tracker,trackid,revisit_time);
            xpred = predictedTrack.State([1 3 5]);
            [phipred,thetapred,rpred] = cart2sph(xpred(1),xpred(2),xpred(3));
            num_trackq_items = num_trackq_items+1;
            trackq(num_trackq_items) = struct('JobType','Track','Priority',3000,...
                'BeamDirection',rad2deg([phipred;thetapred]),'WaveformIndex',1,'Time',revisit_time,...
                'Range',rpred,'TrackID',trackid);
            if current_time < 0.3 || strcmp(current_job.JobType,'Track')
                fprintf('\tCreated track %d at %f m',trackid,rng_est);
            end
        else
            allTracks = [];
        end

    case 'Track'
        % For track job, if there is a detection, update the track and
        % schedule a track job corresponding to the revisit time. If there
        % is no detection, predict and schedule a track job sooner so the
        % target is not lost.
        if ~isempty(detection)
            trackid = current_job.TrackID;
            [~,~,allTracks] = mfradar.Tracker(detection,current_time,trackid);

            rng_est = detection.Measurement(3);
            if rng_est >= 50e3
                updateinterval = 0.5;
            else
                updateinterval = 0.1;
            end

            revisit_time = current_time+updateinterval;
            predictedTrack = predictTracksToTime(mfradar.Tracker,trackid,revisit_time);
            xpred = predictedTrack.State([1 3 5]);
            [phipred,thetapred,rpred] = cart2sph(xpred(1),xpred(2),xpred(3));
            num_trackq_items = num_trackq_items+1;
            trackq(num_trackq_items) = struct('JobType','Track','Priority',3000,...
                'BeamDirection',rad2deg([phipred;thetapred]),'WaveformIndex',1,'Time',revisit_time,...
                'Range',rpred,'TrackID',trackid);

            if current_time < 0.3 || strcmp(current_job.JobType,'Track')
                fprintf('\tTrack %d at %f m',trackid,rng_est);
            end
        else
            trackid = current_job.TrackID;
            [~,~,allTracks] = mfradar.Tracker(detection,current_time,trackid);

            updateinterval = 0.1;  % revisit sooner
            revisit_time = current_time+updateinterval;
            predictedTrack = predictTracksToTime(mfradar.Tracker,trackid,revisit_time);
            xpred = predictedTrack.State([1 3 5]);

            [phipred,thetapred,rpred] = cart2sph(xpred(1),xpred(2),xpred(3));
            num_trackq_items = num_trackq_items+1;
            trackq(num_trackq_items) = struct('JobType','Track','Priority',3000,...
                'BeamDirection',rad2deg([phipred;thetapred]),'WaveformIndex',1,'Time',revisit_time,...
                'Range',rpred,'TrackID',trackid);

            if current_time < 0.3 || strcmp(current_job.JobType,'Track')
                fprintf('\tNo detection, track %d predicted',current_job.TrackID);
            end
        end

end
    
jobq.TrackQueue   = trackq;
jobq.NumTrackJobs = num_trackq_items;
