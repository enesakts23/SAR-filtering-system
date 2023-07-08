model='rtwdemo_eml_aero_radar';
open_system(model)
rtwdemo_eml_aero_radar([],[],[],'compile');
rtwdemo_eml_aero_radar([],[],[],'term');
open_system([model,'/RadarTracker'])
sim(model)
% Create a temporary folder (in your system's temporary folder) for the
% build and inspection process.
currentDir = pwd;
[~,cgDir] = rtwdemodir();
rtwconfiguredemo(model,'GRT')
rtwbuild([model,'/RadarTracker'])
rtwconfiguredemo(model,'ERT')
rtwbuild([model,'/RadarTracker'])
cfile = fullfile(cgDir,'RadarTracker_ert_rtw','RadarTracker.c');
rtwdemodbtype(cfile,'/* Model step', '/* Model initialize', 1, 0);
web(fullfile(cgDir,'RadarTracker_ert_rtw','html','RadarTracker_codegen_rpt.html'))
bdclose(model)
rtwdemoclean;
cd(currentDir)
