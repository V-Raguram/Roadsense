function [result,figureHandle]=startRoadsenseConfidenceDemo(options)
%STARTROADSENSECONFIDENCEDEMO One-command Roadsense 3D confidence launcher.
%   STARTROADSENSECONFIDENCEDEMO opens the verified real-time replay when a
%   cached closed-loop result is available. Set RunModel=true to execute the
%   full Simulink model before replaying its newly generated trajectory.
arguments
    options.PlaybackRate (1,1) double {mustBePositive}=1
    options.RunModel (1,1) logical=false
end
setupRoadsense;
[result,figureHandle]=runRoadsenseConfidenceDemo( ...
    PlaybackRate=options.PlaybackRate,RunModel=options.RunModel);
end
