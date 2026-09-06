function path=generateRoadsenseTuningFigure(comparison,outputDirectory)
%GENERATEROADSENSETUNINGFIGURE Create compact judge-facing profile comparison.
figureHandle=figure("Visible","off","Color","white","Position",[100 100 1150 620]);
cleanup=onCleanup(@() close(figureHandle)); layout=tiledlayout(2,2,"Padding","compact");
names=categorical(comparison.Profile,comparison.Profile);
nexttile; bar(names,[comparison.CollisionFree comparison.Completed comparison.Passed]);
ylabel("Scenarios"); title("Safety and completion"); legend("Collision-free","Completed","Passed","Location","best"); grid on;
nexttile; bar(names,comparison.MeanReadyPercent); yline(70,"--","Acceptance");
ylabel("Ready (%)"); title("Pipeline readiness"); grid on;
nexttile; bar(names,[comparison.WorstLatencyMs comparison.WorstJerkMps3]);
ylabel("Value"); title("Worst latency and jerk"); legend("Latency (ms)","Jerk (m/s^3)","Location","best"); grid on;
nexttile; bar(names,log10(1+comparison.Objective)); ylabel("log10(1 + objective)");
title("Safety-first objective (lower is better)"); grid on;
title(layout,"Roadsense closed-loop tuning comparison","FontWeight","bold");
path=fullfile(outputDirectory,"tuning_comparison.png"); exportgraphics(figureHandle,path,"Resolution",160);
end
