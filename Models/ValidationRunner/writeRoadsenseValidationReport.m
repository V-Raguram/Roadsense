function reportPath=writeRoadsenseValidationReport(summary,configuration,suiteWallTime)
%WRITEROADSENSEVALIDATIONREPORT Write a concise Markdown benchmark report.
reportPath=fullfile(configuration.OutputDirectory,"validation_report.md");
file=fopen(reportPath,"w");
if file<0; error("Roadsense:Validation:Report","Could not create %s.",reportPath); end
cleanup=onCleanup(@() fclose(file));
fprintf(file,"# Roadsense closed-loop validation report\n\n");
fprintf(file,"Contract version: %d  \n",configuration.ContractVersion);
fprintf(file,"Suite wall time: %.2f s  \n",suiteWallTime);
fprintf(file,"Scenario completion rate: %.1f%%  \n",100*mean(summary.Completed));
fprintf(file,"Collision-free rate: %.1f%%  \n",100*mean(summary.CollisionFree));
fprintf(file,"Acceptance rate: %.1f%%\n\n",100*mean(summary.Pass));
fprintf(file,"| ID | Scenario | Complete | Collision-free | Goal | Clearance m | Replan ms | Ready %% | Pass |\n");
fprintf(file,"|---:|---|:---:|:---:|:---:|---:|---:|---:|:---:|\n");
for index=1:height(summary)
    fprintf(file,"| %d | %s | %s | %s | %s | %.3f | %.2f | %.1f | %s |\n", ...
        summary.ScenarioID(index),summary.ScenarioName(index),yesNo(summary.Completed(index)), ...
        yesNo(summary.CollisionFree(index)),yesNo(summary.GoalReached(index)), ...
        summary.MinimumClearanceM(index),summary.MaximumReplanLatencyMs(index), ...
        summary.PipelineReadyPercent(index),yesNo(summary.Pass(index)));
end
fprintf(file,"\n## Acceptance limits\n\n");
fprintf(file,"- Minimum clearance: 0.10 m\n");
fprintf(file,"- Maximum replanning latency: 100 ms\n");
fprintf(file,"- Maximum RMS path curvature: 0.18 1/m\n");
fprintf(file,"- Maximum jerk: 10 m/s^3\n");
fprintf(file,"- Maximum cross-track error: 2.0 m\n");
fprintf(file,"- Minimum pipeline-ready ratio: 70%%\n");
end

function value=yesNo(flag)
if flag; value="yes"; else; value="no"; end
end
