function result = runRoadsenseSemanticMapFusionDemo(saveFigure)
%RUNROADSENSESEMANTICMAPFUSIONDEMO Fuse and visualize a synthetic scene.

arguments
    saveFigure (1,1) logical = false
end
[imageSemantic,lidarGrid,tracks,predictions] = createSyntheticRoadsenseMapInputs;
fusion = RoadsenseSemanticMapFusionSystem;
for warmup = 1:3
    [timestamp,resolution,xLimits,yLimits,staticOccupancy,dynamicOccupancy, ...
        predictedRisk,occupancy,drivability,surfaceCost,combinedCost,semanticLabel, ...
        observedMask,processingTime,sourceOverflow,valid] = ...
        fusion(imageSemantic,lidarGrid,tracks,predictions);
end

limitsX = double(xLimits);
limitsY = double(yLimits);
figure(Name="Roadsense Semantic Map Fusion",Color="white");
tiledlayout(2,3,Padding="compact",TileSpacing="compact");
showLayer(staticOccupancy,"LiDAR static occupancy",limitsX,limitsY);
showLayer(drivability,"Fused drivability",limitsX,limitsY);
showLayer(surfaceCost,"Road-surface cost",limitsX,limitsY);
showLayer(dynamicOccupancy,"Current tracked footprints",limitsX,limitsY);

nexttile;
imagesc(limitsX,limitsY,predictedRisk); axis xy equal tight; colorbar; hold on;
for objectIndex = 1:double(predictions.Count)
    for mode = 1:double(predictions.NumModes(objectIndex))
        plot(squeeze(predictions.Positions(objectIndex,:,mode,1)), ...
            squeeze(predictions.Positions(objectIndex,:,mode,2)),"w-",LineWidth=0.6);
    end
end
xlabel("Forward (m)"); ylabel("Left (m)"); title("Multimodal predicted-risk corridor");

showLayer(combinedCost,"Planner combined cost",limitsX,limitsY);

if saveFigure
    componentDir = fileparts(mfilename("fullpath"));
    root = fileparts(fileparts(componentDir));
    outputDir = fullfile(root,"Results","SemanticMapFusion");
    if ~isfolder(outputDir); mkdir(outputDir); end
    exportgraphics(gcf,fullfile(outputDir,"fused_traversability_demo.png"),Resolution=180);
end

result = struct("Timestamp",timestamp,"Resolution",resolution,"XLimits",xLimits, ...
    "YLimits",yLimits,"StaticOccupancy",staticOccupancy, ...
    "DynamicOccupancy",dynamicOccupancy,"PredictedRisk",predictedRisk, ...
    "Occupancy",occupancy,"Drivability",drivability,"SurfaceCost",surfaceCost, ...
    "CombinedCost",combinedCost,"SemanticLabel",semanticLabel, ...
    "ObservedMask",observedMask,"ProcessingTime",processingTime, ...
    "SourceOverflow",sourceOverflow,"Valid",valid);
end

function showLayer(layer,titleText,xLimits,yLimits)
nexttile;
imagesc(xLimits,yLimits,layer); axis xy equal tight; colorbar;
xlabel("Forward (m)"); ylabel("Left (m)"); title(titleText);
end
