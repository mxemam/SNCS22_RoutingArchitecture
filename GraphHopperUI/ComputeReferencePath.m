function [refSpline, refDataMatrix] = ComputeReferencePath(startPoint, endPoint, apiKey)
    % Algorithm 1: Compute reference path and waypoints between start point (lat,lon) and end point (lat,lon)
    
    refSpline = [];
    refDataMatrix = [];
    
    % Route using GraphHopper online API
    [fcnStatus, ~, rawDataStr] = GHAPI_Routing(startPoint, endPoint, apiKey);
    if (fcnStatus < 1)
        return
    end
    routeDataStruct = jsondecode(rawDataStr);
    
    % Prepare data table
    routeDataTable = GH_ExtractRoutingData(routeDataStruct);
    extendDataTable = GH_ExtendDataWaypoints(routeDataTable);
    adaptDataTable = GH_AdaptPointsByLane(extendDataTable);
    
    % Extract reference path data as [x, y, v_max] (ignore altitude)
    refPathData = adaptDataTable.geoCoords;
    refPathData(:, 3) = adaptDataTable.max_speed*5/18; % Convert speed from km/h to m/s 
    refPathData(1, :) = []; % ignore first line (header)
    % Convert to local coordinates
    originPoint = [refPathData(1, 1:2), 0];
    [xPos, yPos, ~] = latlon2local(refPathData(:, 1), refPathData(:, 2), zeros(length(refPathData), 1), originPoint);
    refPathData = [xPos, yPos, refPathData(:, 3)];
    % Create spline from path
    refSpline = CreateSpline(refPathData(:, 1), refPathData(:, 2));
    
    % Prepare data matrix as [s, x, xp, xpp, xppp, y, yp, ypp, yppp, v]
    refDataLines = length(refSpline.breaks) - 1;
    refDataMatrix = zeros(refDataLines, 10);
    refDataMatrix(1, :) = [0, refSpline.coefs(1, end:-1:1), refSpline.coefs(2, end:-1:1), refPathData(1, 3)];
    for idx = 2:refDataLines
        refDataMatrix(idx, :) = [refSpline.breaks(idx), ...
            refSpline.coefs((2*idx)-1, end:-1:1), refSpline.coefs(2*idx, end:-1:1), refPathData(idx,3)];
    end
end