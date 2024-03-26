function adaptedDataTable = GH_ExtendDataWaypoints(routingDataTable)
    % Add extra waypoints if the distance between two points is too large

    % Add a new column for data sorting
    nrPoints = height(routingDataTable);
    adaptedDataTable = routingDataTable;
    adaptedDataTable.sort_ID = zeros(nrPoints, 1);
    sortIdx = 1;
    % Loop on all existing points
    for idx = 1:(nrPoints-1)
        % Min points resolution
        minDistTwoPnts = max(10, min(adaptedDataTable.max_speed(idx), adaptedDataTable.max_speed(idx+1)));
        if (adaptedDataTable.sort_ID(idx) <= 0)
            adaptedDataTable.sort_ID(idx) = sortIdx;
            sortIdx = sortIdx + 1;
        end
       [adaptedDataTable, sortIdx] = AppendExtraPoints(adaptedDataTable, minDistTwoPnts, idx, idx+1, sortIdx);
    end
    % Sort data table
    adaptedDataTable = sortrows(adaptedDataTable, {'sort_ID'});
end



function [adaptedDataTable, sortIdx] = AppendExtraPoints(adaptedDataTable, minDistPnts, firstIdx, secondIdx, sortIdx)
    % Add extra points if the distance between two points is too large

    % Check distance between points
    distValue = GetEuclideanDistance(adaptedDataTable.crtCoords(firstIdx, :), adaptedDataTable.crtCoords(secondIdx, :));
    if (distValue > minDistPnts)
        % Add a point in the middle, and use data of the first point for streetname, lanes, etc.
        rowData = adaptedDataTable(firstIdx, :);
        rowData.crtCoords = GetMidPoint(adaptedDataTable.crtCoords(firstIdx, :), ...
            adaptedDataTable.crtCoords(secondIdx, :));
        % Fix the point geo data for later usage
        [lat, lon, alt] = local2latlon(rowData.crtCoords(1), rowData.crtCoords(2), rowData.crtCoords(3), ...
            adaptedDataTable.geoCoords(1, :));
        rowData.geoCoords = [lat, lon, alt];
        % Append to the data table
        adaptedDataTable = [adaptedDataTable; rowData];
        lastRowIdx = height(adaptedDataTable);
        % Call function recursively
        [adaptedDataTable, sortIdx] = AppendExtraPoints(adaptedDataTable, minDistPnts, firstIdx, lastRowIdx, sortIdx);
        [adaptedDataTable, sortIdx] = AppendExtraPoints(adaptedDataTable, minDistPnts, lastRowIdx, secondIdx, sortIdx);
        if (adaptedDataTable.sort_ID(lastRowIdx) <= 0)
        % Set the point sorting index
            adaptedDataTable.sort_ID(idx) = sortIdx;
            sortIdx = sortIdx + 1;
        end
    else
        % Set the point sorting index
        adaptedDataTable.sort_ID(secondIdx) = sortIdx;
        sortIdx = sortIdx + 1;
    end
end



function distValue = GetEuclideanDistance(pointA, pointB)
    % Get Euclidean distance between two points
    distValue = sqrt(sum((pointA - pointB).^2));
end



function midPoint = GetMidPoint(pointA, pointB)
    % Get Mid point between two points
    midPoint = (pointA + pointB)./2;
end