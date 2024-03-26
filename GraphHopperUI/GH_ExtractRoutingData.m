function routingDataTable = GH_ExtractRoutingData(routeDataStruct)
    % Parse the input json data struct to extract routing information

    % Check number of data points
    nrDataPoints = length(routeDataStruct.paths.points.coordinates);
    
    % Prepare table columns
    tableCols = {'ID', 'double', 'geoCoords', 'double', 'crtCoords', 'double', ...
        'lanes', 'double', 'max_speed', 'double', 'road_class', 'string', 'street_name', 'string', 'text', 'string'};
    colNames = tableCols(1:2:end);
    colTypes = tableCols(2:2:end);
    tableSize = [nrDataPoints, length(colNames)];
    % Initialize table
    routingDataTable = table('Size', tableSize, 'VariableNames', colNames, 'VariableTypes', colTypes);
    
    % Extract simple data
    routingDataTable.ID = (1:nrDataPoints)';
    % Write coordinates data as [lat, lon, alt] instead of [lon, lat, alt]
    [~, nrCoords] = size(routeDataStruct.paths.points.coordinates);
    if (nrCoords < 3)
        zeroZPosVector = zeros(length(routeDataStruct.paths.points.coordinates(:, 1)), 1);
        routingDataTable.geoCoords = [routeDataStruct.paths.points.coordinates(:, 2), ...
            routeDataStruct.paths.points.coordinates(:, 1), zeroZPosVector];
    else
        routingDataTable.geoCoords = [routeDataStruct.paths.points.coordinates(:, 2), ...
            routeDataStruct.paths.points.coordinates(:, 1), routeDataStruct.paths.points.coordinates(:, 3)];
    end
    % Use latlon2local to convert coordinates to cartesian data
    [xPos, yPos, zPos] = latlon2local(routingDataTable.geoCoords(:, 1), routingDataTable.geoCoords(:, 2), ...
        routingDataTable.geoCoords(:, 3), routingDataTable.geoCoords(1, :));
    routingDataTable.crtCoords = [xPos, yPos, zPos];
    
    % Get lanes data
    for idx = 1:length(routeDataStruct.paths.details.lanes)
        startIdx = routeDataStruct.paths.details.lanes(idx, 1) + 1; % 0-based index
        endIdx = routeDataStruct.paths.details.lanes(idx, 2) + 1; % 0-based index
        routingDataTable.lanes(startIdx:endIdx) = routeDataStruct.paths.details.lanes(idx, 3);
    end
    
    % Get max speed data
    for idx = 1:length(routeDataStruct.paths.details.max_speed)
        startIdx = routeDataStruct.paths.details.max_speed(idx, 1) + 1; % 0-based index
        endIdx = routeDataStruct.paths.details.max_speed(idx, 2) + 1; % 0-based index
        maxSpeedValue = routeDataStruct.paths.details.max_speed(idx, 3);
        if isnan(maxSpeedValue)
            maxSpeedValue = 10; % 10 kmh for parking/service/living areas
        end
        routingDataTable.max_speed(startIdx:endIdx) = maxSpeedValue;
    end
    
    % Get road_class data
    for idx = 1:length(routeDataStruct.paths.details.road_class)
        roadClassCell = routeDataStruct.paths.details.road_class{idx};
        startIdx = roadClassCell{1} + 1; % 0-based index
        endIdx = roadClassCell{2} + 1; % 0-based index
        routingDataTable.road_class(startIdx:endIdx) = roadClassCell{3};
    end
    
    % Get instructions data
    for idx = 1:length(routeDataStruct.paths.instructions)
        instructStruct = routeDataStruct.paths.instructions{idx};
        startIdx = instructStruct.interval(1) + 1; % 0-based index
        endIdx = instructStruct.interval(2) + 1; % 0-based index
        routingDataTable.street_name(startIdx:endIdx) = instructStruct.street_name;
        routingDataTable.text(startIdx:endIdx) = instructStruct.text;
    end
end