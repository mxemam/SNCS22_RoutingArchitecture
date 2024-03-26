function [fcnStatus, fcnMsg, waypointsData] = GH_ParseRoutingData(jsonDataStruct)
    % Parse the input routing json data struct to extract route information
    
    % Initialize outputs
    fcnStatus = -1; % ERROR flag
    fcnMsg = '';  % ERROR message
    waypointsData = []; % Waypoints are either in the format (x,y) or (x,y,z) based on routing options
    
    try
        % Return waypoints data
        waypointsData = jsonDataStruct.paths.points.coordinates(:, :);
        
    catch myExcp
        % Error handling
        fcnMsg = sprintf('ERROR: Unknown Error occured! %s', myExcp.message);
        return
    end
    
    % Success
    fcnStatus = 1;
end