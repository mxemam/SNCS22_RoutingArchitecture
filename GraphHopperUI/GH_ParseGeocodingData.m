function [fcnStatus, fcnMsg, coordsPointData] = GH_ParseGeocodingData(jsonDataStruct)
    % Parse the input Geocoding json data struct to extract point coordinates (lat, lng) information
    
    % Initialize outputs
    fcnStatus = -1; % ERROR flag
    fcnMsg = '';  % ERROR message
    coordsPointData = []; % Point data (lat, lng)
    
    try
        % Analyze input struct
        nrHits = length(jsonDataStruct.hits);
        if (nrHits == 0)
            % No points were found
            fcnMsg = 'ERROR: Invalid Address! Address coordinates could not be found.';
            return
        elseif (nrHits > 1)
            % Multiple points were found
            fcnStatus = 0;
            fcnMsg = ['WARNING: Ambiguous Address! '...
                'Multiple coordinates (' num2str(nrHits) ') were found, selecting the first result.'];
            if (iscell(jsonDataStruct.hits))
                coordsDataStruct = jsonDataStruct.hits{1}.point;
            else
                coordsDataStruct = jsonDataStruct.hits(1).point;
            end
        else
            fcnStatus = 1; % Refresh success flag
            coordsDataStruct = jsonDataStruct.hits.point;
        end
        
        % Extract point data [lat lng]
        coordsPointData(1) = coordsDataStruct.lat;
        coordsPointData(2) = coordsDataStruct.lng;
        
    catch myExcp
        % Error handling
        fcnMsg = sprintf('ERROR: Unknown Error occured! %s', myExcp.message);
        return
    end
    
    % Success
    fcnStatus = 1;
end