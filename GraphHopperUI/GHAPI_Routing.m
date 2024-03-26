function [fcnStatus, fcnMsg, rawDataStr] = GHAPI_Routing(startPoint, endPoint, apiKey, optionsMap)
    % DOKU:
    %   https://docs.graphhopper.com/#tag/Routing-API
    % GUI:
    %   https://graphhopper.com/maps/?point=49.932707%2C11.588051&point=50.3404%2C12.64705
    % API:
    %   https://graphhopper.com/api/1/route?point=49.932707%2C11.588051&point=50.3404%2C12.64705&key=
    
    % Initialize outputs
    fcnStatus = -1; % ERROR flag
    fcnMsg = '';  % ERROR message
    rawDataStr = ''; % Raw json data string
    
    if ~exist('optionsMap', 'var')
        optionsMap = containers.Map();
    end
    SetMissingOptions(optionsMap);
    
    % Create the URL string
    createUrlStr = '';
    createUrlStr = [createUrlStr 'https://graphhopper.com/api/1/route?'];
    createUrlStr = [createUrlStr 'point=' startPoint];
    createUrlStr = [createUrlStr '&' 'point=' endPoint];
    createUrlStr = [createUrlStr '&' 'debug=true'];
    createUrlStr = [createUrlStr '&' 'type=json'];
    createUrlStr = [createUrlStr '&' 'points_encoded=false'];
    createUrlStr = [createUrlStr '&' 'key=' apiKey];
    createUrlStr = [createUrlStr '&' 'locale=' lower(optionsMap('Language'))];
    createUrlStr = [createUrlStr '&' 'vehicle=' lower(optionsMap('VehicleType'))];
    if (optionsMap('bHasElevation'))
        createUrlStr = [createUrlStr '&' 'elevation=true'];
    end
    if (optionsMap('bHasEdgeId'))
        createUrlStr = [createUrlStr '&' 'details=edge_id'];
    end
    if (optionsMap('bHasRoadClass'))
        createUrlStr = [createUrlStr '&' 'details=road_class'];
    end
    if (optionsMap('bHasMaxSpeed'))
        createUrlStr = [createUrlStr '&' 'details=max_speed'];
    end
    if (optionsMap('bHasLaneInfo'))
        createUrlStr = [createUrlStr '&' 'details=lanes'];
    end
    
    % Call the routing API with the URL
    oOptions = weboptions('Timeout', 10, 'ContentType', 'text');
    try
        rawDataStr = webread(createUrlStr, oOptions);
    catch
        fcnMsg = ['ERROR: Wrong Credentials! '...
            'Please check your API key or register for one at https://www.graphhopper.com/developers/'];
        return
    end
    
    % Success flag
    fcnStatus = 1;
end


function optionsMap = SetMissingOptions(optionsMap)
    optionsDataCell = {'Language', 'en', 'VehicleType', 'car', 'bHasElevation', true, 'bHasRoadClass', true, ...
        'bHasMaxSpeed', true, 'bHasLaneInfo', true, 'bHasEdgeId', false};
    for idx = 1:2:length(optionsDataCell)
        if ~optionsMap.isKey(optionsDataCell{idx})
            optionsMap(optionsDataCell{idx}) = optionsDataCell{idx+1};
        end
    end
end