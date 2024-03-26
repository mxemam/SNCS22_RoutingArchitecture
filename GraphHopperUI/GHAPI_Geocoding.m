function [fcnStatus, fcnMsg, rawDataStr] = GHAPI_Geocoding(addressStr, apiKey, infoLangStr)
    % DOKU:
    %   https://docs.graphhopper.com/#tag/Geocoding-API
    % API:
    %   https://graphhopper.com/api/1/geocode?q="Zwergerstr. 31, 85579 Neubiberg"&key=
    
    % Initialize outputs
    fcnStatus = -1; % ERROR flag
    fcnMsg = '';  % ERROR message
    rawDataStr = ''; % Raw json data string
    
    % Create the URL string
    createUrlStr = '';
    createUrlStr = [createUrlStr 'https://graphhopper.com/api/1/geocode?q=' addressStr];
    createUrlStr = [createUrlStr '&' 'debug=true'];
    createUrlStr = [createUrlStr '&' 'key=' apiKey];
    createUrlStr = [createUrlStr '&' 'locale=' lower(infoLangStr)];
    
    % Call the geocoding API with the URL
    oOptions = weboptions('Timeout', 5, 'ContentType', 'text');
    try
        rawDataStr = webread(createUrlStr, oOptions);
    catch
        fcnMsg = ['ERROR: Wrong Credentials! '...
            'Please check your API key or register for one at "https://www.graphhopper.com/developers/".'];
        return
    end
    
    % Success
    fcnStatus = 1;
end