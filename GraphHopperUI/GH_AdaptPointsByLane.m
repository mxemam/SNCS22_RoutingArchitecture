function [adaptedDataTable, nrAdaptedPoints] = GH_AdaptPointsByLane(routingDataTable, laneWidth)
    % Adapt the location of waypoints by lane

    nrAdaptedPoints = 0;
    
    if ~exist('laneWidth', 'var')
        % Assume constant lane width of 3.25 meters
        laneWidth = 3.25;
    end
    
    % Create a spline from the data to determine the heading
    routeSpline = CreateSpline(routingDataTable.crtCoords(:, 1), routingDataTable.crtCoords(:, 2), false);
    
    % Loop on all data points
    adaptedDataTable = routingDataTable;
    nrDataPoints = height(adaptedDataTable);
    for idx = 1: (nrDataPoints-1)
        bAdaptPoint = false;
        
        if (adaptedDataTable.lanes(idx) > 1)
            bAdaptPoint = true;
            % Project point to the right with an offset corresponding to number of lanes
            projDirect = 'r';
            nrmDistOffset = (laneWidth/2) * (adaptedDataTable.lanes(idx) - 1);
        end
        
        if ~bAdaptPoint
            if (idx > 1) && (~strcmp(adaptedDataTable.street_name(idx), adaptedDataTable.street_name(idx-1)))
                % Analyze turning directions
                if contains(adaptedDataTable.text(idx), ' right', 'IgnoreCase', true)
                    % Turn right (en)
                    bAdaptPoint = true;
                    % Project point to the right with half lane width for smoother path
                    projDirect = 'r';
                    nrmDistOffset = (laneWidth/2) * (max(adaptedDataTable.lanes(idx), adaptedDataTable.lanes(idx-1)) - 1);
                elseif contains(adaptedDataTable.text(idx), ' left', 'IgnoreCase', true)
                    % Turn left (en)
                    bAdaptPoint = true;
                    % Project point to the left with half lane width for smoother path
                    projDirect = 'l';
                    nrmDistOffset = (laneWidth/2) * (max(adaptedDataTable.lanes(idx), adaptedDataTable.lanes(idx-1)) - 1);
                elseif strncmpi(adaptedDataTable.text(idx), 'Rechts', 5)
                    % Turn right (de)
                    bAdaptPoint = true;
                    % Project point to the right with half lane width for smoother path
                    projDirect = 'r';
                    nrmDistOffset = (laneWidth/2) * (max(adaptedDataTable.lanes(idx), adaptedDataTable.lanes(idx-1)) - 1);
                elseif strncmpi(adaptedDataTable.text(idx), 'Links', 5)
                    % Turn left (de)
                    bAdaptPoint = true;
                    % Project point to the left with half lane width for smoother path
                    projDirect = 'l';
                    nrmDistOffset = (laneWidth/2) * (max(adaptedDataTable.lanes(idx), adaptedDataTable.lanes(idx-1)) - 1);
                end
            end
        end
        
        if bAdaptPoint
            [xPos, yPos, ~] = GetProjPointData(routeSpline, idx, nrmDistOffset, projDirect);
            % Update crt coords in table
            adaptedDataTable.crtCoords(idx, 1:2) = [xPos, yPos];
            % Update geo coords in table
            [lat, lon, alt] = local2latlon(xPos, yPos, adaptedDataTable.crtCoords(idx, 3), ...
                adaptedDataTable.geoCoords(1, :));
            adaptedDataTable.geoCoords(idx, :) = [lat, lon, alt];
            nrAdaptedPoints = nrAdaptedPoints + 1;
        end
    end
end


function [xPos, yPos, psiVal] = GetProjPointData(routeSpline, idx, nrmDistOffset, projDirect)
    % Get projection point data in Cartesian coordinates

    if ~exist('projDirect', 'var')
        projDirect = 'r';
    end
    % (xm, ym) of road
    x_road = routeSpline.coefs((2*idx)-1, end);
    y_road = routeSpline.coefs(2*idx, end);
    % (xm', ym') of road
    xp_road = routeSpline.coefs((2*idx)-1, end-1);
    yp_road = routeSpline.coefs(2*idx, end-1);
    % psi(t) = arctan(ym'/xm')
    psiVal = atan2(yp_road, xp_road);
    if strcmp(projDirect, 'r')
        % x(t) = xm + (sin(psi) * r)
        xPos = x_road + nrmDistOffset * sin(psiVal);
        % y(t) = ym + (-cos(psi) * r)
        yPos = y_road - nrmDistOffset * cos(psiVal);
    else
        % x(t) = xm + (-sin(psi) * r)
        xPos = x_road - nrmDistOffset * sin(psiVal);
        % y(t) = ym + (+cos(psi) * r)
        yPos = y_road + nrmDistOffset * cos(psiVal);
    end
end