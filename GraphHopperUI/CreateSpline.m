function dataSpline = CreateSpline(xPoints, yPoints, bPlotData)
    % Create natural cubic spline from the input (x,y) points
    
    if ~exist('bPlotData', 'var')
        bPlotData = true;
    end
    
    % Reshape input arrays
    [nrRows, ~] = size(xPoints);
    if (nrRows > 1)
        xPoints = xPoints';
    end
    [nrRows, ~] = size(yPoints);
    if (nrRows > 1)
        yPoints = yPoints';
    end
    dataPoints = [xPoints; yPoints];

    % Calculate natural cubic spline
    dataSpline = cscvn(dataPoints);

    % Plot data if desired
    if bPlotData
        % Redistribute breaks for better display
        dispBreaks = linspace(dataSpline.breaks(1), dataSpline.breaks(end), 1001);
        % Evaluate spline at new breaks
        splineDataPoints = ppval(dataSpline, dispBreaks);
        % Plot original data points and spline
        figure;
        plot(dataPoints(1,:), dataPoints(2,:), 'ro', splineDataPoints(1,:), splineDataPoints(2,:),'b-');
    end
end