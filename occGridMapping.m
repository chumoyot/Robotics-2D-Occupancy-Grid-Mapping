% Robotics: Estimation and Learning 
% WEEK 3
% 
% Complete this function following the instruction. 
function map = occGridMapping(ranges, scanAngles, pose, param)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Parameters 
% 
% the number of grids for 1 meter.
resol = param.resol;
% the initial myMap size in pixels
map = zeros(param.size);
% the origin of the myMap in pixels
origin = param.origin; 

% 4. Log-odd parameters 
lo_occ = param.lo_occ;
lo_free = param.lo_free; 
lo_max = param.lo_max;
lo_min = param.lo_min;


N = size(pose,2);
numScans = size(scanAngles);
for j = 1:N 
    x = pose(1, j);
    y = pose(2, j);
    theta = pose(3, j); %san angles

    ix_robot = ceil(x * resol) + origin(1);
    iy_robot = ceil(y * resol) + origin(2);
    

    % Find grids hit by the rays (in the gird myMap coordinate)
    rays = ranges(:, j);
    x_occ = rays .* cos(scanAngles + theta) + x;
    y_occ = -rays .* sin(scanAngles + theta) + y;
    ix_occ = ceil(x_occ * resol) + origin(1);
    iy_occ = ceil(y_occ * resol) + origin(2);

    % Find occupied-measurement cells and free-measurement cells
    occ = sub2ind(size(map), iy_occ, ix_occ); % Convert to 1ndices

    free = [];
    for k = 1:numScans
        [ix_free, iy_free] = bresenham(ix_robot, iy_robot, ix_occ(k), iy_occ(k));  
        free = [free; iy_free, ix_free]; %union of the free cells
    end
    free = sub2ind(size(map), free(:, 1), free(:, 2)); % Convert to 1d

    % Update the log-odds
    map(occ) = map(occ) + lo_occ;
    map(free) = map(free) - lo_free;

    % Saturate the log-odd values
    map(map > lo_max) = lo_max; %upper limit of the log odd
    map(map < lo_min) = lo_min; %lower limit of the log odd

    
end

end
