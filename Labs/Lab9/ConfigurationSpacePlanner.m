cos% Function to Create a Viable Configuration Space for the Ottia SCARA Arm
% Units: mm, rad.
% Origin is Joint A Axis.
% Returns: a Binary Matrix Representing the Revolute-Revolute SCARA 
% Configuration Space in ThetaA-ThetaB Coordinates where Theta A is the 
% Rows of the Matrix and encodes the Angle of the First Joint of the Arm.
% Angles of each joint are referenced with 0deg pointing straight forward.
% For Joint A this implies bisecting the freespace, for Joint B this
% implies lying parallel to the axis of linkage AB.
% Optionally: if do_plot is enabled, all relevant coordinate spaces will be
% plotted and the run will be timed.
function config_AB = ConfigurationSpacePlanner(do_plot)
if do_plot; tic; end
% Joint Angles Referenced from Pointing Straight Forward:
min_th_A = -pi * 10/20; % Minimum /Attainable/ Angle for Joint A in Freespace (no obstacles)
max_th_A = pi * 10/20; % Max
min_th_B = -pi; % Minimum /Attainable/ Angle for Joint B in Freespace (no obstacles)
max_th_B = pi; % Max
step_th = ( (360/360) ) * (pi / 180); % Minimum meaningful (or attainable) change in th.
step_th = step_th / 2; % Gear Reduction
%step_th = step_th / 8; % Microstepping

len_AB = 3.75*25.4; % Distance from Joint A to Joint B
len_BF = 2.5*25.4; % Distance from Joint B to End Effector

width_arm = 75; % Width of Arm Segment (Arm Segment is Slot Shaped)

%% Create Workspace (in X-Y Space):
res_xy =  step_th * (len_AB+len_BF) % mm/cell Smallest Meaningful Size (Arc-length of smallest attainable Delta TH * length of extended arm)
box_width = 2*(len_AB + len_BF + 2 * width_arm);
box_width_cells = box_width/res_xy; % Convert to Cells
box_width_cells = floor(box_width_cells/2)*2 + 1; % Round Up to Nearest Odd Number
WS_XY = zeros(box_width_cells);

% Add Obstacles:
XY_mid = ceil(box_width_cells/2);

% Draw a Rectangular Zone of Exclusion (obstacle) in the Workspace with
% upper left corner at (x1,y1) and lower right corner at (x2,y2) where
% (0,0) is the position of Joint A. Units in mm.
function drawRect(x1,y1,x2,y2)
    ya = min(y1,y2); yb = max(y1,y2);
    xa = min(x1,x2); xb = max(x1,x2);
    WS_XY( XY_mid+ceil(ya/res_xy):XY_mid+ceil(yb/res_xy), XY_mid+ceil(xa/res_xy):XY_mid+ceil(xb/res_xy) ) = 1;
end

drawRect(-3*25.4, -8*25.4, 3*25.4, -5.5*25.4);

% Add Boundaries:
drawRect(-box_width/2,-box_width/2, -7*25.4,box_width/2);
drawRect(7*25.4,-box_width/2, box_width/2,box_width/2);
drawRect(-box_width/2,box_width/2, box_width/2,1*25.4);
drawRect(-box_width/2,-box_width/2, box_width/2,-8*25.4);

%% Generate X-Y Config Space (buffer workspace):
buff = 0.5 * 25.4; % 0.5in
buff_cells = ceil(buff/res_xy)+0.5; % Convert Buffer to Cells and Ensure Twice the value is Odd (width of mask)

config_XY = buffer_mat(WS_XY, buff_cells);

%% Generate A-B Config Space and XY-Accessibility Graph:
access_XY = ones(size(config_XY)); % All-Locations Inaccessible (walls) by default

ths_A = min_th_A : step_th : max_th_A;
ths_B = min_th_B : step_th : max_th_B;
Nidx = ceil(2*pi/step_th + 1); % Total Number of Indices for each Joint
config_AB = zeros(Nidx, Nidx); % Initialize with Complete FreeSpace (-pi to pi for each joint)
function idx=th2idx(th); idx=round((th+pi) ./ step_th)+1; end % Convert Angle to Index in Config. Matrix
% Impose Joint Angle Limits:
config_AB( th2idx(-pi:step_th:min_th_A), : ) = 1;
config_AB( :, th2idx(-pi:step_th:min_th_B) ) = 1;
config_AB( :, th2idx(max_th_B:step_th:pi) ) = 1;
config_AB( th2idx(max_th_A:step_th:pi), : ) = 1;

% Iterate over Angle Space and find Corresponding Cell Value in XY Space.
x=0; y=0; i=0; j=0; % Init. (avoid freq. gc and reallocation)
a=1; b=1;
while a <= numel(ths_A)
thA = ths_A(a);
    b = 1;
    while b <= numel(ths_B)
    thB = ths_B(b);
        x = len_AB * cos(thA + pi/2) + len_BF * cos(thA+pi/2 + thB);
        y = len_AB * sin(thA + pi/2) + len_BF * sin(thA+pi/2 + thB);
        
        i = ceil(box_width_cells/2) - round(y/res_xy);
        j = ceil(box_width_cells/2) + round(x/res_xy);
        
        acc = config_XY(i,j) | intersects_obstacle(thA+pi/2,thB); % Whether Location is Accessible
        config_AB(th2idx(thA), th2idx(thB)) = acc;
        access_XY(i,j) = acc;
    b = b+1;
    end % thB
a = a+1;
end % thA

% Determines Whether Joint Config (tha, thb) will Cause the Arm to Intersect
% an Obstacle in the XY Configuration Space:
function hits = intersects_obstacle(tha, thb, step)
    step_size = res_xy;
    if nargin > 2
        step_size = step;
    end
    hits = 0;
    
    mid_cell = ceil(box_width_cells/2); % Middle Cell Location
    
    n = 0;
    Ntot = round(len_AB / step_size);
    xi = 0; yi = 0;
    dx = step_size * cos(tha); dx = dx/res_xy;
    dy = step_size * sin(tha); dy = dy/res_xy;
    % Iterate in Steps of res_xy from Joint A to Joint B
    while(n <= Ntot && ~hits)
        xi = xi + dx;
        yi = yi + dy;
        
        ii = mid_cell - round(yi);
        ji = mid_cell + round(xi);
        
        hits = hits | config_XY(ii,ji);
    n = n + 1;
    end
    % Iterate in Steps of res_xy from Joint B to End Effector
    n = 0;
    Ntot = round(len_BF / step_size);
    dx = step_size * cos(tha+thb); dx = dx/res_xy;
    dy = step_size * sin(tha+thb); dy = dy/res_xy;
    % Iterate in Steps of res_xy from Joint A to Joint B
    while(n <= Ntot && ~hits)
        xi = xi + dx;
        yi = yi + dy;
        
        ii = mid_cell - round(yi);
        ji = mid_cell + round(xi);
        
        hits = hits | config_XY(ii,ji);
    n = n + 1;
    end
end %intersects_obstacle

%% Display:
if(do_plot)
    figure();
    imagesc(WS_XY);
    title({'XY Workspace', '(no arm constraints)'});
    % Generate Tick Marks:
    dist_spacing = 0.5*25.4; % Tick Increments of Half Inches
    box_rad_r = round(box_width/2/dist_spacing)*dist_spacing; % Box "Radius" (Half-Width) Rounded to Nearest Inc. of dist_spacing
    dist_ticks = (-box_rad_r : dist_spacing : box_rad_r);
    dist_tick_cells = (dist_ticks + box_rad_r + dist_spacing/2) ./ res_xy;
    xticks(dist_tick_cells); yticks(dist_tick_cells);
    xticklabels(num2cell(dist_ticks / 25.4)); yticklabels(num2cell(dist_ticks / 25.4)); % Units Inches
    xlabel('X-Position [in]', 'Interpreter', 'latex');
    ylabel('Y-Position [in]', 'Interpreter', 'latex');
    grid on

    figure();
    imagesc(config_XY);
    title({'XY Configuration Space', '(buffered, no arm constraints)'});
    xticks(dist_tick_cells); yticks(dist_tick_cells);
    xticklabels(num2cell(dist_ticks / 25.4)); yticklabels(num2cell(dist_ticks / 25.4)); % Units Inches
    xlabel('X-Position [in]', 'Interpreter', 'latex');
    ylabel('Y-Position [in]', 'Interpreter', 'latex');
    grid on

    figure();
    imagesc(access_XY);
    title({'XY Configuration Space', '(with arm constraints and buffer)'});
    xticks(dist_tick_cells); yticks(dist_tick_cells);
    xticklabels(num2cell(dist_ticks / 25.4)); yticklabels(num2cell(dist_ticks / 25.4)); % Units Inches
    xlabel('X-Position [in]', 'Interpreter', 'latex');
    ylabel('Y-Position [in]', 'Interpreter', 'latex');
    grid on

    figure();
    imagesc(config_AB);
    title('Joint Angle Configuration Space');

    th_ticks = (-360:15:360);
    ts = th2idx(th_ticks*pi/180);
    xticks(ts); yticks(ts);
    xticklabels(num2cell(th_ticks)); yticklabels(num2cell(th_ticks));
    xlabel('$$\theta_{2}=\theta_{B}  [deg]$$', 'Interpreter', 'latex');
    ylabel('$$\theta_{1}=\theta_{A}  [deg]$$', 'Interpreter', 'latex')
    grid on

    toc
end
end % #ConfigurationSpacePlanner