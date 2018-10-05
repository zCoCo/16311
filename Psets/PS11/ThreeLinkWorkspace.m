function ThreeLinkWorkspace()
    tic;
    
    % Linkage Sizes:
    LAB = 100;
    LBC = 1.5*LAB;
    LCF = 2.0*LAB;
    
    deg = pi/180;
    step_th = 0.5*deg; % Angular Step Size
    
    % Permissible Joint Angle Ranges:
    thsA = (-pi : step_th : pi);
    thsB = (-pi : step_th : pi);
    thsC = (-pi : step_th : pi);
    
    % Allocate Workspace:
    res_xy = step_th * (LAB + LBC + LCF); % Min. XY Step Size
    width_cells = ceil(2.1 * (LAB + LBC + LCF) / res_xy)
    
    mid_XY = width_cells / 2;
    
    WS = zeros(width_cells); % Prepopulate Workspace with Maximally Sized Freespace (Assuming no solutions for all-space)
    
    % Generate Workspace (value indicates number of solutions):
    for tha = thsA
        for thb = thsB
            for thc = thsC
                xx = LAB * cos(tha) + LBC * cos(tha+thb) + LCF * cos(tha+thb+thc);
                yy = LAB * sin(tha) + LBC * sin(tha+thb) + LCF * sin(tha+thb+thc);
                
                ii = round( mid_XY + yy / res_xy );
                jj = round( mid_XY + xx / res_xy );
                
                WS(ii,jj) = WS(ii,jj) + 1; % Add Solution to Region
            end % loop: thsC
        end % loop: thsB
    end % loop: thsA
    
    % Display Workspace:
    figure();
    imagesc(WS);
    
    % Generate Tick Marks:
    dist_spacing = LAB;
    box_rad_r = ceil(width_cells*res_xy/2/dist_spacing)*dist_spacing; % Box "Radius" (Half-Width) Rounded to Nearest Largest Inc. of dist_spacing
    ticks = (-box_rad_r : dist_spacing : box_rad_r);
    tick_cells = ticks ./ res_xy +  width_cells / 2;
    xticks(tick_cells); yticks(tick_cells);
    xticklabels(num2cell(ticks/LAB)); yticklabels(num2cell(ticks/LAB)); % Multiples of Arm Lengths
    grid on
    xlabel('X-Position [Multiples of $$L_{1}$$]', 'Interpreter', 'latex');
    ylabel('Y-Position [Multiples of $$L_{1}$$]', 'Interpreter', 'latex');
    title({'Workspace of Three-Link Revolute Arm', 'Special Case: $$L_{1}+L_{2}>L_{3}$$', 'with $$L_{2}=1.5*L_{1}, L_{3}=2*L_{1}$$'}, 'Interpreter', 'latex');

    toc
end