function Lab9()
    config = ConfigurationSpacePlanner(1);
    voronoi_roadmap(config);
    % Generalized Voronoi Coords:
    gvc = [ ...
        12.5, 191.5; ...
        52.99, 287.5; ...
        106.5, 352.5; ...
        232.9, 283.7; ...
        353.5, 233.5; ...
        499.5, 220.4; ...
        622.5, 278.5; ...
        693.5, 207.5; ...
        614.6, 368.6; ...
        666.4, 430.5; ...
        700.3, 500.6; ...
        489.1, 438.2; ...
        368.1, 485.9; ...
        227.5, 502.2; ...
        99.97, 441.5; ...
        21.5, 521.5 ...
        ];
    
    ths = ( (gvc - 1) .* ((360./360) .* (pi./180) ./ 2) ) - pi;
    %Switch Columns (so thA is first):
    tmp = ths(:,1); ths(:,1) = ths(:,2); ths(:,2) = tmp;

end % #Lab9