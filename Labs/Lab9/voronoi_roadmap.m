% Generates a Roadmap through the Given Binary Configuration Space Matrix
% using the Voronoi Tesselation which optimizes distance between the 
% Roadmap and Obstacles at all Points.
function voronoi_roadmap(cfg)
    config = cfg;
    % Add Thin Boundary Around Configuration Space (prevent overflow):
    [wrows,wcols] = size(config);
    config(:,1) = 1;
    config(1,:) = 1;
    config(:,wcols) = 1;
    config(wrows,:) = 1;
    
    B = bwboundaries(config); % Get Points along Boundary of Accessible and Inaccessible Space
    Bmat = cell2mat(B);
    
    [vs,cs] = voronoin(Bmat);
    
    % Collect Voronoi Edges which Don't Intersect an Obstacle -> Roadmap
    % Edges and Create a Connectivity Matrix for Use in A* Later.
    MC = zeros(numel(vs)); % Initialize Connectivity Matrix (starts at max-size, will be trimmed to len(vrm)xlen(vrm));
    vrm = []; % Initialize Vector of Roadmap Vertices    
    for i = (1:numel(cs)) % Loop Through each Voronoi Cell
        c = cs{i};
        xs = []; ys = [];
        for j = (1:numel(c)) % Check Each Pair of Vertices
            n = c(j);
            m = n+1; if(n==numel(c)); m = 1; end % <-Loop Around
            xa = vs(n,2); ya = vs(n,2);
            xb = vs(m,2); yb = vs(m,2);
            if(cfg(ya,xa) == 0 && cfg(yb,xb) == 0) % If neither vertex is in an obstacle, add to roadmap
                % Check to see if vertex already is in roadmap and, thus,
                % has index in connectivity matrix:
                ia = find(vrm(:,1)==xa & vrm(:,2)==ya);
                if(isempty(idx)) % Vertex not yet in roadmap. Add it and assign index.
                    vrm = [vrm; xa,ya];
                    [ia, ~] = size(vrm);
                end
                ib = find(vrm(:,1)==xb & vrm(:,2)==yb);
                if(isempty(idx)) % Vertex not yet in roadmap. Add it and assign index.
                    vrm = [vrm; xb,yb];
                    [ib, ~] = size(vrm);
                end
                MC(ia,ib) = 1; MC(ib,ia) = 1; % Add entry to Connectity Matrix. (can speed up this triangular matrix with sparse matrix?)
            end
        end
    end
    
    % Check to See if Vertex Already in Roadmap:
    
    figure();
    imagesc(cfg);
    hold on
    for i = (1:numel(cs))
        c = cs{i};
        xs = []; ys = [];
        for j = (1:numel(c))
            n = c(j);
            xs = [xs, vs(n,2)];
            ys = [ys, vs(n,1)];
        end
        plot(xs,ys);
    end
    plot(Bmat(:,2), Bmat(:,1));
    
end % #voronoi_roadmap