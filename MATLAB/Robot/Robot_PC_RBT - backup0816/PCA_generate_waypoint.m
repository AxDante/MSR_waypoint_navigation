% -----------------------------------------------------------
% Author: AxDante <knight16729438@gmail.com>
% Singapore University of Technology and Design
% Created: April 2018
% Modified: August 2018
% -----------------------------------------------------------

%  --- Function Inputs ---
% gs: grid_size (1x2 array)
% gw: grid_width (int)
% Gobs: grid obstacle map (gs(1)xgs(2) array)
% rcg: robot center grid (1x2 array)
% Wpp: starting Waypoint profile (nx3 array)
% Rsp: Robot sweeping profile (nx2 array)
% rf: robot form
%
% --- Function Outputs ---
% Wp: generated waypoints (nx3 array)
% Wpp: final Waypoint profile (nx3 array)
%
% --- Function Variables ---
% Rsd: robot sweeping direction
% GA: grid availability cell array
% GSC: grid shape change cell array
% scg: starting center grid

% ---------------------------


function [Wp, Wpp] = PCA_generate_waypoint(gs, gw, Gobs, rcg, Wpp, Rsp, Rss, rf)

    Wpp
    

    Rsd = [1; 0; -1; 0; 1; 0; -1; 0; 1];
                     
    Gvis = zeros(gs(1),gs(2));
    
    for obsidx = 1:size(Gobs,1)
        Gvis(Gobs(obsidx,1), Gobs(obsidx,2)) = -1;
    end
  
    for rowidx = 1:gs(1)
        for colidx = 1:gs(2)
             GA{rowidx,colidx} = PCA_get_grid_availability(rowidx,colidx, gs, Gobs);
             GSC{rowidx,colidx} = PCA_get_grid_shape_change(rowidx,colidx, gs, Gobs);
        end
    end
    
    scg = rcg;
    Wp = [];
    
    stripe_num = size(Rss,2); % Number of stripes in the graph.
    
    
    
    % Stripe navigation
    for stridx = 1:stripe_num
        
        % Set goal center grid to the one specified in waypoint properties.
        gcg = [Wpp(stridx*2,1) Wpp(stridx*2,2)];
        
        % Indicating the borders of the stripe
        bound_row = [Rss(stridx,1) Rss(stridx,2)];
        bound_col = [1 gs(2)];
        
        % Begin stripe path planning
        [Wp_s, Gvis_best] = PCA_stripe_path_planning(gs, rf, 2 , Gvis, scg, gcg, GA, GSC, bound_row, bound_col, Rsd(stridx)); % segemented Wp
        
        % Append waypoint
        Wp = [Wp; Wp_s];
        
        Gvis = Gvis_best;
        
        % Perform stripe waypoint navigation if the current stripe is not
        % the last stripe.
        if stridx ~= stripe_num
            % Ideal: A star algorithm
            % Current: Hacked, direct movement
            scg = [Wpp(stridx*2+1,1) Wpp(stridx*2+1,2)];
            Wp = [Wp; [scg 2]];
        end 
    end
    
    
    % Converstion between grid based coordinate to workspace coordinates
    Wp(:, 1:2) = (Wp(:, 1:2) - 0.5)*gw;
    Wpp = (Wpp-0.5)*gw;
    
end