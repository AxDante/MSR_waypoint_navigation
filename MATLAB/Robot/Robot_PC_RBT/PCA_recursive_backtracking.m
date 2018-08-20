% -----------------------------------------------------------
% Author: AxDante <knight16729438@gmail.com>
% Singapore University of Technology and Design
% Created: April 2018
% Modified: August 2018
% -----------------------------------------------------------

%  --- Function Inputs ---
% gs: grid_size (1x2 array)
% ccg: current center grid (1x2 array)
% gcg: goal center grid  (1x2 array)
% GA: grid availability cell array
% GSC: grid shape change cell array
% cost: current path cost (float)
% cost_best: best path cost (float)
% Wp: current waypoint series (n x 3 array)
% Wp_best: waypoint series with minimum cost (n x 3 array)
% grf: goal robot form (int)
% rows: navigation row boundary (1x2 array)
% cols: navigation column boundary (1x2 array)


% --- Function Outputs ---
% Gvis: visited grid array;
% Gvis: visited grid array;
% cost: current path cost (float)
% cost_best: best path cost (float)
% Wp: current waypoint series (n x 3 array)
% Wp_best: waypoint series with minimum cost (n x 3 array)
% -------------------------




function [Wp_best, Wp, cost_best, cost, Gvis_best, Gvis, grid_missed_best, Grid_miss] = PCA_recursive_backtracking(gs, ccg, gcg, GA, GSC, cost, cost_best, Wp, Wp_best, grf , closed_ncg, boundary, Gvis, Gvis_best, rows_init, row_sweep_dir, up_allow, down_allow, repeat_allow, grid_missed_best, Grid_miss)

    Grid_miss = [];
    
    if cost > cost_best || cost > 100
        return
    end
    
    
    cost_shapeshift = 2;

    rgp = cell(8);
    rgp{1} = [0 -1; 0 0; 0 1; 0 2];
    rgp{2} = [0 -1; 0 0; 1 0; 1 -1];
    rgp{8} = [-1 0; 0 0; 1 0; 2 0];
              
    rmc = ['F', 'R', 'B', 'L', '2', '8', '1']; % Robot movement commands
    ncg = [];

    shapes = [1, 2, 8];
    
    for idxrmc = 1:size(rmc,2)
        
        invalid_move = false;
        up_move = 0;
        down_move = 0;
        shapeshift = 0;
        switch(rmc(idxrmc))
            case 'F'
                if (row_sweep_dir == 1)
                    ncg = ccg + [1 0];
                else
                    ncg = ccg;
                    invalid_move = true;
                end
            case 'R'
                ncg = ccg + [0 1];
                if (row_sweep_dir ~= 0)
                    up_move = 1;
                end
            case 'B'
                 if (row_sweep_dir == -1)
                    ncg = ccg + [-1 0];
                 else
                    ncg = ccg;
                    invalid_move = true;
                 end
            case 'L'
                ncg = ccg + [0 -1];
                if (row_sweep_dir ~= 0)
                    down_move = 1;
                end
            case '8'
                ncg = ccg;
                shapeshift = 8;
            case '2'
                ncg = ccg;
                shapeshift = 2;
            case '1'
                ncg = ccg;
                shapeshift = 1;
        end
       
        if (shapeshift ~= 0)
            
            is_non_repeat = false;
            is_transform_clear = false;

            % Proceed only if the previous two robot waypoints are not in
            % the same shape of the next shape.
            for intidx = 1:size(shapes,2)  
                if  (size(Wp,1) == 1)
                    if  shapeshift == shapes(intidx) && (Wp(end,3) ~= shapeshift)
                        is_non_repeat = true;
                    end
                else
                    if  shapeshift == shapes(intidx) && (Wp(end,3) ~= shapeshift) && ~(Wp(end-1,3) == shapeshift) && ~isequal(Wp(end,1:2),Wp(end-1,1:2))
                        is_non_repeat = true;
                    end
                end
            end
            
            % Check grid shape change
            gsc = GSC{ncg(1), ncg(2)};
            if gsc(Wp(end,3), shapeshift) == 1
                is_transform_clear = true;
            end
                
            if (is_non_repeat && is_transform_clear)    
                
                Wp_temp = Wp;
                ccg_temp = ccg;
                cost_temp = cost;
                Gvis_temp = Gvis;     
                updir_temp = up_allow;
                downdir_temp = down_allow;
                repeat_temp = repeat_allow;
                
                cost = cost + cost_shapeshift;
                Wp = [Wp; ncg(1) ncg(2) shapeshift];
                
                [Wp_best, Wp, cost_best, cost, Gvis_best, Gvis, grid_missed_best, Grid_miss] = PCA_recursive_backtracking(gs, ccg, gcg, GA, GSC, cost, cost_best, Wp, Wp_best, grf, closed_ncg, boundary, Gvis, Gvis_best, rows_init, row_sweep_dir, up_allow, down_allow, repeat_allow, grid_missed_best);
                    
                up_allow = updir_temp;
                down_allow = downdir_temp;
                repeat_allow = repeat_temp;
                Gvis = Gvis_temp;
                ccg = ccg_temp;
                Wp = Wp_temp;
                cost = cost_temp;
                    
            end
        else
            
            repeat_temp = repeat_allow;
            if (up_move == 1)
                updir_temp = up_allow;
                if (up_allow > 0)
                    up_allow = up_allow - 1;
                else
                    invalid_move = true;
                end
            end
            if (down_move == 1)
                downdir_temp = down_allow;
                if (down_allow > 0)
                    down_allow = down_allow - 1;
                else
                    invalid_move = true;
                end
            end
            
            %{
            if (rows(1)~=0 || rows(2) ~= 0)
                bound_row = rows;
                bound_col = [1 gs(1)];
            elseif (cols(1)~=0 || cols(2) ~= 0)
                bound_col = cols;
                bound_row = [1 gs(2)];
            end
            
            bound_col
            bound_row
            %}
            
            Rgp =  ncg + rgp{Wp(end,3)};
            for rgpidx = 1:size(Rgp,1)
                if ~(Rgp(rgpidx,2) >= boundary(1) && Rgp(rgpidx,2) <= boundary(2) && ...
                        Rgp(rgpidx,1) >= boundary(3) && Rgp(rgpidx,1) <= boundary(4) && ...
                    Rgp(rgpidx,1) > 0 && Rgp(rgpidx,2) > 0)
                    invalid_move = true;
                end 
            end
            
            % Check if the next center grid 

            
            if (~invalid_move)
               %disp(['=================================='])
               %disp(['going from (',num2str(ccg(1)), ', ', num2str(ccg(2)), ') to (', num2str(ncg(1)), ', ', num2str(ncg(2)) ,').']);
               %Wp
                
                for wpidx = 1:size(Wp,1)
                    if ncg == Wp(wpidx, 1:2)
                        repeat_allow = repeat_allow -1;
                    end
                end
                
                if repeat_allow > 0 && (~(ncg(1) == closed_ncg(1) && ncg(2) == closed_ncg(2)))
                    
                    %[ncg(1), ncg(2)]
                    %Rgp
                    ga = GA{ncg(1), ncg(2)};

                    if ga(Wp(end,3)) == 1 

                        Wp_temp = Wp;
                        ccg_temp = ccg;
                        cost_temp = cost;
                        Gvis_temp = Gvis;
                        
                        updir_temp = up_allow;
                        downdir_temp = down_allow;
                        
                        closed_ncg = ncg;

                        Wp = [Wp; ncg(1) ncg(2) Wp(end,3)];
                        ccg = ncg;
                        cost = cost+1;
                        
                        for rgpidx = 1:size(Rgp,1)
                            if Gvis(Rgp(rgpidx,1), Rgp(rgpidx,2)) == 1
                                cost = cost + 0.2;
                            end
                            Gvis(Rgp(rgpidx,1), Rgp(rgpidx,2)) = 1;
                        end

                        % Check if next robot configuration equals to the
                        % goal configuration
                        if (isequal(ncg,gcg) && Wp(end,3) == grf) 
                            
                            Grid_miss = [];
                            for rowidx =1:rows_init(2)
                                for colidx = 1:gs(2)
                                    if Gvis(rowidx, colidx) == 1
                                        cost = cost ;
                                    elseif Gvis(rowidx, colidx) == 0
                                        cost = cost + 5;
                                        Grid_miss = [Grid_miss; rowidx, colidx];
                                    end
                                end
                            end
                            
                            if (cost < cost_best)
                                Wp_best = Wp;
                                cost_best = cost;
                                Gvis_best = Gvis;
                                grid_missed_best = Grid_miss;
                                %return
                            else
                                %return
                                %[Wp_best, Wp, cost_best, cost, Gvis_best, Gvis] = recurse_cost(gs, ccg, gcg, GA, GSC, cost, cost_best, Wp, Wp_best, end_shape, closed_ncg, boundary, Gvis, Gvis_best, rows_init,  row_sweep_dir , updir, downdir);
                            end
                        else
                            [Wp_best, Wp, cost_best, cost, Gvis_best, Gvis, grid_missed_best, Grid_miss] = PCA_recursive_backtracking(gs, ccg, gcg, GA, GSC, cost, cost_best, Wp, Wp_best, grf, closed_ncg, boundary, Gvis, Gvis_best, rows_init,  row_sweep_dir , up_allow, down_allow, repeat_allow, grid_missed_best);
                        end
                        
                        Gvis = Gvis_temp;
                        ccg = ccg_temp;
                        Wp = Wp_temp;
                        cost = cost_temp;
                        
                    end
                end
            end
            
            if (up_move == 1)
                up_allow = updir_temp;
            end
            if (down_move == 1)
                down_allow = downdir_temp;
            end
            repeat_allow = repeat_temp;
        end
    end
end