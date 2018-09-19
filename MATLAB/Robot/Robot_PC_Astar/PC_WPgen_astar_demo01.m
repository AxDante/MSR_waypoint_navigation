function [Wp, Wp_hack] = PC_WPgen_astar_demo01(gs, gw, Gobs, rcg)

    Wp_hack = [2 2 2;
                   12 2 2;
                   12 4 2;
                   2 4 2;
                   2 6 2;
                   12 6 2];
    
    sweep_width = [4 2 3 3 2 2 2 ...
                           1];           
               
   
    Gvis = zeros(gs(1),gs(2));
    
    scg = rcg;
    Wp = [];
    for idx = 1: size(Wp_hack,1)
        gcg = [ceil(Wp_hack(idx,1)) ceil(Wp_hack(idx,2))];
        Wp_s = PC_Astar_origin_algorithm(gs, Gobs, Wp_hack(idx,3), scg, gcg); %segemented Wp
        Wp_s(:, 3) = Wp_hack(idx,3);
        Wp = [Wp; Wp_s];
        scg = gcg;
    end
        
    Wp(:, 1:2) = (Wp(:, 1:2) - 0.5)*gw;
    Wp_hack = (Wp_hack-0.5)*gw;
end