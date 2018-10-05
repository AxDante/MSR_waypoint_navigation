function [Wp, Wp_hack] = PC_WPgen_PCA(gs, gw, Gobs, rcg)
 
    Gobs = zeros(15);
    Gobs(5,2) = 1;
    Gobs(5,4) = 1;
    Gobs(2,7) = 1;


    Wp_hack = [2 2 2;
                   8 8 3;
                   2 2 1;
                   8 8 8];
    
    gs = [15 15];
               
    tot_step = 2000;
    scg = rcg;
    Wp = [];
    for idx = 1: size(Wp_hack,1)
        gcg = [ceil(Wp_hack(idx,1)) ceil(Wp_hack(idx,2))];
        Wp_s = PC_Astar_rev_algorithm(gs, Gobs, Wp_hack(idx,3), scg, gcg); %segemented Wp
        Wp_s(:, 3) = Wp_hack(idx,3);
        Wp = [Wp; Wp_s];
        scg = gcg;
    end
    
    %Wp(:, 1:2) = (Wp(:, 1:2) - 0.5)*gw;
    %Wp_hack = (Wp_hack-0.5)*gw;
end