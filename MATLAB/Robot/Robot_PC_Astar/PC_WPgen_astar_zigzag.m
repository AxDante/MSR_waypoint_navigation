function Wp = PC_WPgen_astar_zigzag(gs, gw, Gobs, shape, rcg, is_print_wp_gen_info)

    Wp_zz = [];
    
    for idx = 1: gs(2)
        if (mod(idx, 4) == 1)
            Wp_zz = [Wp_zz; 0.5  (idx+0.5) 2];
        end
        if (mod(idx, 4) == 2)
            Wp_zz = [Wp_zz; (gs(1) - 1.5)  (idx-0.5)  2];
        end
        if (mod(idx, 4) == 3)
            Wp_zz = [Wp_zz; (gs(1) - 1.5) (idx+0.5) 2];
        end
        if (mod(idx, 4) == 0)
            Wp_zz = [Wp_zz; 0.5  (idx-0.5) 2];
        end
    end

    scg = rcg;
    Wp = [];
    for idx = 1: size(Wp_zz,1)
        gcg = [ceil(Wp_zz(idx,1)) ceil(Wp_zz(idx,2))];
        Wp_s = Robot_PC_Astar_algorithm(gs, Gobs, shape, scg, gcg); %segemented Wp
        Wp_s(:, 3) = shape;
        Wp = [Wp; Wp_s];
        scg = gcg;
    end
        
    Wp(:, 1:2) = (Wp(:, 1:2) - 0.5)*gw;
end