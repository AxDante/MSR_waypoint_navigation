function [Wp, Wp_hack] = GraphTheory(gs, gw, Gobs, rcg)

    Gvis = zeros(gs(1),gs(2));

    tot_row = gs(1);
    tot_col = gs(2);

    tot_stripe = ceil(gs(1)/2);

    col_size = tot_col;
    row_size = 2;

    shape = 2;
    
    allowedShape = [2 8];
    
    tot_allowedShape = size(allowedShape,2);

    if isempty(Gobs)
        Gobs = [2 5; 2 6; 1 11; 1 13];
        [E, W]= CreateGraph(row_size, col_size, Gobs, shape, allowedShape);
        E = [E;1 17 2; 18 2 2; 5 21 2; 22 6 2; 9 25 2; 28 12 2; 29 13 2; 30 14 2];
        W = [W 1 1 1 1 1 1 1 1]; 
    end

    for intidx = 1:size(Gobs,1)
        stripeidx = ceil(Gobs(intidx,1)/2);
        Gobs_feed{stripeidx} = [];
        
    end
    
    for intidx = 1:size(Gobs,1)
        stripeidx = ceil(Gobs(intidx,1)/2);
        Gobs_feed{stripeidx} =  [Gobs_feed{stripeidx}; Gobs(intidx,:)];
    end

    
    starting_row = 1;
    sweepingDirection = 1; % 1: right; 2:left

    tic
    Wp = [];

    for stripeidx = 1:tot_stripe
       
    
        n_node = row_size*col_size*tot_allowedShape;
        

        E_matrix = zeros(n_node,n_node);


        for nodeidx = 1:size(E,1)
            E_matrix(E(nodeidx,1),E(nodeidx,2)) = 1;
        end
        
        
        E_dig = digraph(E_matrix);

        node_topo = toposort(E_dig);
        
        
        D_max = zeros(1, n_node);
        Path = {};
        Path{1} = [17];
        for node = node_topo
            d_max = 0;
            for Eidx = 1:size(E,1)
                if E(Eidx,2) == node
                    if D_max(E(Eidx,1)) + W(Eidx) > d_max
                        d_max = D_max(E(Eidx,1)) + W(Eidx);
                        Path{node} = [Path{E(Eidx,1)} node];
                    end
                end
            end
           D_max(node) = d_max;
        end

        %D_max
        Path{32};
        
        Wp_stripe = convertNodeToWp(Path{32}, col_size, starting_row, allowedShape);
        Wp = [Wp; Wp_stripe];
        disp(['Stripe No: ', num2str(stripeidx), ' navigation complete!']);
        toc
        disp(['================']);
        starting_row = starting_row + 2;
        
    end
    
    Wp;
    Wp(:, 1:2) = (Wp(:, 1:2) - 0.5)*gw;
    %Wp_hack = (Wp_hack-0.5)*gw;
    
end





