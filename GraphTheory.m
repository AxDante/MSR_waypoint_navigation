
function [Wp, Wp_hack] = GraphTheory(gs, gw, Gobs, rcg)

Gvis = zeros(gs(1),gs(2));

tot_row = gs(1);
tot_col = gs(2);

tot_stripe = ceil(gs(1)/2);

if isempty(Gobs)
    Gobs = [2 3; 2 4; 2 6; 2 7; 1 9; 1 10];
    [E, W]= CreateGraph(row_size, col_size, Gobs);
    E = [E;1 17; 18 2; 5 21; 22 6; 9 25; 28 12; 29 13; 30 14];
    W = [W 1 1 1 1 1 1 1 1];
end


for intidx = 1:size(Gobs,1)
    stripeidx = ceil(Gobs(intidx,1)/2);
    Gobs_feed{stripeidx} =  [Gobs_feed{intidx}; Gobs(intidx,:)];
end


starting_row = 1;
sweepingDirection = 1; % 1: right; 2:left

tic
col_size = tot_col;
row_size = 2;
Wp = [];

    for stripeidx = 1:tot_stripe
        
        
        
        
        [E, W]= CreateGraph(row_size, col_size, Gobs_feed{stripeidx}, shape);
        E = [E; 1 17; 18 2; 5 21; 22 6; 9 25; 28 12; 29 13; 30 14];
        W = [W 1 1 1 1 1 1 1 1];
    
    
        n_node = row_size*col_size;
        

        E_matrix = zeros(n_node,n_node);


        for nodeidx = 1:size(E,1)
            E_matrix(E(nodeidx,1),E(nodeidx,2)) = 1;
        end
        E_dig = digraph(E_matrix);

        node_topo = toposort(E_dig);

        D_max = zeros(1, n_node);
        Path = {};
        Path{1} = [1];
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
        
        
        Path{16}
        Wp_stripe = convertNodeToWp(Path{16}, col_size, starting_row);
        Wp = [Wp; Wp_stripe];
        disp(['Stripe No: ', num2str(stripeidx), ' Done!']);
        toc
        disp(['================']);
        starting_row = 
        
    end
    
    Wp
    Wp(:, 1:2) = (Wp(:, 1:2) - 0.5)*gw;
    Wp_hack = (Wp_hack-0.5)*gw;
    
end





