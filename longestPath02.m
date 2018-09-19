
x = -100;
o = 1;


%{
E = [1 2; 2 3; 3 4; 4 5; 5 6; 6 7; 7 8; 8 9; 9 10;...
        11 12; 12 13; 13 14; 14 15; 15 16; 16 17; 17 18; 18 19; 19 20
        2 11; 13 3; 4 14; 16 6; 17 7; 18 8; 19 9];       
    
W = [1 1 1 1 1 1 1 -5 1 1 ...
        1 1 1 1 1 1 1 1 1 1 ...
        1 1 1 1 1 1 1];
    
%}

tic
row = 2;
col = 16;
n_node = row*col;
obs = [2 3; 2 4; 2 6; 2 7; 1 9; 1 10];
[E, W]= CreateGraph(row, col, obs);
E = [E;
       1 17; 18 2; 5 21; 22 6; 9 25; 28 12; 29 13; 30 14];
W = [W 1 1 1 1 1 1 1 1];

    
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

D_max
%Path{16}
toc