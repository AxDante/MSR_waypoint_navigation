function [Wp_out, total_dis] = alg_greedy_path(Wp)

    count_wp = size(Wp,1);
    visited = zeros(count_wp,1);
    visited(1) = 1;
    total_dis = 0;
    
    ccg = Wp(1, 1:2);
    
    Wp_out = [Wp(1,:)];
    
    while any(visited(:) == 0)
        shortest_idx = 0;
        shortest_dis = 1000;
        for intidx = 1:count_wp
            if visited(intidx) == 0
                dis = abs(Wp(intidx,1) - ccg(1)) + abs(Wp(intidx,2) - ccg(2));
                if dis < shortest_dis
                    shortest_idx = intidx;
                    shortest_dis = dis;
                end
            end
        end
        total_dis = total_dis + shortest_dis;
        visited(shortest_idx) = 1;
        ccg = Wp(shortest_idx, 1:2);
        Wp_out = [Wp_out; Wp(shortest_idx,:)];
    end
 
end