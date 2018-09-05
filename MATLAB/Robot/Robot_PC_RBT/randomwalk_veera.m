function [Wp_out, total_dis] = randomwalk_veera(Wp, rng_seed)

    rand('seed', rng_seed);
    
    count_wp = size(Wp,1);
    rng_array = randperm(count_wp-1)+1;
    
    Wp_out(1,:) = Wp(1,:);
    total_dis = 0;
    
    for intidx = 1:count_wp-1
        Wp_out(intidx+1,:) = Wp(rng_array(intidx), :);
        dis = abs(Wp_out(end, 1) - Wp_out(end-1,1)) + abs(Wp_out(end, 2) - Wp_out(end-1,2));
        total_dis = total_dis + dis;
    end

end