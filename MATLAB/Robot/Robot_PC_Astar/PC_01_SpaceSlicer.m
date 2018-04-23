max_step = 20;
pause_time = 1;
time_interval = 0.1;
max_slice_approach = 20;


gs = [8 8];


obstacle_map = [3 4;
                        3 5;
                        4 4;
                        5 4; ];
Grid_visited = zeros(gs(1),gs(2),max_step);
Grid_obs = zeros(gs(1),gs(2));

Grid_inslice = zeros(gs(1),gs(2));

for intidx = 1:size(obstacle_map,1)
    Grid_obs(obstacle_map(intidx,1),obstacle_map(intidx,2)) = 1;
    Grid_inslice(obstacle_map(intidx,1),obstacle_map(intidx,2)) = -1;
end

is_all_covered = false;

Slice_final = [];
slice_approach = 0;
Grid_inslice;
while ismember(0, Grid_inslice) || slice_approach > max_slice_approach
    slice_approach = slice_approach +1;
    Slice_candidates=[];
    for rowidx = 1:gs(1)
        for colidx = 1:gs(2)
            % Find Avaliable grid for biggest slice
            is_possible_slice_start = true;
            if (Grid_inslice(rowidx, colidx)~= 0)
                is_possible_slice_start = false;
            end
            if (rowidx - 1 > 0)
                if Grid_inslice(rowidx-1, colidx) == 0
                    is_possible_slice_start = false;
                end
            end
            if (colidx - 1 > 0)
                if Grid_inslice(rowidx, colidx-1) == 0
                    is_possible_slice_start = false;
                end
            end            
            if (is_possible_slice_start) 
                max_slice_end = [rowidx colidx];
                max_slice_size = 0;
                for nrowidx = rowidx:gs(1)
                    for ncolidx = colidx:gs(2)
                        slice_size = 0;
                        if any(Grid_inslice(rowidx:nrowidx, colidx:ncolidx)) == 0
                            slice_size = (nrowidx-rowidx+1)*(ncolidx-colidx+1);
                            if slice_size > max_slice_size
                                max_slice_size = slice_size;
                                max_slice_end = [nrowidx, ncolidx];
                            end
                        end
                    end
                end
                Slice_candidates = [Slice_candidates;
                                       rowidx, colidx, max_slice_end(1), max_slice_end(2), max_slice_size];
            end
        end
    end
    max_Sliceidx = 0;
    max_Slice_size = 0;
    for Sliceidx = 1:size(Slice_candidates,1)
       if Slice_candidates(Sliceidx, 5) > max_Slice_size
           max_Slice_size = Slice_candidates(Sliceidx, 5);
           max_Sliceidx = Sliceidx;
       end
    end
    for rowidx = Slice_candidates(max_Sliceidx,1): Slice_candidates(max_Sliceidx,3)
        for colidx = Slice_candidates(max_Sliceidx,2): Slice_candidates(max_Sliceidx,4)
            Grid_inslice(rowidx,colidx) = 1;
        end
    end
    Slice_final = [Slice_final; Slice_candidates(max_Sliceidx,:)];
end
Slice_final

Wpd_approach = [];
max_wpd_approach = 10;

[Slice_Info, Slice_Wp, Slice_Info_size] = get_slice_info(Slice_final);

min_distance = 10^10;
min_slice_idx = 0;
min_slice_sequence = [];
min_des_sequence = [];

for intidx = 1:prod(Slice_Info_size)
    des_sequence = [];
    slice_sequence = [];
    slice_info = [];
    perms_slice = perms(2:size(Slice_final,1));
    for sliceidx = 1:size(perms_slice,1)
        des_quotients = intidx;
        tot_distance = 0;
        des_remainings = 0;
        slice_sequence = [1 perms_slice(sliceidx,:)];
        last_slice_info = [1 1 1 1 0 0];
        for desidx = 1:size(Slice_final,1)
            des_remainings = mod(des_quotients, Slice_Info_size(slice_sequence(desidx)))+1;
            slice_info = Slice_Info{slice_sequence(desidx), des_remainings};
            tot_distance = tot_distance + slice_info(6);
            tot_distance = tot_distance + abs(slice_info(1)-last_slice_info(3)) + abs(slice_info(2)-last_slice_info(4));
            des_quotients = floor(des_quotients/Slice_Info_size(slice_sequence(desidx)));
            des_sequence = [des_sequence, des_remainings];
            last_slice_info = slice_info;
        end
        if tot_distance < min_distance
            min_distance = tot_distance;
            min_slice_idx = desidx;
            min_des_sequence = des_sequence;
            min_slice_sequence = slice_sequence;
        end
    end
end
min_slice_sequence
