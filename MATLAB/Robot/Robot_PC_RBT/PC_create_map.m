
filename = 'gbp_10_01';

create_obs_map = 'obs_10_01';

create_gs = [10,10];        % Map grid size
create_rcg = [2, 2];        % Robot starting center grid

create_Wp = [9 2 2;
                   9 3 2;
                   2 3 2;
                   2 6 2;
                   9 6 2;
                   9 8 2;
                   2 7 2;
                   2 9 2;
                   9 10 2];

create_Row_sweep = [1 2;
                         0 0;
                         3 4;
                         0 0;
                         5 6;
                         0 0;
                         7 8;
                         0 0;
                         9 10];
                     
save(['navmap/', filename], 'create_obs_map', 'create_gs', 'create_rcg', ...
    'create_Wp', 'create_Row_sweep')