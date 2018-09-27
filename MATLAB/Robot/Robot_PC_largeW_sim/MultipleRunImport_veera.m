function [Wp_series_out, Wp_hack_out]= MultipleRunImport_Veera(gs, gw, WaypointMapMode)
   
   Wp_series_init = {};
   Wp_hack_init = {};
   
   if strcmp(WaypointMapMode, 'empty')
        runs = 5;

       Wp_series_init{1}=  [1 1 6;  
                4 1 27;     %27
                5 1 5+7;
                3 3 7+7;
                3 4 5;
                1 4 6;
                1 7 6;
                3 7 5;
                4 5 5+7;
                5 8 5;
                3 9 7+7;
                1 10 13;
                6 10 13;
                6 7 6;
                5 7 21;
                5 3 14;
                6 2 14;
                9 1 27;
                9 3 21;
                8 6 20;
                9 5 21;
                9 7 21;
                8 7 5;
                8 9 14;
                10 9 20];                   
      Wp_hack_init{1} = Wp_series_init{1};

       % Zigzag
       Wp_series_init{2}=  [1 1 2;  
                1 9 2;     
                2 9 2;
                2 1 2;
                3 1 2;
                3 9 2;
                4 9 2;
                4 1 2;
                5 1 2;
                5 9 2;
                6 9 2;
                6 1 2;
                7 1 2;
                7 9 2;
                8 9 2;
                8 1 2;
                9 1 2;
                9 9 2];                        
       Wp_hack_init{2} = Wp_series_init{2};

       % Spiral
       Wp_series_init{3}=  [5 5 2;  
                5 6 2;     
                6 6 2;
                6 4 2;
                4 4 2;
                4 7 2;
                7 7 2;
                7 3 2;
                3 3 2;
                3 8 2;
                8 8 2;
                8 2 2;
                2 2 2;
                2 9 2;
                9 9 2;
                9 1 2;
                1 1 2;
                1 9 2];
       Wp_hack_init{3} =  Wp_series_init{3};
     
       [Wp_greedy, total_dis_greedy] = alg_greedy_path(Wp_series_init{1});     
       [Wp_rw, total_dis_rw] = alg_random_path(Wp_series_init{1}, 1);     
       Wp_series_init{4} = Wp_greedy;
       Wp_hack_init{4} =  Wp_series_init{4};
       Wp_series_init{5} = Wp_rw;
       Wp_hack_init{5} =  Wp_series_init{5};
        
    
   elseif strcmp(WaypointMapMode, 'obs1')
  
       %OBS MAP 01
         Wp_orig = [1 1 4+7;
           2 3 3+14;
           1 4 4+7;
           3 5 4+21;
           2 6 2;
           1 8 18;
           5 8 3+14;
           4 9 4+21;
           3 10 3;
           4 11 4+14;
           5 10 4+14;
           6 9 3+14;
           6 4 4+7;
           4 4 2+7;
           5 5 2+21;
           5 1 4;
           6 2 3+14;
           8 3 4;
           9 1 3;
           10 1 4+7;
           11 4 4+21;
           10 5 3+21;
           11 5 3;
           11 10 3+21;
           9 11 4+14;
           8 11 4+21;
           8 7 3+7;
           10 7 3+7];

       Wp_series_init{1} = [1 1 4+7;
           2 3 3+14;
           1 4 4+7;
           3 5 4+21;
           2 6 2;
           1 8 4+14;   4 8 4+14; 
           5 8 3+14;   4 8 25;  
           4 9 4+21;   4 10 3; 
           3 10 3;  4 10 3; 
           4 11 4+14;
           5 10 4+14;    4 10 4+14; 4 9 3+14; 
           6 9 3+14; 5 9 3+14;   5 8 4+7; 5 4 4+7;  
           6 4 4+7;    4 4 4+7; 
           4 4 2+7;
           5 5 2+21;
           5 1 4;
           6 2 3+14;
           8.1 3 4;
           9 1 3;
           10 1 4+7;
           11 4 4+21;
           10 5 3+21;
           11 5 3;
           11 10 3+21;
           9 11 4+14;
           8 11 4+21;  9 11 4+21;  
           8 7 3+7;
           10 7 3+7];
         Wp_hack_init{1} = Wp_orig;

          Wp_series_init{2}=   [1 1 2;
           1 7 2;
           2 7 2;
           2 1 2;
           3 1 2;
           3 10 2;
           4 10 2;
           4 1 2;
           5 1 2;
           5 10 2;
           8 10 2;
           8 7 2;
           7 7 2;
           9 7 2;
           9 3 2;
           6 3 2;
           6 1 2;
           7 1 2;
           7 3 2;
           8 3 2;
           8 1 2;
           9 1 2;
           9 10 2;
           10 10 2;
           10 1 2]; 
       Wp_hack_init{2} = Wp_series_init{2}; 

       % Spiral
       Wp_series_init{3}=  [5 5 2;  
                5 7 2;     
                9 7 2;
                9 3 2;
                4 3 2;
                4 7 2;
                9 7 2;
                9 3 2;
                3 3 2;
                3 8 2;
                5 8 2;
                5 7 2;
                8 7 2;
                8 8 2;
                8 7 2;
                9 7 2;
                9 2 2;
                2 2 2;
                2 7 2;
                3 7 2;
                3 9 2;
                5 9 2;
                5 10 2;
                8 10 2;
                8 9 2;
                9 9 2;
                9 1 2;
                1 1 2;
                1 7 2;
                3 7 2;
                3 10 2;
                10 10 2;
                10 1 2];
       Wp_hack_init{3} = Wp_series_init{3}; 

        Wp_series_init{4}=  [
         1     1    11
         2 3 3+14;
           1 4 4+7;
           3 5 4+21;
           2 6 2;
           1 8 4+14;
           4 8 4+14;  %add
           5 8 3+14;
           5 9 4+21;  %add
           4 9 4+21;
           4 10 3; %add
         3    10     3
         4    11    18
         5    10    18
         4 10 4+14; %add
         4 9 3+14;  %add
         6     9    17;
         4 9 17; %add
         4 11 17; %add
         8 11 17; %add
         8    11    25
         9    11    18
        11    10    24
        10     7    10
        10     5    24
        11     5     3
        11     4    25
         8     3     4
         6 3 11 %add
         6     4    11
         4 4 11 %add
         4     4     9
         5     5    23
         5     1     4
         6     2    17
         9     1     3
        10     1    11
        10 7 10
         8     7    10];

        Wp_hack_init{4} =    [
         1     1    11
         2     3    17
         1     4    11
         3     5    25
         2     6     2
         1     8    18
         5     8    17
         4     9    25
         3    10     3
         4    11    18
         5    10    18
         6     9    17
         8    11    25
         9    11    18
        11    10    24
        10     7    10
        10     5    24
        11     5     3
        11     4    25
         8     3     4
         6     4    11
         4     4     9
         5     5    23
         5     1     4
         6     2    17
         9     1     3
        10     1    11
         8     7    10 ];

        Wp_series_init{5}=  [
         1     1    11
         10 1 10
        10     7    10
        9 11 18
        5 11 18
         5    10    18
         4 10 3
         3    10     3
         5 10 3
         9    11    18
         3 11 18
         3 7 18
         2     6     2
         4     9    25
         4     4     9
        10 4 9
        11    10    24
        9 8 17
        6 8 17
         6     9    17
         6 8 17
         9 8 17
        11     5     3
        11 4 25
        4 4 25
         3     5    25
         5 4 24
         9 4 24
        10     5    24
        10 11 18
         4    11    18
         8    11    25
         10 11 25
         10 4 25
         6     2    17
         1     8    18
         1 3 4
         8     3     4
         1     4    11
         6     4    11
         6 1 11
         9     1     3
         2     3    17
        10     1    11
        11     4    25
         5     1     4
         5 7 4
         5     8    17
         5     5    23
         5 7 23
         8 7 23
         8     7    10 ];

        Wp_hack_init{5} =    [
        1     1    11
        10     7    10
         5    10    18
         3    10     3
         9    11    18
         2     6     2
         4     9    25
         4     4     9
        11    10    24
         6     9    17
        11     5     3
         3     5    25
        10     5    24
         4    11    18
         8    11    25
         6     2    17
         1     8    18
         8     3     4
         1     4    11
         6     4    11
         9     1     3
         2     3    17
        10     1    11
        11     4    25
         5     1     4
         5     8    17
         5     5    23
         8     7    10];
     
   elseif strcmp(WaypointMapMode, 'obs2')
   end
   
   for runidx = 1:runs   
       
    Wp = Wp_series_init{runidx};
    Wp_out = [];
    Wp_out(:,1) = Wp(:,2);
    Wp_out(:,2) = gs(2)+1-Wp(:,1); 
    Wp_out(:,3) = Wp(:,3); 
    Wp_out(:,1:2) = (Wp_out(:,1:2)-0.5)*gw;
    Wp_series_out{runidx} = Wp_out;
    
    Wp = Wp_hack_init{runidx};
    Wp_out = [];
    Wp_out(:,1) = Wp(:,2);
    Wp_out(:,2) = gs(2)+1-Wp(:,1); 
    Wp_out(:,3) = Wp(:,3); 
    Wp_out(:,1:2) = (Wp_out(:,1:2)-0.5)*gw;
    Wp_hack_out{runidx} = Wp_out;
   end
end