function c = load28waypoints()
% LOADGR96 Loads the data file 
% NAME: gr96
% TYPE: TSP
% COMMENT: Africa-Subproblem of 666-city TSP (Groetschel)
% DIMENSION: 96
% EDGE_WEIGHT_TYPE: GEO
% DISPLAY_DATA_TYPE: COORD_DISPLAY
% NODE_COORD_SECTION
clear all;
temp_x = [ 
    1, 1 1 1 2   2 1 3 1
    2, 4 1 4 2   5 1 6 1
    3, 8 1 7 1   8 2 8 3
    4, 12 3 11 3   11 2 11 1
    5, 3 2 2 2   2 3 2 4
    6, 5 3 5 2   4 3 3 3
    7, 6 2 6 3   7 3 7 2
    8, 1 5 2 5   1 4 1 3
    9, 9 4 9 3   8 4 7 4
    10, 3 5 3 4   4 5 4 4
    11, 5 5 5 4   6 5 6 4
    12, 11 4 11 3   10 5 11 6
    13, 8 5 7 5   7 6 7 7
    14, 10 5 9 5  10 6 10 7
    15, 1 7 1 6  1 8 1 9
    16, 2 7 2 6   2 8 2 9
    17, 3 8 4 8   3 7 3 6
    18, 4 6 4 7   5 6 6 6
    19, 9 6 8 6    8 7 8 8
    20, 11 8 11 7   10 8 9 8
    21, 7 8 7 9   8 9 9 9
    22, 5 10 5 9   4 9 3 9
    23, 5 11 6 11   6 10 6 9
    24, 11 9 10 9   11 10 11 11
    25, 1 10 1 11   2 10 3 10
    26, 4 11 4 10   3 11 2 11
    27, 7 10 6 11   8 11 9 11
    28, 10 11 10 10     9 10 8 10];

%cities = [temp_x(:,2)';temp_x(:,3)'];
%cities = temp_x;
save cities.mat cities -V6;