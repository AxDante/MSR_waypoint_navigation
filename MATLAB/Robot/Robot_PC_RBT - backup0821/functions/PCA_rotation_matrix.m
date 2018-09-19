% -----------------------------------------------------------
% Author: AxDante <knight16729438@gmail.com>
% Singapore University of Technology and Design
% Created: April 2018
% Modified: April 2018
% -----------------------------------------------------------

function R = PCA_rotation_matrix(M, config)
    shape = mod(config-1, 7)+1;
    deg = pi/2*floor((config-1)/7);
    rotation_matrix = [cos(deg) -sin(deg);
                sin(deg) cos(deg)];
    full_rotation = M*rotation_matrix;
    R = round(full_rotation(shape*3-2:shape*3, :));
end