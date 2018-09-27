function Wp = convertNodeToWp(node_array, col_size, starting_row, allowedShape)
    Wp = [];
    tot_allowedShape = size(allowedShape,2);
    for intidx = 1:size(node_array,2)
        morphLyrNode = mod(node_array(intidx)-1,(tot_allowedShape*col_size*2))+1;
        morphLyr = floor(node_array(intidx)/(tot_allowedShape*col_size*2))+1;
        Wp = [Wp; mod(morphLyrNode-1,col_size)+1 floor((morphLyrNode-1)/col_size)+starting_row allowedShape(morphLyr)];
    end

end