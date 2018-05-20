function Wp = convertNodeToWp(node_array, col_size, starting_row)
    Wp = [];
    
    for intidx = 1:size(node_array,2)
        Wp = [Wp; mod(node_array(1,intidx)-1,col_size)+1 floor(node_array(1,intidx)/col_size)+starting_row];
    end

end