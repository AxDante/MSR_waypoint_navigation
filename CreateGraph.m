function [E, W]= CreateGraph(row_size, col_size, Gobs, shape, allowedShape)
    
    E = [];
    W = ones(1, row_size*col_size);
    tot_allowedShape = size(allowedShape,2);
    
    for shapeidx = 1:tot_allowedShape
        for rowidx = 1:row_size
            for colidx = 1:col_size
                if colidx ~= col_size
                    E = [E; (rowidx-1)*col_size+colidx,  (rowidx-1)*col_size+colidx+1 allowedShape(shapeidx)];
                end
            end
        end
    end
    
    for rowidx = 1:row_size
        for colidx = 1:col_size
            Ga{colidx, rowidx} = checkNodeValidity(colidx, rowidx, [col_size row_size], fliplr(Gobs));
        end
    end
    
    
    %% Node connection between morphologies
    for shape = allowedShape
        ShapeTraverse{shape} = [];
        for colidx = 1:col_size
            invalidNodes = 0;
            for rowidx = 1:row_size
                ga = Ga{colidx, rowidx};
                if ga(shape) == 0 % Invalid action 
                    invalidNodes = invalidNodes + 1;
                end
            end
            if invalidNodes == row_size
                ShapeTraverse{shape} = [ShapeTraverse{shape} 0];
            else
                ShapeTraverse{shape} = [ShapeTraverse{shape} 1];
            end
        end
    end
    
    for colidx = 1:col_size
        ShapeTraverse{2}
        ShapeTraverse{8}
    end
    
    
    %%
    for Eidx = 1:size(E,1)
        scndNodeIdx = E(Eidx,1);
        scndNodeGrid = [mod(E(Eidx,1)-1, col_size)+2, floor(E(Eidx,1)/col_size)+1];
        ga = Ga{scndNodeGrid(1), scndNodeGrid(2)};
        if ga(shape) == 0 % If action is invalid on the second node
            W(Eidx) = 0.01;
        end
    end
    
    for garowidx = 1: size(Ga,1)
        for gacolidx = 1: size(Ga,2)
        end
    end
    
    gridDirection = ones(1, col_size);
    prevDirection = -1;
    gridDirection(1) = -1;
    prevObsPosition = 1;
    currentObsPosition = 0;
    for colidx = 1:col_size
        isObsPresent = false;
        for rowidx = 1:row_size
            for gobsidx = 1 : size(Gobs, 1)
                if (Gobs(gobsidx, 1) == rowidx) && (Gobs(gobsidx, 2) == colidx) 
                    isObsPresent = true;
                    currentObsPosition = rowidx;
                end
            end
        end
        if isObsPresent == true
            gridDirection(colidx) = 0;
            if (colidx > 1)
                if (prevDirection ~= 0 && gridDirection(colidx-1) == 0)
                elseif (prevDirection ~= 0 && gridDirection(colidx-1) == prevDirection)
                end
            end
        else
            if (colidx > 1)
                if (prevDirection ~= 0 && gridDirection(colidx-1) == 0)
                    if (currentObsPosition ~= prevObsPosition)
                        prevDirection = -prevDirection;
                        prevObsPosition = currentObsPosition;
                        currentObsPosition = 0;
                    end
                    gridDirection(colidx) = prevDirection;
                elseif (prevDirection ~= 0 && gridDirection(colidx-1) == prevDirection)
                    gridDirection(colidx) = prevDirection;
                end
            end
        end
    end
    gridDirection
end