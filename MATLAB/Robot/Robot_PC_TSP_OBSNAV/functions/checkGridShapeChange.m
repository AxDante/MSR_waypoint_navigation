


function gsc = checkGridShapeChange(rowidx,colidx, gs, Gobs)

    Rgp = [0 -1; 0 1; 0 2;                         % Relative grid positions between modules
                  0 -1; 1 0; 1 -1;
                 -1 0; 0 1; 1 1;
                  -1 0; 0 1; 0 2;
                  1 0; 0 1; -1 1;
                  1 -1; 1 0; 2 0;
                  1 -1; 1 0; 2 -1];         
    
     Scp = cell(8,8);
     
     Scp{2, 8} = [-1 -1; -1 0; 0 -1; 0 0; 1 -1; 1 0; 2 -1; 2 0];
     Scp{8, 2} = [-1 -1; -1 0; 0 -1; 0 0; 1 -1; 1 0; 2 -1; 2 0];
     
     gsc = zeros(8,8);
     
     
     for shape_prev = 1:8          
         for shape_next = 1:8
             
            isvalid = true;
            if size(Scp{shape_prev, shape_next},1) ~= 0
                Rg = Scp{shape_prev, shape_next}+[rowidx, colidx];
                for idx = 1:size(Rg,1)   
                    if (Rg(idx,1) > gs(1) || Rg(idx,1) <= 0 || ...
                            Rg(idx,2) > gs(2) || Rg(idx,2) <= 0)
                        isvalid = false;
                    else
                        for obsidx = 1:size(Gobs,1)
                            if (Rg(idx,1) == Gobs(obsidx,1) && Rg(idx,2) == Gobs(obsidx,2))
                                isvalid = false;
                            end
                        end
                    end
                end
                if (isvalid)
                    gsc(shape_prev, shape_next) = 1;
                end
            end
         end
     end         
end