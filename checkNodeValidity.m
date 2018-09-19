function ga = checkNodeValidity(rowidx,colidx, gs, Gobs)

    Rgp = [0 -1; 0 1; 0 2;                         % Relative grid positions between modules
                  0 -1; 1 0; 1 -1;
                 -1 0; 0 1; 1 1;
                  -1 0; 0 1; 0 2;
                  1 0; 0 1; -1 1;
                  1 -1; 1 0; 2 0;
                  1 -1; 1 0; 2 -1];         
     
     RRgp = zeros(8,3,2);
     ga = zeros(1,8);
     
     
     for shape = 1:8          
         newShape = rotationMatrix(Rgp, shape);
         Rg = [ newShape(1,:);                                % Robot Grid values
                     0 0;
                      newShape(2,:);
                      newShape(3,:)] +[rowidx, colidx];
        
        isvalid = true;
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
            ga(shape) = 1;
        end
     end         
end