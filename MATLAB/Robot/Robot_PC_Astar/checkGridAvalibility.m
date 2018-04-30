function GA = checkGridAvalibility(rowidx,colidx, gs, Gobs)

    Rgp = [0 -1; 0 1; 0 2;                         % Relative grid positions between modules
                  0 -1; 1 0; 1 -1;
                 -1 0; 0 1; 1 1;
                  -1 0; 0 1; 0 2;
                  1 0; 0 1; -1 1;
                  1 -1; 1 0; 2 0;
                  1 -1; 1 0; 2 -1];         
     
     RRgp = zeros(8,3,2);
     GA = zeros(1,8);
     
     
     for shape = 1:8          
         rotationMatrix(Rgp, shape)
         newShape = zeros(1,3,2);
         newShape(1,:,:) = rotationMatrix(Rgp, shape);
         RRgp(shape,:,:) = newShape;       % Rotated Relative grid positions
         size(RRgp(shape,1,:))
         Rg = [ RRgp(shape,1,:);                                % Robot Grid values
                     0 0;
                     RRgp(shape,2,:);
                     RRgp(shape,3,:)] +[rowidx, colidx];
        
        isvalid = true;
        for idx = 1:size(Rg,1)   
            if (Rg(idx,1) > gs(1) || Rg(idx,1) <= 0 || ...
                    Rg(idx,2) > gs(2) || Rg(idx,2) <= 0)
                isvalid = false;
            else
                if Gobs(Rg(idx,1), Rg(idx,2)) == 1
                    isvalid = false;
                end
            end
        end
        if (isvalid)
            GA(shape) = 1;
        end
     end         
end