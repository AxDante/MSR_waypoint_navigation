function [Wp, Gvis] = wpMove(Wp, x, y, shape, Gvis)
    Wp = [Wp; x y shape];
    
    Rgp = [0 -1; 0 1; 0 2;
              0 -1; 1 0; 1 -1;
             -1 0; 0 1; 1 1;
              -1 0; 0 1; 0 2;
              1 0; 0 1; -1 1;
              1 -1; 1 0; 2 0;
              1 -1; 1 0; 2 -1];
   
   RRgp = rotationMatrix(Rgp, shape);
   
   Rg = [ RRgp(1,:);
             0 0;
             RRgp(2,:);
             RRgp(3,:)] + [x y];
   for idx = 1:4
       Gvis(Rg(idx,1), Rg(idx,2)) = 1;
   end
                              
end