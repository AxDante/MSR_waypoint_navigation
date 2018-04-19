

pause_time = 1;
    
gs = [25 25];
               
Gobs = [4, 4;
                4, 5;
                5, 5;
                5, 6];
    
rrange = 2;
nbrange = 1;
mu = 1;     
B = 1;
D = 1;

            
   for selfrow = 1:gs(2)
       for selfcol = 1:gs(1)
           
           for shiftrow = -rrange:rrange
               for shiftcol = -rrange:rrange
                   nextrow = selfrow + shiftrow;
                   nextcol = selfcol + shiftcol;
                   if (nextrow <= gs(2) && nextrow > 0 && nextcol <= gs(1) && nextcol > 0)
                        Weight(selfrow, selfcol, nextrow, nextcol) = mu/norm([nextrow, nextcol]-[selfrow,selfcol]);
                   end
               end
           end
       end
   end
   Weight
   
   
   
   tot_step = 20;
   
   
   for step = 1:tot_step
       pause(pause_time)
       
       for selfrow = 1:gs(2)
           for selfcol = 1:gs(1)
               
               sums = 0;
               
               for shiftrow = -nbrange:nbrange
                   for shiftcol = -nbrange:nbrange
                       nextrow = selfrow + shiftrow;
                       nextcol = selfcol + shiftcol;
                       if (shiftrow == 0 && shiftcol == 0)
                           a = 1+1;
                       elseif (nextrow <= gs(2) && nextrow > 0 && nextcol <= gs(1) && nextcol > 0)
                           sums = sums +  = mu/norm([nextrow, nextcol]-[selfrow,selfcol]);
                       end
                       
                   end
               end
           end
       end
       
       
       
   end
   
   
    %{
    
    scg = rcg;
    Wp = [];
    
    
    
    for idx = 1: size(Wp_hack,1)
        gcg = [ceil(Wp_hack(idx,1)) ceil(Wp_hack(idx,2))];
        Wp_s = PC_Astar_origin_algorithm(gs, Gobs, Wp_hack(idx,3), scg, gcg); %segemented Wp
        Wp_s(:, 3) = Wp_hack(idx,3);
        Wp = [Wp; Wp_s];
        scg = gcg;
    end
   
    
    
    
    for step = 1: tot_step
        
        
        
        
        
    end
    
    Wp(:, 1:2) = (Wp(:, 1:2) - 0.5)*gw;
    Wp_hack = (Wp_hack-0.5)*gw;
   %}