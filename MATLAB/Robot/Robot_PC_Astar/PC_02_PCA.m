

max_step = 20;
pause_time = 1;
time_interval = 0.1;

gs = [25 25];

Grid_visited = zeros(gs(1),gs(2),max_step);
Grid_obs = zeros(gs(1),gs(2));
obstacle_map = [4, 4;
                4, 5;
                5, 5;
                5, 6];

            
rrange = 2;
nbrange = 1;
mu = 0.11;     
A = 80;
B = 1;
D = 1;
E = 1;
            
Activity = ones(gs(1),gs(2),max_step)*E;
for intidx = 1:size(obstacle_map,1)
    Grid_obs(obstacle_map(intidx,1),obstacle_map(intidx,2)) = 1;
    Activity(obstacle_map(intidx,1),obstacle_map(intidx,2),1) = -E;
end





   for selfrow = 1:gs(1)
       for selfcol = 1:gs(2)
           
           for shiftrow = -rrange:rrange
               for shiftcol = -rrange:rrange
                   nextrow = selfrow + shiftrow;
                   nextcol = selfcol + shiftcol;
                   if (nextrow <= gs(1) && nextrow > 0 && nextcol <= gs(2) && nextcol > 0)
                        Weight(selfrow, selfcol, nextrow, nextcol) = mu/norm([nextrow, nextcol]-[selfrow,selfcol]);
                   end
                   if (shiftrow == selfrow && shiftcol == selfcol)
                       Weight(selfrow, selfcol, nextrow, nextcol) = 0;
                   end
               end
           end
       end
   end
   
   for step = 1:20
       pause(pause_time)
       
       for selfrow = 1:gs(1)
           for selfcol = 1:gs(2)
               
               sums = 0;
               
               for shiftrow = -nbrange:nbrange
                   for shiftcol = -nbrange:nbrange
                       nextrow = selfrow + shiftrow;
                       nextcol = selfcol + shiftcol;
                       if (shiftrow == 0 && shiftcol == 0)
                           a = 1+1;
                       elseif (nextrow <= gs(1) && nextrow > 0 && nextcol <= gs(2) && nextcol > 0)
                           
                           if Activity(nextrow, nextcol, step) <= 0
                               nextactivity = 0;
                           else
                               nextactivity =  Activity(nextrow, nextcol, step);
                           end
                           sums = sums + nextactivity*Weight(selfrow, selfcol, nextrow, nextcol);
                           if (selfrow == 4 && selfcol == 5)
                                disp(['row ', num2str(nextrow), ' col ', num2str(nextcol), ' activity '...
                                    ,num2str(nextactivity*Weight(selfrow, selfcol, nextrow, nextcol))]);
                           end
                       end
                   end
               end
               if (Grid_visited(selfrow,selfcol,step) == 0)
                   excitatory_input = E;
               else
                   excitatory_input = 0;
               end
               if (Grid_obs(selfrow,selfcol) == 1)
                   inhibitory_input = E;
                   
                   Activity(selfrow, selfcol, step+1) = Activity(selfrow, selfcol, step);
                                                   
               else
                   inhibitory_input = 0;
                   
                    Activity(selfrow, selfcol, step+1) = Activity(selfrow, selfcol, step) +...
                                                (-A*Activity(selfrow, selfcol, step) +...
                                               (B-Activity(selfrow, selfcol, step))*(excitatory_input + sums)+...
                                               -(D+Activity(selfrow, selfcol, step))*inhibitory_input)*time_interval;
               end
               
               
               if Activity(selfrow, selfcol, step+1) > E
                   Activity(selfrow, selfcol, step+1) = E;
               elseif Activity(selfrow, selfcol, step+1) < -E
                   Activity(selfrow, selfcol, step+1) = -E;
               end
           end
       end
       
       figure(1)
       title('Robot Coverage Map')
       imagesc(flipud(transpose(Activity(:,:,step+1))))
       colormap('parula');
       colorbar       
       caxis([-E E])
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