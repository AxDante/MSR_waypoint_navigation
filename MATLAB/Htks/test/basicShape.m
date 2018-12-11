clear 
clc
clf
cla 
close all


max_step = 1000;

size_ws = [5 5];
size_cvg = [50 50];

s_tl = 1;
s_rt = 1/12;

d_r = 10;
d_cvg = size_ws./size_cvg*d_r;
wp = zeros(max_step,5);
wp(1,:) = [25,25, 0, 4/3, 4/3];

Line_Rbt = [];

figure(1)
set(figure(1), 'Position', [100, 100, 1020, 900])
%axis([-d_r*1.5 d_r*(size_ws(1)+1.5) -d_r*1.5 d_r*(size_ws(2)+1.5)])
axis([0 d_r*(size_ws(1)) 0 d_r*(size_ws(2))])
title('hTetro Waypoint Map')
hold on


wp_cur = 1;
Wp = [25, 25, 0, 4/3, 4/3;
           55, 25, 0, 4/3, 4/3;
           40, 32, 1/3, 4/3, 4/3;
           30, 12, 1/3, 1/3, 4/3;
           20, 20, 2/3, 1/3, 4/3;
           15, 25, 2/3, 4/3, 8/3;
           45, 45, 1, 4/3, 8/3;];

       %shape 1  [4/3 4/3]
       %shape A  1/3, 4/3;
step = 1;

clims = [-1000, 800];  
cvg_dflt = -900;
cvg_clrmap = 'jet';
Cvg = ones(size_cvg(2),size_cvg(1),max_step)*cvg_dflt;
area_trg = sqrt(3)/4*d_r^2;

Line_Rbt = [];

while step < max_step && wp_cur <= size(Wp,1)
   
    if abs(Wp(wp_cur, 3) - wp(step,3)) > 1e-10 || abs(Wp(wp_cur, 4) - wp(step,4)) > 1e-10 ||abs(Wp(wp_cur, 5) - wp(step,5)) > 1e-10
       a_add = [0 0 0];
       if Wp(wp_cur, 3) - wp(step,3) > 1e-10
           a_add = a_add + [s_rt 0 0];
       elseif Wp(wp_cur, 3) - wp(step,3) < -1e-10
           a_add = a_add + [-s_rt 0 0];
       end
       if Wp(wp_cur, 4) - wp(step,4) > 1e-10
           a_add = a_add + [0 s_rt 0];
       elseif Wp(wp_cur, 4) - wp(step,4) < -1e-10
           a_add = a_add + [0 -s_rt 0];
       end
       if Wp(wp_cur, 5) - wp(step,5) > 1e-10
           a_add = a_add + [0 0 s_rt];
       elseif Wp(wp_cur, 5) - wp(step,5) < -1e-10
           a_add = a_add + [0 0 -s_rt];
       end
       wp(step+1, :)  = wp(step, :) + [0 0 a_add(1) a_add(2) a_add(3)];
        
    elseif abs(Wp(wp_cur, 1) - wp(step,1)) < 1e-3 && abs( Wp(wp_cur, 2) - wp(step,2))< 1e-3
        wp(step+1, :)  = wp(step, :);
        wp_cur = wp_cur + 1;
    elseif abs(Wp(wp_cur, 1) - wp(step,1)) > abs(Wp(wp_cur, 2) - wp(step,2)) 
        if Wp(wp_cur, 1) - wp(step,1) > 0
            wp(step+1, :) = wp(step, :) + [s_tl 0 0 0 0];
        else
            wp(step+1, :) = wp(step, :) + [-s_tl 0 0 0 0];
        end
    elseif abs(Wp(wp_cur, 1) - wp(step,1)) <= abs(Wp(wp_cur, 2) - wp(step,2)) 
        if Wp(wp_cur, 2) - wp(step,2) > 0
            wp(step+1, :) = wp(step, :) + [0 s_tl 0 0 0];
        else
            wp(step+1, :) = wp(step, :) + [0 -s_tl 0 0 0];
        end
    end

    a_vc13 = 1/3 - wp(step+1, 3);
    a_vc24 = 1/6 + wp(step+1, 3);
    a_v11 = -wp(step+1, 4) - wp(step+1, 3) + 11/6;
    a_v12 = -wp(step+1, 4) - wp(step+1, 3) + 3/2;
    a_v21 = wp(step+1, 5) + wp(step+1, 3) - 3/2;
    a_v22 = wp(step+1, 5) + wp(step+1, 3) - 7/6; 
    
    p_vc1 = wp(step+1, 1:2) + d_r/2 * [-cos(pi*a_vc13) sin(pi*a_vc13)];
    p_vc2 = wp(step+1, 1:2) + d_r*sqrt(3)/2 * [cos(pi*a_vc24) sin(pi*a_vc24)];
    p_vc3 = wp(step+1, 1:2) + d_r/2 * [cos(pi*a_vc13) -sin(pi*a_vc13)];
    p_vc4 = wp(step+1, 1:2) + d_r*sqrt(3)/2 * [-cos(pi*a_vc24) -sin(pi*a_vc24)];

    p_v11 = p_vc1 + d_r * [-sin(pi*a_v11) -cos(pi*a_v11)];
    p_v12 = p_vc1 + d_r * [-sin(pi*a_v12) -cos(pi*a_v12)];
    p_v21 = p_vc2 + d_r * [sin(pi*a_v21) -cos(pi*a_v21)];
    p_v22 = p_vc2 + d_r * [sin(pi*a_v22) -cos(pi*a_v22)];
    
    
    figure(1)
    if (~isempty(Line_Rbt))
        delete(Line_Rbt);
    end
    Line_Rbt = [];
    
    Line_Rbt(1) = line([p_vc1(1) p_vc2(1)], [p_vc1(2) p_vc2(2)], 'Color', 'black', 'LineWidth', 2);
    Line_Rbt(2) = line([p_vc2(1) p_vc3(1)], [p_vc2(2) p_vc3(2)], 'Color', 'black', 'LineWidth', 2);
    Line_Rbt(3) = line([p_vc3(1) p_vc4(1)], [p_vc3(2) p_vc4(2)], 'Color', 'black', 'LineWidth', 2);
    Line_Rbt(4) = line([p_vc4(1) p_vc1(1)], [p_vc4(2) p_vc1(2)], 'Color', 'black', 'LineWidth', 2);

    Line_Rbt(5) = line([p_vc1(1) p_v11(1)], [p_vc1(2) p_v11(2)], 'Color', 'black', 'LineWidth', 2);
    Line_Rbt(6) = line([p_v11(1) p_v12(1)], [p_v11(2) p_v12(2)], 'Color', 'black', 'LineWidth', 2);
    Line_Rbt(7) = line([p_v12(1) p_vc1(1)], [p_v12(2) p_vc1(2)], 'Color', 'black', 'LineWidth', 2);

    Line_Rbt(8) = line([p_vc2(1) p_v21(1)], [p_vc2(2) p_v21(2)], 'Color', 'black', 'LineWidth', 2);
    Line_Rbt(9) = line([p_v21(1) p_v22(1)], [p_v21(2) p_v22(2)], 'Color', 'black', 'LineWidth', 2);
    Line_Rbt(10) = line([p_v22(1) p_vc2(1)], [p_v22(2) p_vc2(2)], 'Color', 'black', 'LineWidth', 2);

    
    Vs = [p_vc1 p_vc3  p_vc4;
            p_vc1 p_vc2 p_vc3;
            p_vc1 p_v11 p_v12;
            p_vc2 p_v21 p_v22];
   
    p_b = [mean([p_vc1; p_vc3; p_vc4]);
                 mean([p_vc1; p_vc2; p_vc3]);
                 mean([p_vc1; p_v11; p_v12]);
                 mean([p_vc2; p_v21; p_v22])];
    
    for  idxx = 1:size_cvg(1)
        for idxy = 1:size_cvg(2)
            sample_pos = [(idxx-0.5) (idxy-0.5)]*d_cvg(1);
            within = false;
            Cvg(idxx,idxy,step+1) = Cvg(idxx,idxy,step);
            if (Cvg(idxx,idxy,step+1) > -900 && Cvg(idxx,idxy,step+1) < -10)
                Cvg(idxx,idxy,step+1) = 0;
            end
            for blkidx = 1:4
                if (p_b(blkidx,:)-sample_pos < 0.6*d_r)
                   trig_x = [sample_pos(1) Vs(blkidx,1) Vs(blkidx,3)];
                   trig_y = [sample_pos(2) Vs(blkidx,2) Vs(blkidx,4)];
                   area_1 = polyarea(trig_x,trig_y);
                   trig_x = [sample_pos(1) Vs(blkidx,3) Vs(blkidx,5)];
                   trig_y = [sample_pos(2) Vs(blkidx,4) Vs(blkidx,6)];
                   area_2 = polyarea(trig_x,trig_y);
                   trig_x = [sample_pos(1) Vs(blkidx,5) Vs(blkidx,1)];
                   trig_y = [sample_pos(2) Vs(blkidx,6) Vs(blkidx,2)];
                   area_3 = polyarea(trig_x,trig_y);
                   area_1 + area_2 + area_3 - area_trg;
                   if area_1 + area_2 + area_3 <= area_trg + 0.01
                       within = true;
                   end
               end
               if (within)            
                   if (Cvg(idxx,idxy,step+1) < -10)
                        Cvg(idxx,idxy,step+1) = 0;
                   else
                        Cvg(idxx,idxy,step+1) = Cvg(idxx,idxy,step) + 4;
                   end
                end
            end
        end
    end
    
    
        figure(2)
        set(figure(2), 'Position', [1000, 100, 1020, 900])
        title('hTetro Coverage Map')
        imagesc(flipud(transpose(Cvg(:,:,step+1))), clims)
       % axis([-d_r*1.5 d_r*(size_ws(1)+1.5) -d_r*1.5 d_r*(size_ws(2)+1.5)])
        cmap = colormap(cvg_clrmap);
        cmap(1,:) = zeros(1,3);
        colormap(cmap);
        colorbar
    
    
    step = step + 1;
    pause(0.1)
    
end
