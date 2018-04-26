%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Slice Info Generator
% Produces zigzag path and calculate path cost for each slices
% 04-22-2018
% Author: Ping-Cheng Ku
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [Slice_Info, Slice_Wp, Slice_Info_size] = get_slice_info(Slice_final)
    
    Slice_Info = {};
    Slice_Wp = {};
    Slice_Info_size = [];
    
    for sliceidx = 1:size(Slice_final,1)
        slice_count = 0;
        slice_info = [];
        
        ssrow = Slice_final(sliceidx,1);
        sscol = Slice_final(sliceidx,2);
        serow = Slice_final(sliceidx,3);
        secol = Slice_final(sliceidx,4);
        
        row_w = serow - ssrow + 1;
        col_w = secol - sscol + 1;
        % Row and column are odd
        if  mod(row_w,2) == 1
            if mod(col_w,2) == 1
               if row_w == 1 && col_w == 1
                   slice_info = [ssrow, sscol, serow, secol, 0 , 0];
                   slice_count = slice_count+1;
                   Slice_Wp{sliceidx,slice_count} = slice_wp;
                   Slice_Info{sliceidx,slice_count} = slice_info;
               elseif row_w == 1
                   slice_info = [ssrow, sscol, serow, secol, 2, secol-sscol];
                   slice_count = slice_count+1;
                   slice_wp = [];
                   for rowidx = 0:col_w-1
                     slice_wp = [slice_wp; ssrow, sscol+rowidx];
                   end
                   Slice_Wp{sliceidx,slice_count} = slice_wp;
                   Slice_Info{sliceidx,slice_count} = slice_info;
                   %==========================
                   slice_info = [serow, secol, ssrow, sscol, 4, secol-sscol];
                   slice_count = slice_count+1;
                   slice_wp = [];
                   for rowidx = 0:col_w-1
                     slice_wp = [slice_wp; serow, secol-rowidx];
                   end
                   Slice_Wp{sliceidx,slice_count} = slice_wp;
                   Slice_Info{sliceidx,slice_count} = slice_info;
                   %==========================
               elseif col_w == 1
                   slice_info = [ssrow, sscol, serow, secol, 3, serow-ssrow];
                   slice_count = slice_count+1;
                   slice_wp = [];
                   for rowidx = 0:row_w-1
                     slice_wp = [slice_wp; ssrow+rowidx, sscol];
                   end
                   Slice_Wp{sliceidx,slice_count} = slice_wp;
                   Slice_Info{sliceidx,slice_count} = slice_info;
                   %==========================
                   slice_info = [serow, secol, ssrow, sscol, 1, serow-ssrow];
                   slice_count = slice_count+1;
                   slice_wp = [];
                   for rowidx = 0:row_w-1
                     slice_wp = [slice_wp; serow-rowidx, secol];
                   end
                   Slice_Wp{sliceidx,slice_count} = slice_wp;
                   Slice_Info{sliceidx,slice_count} = slice_info;
                   %==========================
               else
                   slice_info = [ssrow, sscol, serow, secol, 2, row_w*col_w-1];
                   slice_count = slice_count+1;
                   slice_wp = [];
                   direction = 1;
                   for rowidx = 0:row_w-1
                       for colidx = 0:col_w-1
                           if direction == 1
                               slice_wp = [slice_wp; ssrow+rowidx, sscol+colidx];
                           elseif direction == -1
                               slice_wp = [slice_wp; ssrow+rowidx, secol-colidx];
                           end
                       end
                       direction = -direction;
                   end
                   Slice_Wp{sliceidx,slice_count} = slice_wp;
                   Slice_Info{sliceidx,slice_count} = slice_info;
                   %==========================
                   slice_info = [ssrow, sscol, serow, secol, 3, row_w*col_w-1];
                   slice_count = slice_count+1;
                   slice_wp = [];
                   direction = 1;
                   for colidx = 0:col_w-1
                       for rowidx = 0:row_w-1
                           if direction == 1
                               slice_wp = [slice_wp; ssrow+rowidx, sscol+colidx];
                           elseif direction == -1
                               slice_wp = [slice_wp; serow-rowidx, sscol+colidx];
                           end
                       end
                       direction = -direction;
                   end
                   Slice_Wp{sliceidx,slice_count} = slice_wp;
                   Slice_Info{sliceidx,slice_count} = slice_info;
                   %==========================
                   slice_info = [ssrow, secol, serow, sscol, 3, row_w*col_w-1];
                   slice_count = slice_count+1;
                   slice_wp = [];
                   direction = 1;
                   for colidx = 0:col_w-1
                       for rowidx = 0:row_w-1
                           if direction == 1
                               slice_wp = [slice_wp; ssrow+rowidx, secol-colidx];
                           elseif direction == -1
                               slice_wp = [slice_wp; serow-rowidx, secol-colidx];
                           end
                       end
                       direction = -direction;
                   end
                   Slice_Wp{sliceidx,slice_count} = slice_wp;
                   Slice_Info{sliceidx,slice_count} = slice_info;
                   %==========================
                   slice_info = [ssrow, secol, serow, sscol, 4, row_w*col_w-1];
                   slice_count = slice_count+1;
                   slice_wp = [];
                   direction = -1;
                   for rowidx = 0:row_w-1
                       for colidx = 0:col_w-1
                           if direction == 1
                               slice_wp = [slice_wp; ssrow+rowidx, sscol+colidx];
                           elseif direction == -1
                               slice_wp = [slice_wp; ssrow+rowidx, secol-colidx];
                           end
                       end
                       direction = -direction;
                   end
                   Slice_Wp{sliceidx,slice_count} = slice_wp;
                   Slice_Info{sliceidx,slice_count} = slice_info;
                   %==========================
                   slice_info = [serow, secol, ssrow, sscol, 1, row_w*col_w-1];
                   slice_count = slice_count+1;
                   slice_wp = [];
                   direction = -1;
                   for colidx = 0:col_w-1
                       for rowidx = 0:row_w-1
                           if direction == 1
                               slice_wp = [slice_wp; ssrow+rowidx, secol-colidx];
                           elseif direction == -1
                               slice_wp = [slice_wp; serow-rowidx, secol-colidx];
                           end
                       end
                       direction = -direction;
                   end
                   Slice_Wp{sliceidx,slice_count} = slice_wp;
                   Slice_Info{sliceidx,slice_count} = slice_info;
                   %==========================
                   slice_info = [serow, secol, ssrow, sscol, 4, row_w*col_w-1];
                   slice_count = slice_count+1;
                   slice_wp = [];
                   direction = -1;
                   for rowidx = 0:row_w-1
                       for colidx = 0:col_w-1
                           if direction == 1
                               slice_wp = [slice_wp; serow-rowidx, sscol+colidx];
                           elseif direction == -1
                               slice_wp = [slice_wp; serow-rowidx, secol-colidx];
                           end
                       end
                       direction = -direction;
                   end
                   Slice_Wp{sliceidx,slice_count} = slice_wp;
                   Slice_Info{sliceidx,slice_count} = slice_info;
                   %==========================
                   slice_info = [serow, sscol, ssrow, secol, 2, row_w*col_w-1];
                   slice_count = slice_count+1;
                   slice_wp = [];
                   direction = 1;
                   for rowidx = 0:row_w-1
                       for colidx = 0:col_w-1
                           if direction == 1
                               slice_wp = [slice_wp; serow-rowidx, sscol+colidx];
                           elseif direction == -1
                               slice_wp = [slice_wp; serow-rowidx, secol-colidx];
                           end
                       end
                       direction = -direction;
                   end
                   Slice_Wp{sliceidx,slice_count} = slice_wp;
                   Slice_Info{sliceidx,slice_count} = slice_info;
                   %==========================
                   slice_info = [serow, sscol, ssrow, secol, 1, row_w*col_w-1];
                   slice_count = slice_count+1;
                   slice_wp = [];
                   direction = -1;
                   for colidx = 0:col_w-1
                       for rowidx = 0:row_w-1
                           if direction == 1
                               slice_wp = [slice_wp; ssrow+rowidx, sscol+colidx];
                           elseif direction == -1
                               slice_wp = [slice_wp; serow-rowidx, sscol+colidx];
                           end
                       end
                       direction = -direction;
                   end
                   Slice_Wp{sliceidx,slice_count} = slice_wp;
                   Slice_Info{sliceidx,slice_count} = slice_info;
               end
            end
        end
        % Row and column are even
        if mod(row_w,2) == 0
            if mod(col_w,2) == 0
                   slice_info = [ssrow, sscol, ssrow, secol, 3, row_w*col_w-1];
                   slice_count = slice_count+1;
                   slice_wp = [];
                   direction = 1;
                   for colidx = 0:col_w-1
                       for rowidx = 0:row_w-1
                           if direction == 1
                               slice_wp = [slice_wp; ssrow+rowidx, sscol+colidx];
                           elseif direction == -1
                               slice_wp = [slice_wp; serow-rowidx, sscol+colidx];
                           end
                       end
                       direction = -direction;
                   end
                   Slice_Wp{sliceidx,slice_count} = slice_wp;
                   Slice_Info{sliceidx,slice_count} = slice_info;
                   %==========================
                   slice_info = [ssrow, sscol, serow, sscol, 2, row_w*col_w-1];
                   slice_count = slice_count+1;
                   slice_wp = [];
                   direction = 1;
                   for rowidx = 0:row_w-1
                       for colidx = 0:col_w-1
                           if direction == 1
                               slice_wp = [slice_wp; ssrow+rowidx, sscol+colidx];
                           elseif direction == -1
                               slice_wp = [slice_wp; ssrow+rowidx, secol-colidx];
                           end
                       end
                       direction = -direction;
                   end
                   Slice_Wp{sliceidx,slice_count} = slice_wp;
                   Slice_Info{sliceidx,slice_count} = slice_info;
                   %==========================
                   slice_info = [ssrow, secol, serow, secol, 4, row_w*col_w-1];
                   slice_count = slice_count+1;
                   slice_wp = [];
                   direction = -1;
                   for rowidx = 0:row_w-1
                       for colidx = 0:col_w-1
                           if direction == 1
                               slice_wp = [slice_wp; ssrow+rowidx, sscol+colidx];
                           elseif direction == -1
                               slice_wp = [slice_wp; ssrow+rowidx, secol-colidx];
                           end
                       end
                       direction = -direction;
                   end
                   Slice_Wp{sliceidx,slice_count} = slice_wp;
                   Slice_Info{sliceidx,slice_count} = slice_info;
                   %==========================
                   slice_info = [ssrow, secol, ssrow, sscol, 3, row_w*col_w-1];
                   slice_count = slice_count+1;
                   slice_wp = [];
                   direction = 1;
                   for colidx = 0:col_w-1
                       for rowidx = 0:row_w-1
                           if direction == 1
                               slice_wp = [slice_wp; ssrow+rowidx, secol-colidx];
                           elseif direction == -1
                               slice_wp = [slice_wp; serow-rowidx, secol-colidx];
                           end
                       end
                       direction = -direction;
                   end
                   Slice_Wp{sliceidx,slice_count} = slice_wp;
                   Slice_Info{sliceidx,slice_count} = slice_info;
                   %==========================
                   slice_info = [serow, secol, ssrow, secol, 4, row_w*col_w-1];
                   slice_count = slice_count+1;
                   slice_wp = [];
                   direction = -1;
                   for rowidx = 0:row_w-1
                       for colidx = 0:col_w-1
                           if direction == 1
                               slice_wp = [slice_wp; serow-rowidx, sscol+colidx];
                           elseif direction == -1
                               slice_wp = [slice_wp; serow-rowidx, secol-colidx];
                           end
                       end
                       direction = -direction;
                   end
                   Slice_Wp{sliceidx,slice_count} = slice_wp;
                   Slice_Info{sliceidx,slice_count} = slice_info;
                   %==========================
                   slice_info = [serow, secol, serow, sscol, 1, row_w*col_w-1];
                   slice_count = slice_count+1;
                   slice_wp = [];
                   direction = -1;
                   for colidx = 0:col_w-1
                       for rowidx = 0:row_w-1
                           if direction == 1
                               slice_wp = [slice_wp; ssrow+rowidx, secol-colidx];
                           elseif direction == -1
                               slice_wp = [slice_wp; serow-rowidx, secol-colidx];
                           end
                       end
                       direction = -direction;
                   end
                   Slice_Wp{sliceidx,slice_count} = slice_wp;
                   Slice_Info{sliceidx,slice_count} = slice_info;
                   %==========================
                   slice_info = [serow, sscol, ssrow, sscol, 2, row_w*col_w-1];
                   slice_count = slice_count+1;
                   slice_wp = [];
                   direction = 1;
                   for rowidx = 0:row_w-1
                       for colidx = 0:col_w-1
                           if direction == 1
                               slice_wp = [slice_wp; ssrow+rowidx, sscol+colidx];
                           elseif direction == -1
                               slice_wp = [slice_wp; ssrow+rowidx, secol-colidx];
                           end
                       end
                       direction = -direction;
                   end
                   Slice_Wp{sliceidx,slice_count} = slice_wp;
                   Slice_Info{sliceidx,slice_count} = slice_info;
                   %==========================
                   slice_info = [serow, sscol, serow, secol, 1, row_w*col_w-1];
                   slice_count = slice_count+1;
                   slice_wp = [];
                   direction = -1;
                   for colidx = 0:col_w-1
                       for rowidx = 0:row_w-1
                           if direction == 1
                               slice_wp = [slice_wp; ssrow+rowidx, sscol+colidx];
                           elseif direction == -1
                               slice_wp = [slice_wp; serow-rowidx, sscol+colidx];
                           end
                       end
                       direction = -direction;
                   end
                   Slice_Wp{sliceidx,slice_count} = slice_wp;
                   Slice_Info{sliceidx,slice_count} = slice_info;
            end
        end
        % Row is odd & col is even
        if mod(row_w,2) == 1
            if mod(col_w,2) == 0
                if row_w == 1 
                   slice_info = [ssrow, sscol, serow, secol, 2, secol-sscol];
                   slice_count = slice_count+1;
                   slice_wp = [];
                   for rowidx = 0:col_w-1
                     slice_wp = [slice_wp; ssrow, sscol+rowidx];
                   end
                   Slice_Wp{sliceidx,slice_count} = slice_wp;
                   Slice_Info{sliceidx,slice_count} = slice_info;
                   %==========================
                   slice_info = [serow, secol, ssrow, sscol, 4, secol-sscol];
                   slice_count = slice_count+1;
                   slice_wp = [];
                   for rowidx = 0:col_w-1
                     slice_wp = [slice_wp; serow, secol-rowidx];
                   end
                   Slice_Wp{sliceidx,slice_count} = slice_wp;
                   Slice_Info{sliceidx,slice_count} = slice_info;
                   %==========================
                else
                   slice_info = [ssrow, sscol, ssrow, secol, 3, row_w*col_w-1];
                   slice_count = slice_count+1;
                   slice_wp = [];
                   direction = 1;
                   for colidx = 0:col_w-1
                       for rowidx = 0:row_w-1
                           if direction == 1
                               slice_wp = [slice_wp; ssrow+rowidx, sscol+colidx];
                           elseif direction == -1
                               slice_wp = [slice_wp; serow-rowidx, sscol+colidx];
                           end
                       end
                       direction = -direction;
                   end
                   Slice_Wp{sliceidx,slice_count} = slice_wp;
                   Slice_Info{sliceidx,slice_count} = slice_info;
                   %==========================
                   slice_info = [ssrow, sscol, serow, secol, 2, row_w*col_w-1];
                   slice_count = slice_count+1;
                   slice_wp = [];
                   direction = 1;
                   for rowidx = 0:row_w-1
                       for colidx = 0:col_w-1
                           if direction == 1
                               slice_wp = [slice_wp; ssrow+rowidx, sscol+colidx];
                           elseif direction == -1
                               slice_wp = [slice_wp; ssrow+rowidx, secol-colidx];
                           end
                       end
                       direction = -direction;
                   end
                   Slice_Wp{sliceidx,slice_count} = slice_wp;
                   Slice_Info{sliceidx,slice_count} = slice_info;
                   %==========================
                   slice_info = [ssrow, secol, serow, sscol, 4, row_w*col_w-1];
                   slice_count = slice_count+1;
                   slice_wp = [];
                   direction = -1;
                   for rowidx = 0:row_w-1
                       for colidx = 0:col_w-1
                           if direction == 1
                               slice_wp = [slice_wp; ssrow+rowidx, sscol+colidx];
                           elseif direction == -1
                               slice_wp = [slice_wp; ssrow+rowidx, secol-colidx];
                           end
                       end
                       direction = -direction;
                   end
                   Slice_Wp{sliceidx,slice_count} = slice_wp;
                   Slice_Info{sliceidx,slice_count} = slice_info;
                   %==========================
                   slice_info = [ssrow, secol, ssrow, sscol, 3, row_w*col_w-1];
                   slice_count = slice_count+1;
                   slice_wp = [];
                   direction = 1;
                   for colidx = 0:col_w-1
                       for rowidx = 0:row_w-1
                           if direction == 1
                               slice_wp = [slice_wp; ssrow+rowidx, secol-colidx];
                           elseif direction == -1
                               slice_wp = [slice_wp; serow-rowidx, secol-colidx];
                           end
                       end
                       direction = -direction;
                   end
                   Slice_Wp{sliceidx,slice_count} = slice_wp;
                   Slice_Info{sliceidx,slice_count} = slice_info;
                   %==========================
                   slice_info = [serow, secol, ssrow, sscol, 4, row_w*col_w-1];
                   slice_count = slice_count+1;
                   slice_wp = [];
                   direction = -1;
                   for rowidx = 0:row_w-1
                       for colidx = 0:col_w-1
                           if direction == 1
                               slice_wp = [slice_wp; serow-rowidx, sscol+colidx];
                           elseif direction == -1
                               slice_wp = [slice_wp; serow-rowidx, secol-colidx];
                           end
                       end
                       direction = -direction;
                   end
                   Slice_Wp{sliceidx,slice_count} = slice_wp;
                   Slice_Info{sliceidx,slice_count} = slice_info;
                   %==========================
                   slice_info = [serow, secol, serow, sscol, 1, row_w*col_w-1];
                   slice_count = slice_count+1;
                   slice_wp = [];
                   direction = -1;
                   for colidx = 0:col_w-1
                       for rowidx = 0:row_w-1
                           if direction == 1
                               slice_wp = [slice_wp; ssrow+rowidx, secol-colidx];
                           elseif direction == -1
                               slice_wp = [slice_wp; serow-rowidx, secol-colidx];
                           end
                       end
                       direction = -direction;
                   end
                   Slice_Wp{sliceidx,slice_count} = slice_wp;
                   Slice_Info{sliceidx,slice_count} = slice_info;
                   %==========================
                   slice_info = [serow, sscol, ssrow, secol, 2, row_w*col_w-1];
                   slice_count = slice_count+1;
                   slice_wp = [];
                   direction = 1;
                   for rowidx = 0:row_w-1
                       for colidx = 0:col_w-1
                           if direction == 1
                               slice_wp = [slice_wp; serow-rowidx, sscol+colidx];
                           elseif direction == -1
                               slice_wp = [slice_wp; serow-rowidx, secol-colidx];
                           end
                       end
                       direction = -direction;
                   end
                   Slice_Wp{sliceidx,slice_count} = slice_wp;
                   Slice_Info{sliceidx,slice_count} = slice_info;
                   %==========================
                   slice_info = [serow, sscol, serow, secol, 1, row_w*col_w-1];
                   slice_count = slice_count+1;
                   slice_wp = [];
                   direction = -1;
                   for colidx = 0:col_w-1
                       for rowidx = 0:row_w-1
                           if direction == 1
                               slice_wp = [slice_wp; ssrow+rowidx, sscol+colidx];
                           elseif direction == -1
                               slice_wp = [slice_wp; serow-rowidx, sscol+colidx];
                           end
                       end
                       direction = -direction;
                   end
                   Slice_Wp{sliceidx,slice_count} = slice_wp;
                   Slice_Info{sliceidx,slice_count} = slice_info;
                end
            end
        end

        % Row is even & col is odd
        if mod(row_w,2) == 0
            if mod(col_w,2) == 1
                if col_w == 1
                   slice_info = [ssrow, sscol, serow, secol, 3, serow-ssrow];
                   slice_count = slice_count+1;
                   slice_wp = [];
                   for rowidx = 0:row_w-1
                     slice_wp = [slice_wp; ssrow+rowidx, sscol];
                   end
                   Slice_Wp{sliceidx,slice_count} = slice_wp;
                   Slice_Info{sliceidx,slice_count} = slice_info;
                   %==========================
                   slice_info = [serow, secol, ssrow, sscol, 1, serow-ssrow];
                   slice_count = slice_count+1;
                   slice_wp = [];
                   for rowidx = 0:row_w-1
                     slice_wp = [slice_wp; serow-rowidx, secol];
                   end
                   Slice_Wp{sliceidx,slice_count} = slice_wp;
                   Slice_Info{sliceidx,slice_count} = slice_info;
                   %==========================
                else
                   slice_info = [ssrow, sscol, serow, secol, 3, row_w*col_w-1];
                   slice_count = slice_count+1;
                   slice_wp = [];
                   direction = 1;
                   for colidx = 0:col_w-1
                       for rowidx = 0:row_w-1
                           if direction == 1
                               slice_wp = [slice_wp; ssrow+rowidx, sscol+colidx];
                           elseif direction == -1
                               slice_wp = [slice_wp; serow-rowidx, sscol+colidx];
                           end
                       end
                       direction = -direction;
                   end
                   Slice_Wp{sliceidx,slice_count} = slice_wp;
                   Slice_Info{sliceidx,slice_count} = slice_info;
                   %==========================
                   slice_info = [ssrow, sscol, serow, sscol, 2, row_w*col_w-1];
                   slice_count = slice_count+1;       
                   slice_wp = [];
                   direction = 1;
                   for rowidx = 0:row_w-1
                       for colidx = 0:col_w-1
                           if direction == 1
                               slice_wp = [slice_wp; ssrow+rowidx, sscol+colidx];
                           elseif direction == -1
                               slice_wp = [slice_wp; ssrow+rowidx, secol-colidx];
                           end
                       end
                       direction = -direction;
                   end
                   Slice_Wp{sliceidx,slice_count} = slice_wp;
                   Slice_Info{sliceidx,slice_count} = slice_info;
                   %==========================
                   slice_info = [ssrow, secol, serow, secol, 4, row_w*col_w-1];
                   slice_count = slice_count+1;
                   slice_wp = [];
                   direction = -1;
                   for rowidx = 0:row_w-1
                       for colidx = 0:col_w-1
                           if direction == 1
                               slice_wp = [slice_wp; ssrow+rowidx, sscol+colidx];
                           elseif direction == -1
                               slice_wp = [slice_wp; ssrow+rowidx, secol-colidx];
                           end
                       end
                       direction = -direction;
                   end
                   Slice_Wp{sliceidx,slice_count} = slice_wp;
                   Slice_Info{sliceidx,slice_count} = slice_info;
                   %==========================
                   slice_info = [ssrow, secol, serow, sscol, 3, row_w*col_w-1];
                   slice_count = slice_count+1;
                   slice_wp = [];
                   direction = 1;
                   for colidx = 0:col_w-1
                       for rowidx = 0:row_w-1
                           if direction == 1
                               slice_wp = [slice_wp; ssrow+rowidx, secol-colidx];
                           elseif direction == -1
                               slice_wp = [slice_wp; serow-rowidx, secol-colidx];
                           end
                       end
                       direction = -direction;
                   end
                   Slice_Wp{sliceidx,slice_count} = slice_wp;
                   Slice_Info{sliceidx,slice_count} = slice_info;
                   %==========================
                   slice_info = [serow, secol, ssrow, secol, 4, row_w*col_w-1];
                   slice_count = slice_count+1;
                   slice_wp = [];
                   direction = -1;
                   for rowidx = 0:row_w-1
                       for colidx = 0:col_w-1
                           if direction == 1
                               slice_wp = [slice_wp; serow-rowidx, sscol+colidx];
                           elseif direction == -1
                               slice_wp = [slice_wp; serow-rowidx, secol-colidx];
                           end
                       end
                       direction = -direction;
                   end
                   Slice_Wp{sliceidx,slice_count} = slice_wp;
                   Slice_Info{sliceidx,slice_count} = slice_info;
                   %==========================
                   slice_info = [serow, secol, ssrow, sscol, 1, row_w*col_w-1];
                   slice_count = slice_count+1;
                   slice_wp = [];
                   direction = -1;
                   for colidx = 0:col_w-1
                       for rowidx = 0:row_w-1
                           if direction == 1
                               slice_wp = [slice_wp; ssrow+rowidx, secol-colidx];
                           elseif direction == -1
                               slice_wp = [slice_wp; serow-rowidx, secol-colidx];
                           end
                       end
                       direction = -direction;
                   end
                   Slice_Wp{sliceidx,slice_count} = slice_wp;
                   Slice_Info{sliceidx,slice_count} = slice_info;
                   %==========================
                   slice_info = [serow, sscol, ssrow, sscol, 2, row_w*col_w-1];
                   slice_count = slice_count+1;                  
                   slice_wp = [];
                   direction = 1;
                   for rowidx = 0:row_w-1
                       for colidx = 0:col_w-1
                           if direction == 1
                               slice_wp = [slice_wp; serow-rowidx, sscol+colidx];
                           elseif direction == -1
                               slice_wp = [slice_wp; serow-rowidx, secol-colidx];
                           end
                       end
                       direction = -direction;
                   end
                   Slice_Wp{sliceidx,slice_count} = slice_wp;
                   Slice_Info{sliceidx,slice_count} = slice_info;
                   %==========================
                   slice_info = [serow, sscol, ssrow, secol, 1, row_w*col_w-1];
                   slice_count = slice_count+1;
                   slice_wp = [];
                   direction = -1;
                   for colidx = 0:col_w-1
                       for rowidx = 0:row_w-1
                           if direction == 1
                               slice_wp = [slice_wp; ssrow+rowidx, sscol+colidx];
                           elseif direction == -1
                               slice_wp = [slice_wp; serow-rowidx, sscol+colidx];
                           end
                       end
                       direction = -direction;
                   end
                   Slice_Wp{sliceidx,slice_count} = slice_wp;
                   Slice_Info{sliceidx,slice_count} = slice_info;
                   %==========================
                end
            end
        end
        Slice_Info_size = [Slice_Info_size; slice_count];
    end
end