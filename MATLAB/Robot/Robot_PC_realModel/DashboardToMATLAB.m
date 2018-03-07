
txt_endLine_new = [0 0];

is_initialized = false;
is_grid_on = true;


figure(1)
hold on
axis([-grid_w grid_w*11 -grid_w grid_w*11])
if (is_grid_on)
    for idxx = 1:(grid_size(1) + 1)
        line(grid_w*[(idxx-1) (idxx-1)], grid_w*[0 grid_size(2)], 'Color', 'black');
    end
    for idxy = 1:(grid_size(2) + 1)
        line(grid_w*[0 grid_size(1)], grid_w*[(idxy-1) (idxy-1)], 'Color', 'black');
    end
end


for intidx = 1:400000
    fid = fopen('C:\Marvelmind\dashboard\logs\TestLog.txt','rt');
    txt_Streaming = textscan(fid, '%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f', 'delimiter', ',','collectoutput',true...
          , 'HeaderLines', 10);
    txt_Streaming=txt_Streaming{1};
    fclose(fid);
    txt_rows=size(txt_Streaming,1);
    txt_endLine = txt_Streaming(end, 5:6);
    
    if (~is_initialized && (txt_endLine(1) ~= 0 &&  ~isnan(txt_endLine(1))  &&  txt_endLine(2) ~= 0 &&  ~isnan(txt_endLine(2))))
        txt_endLine_new = txt_endLine;
        is_initialized = true;
    elseif  (txt_endLine(1) ~= 0 &&  ~isnan(txt_endLine(1))  &&  txt_endLine(2) ~= 0 &&  ~isnan(txt_endLine(2)) )
        if (txt_endLine_new(1) ~= txt_endLine(1) || txt_endLine_new(2) ~= txt_endLine(2)) 
            if norm(txt_endLine_new - txt_endLine) < 0.4
                line([txt_endLine_new(1) txt_endLine(1)], [txt_endLine_new(2) txt_endLine(2)]);
                txt_endLine_new = txt_endLine;
            end
        end
    end
    pause(0.3)
end

