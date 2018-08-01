function [runs, Wp_series] = MultipleRunImport_veera(file, gw)

    runs = 5;
                    
    Wp_series_init(1,:,:)=  [1 1 6;  
            4 1 27;     %27
            5 1 5+7;
            3 3 7+7;
            3 4 5;
            1 4 6;
            1 7 6;
            3 7 5;
            4 5 5+7;
            5 8 5;
            3 9 7+7;
            1 10 13;
            6 10 13;
            6 7 6;
            5 7 21;
            5 3 14;
            6 2 14;
            9 1 27;
            9 3 21;
            8 6 20;
            9 5 21;
            9 7 21;
            8 7 5;
            8 9 14;
            10 9 20];                   
   
   Wp_series(1,:,1) =  Wp_series_init(1,:,2); 
   Wp_series(1,:,2) =  11-Wp_series_init(1,:,1); 
   Wp_series(1,:,3) =  Wp_series_init(1,:,3);       
        
   Wp_series(:,:,1:2) = (Wp_series(:, :,1:2)-0.5)*gw;        
                    
end