function Wp_series_out = MultipleRunImport_veera(file, gw)

    runs = 5;
    %{                
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
  
   % Zigzag
   Wp_series_init(2,:,:)=  [1 1 2;  
            1 9 2;     
            2 9 2;
            2 1 2;
            3 1 2;
            3 9 2;
            4 9 2;
            4 1 2;
            5 1 2;
            5 9 2;
            6 9 2;
            6 1 2;
            7 1 2;
            7 9 2;
            8 9 2;
            8 1 2;
            9 1 2;
            9 9 2];                        
   
   % Spiral
   Wp_series_init(3,:,:)=  [5 5 2;  
            5 6 2;     
            6 6 2;
            6 4 2;
            4 4 2;
            4 7 2;
            7 7 2;
            7 3 2;
            3 3 2;
            3 8 2;
            8 8 2;
            8 2 2;
            2 2 2;
            2 9 2;
            9 9 2;
            9 1 2;
            1 1 2;
            1 9 2];
    %}
   Wp_series_init = {};
    
   Wp_series_init{1}=  [1 1 6;  
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
  
   % Zigzag
   Wp_series_init{2}=  [1 1 2;  
            1 9 2;     
            2 9 2;
            2 1 2;
            3 1 2;
            3 9 2;
            4 9 2;
            4 1 2;
            5 1 2;
            5 9 2;
            6 9 2;
            6 1 2;
            7 1 2;
            7 9 2;
            8 9 2;
            8 1 2;
            9 1 2;
            9 9 2];                        
   
   % Spiral
   Wp_series_init{3}=  [5 5 2;  
            5 6 2;     
            6 6 2;
            6 4 2;
            4 4 2;
            4 7 2;
            7 7 2;
            7 3 2;
            3 3 2;
            3 8 2;
            8 8 2;
            8 2 2;
            2 2 2;
            2 9 2;
            9 9 2;
            9 1 2;
            1 1 2;
            1 9 2];
    
   [Wp_greedy, total_dis_greedy] = greedypath_veera(Wp_series_init{1});     
   [Wp_rw, total_dis_rw] = randomwalk_veera(Wp_series_init{1}, 1);      
   Wp_series_init{4} = Wp_greedy;
   Wp_series_init{5} = Wp_rw;
   
   for runidx = 1:runs   
       
    Wp = Wp_series_init{runidx};
    Wp_out = [];
    Wp_out(:,1) = Wp(:,2);
    Wp_out(:,2) = 11-Wp(:,1); 
    Wp_out(:,3) = Wp(:,3); 
    Wp_out(:,1:2) = (Wp_out(:,1:2)-0.5)*gw;
    Wp_series_out{runidx} = Wp_out;
   end
end