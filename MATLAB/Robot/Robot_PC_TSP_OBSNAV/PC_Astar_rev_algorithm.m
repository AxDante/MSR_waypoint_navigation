%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% A* ALGORITHM Demo
% Interactive A* search demo
% 04-26-2005
% Copyright 2009-2010 The MathWorks, Inc.
% 
% Revised by Ping-Cheng Ku on  04-04-2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function Wp = PC_Astar_rev_algorithm(gs, Gobs, shape, scg, gcg)
    
    %Linear Motion
    rcg = scg;
    Map=2*(ones(gs(1),gs(2)));
    Map(gcg(1),gcg(2))=0;%Initialize MAP with location of the target

    
    for idxx = 1:gs(1)
        for idxy = 1:gs(2)
            if Gobs(idxx, idxy) == 1
                Map(idxx,idxy) = -1;
            end
        end
    end

    %Map (0: goal, -1:obs, 1: start, 2:others)
    
    Map(scg(1),scg(2))=1;
    OPEN=[];
    CLOSED=Gobs;
    
    
    CLOSED_COUNT=size(CLOSED,1);
    %set the starting node as the first node
    OPEN_COUNT=1;
    path_cost=0;
    goal_distance=distance(rcg(1),rcg(2),gcg(1),gcg(2));
    OPEN(OPEN_COUNT,:)=insert_open(rcg(1),rcg(2),rcg(1),rcg(2),path_cost,goal_distance,goal_distance);
    OPEN(OPEN_COUNT,1)=0;
    CLOSED_COUNT=CLOSED_COUNT+1;
    CLOSED(CLOSED_COUNT,1)=rcg(1);
    CLOSED(CLOSED_COUNT,2)=rcg(2);
    NoPath=1;

    while((rcg(1) ~= gcg(1) || rcg(2) ~= gcg(2)) && NoPath == 1)
     [exp_array]=Robot_PC_expand_array_rev(rcg,path_cost,gcg,gs,shape,Gobs);
     exp_count=size(exp_array,1);
     for i=1:exp_count
        flag=0;
        for j=1:OPEN_COUNT
            if(exp_array(i,1) == OPEN(j,2) && exp_array(i,2) == OPEN(j,3) )
                OPEN(j,8)=min(OPEN(j,8),exp_array(i,5)); %#ok<*SAGROW>
                if OPEN(j,8)== exp_array(i,5)
                    %UPDATE PARENTS,gn,hn
                    OPEN(j,4)=rcg(1);
                    OPEN(j,5)=rcg(2);
                    OPEN(j,6)=exp_array(i,3);
                    OPEN(j,7)=exp_array(i,4);
                end%End of minimum fn check
                flag=1;
            end%End of node check
        end%End of j for
        if flag == 0
            OPEN_COUNT = OPEN_COUNT+1;
            OPEN(OPEN_COUNT,:)=insert_open(exp_array(i,1),exp_array(i,2),rcg(1),rcg(2),exp_array(i,3),exp_array(i,4),exp_array(i,5));
        end%End of insert new element into the OPEN list
     end
     %Find out the node with the smallest fn 
      index_min_node = min_fn(OPEN,OPEN_COUNT,gcg(1),gcg(2));
      if (index_min_node ~= -1)    
       %Set rcg(1) and rcg(2) to the node with minimum fn
       rcg(1)=OPEN(index_min_node,2);
       rcg(2)=OPEN(index_min_node,3);
       path_cost=OPEN(index_min_node,6);%Update the cost of reaching the parent node
      %Move the Node to list CLOSED
      CLOSED_COUNT=CLOSED_COUNT+1;
      CLOSED(CLOSED_COUNT,1)=rcg(1);
      CLOSED(CLOSED_COUNT,2)=rcg(2);
      OPEN(index_min_node,1)=0;
      else
          %No path exists to the Target!!
          NoPath=0;%Exits the loop!
      end%End of index_min_node check
    end%End of While Loop

    i=size(CLOSED,1);
    Optimal_path=[];
    rcg(1)=CLOSED(i,1);
    rcg(2)=CLOSED(i,2);
    i=1;
    Optimal_path(i,1)=rcg(1);
    Optimal_path(i,2)=rcg(2);
    i=i+1;

    if ( (rcg(1) == gcg(1)) && (rcg(2) == gcg(2)))
       %Traverse OPEN and determine the parent nodes
       parent_x=OPEN(node_index(OPEN,rcg(1),rcg(2)),4);%node_index returns the index of the node
       parent_y=OPEN(node_index(OPEN,rcg(1),rcg(2)),5);

       while( parent_x ~= scg(1) || parent_y ~= scg(2))
               Optimal_path(i,1) = parent_x;
               Optimal_path(i,2) = parent_y;
               %Get the grandparents:-)
               inode=node_index(OPEN,parent_x,parent_y);
               parent_x=OPEN(inode,4);%node_index returns the index of the node
               parent_y=OPEN(inode,5);
               i=i+1;
       end
       Wp = flipud(Optimal_path);
    else
        Wp = [];
    end

end





