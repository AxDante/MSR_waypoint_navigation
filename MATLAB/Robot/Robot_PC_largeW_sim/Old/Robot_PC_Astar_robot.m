%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% A* ALGORITHM Demo
% Interactive A* search demo
% 04-26-2005
%   Copyright 2009-2010 The MathWorks, Inc.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
addpath('C:\Users\IceFox\Desktop\ERMINE\MATLAB\Robot_PC')
addpath('C:\Users\IceFox\Desktop\ERMINE\MATLAB\Robot_PC\Maps')
%function Wp = Robot_PC_Astar_robot(gs, Gobs, )

gs = [10 10];
Gobs = zeros(gs(1),gs(2));
MAX_VAL=10;
shape = 8;

Gobs(3,5) = 1;
Gobs(4,5) = 1;
Gobs(5,5) = 1;
Gobs(6,5) = 1;
Gobs(8,3) = 1;
Gobs(3,7) = 1;

scg = [1 2]; % start center grid
gcg = [7 7]; % goal center grid
rcg = scg;
MAP=2*(ones(gs(1),gs(2)));

% Obtain Obstacle, Target and Robot Position
% Initialize the MAP with input values
% Obstacle=-1,Target = 0,Robot=1,Space=2
j=0;
x_val = 1;
y_val = 1;
axis([1 gs(1)+1 1 gs(2)+1])
grid on;
hold on;
n=0;%Number of Obstacles

MAP(gcg(1),gcg(2))=0;%Initialize MAP with location of the target
plot(gcg(1)+.5,gcg(2)+.5,'gd');
text(gcg(1)+1,gcg(2)+.5,'Target')

for idxx = 1:gs(1)
    for idxy = 1:gs(2)
        if Gobs(idxx, idxy) == 1
            MAP(idxx,idxy) = -1;
            plot(idxx+0.5, idxy+0.5, 'ro');
        end
    end
end
 

MAP(scg(1),scg(2))=1;
plot(scg(1)+0.5,scg(2)+0.5,'bo');
%End of obstacle-Target pickup

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%LISTS USED FOR ALGORITHM
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%OPEN LIST STRUCTURE
%--------------------------------------------------------------------------
%IS ON LIST 1/0 |X val |Y val |Parent X val |Parent Y val |h(n) |g(n)|f(n)|
%--------------------------------------------------------------------------
OPEN=[];
%CLOSED LIST STRUCTURE
%--------------
%X val | Y val |
%--------------
% CLOSED=zeros(MAX_VAL,2);
CLOSED=[];

%Put all obstacles on the Closed list
k=1;%Dummy counter
for i=1:gs(1)
    for j=1:gs(2)
        if(MAP(i,j) == -1)
            CLOSED(k,1)=i; 
            CLOSED(k,2)=j; 
            k=k+1;
        end
    end
end
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

%for idxclo = 1:size(CLOSED,1)
%    Gobs(CLOSED(idxclo,1), CLOSED(idxclo,2)) = 1;
%end
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% START ALGORITHM
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
while((rcg(1) ~= gcg(1) || rcg(2) ~= gcg(2)) && NoPath == 1)
%  plot(rcg(1)+.5,rcg(2)+.5,'go');
 exp_array=Robot_PC_expand_array(rcg,path_cost,gcg,CLOSED,gs,shape,Gobs);
 exp_count=size(exp_array,1);
 %UPDATE LIST OPEN WITH THE SUCCESSOR NODES
 %OPEN LIST FORMAT
 %--------------------------------------------------------------------------
 %IS ON LIST 1/0 |X val |Y val |Parent X val |Parent Y val |h(n) |g(n)|f(n)|
 %--------------------------------------------------------------------------
 %EXPANDED ARRAY FORMAT
 %--------------------------------
 %|X val |Y val ||h(n) |g(n)|f(n)|
 %--------------------------------
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
            end;%End of minimum fn check
            flag=1;
        end;%End of node check
%         if flag == 1
%             break;
    end;%End of j for
    if flag == 0
        OPEN_COUNT = OPEN_COUNT+1;
        OPEN(OPEN_COUNT,:)=insert_open(exp_array(i,1),exp_array(i,2),rcg(1),rcg(2),exp_array(i,3),exp_array(i,4),exp_array(i,5));
     end;%End of insert new element into the OPEN list
 end;%End of i for
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %END OF WHILE LOOP
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
  end;%End of index_min_node check
end;%End of While Loop
%Once algorithm has run The optimal path is generated by starting of at the
%last node(if it is the target node) and then identifying its parent node
%until it reaches the start node.This is the optimal path

i=size(CLOSED,1);
Optimal_path=[];
rcg(1)=CLOSED(i,1);
rcg(2)=CLOSED(i,2);
i=1;
Optimal_path(i,1)=rcg(1);
Optimal_path(i,2)=rcg(2);
i=i+1;

if ( (rcg(1) == gcg(1)) && (rcg(2) == gcg(2)))
    inode=0;
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
    end;
 j=size(Optimal_path,1);
 %Plot the Optimal Path!
 p=plot(Optimal_path(j,1)+.5,Optimal_path(j,2)+.5,'bo');
 j=j-1;
 for i=j:-1:1
  pause(.25);
  set(p,'XData',Optimal_path(i,1)+.5,'YData',Optimal_path(i,2)+.5);
 drawnow ;
 end;
 plot(Optimal_path(:,1)+.5,Optimal_path(:,2)+.5);
else
 pause(1);
 h=msgbox('Sorry, No path exists to the Target!','warn');
 uiwait(h,5);
end

    





