function [mat,no_edge_const]=EdgesRandomCost(n,node_node)	 
	 costs=[1,2,4,8,16];
	 no_edge_const = 999999;
	 mat=[];
	 for i = 1:n do
	     for j = i:n do
	     	 cost = no_edge_const;
	     	 if (node_node(i,j) == 1) then
		    r = rand();
		    if (r < 0.2) then
		       cost = 1;
		    elseif (r < 0.4) then
		       cost = 2;
		    elseif (r < 0.6) then
		       cost = 4;
		    elseif (r < 0.8) then
		       cost = 8;
		    else cost = 16;
		    end;
		    mat(i,j) = cost;
		 else 
		      mat(i,j) = cost;
		      //disp(strcat(['i: ',string(i),', j: ',string(j),', ',string(mat(i,j))]));
		 end;
		 mat(j,i) = cost;
	     end;
	 end
endfunction