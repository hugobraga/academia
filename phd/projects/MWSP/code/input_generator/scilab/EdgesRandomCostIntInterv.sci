function [mat,no_edge_const]=EdgesRandomCostIntInterv(n,node_node)	 
	 no_edge_const = 999999;
	 mat=[];
	 for i = 1:n do
	     for j = i:n do
	     	 cost = no_edge_const;
	     	 if (node_node(i,j) == 1) then
		    cost = ceil(rand(1)*10);
		 end;
		 mat(i,j) = cost;
		 mat(j,i) = cost;
	     end;
	 end
endfunction