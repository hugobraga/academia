function [mat,no_edge_const]=EdgesUnitCost(n,node_node)	 
	 no_edge_const = 999999;
	 mat=[];
	 for i = 1:n do
	     for j = i:n do
	     	 cost = no_edge_const;
	     	 if (node_node(i,j) == 1) then
		    cost = 1;
		 end;
		 mat(i,j) = cost;
		 mat(j,i) = cost;
	     end;
	 end
endfunction