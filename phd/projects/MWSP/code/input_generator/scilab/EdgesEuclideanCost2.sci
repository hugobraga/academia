function [mat,no_edge_const]=EdgesEuclideanCost2(n,node_node, lado_quad)
	 no_edge_const = 999999;
	 pos=grand(n, 2, "unf", 1, lado_quad);
	 mat=[];
	 for i = 1:n do
	     for j = i:n do
	     	 cost = no_edge_const;
	     	 if (node_node(i,j) == 1) then
		    cost = sqrt((pos(i, 1) - pos(j, 1))^2 + (pos(i, 2) - pos(j, 2))^2);
		 end;
		 mat(i,j) = cost;
		 mat(j,i) = cost;
	     end;
	 end
endfunction
