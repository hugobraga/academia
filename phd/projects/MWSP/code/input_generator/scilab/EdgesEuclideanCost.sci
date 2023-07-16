function [mat,no_edge_const]=EdgesEuclideanCost(n,node_node, dist, chosen_nodes)	 
	 no_edge_const = 999999;
	 mat=[];
	 for i = 1:n do
	     for j = i:n do
	     	 cost = no_edge_const;
		 ni = chosen_nodes(i);
		 nj = chosen_nodes(j);
	     	 if (node_node(i,j) == 1) then
		    cost = dist(ni,nj);
		 end;
		 mat(i,j) = cost;
		 mat(j,i) = cost;
	     end;
	 end
endfunction
