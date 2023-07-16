function WritingGraph(_dir,name,n,node_node,cost,no_edge_const)
	 fd = mopen(_dir+name,'wt');
	 mfprintf(fd,'[');
	 //mfprintf(fd_dist,'[');
	 for i = 1:n do
	     row = '[';
	     for j = 1:n do
	     	 if (node_node(i,j) == 1) then //existe aresta
		    row = strcat([row, string(cost(i,j))]);
		 else row = strcat([row, string(no_edge_const)]);
		 end;

		 if (j < n) then 
		    row = strcat([row, ', ']);
		 end;
	     end;
	     row = strcat([row, ']']);

	     if (i < n) then 
	     	row = strcat([row, ',']);
	     end;
	     mfprintf(fd,'%s\n', row);
	 end
	 mfprintf(fd,']');
	 mclose(fd);
endfunction