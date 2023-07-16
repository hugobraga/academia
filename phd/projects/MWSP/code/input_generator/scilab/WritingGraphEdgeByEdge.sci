function WritingGraphEdgeByEdge(_dir,name,n,ne,node_node,cost)
	 fd = mopen(_dir+name,'wt');	 
	 mfprintf(fd, '%d %d\n', n, ne);
	 for i = 1:n do
	     for j = i+1:n do
	     	 if (node_node(i,j) == 1) then //existe aresta
		    mfprintf(fd, '%d %d %d\n', i-1, j-1, round(cost(i,j)));
		 end;
	     end;
	 end
	 mclose(fd);
endfunction