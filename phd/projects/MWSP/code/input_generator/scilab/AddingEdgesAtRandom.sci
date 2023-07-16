function [g,d,c]=AddingEdgesAtRandom(n, per)
	 g = make_graph('temp', 0, n, [1], [1]);
	 d = [];
	 d = zeros(n,1);
	 for _i = 1:n do
	     temp = _i+1;
	     dist(_i,_i) = 0;
	     for _j = temp:n do
	     	 flag = rand();
		 // disp(string(_i) + "-" + string(_j) + ", rand: ");
		 // disp(flag);
		 if (flag < per) then
		    g = add_edge([_i], [_j], g);
		    d(_i) = d(_i) + 1;
		    d(_j) = d(_j) + 1;
		 end;
	     end;
	 end
	 edge = [1 1];
	 g = delete_edges(edge, g);

	c = 0;
	// disp(g);
	if ((edge_number(g) > 0) & (is_connex(g)==1))
	   c = 1;
	end 

endfunction
