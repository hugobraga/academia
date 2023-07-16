function ScriptWaxman(tmpdir, tmpfile)
	 script_dir = '~/Dropbox/USP/git/doutorado/projetos/MWSP/implementacao/gerar_entrada/scilab/';
	 exec (script_dir+'EdgesRandomCost.sci');
	 exec (script_dir+'EdgesRandomCostIntInterv.sci');
	 exec (script_dir+'EdgesUnitCost.sci');
	 exec (script_dir+'EdgesEuclideanCost2.sci');
	 exec (script_dir+'AddingEdgesWaxman.sci');
	 exec (script_dir+'AddingEdgesAtRandom.sci');
	 exec (script_dir+'WritingGraph.sci');
	 exec (script_dir+'WritingGraphEdgeByEdge.sci');
	 exec (script_dir+'NtgWaxman2.sci');

	 sizes=[16, 32, 64, 16, 32, 64];
	 edges_perc=[4, 4, 4, 8, 8, 8];
	 // edge_type = [];
	 // edge_type.append("unit", "random", "euclidean");
	 edge_type = ['unit', 'random', 'euclidean'];
	 // add_edge_type = [];
	 // add_edge_type.append("random");
	 add_edge_type = ['waxman'];
	 qtInst = 50;
	 //waxman parameters
	 b = 0.14
	 a = [3.8, 1.52, 0.71, 18, 3.8, 1.55]
	 lado = 100

	 for ec = 1:size(edge_type, 2)
	   for ae = 1:size(add_edge_type, 2)
	     for j = 1:length(sizes)
	       //for k = 1:size(edges_perc,2)
	       for k = 1:1
		 sum_degree = 0;
		 for i = 1:qtInst 
		   //disp(strcat(["size2: ", string(sizes(j)), ", perc: ", string(edges_perc(k)), ", inst: ", string(i)]));
		   c = 0;
		   g = [];
		   d = [];
		   count = 1;
		   //disp("ae: " + string(ae));
		   while (c == 0) do //enquanto o grafo não é conexo

			 [g,d,c]=AddingEdgesWaxman(a(j), b, sizes(j), lado);

			 //nc = strong_connex(g);
			 //disp("componentes: " + string(nc));
			 count = count + 1;
		   end;

		   _sum = 0;
		   for c = 1:sizes(j)
		       _sum = _sum + d(c);
		       // disp("grau de " + string(c-1) + ": " + string(d(c)));
		   end
		   _degree = _sum/sizes(j);
		   sum_degree = sum_degree + _degree;
		   //disp("grau medio: " + string(_degree));

		   node_node = graph_2_mat(g,'node-node');
		   random_type = ["random"];
		   res = strcmp(edge_type(ec), random_type);
		   if (res == 0) then
		     //[cost,no_edge_const] = EdgesRandomCostIntInterv(sizes(j),node_node);
		     [cost,no_edge_const] = EdgesRandomCost(sizes(j),node_node);
		   else 
		     unit_type = ["unit"];
		     res = strcmp(edge_type(ec), unit_type);
		     if (res == 0) then //unit
		       [cost,no_edge_const] = EdgesUnitCost(sizes(j),node_node);
		     else //euclidean
		       [cost,no_edge_const] = EdgesEuclideanCost2(sizes(j),node_node, lado);
		     end		   
		   end;
		   ne = edge_number(g);
		   edge_cost = [];
		   for e = 1:ne
		       temp_cost = cost(g.edges(e).head,g.edges(e).tail);
		       edge_cost(e) = temp_cost;
		   end
		   edge_cost = edge_cost'; //transposta
		   g = add_edge_data(g,'length',edge_cost);

		   WritingGraphEdgeByEdge(tmpdir, strcat([string(i), "-", string(edges_perc(j)), "-", string(sizes(j)), "-", edge_type(ec), "-", add_edge_type(ae), "-", tmpfile]),sizes(j),ne,node_node,cost);

		 end;
		 disp("media das instancias, grau: " + string(sum_degree/qtInst));
	       end;
	     end;
	   end;
	 end;

endfunction