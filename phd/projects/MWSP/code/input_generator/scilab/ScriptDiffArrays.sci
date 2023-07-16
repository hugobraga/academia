function ScriptDiffArrays(tmpdir, tmpfile)
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

	 sizes=[10, 20, 50];
	 // sizes=[16, 32, 64];
	 qt_edges=[15, 20, 0, 0, 0, 0, 0, 0; 30, 40, 0, 0, 0, 0, 0, 0; 0, 0, 0, 0, 0, 0, 0, 0];
	 nome_param = ["edge"];
	 // sizes_dens = (sizes, edges_perc);
	 // edges_perc=[4, 8];
	 // edges_perc=[4, 8];
	 // edge_type = [];
	 // edge_type.append("unit");
	 edge_type = ['unit', 'random', 'euclidean'];
	 //edge_type = ['unit', 'random', 'euclidean'];
	 // add_edge_type = [];
	 // add_edge_type.append("random");
	 add_edge_type = ['random'];
	 qtInst = 20;
	 lado = 100;

	mat_size = size(qt_edges);
	 for ec = 1:size(edge_type, 2)
	   for ae = 1:size(add_edge_type, 2)
	     for j = 1:length(sizes)
	       for k = 1:mat_size(2)//numero de colunas
	       //for k = 1:1
	         if (qt_edges(j,k) > 0)
		   sum_degree = 0;
		   for i = 1:qtInst 
		     //disp(strcat(["size2: ", string(sizes(j)), ", perc: ", string(edges_perc(j,k)), ", inst: ", string(i)]));
		     c = 0;
		     g = [];
		     d = [];
		     count = 1;
		     disp("ae: " + string(ae));
		     while (c == 0) do //enquanto o grafo não é conexo
			   // perc = edges_perc(j,k)/(sizes(j)-1);
			   perc = (2 * qt_edges(j,k))/(sizes(j) * (sizes(j)-1));
			   if (qt_edges(j,k) < 1)
			      perc = qt_edges(j,k);
			   end;
			   [g,d,c]=AddingEdgesAtRandom(sizes(j), perc);

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
		     disp(string(_degree));

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

		     qt_edges_str = '';
		     if (qt_edges(j,k) < 1)
			qt_edges_str = string(qt_edges(j,k)*100);
		     else
			  qt_edges_str = string(qt_edges(j,k));
		     end;

		     WritingGraphEdgeByEdge(tmpdir, strcat([string(i), "-", qt_edges_str, "-", string(sizes(j)), "-", nome_param(1), "-", edge_type(ec), "-", add_edge_type(ae), "-", tmpfile]),sizes(j),ne,node_node,cost);		   


		   end;
		   disp("media das instancias, grau: " + string(sum_degree/qtInst));

		 end;
	       end;
	     end;
	   end;
	 end;

endfunction