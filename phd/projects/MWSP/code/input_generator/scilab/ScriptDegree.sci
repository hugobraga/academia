function ScriptDegree(tmpdir, tmpfile)
	 script_dir = '~/Dropbox/USP/git/doutorado/projetos/MWSP/implementacao/gerar_entrada/scilab/';
	 // script_dir = '~/Dropbox/documentos/';
	 exec (script_dir+'EdgesRandomCost.sci');
	 exec (script_dir+'EdgesRandomCostIntInterv.sci');
	 exec (script_dir+'EdgesUnitCost.sci');
	 exec (script_dir+'EdgesEuclideanCost2.sci');
	 exec (script_dir+'AddingEdgesWaxman.sci');
	 exec (script_dir+'AddingEdgesAtRandom.sci');
	 exec (script_dir+'WritingGraph.sci');
	 exec (script_dir+'WritingGraphEdgeByEdge.sci');
	 exec (script_dir+'NtgWaxman2.sci');

	 sizes=[80, 90, 100];
	 nome_param = ["edge"];
	 degrees=[4, 8];
	 edge_type = ['random', 'euclidean'];
	 add_edge_type = ['random'];
	 qtInst = 20;
	 lado = 100;

	 for ec = 1:size(edge_type, 2)
	   for ae = 1:size(add_edge_type, 2)
	     for j = 1:length(sizes)
	     //se vai utilizar graus ou qt de arestas
	       //--
	       for k = 1:length(degrees)//numero de colunas
		   sum_degree = 0;
		   for i = 1:qtInst 
		     c = 0;
		     g = [];
		     d = [];
		     count = 1;
		     disp("ae: " + string(ae));
		     while (c == 0) do //enquanto o grafo não é conexo
		     	   disp("buscando grafo conexo");
			   disp("grau: " + string(degrees(k)) + ", tam: " + string(sizes(j)));
			   perc = (degrees(k) * sizes(j))/(sizes(j) * (sizes(j)-1));
			   disp("perc: " + string(perc));
			   disp("tam: " + string(sizes(j)));
			   [g,d,c]=AddingEdgesAtRandom(sizes(j), perc);

			   count = count + 1;
		     end;

		     _sum = 0;
		     for c = 1:sizes(j)
			 _sum = _sum + d(c);
		     end
		     _degree = _sum/sizes(j);
		     sum_degree = sum_degree + _degree;
		     disp(string(_degree));

		     node_node = graph_2_mat(g,'node-node');
		     random_type = ["random"];
		     res = strcmp(edge_type(ec), random_type);
		     if (res == 0) then
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

		     degrees_str = '';
		     
		     //outra parte para ajustar o grau
		     degrees_str = string(degrees(k));

		     WritingGraphEdgeByEdge(tmpdir, strcat([string(i), "-", degrees_str, "-", string(sizes(j)), "-", edge_type(ec), "-", add_edge_type(ae), "-", tmpfile]),sizes(j),ne,node_node,cost);

		   end;
		   disp("media das instancias, grau: " + string(sum_degree/qtInst));

	       end;
	     end;
	   end;
	 end;

endfunction