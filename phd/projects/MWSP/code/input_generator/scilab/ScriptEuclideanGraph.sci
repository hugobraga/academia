function ScriptEuclideanGraph(tmpdir, tmpfile)
	 script_dir = '~/Dropbox/USP/git/doutorado/projetos/MWSP/implementacao/gerar_entrada/scilab/';
	 input_dir = script_dir + 'input/';
	 exec (script_dir+'EdgesRandomCost.sci');
	 exec (script_dir+'EdgesRandomCostIntInterv.sci');
	 exec (script_dir+'EdgesUnitCost.sci');
	 exec (script_dir+'EdgesEuclideanCost.sci');
	 exec (script_dir+'AddingEdgesWaxman.sci');
	 exec (script_dir+'AddingEdgesAtRandom.sci');
	 exec (script_dir+'WritingGraph.sci');
	 exec (script_dir+'WritingGraphEdgeByEdge.sci');
	 exec (script_dir+'NtgWaxman2.sci');

	 max_size = 131;
	 sizes=[60, 80, 100, 120, 131];
	 input_file=["131-dist-vlsi.dat", "131-dist-vlsi.dat", "131-dist-vlsi.dat", "131-dist-vlsi.dat", "131-dist-vlsi.dat"];
	 edges_perc=[0.2, 0.4, 0.6, 0.8];
	 edge_type = ["euclidean"];
	 compl_name = ["-131-"];
	 qtInst = 10;

	 for j = 1:length(sizes)
	     for k = 1:length(edges_perc)
	     	 for l = 1:qtInst
		   disp(strcat(["size: ", string(sizes(j)), ", perc: ", string(edges_perc(k)), ", inst: ", string(l)]));
		   dist = fscanfMat(input_dir+input_file(j));

		   c = 0;
		   g = [];
		   d = [];

		   Random_max = grand(1, "prm", 1:max_size);
		   Random_j = Random_max(1:sizes(j));
		   while (c == 0) do //enquanto o grafo não é conexo
			 [g,d,c]=AddingEdgesAtRandom(sizes(j), edges_perc(k));
			 nc = strong_connex(g);
		   end;

		   //atribuindo os custos
		   node_node = graph_2_mat(g,'node-node');
		   [cost,no_edge_const] = EdgesEuclideanCost(sizes(j),node_node, dist, Random_j);

		   // no_edge_const = 999999;
		   // mat=[];
		   // for i = 1:n do
		   //   for j = i:n do
		   //     cost = no_edge_const;
		   //     if (node_node(i,j) == 1) then
		   //       cost = a1(i,j);
		   //     end;
		   //     mat(i,j) = cost;
		   //     mat(j,i) = cost;
		   //   end;
		   // end;

		   //invertando a matriz para adicionar o custo nas arestas
		   ne = edge_number(g);
		   edge_cost = [];
		   for e = 1:ne
		     temp_cost = cost(g.edges(e).head,g.edges(e).tail);	     
		     edge_cost(e) = temp_cost;
		   end
		   edge_cost = edge_cost'; //transposta
		   g = add_edge_data(g,'length',edge_cost);
		   //---------------

		   //WritingGraph(tmpdir, strcat([string(l), "-", string(edges_perc(k)*100), "-", string(sizes(j)), "-", edge_type(1), "-", tmpfile]),sizes(j),node_node,cost,no_edge_const);
		   WritingGraphEdgeByEdge(tmpdir, strcat([string(l), "-", string(edges_perc(k)*100), "-", string(sizes(j)), "-", edge_type(1), compl_name(1), tmpfile]),sizes(j),ne,node_node,cost);

		 end;
	     end;
	 end;

endfunction
