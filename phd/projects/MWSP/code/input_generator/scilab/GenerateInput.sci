function [node_node,g,edge_cost]=GenerateInput(n, tmpdir, tmpfile, distfile, degreefile, qtInst)
	 script_dir = '~/Dropbox/USP/git/doutorado/projetos/MWSP/implementacao/gerar_entrada/scilab/';
	 exec (script_dir+'EdgesRandomCost.sci');
	 exec (script_dir+'EdgesRandomCostIntInterv.sci');
	 exec (script_dir+'EdgesUnitCost.sci');
	 exec (script_dir+'AddingEdgesWaxman.sci');
	 exec (script_dir+'AddingEdgesAtRandom.sci');
	 exec (script_dir+'WritingGraph.sci');
	 exec (script_dir+'NtgWaxman2.sci');
	 //a = 0.85;
	 //a = 0.5;
	 a = 0.95;
	 //b = 0.15;
	 b = 0.5;
	 //n = 30;
	 //n = 12; 
	 //l = 400;
	 l = 200;

	 per = 0.4
	 for k = 1:qtInst do

	 c = 0;
	 g = [];
	 d = [];
	 count = 1;
	 while (c == 0) do //enquanto o grafo não é conexo
	       //[g,d,c]=AddingEdgesWaxman(a,b,n,l);
	       [g,d,c]=AddingEdgesAtRandom(n, per);
	       nc = strong_connex(g);
	       disp("num. de componentes: ", nc);
	       disp(d);
	       count = count + 1;
	 end
	 //disp(d);
	 row = '';
	 for i = 1:n do
	     row = strcat([row, ", ", string(d(i))]);
	 end;	 
	 fd = mopen(tmpdir+strcat([string(k), "-", string(per*100), "-", string(n), "-random-", degreefile]),'wt');
	 mfprintf(fd,'%s\n', row);
	 mclose(fd);
	 node_node = graph_2_mat(g,'node-node');
	 //[cost,no_edge_const] = EdgesRandomCostIntInterv(n,node_node);
	 //[cost,no_edge_const] = EdgesRandomCost(n,node_node);
	 [cost,no_edge_const] = EdgesUnitCost(n,node_node);

	 ne = edge_number(g);
	 edge_cost = [];
	 for i = 1:ne do
	     temp_cost = cost(g.edges(i).head,g.edges(i).tail);	     
	     edge_cost(i) = temp_cost;
	 end
	 edge_cost = edge_cost'; //transposta
	 g = add_edge_data(g,'length',edge_cost);

  	 dist = [];
	 for i = 1:n do
	     temp = i+1;
	     dist(i,i) = 0;
	     for j = temp:n do
	     	 [p,lp]=shortest_path(i,j,g,'length');
		 if (size(p) == 1) then //existe aresta entre os vertices
		    dist(i,j) = cost(i,j);
		    dist(j,i) = cost(j,i);
		 else
		    dist(i,j) = lp;
		    dist(j,i) = lp;
		 end;
	     end;
	 end

	 //disp("per*100: ", string(per*100));
	 WritingGraph(tmpdir, strcat([string(k), "-", string(per*100), "-", string(n), "-unit-", tmpfile]),n,node_node,cost,no_edge_const);

	 end
endfunction