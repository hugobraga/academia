function [g,degree]=NtgWaxman2(alph,bet,n,L)
	degree=zeros(n,1);
	nodx=XYRand(n,L);
	nody=XYRand(n,L);
	ta=[];
	he=[];
	while (length(ta) == 0)
	for i=1:n
		for j=i+1:n
			d=Distance(i,j,nodx,nody);
			p=alph*exp((-d)/(bet*L));
			if ((rand()<p))
				degree(i)=degree(i)+1;
				degree(j)=degree(j)+1;
				ta=[ta i];
				he=[he j];
			end
		end 
	end
	end
	// disp(ta);
	// disp(he);
	g=make_graph('network',0,n,ta,he);
	g('node_x')=nodx;
	g('node_y')=nody;
	[g]=EdgeLength(g);
endfunction
