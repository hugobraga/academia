function [g,d,c]=AddingEdgesWaxman(a,b,n,l)
	// [g,d]=NtgWaxman2(a,b,n,l);
	[g,d]=NtgWaxman(a,b,n,l);
	c = 0;
	if (is_connex(g)==1)
	   c = 1;
	end 
endfunction
