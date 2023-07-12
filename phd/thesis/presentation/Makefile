BASE_NAME = 181214-defesa

LATEX     = latex
PDFLATEX  = pdflatex
BIBTEX    = bibtex
MAKEINDEX = makeindex


$(BASE_NAME).pdf: $(BASE_NAME).tex 
	$(PDFLATEX) $<
	# $(BIBTEX) $(BASE_NAME)
	$(PDFLATEX) $< 
	$(PDFLATEX) $<

# all:
# 	$(PDFLATEX) $(BASE_NAME).tex

clean:
	rm -f *.aux *.log *.nav *.toc *.snm *.out *.vrb ${BASE_NAME}.pdf *.ps

cleanall: clean
	rm -f *.pdf

handout:
	$(PDFLATEX) -jobname=presentation.handout '\PassOptionsToClass{handout}{beamer} \input{$(BASE_NAME)}'
