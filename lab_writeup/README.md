Yes you do need to run pdflatex multiple times for bibliography stuff.

```
sudo apt-get install texlive-latex-base texlive texlive-latex-extra texlive-fonts-extra texlive-science cm-super texlive-publishers
pdflatex main.tex
bibtex main
pdflatex main.tex
pdflatex main.tex
```