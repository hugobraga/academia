set title "Max Stretch Fac x Density" font ", 20"
set terminal postscript eps
set output "den-max_fac.eps"
set xlabel "Density" font ", 20"
set ylabel "Max Factor" offset 1.5,0 font ", 20"
set xr [0:100]
set yr [0:7]
set xtics font ", 17"
set ytics font ", 17"
set style data linespoints
set pointsize 2
#set autoscale x  
#set autoscale y 
set key left top spacing 1.25 at graph 0.05,0.98
plot "den-max_fac.dat" using 2:3 title '10' lt 3 pt 6 ps 3, "den-max_fac.dat" using 5:6 title '20'  lt -1 pt 5, "den-max_fac.dat" using 8:9 title '30' lt 3 pt 4 ps 4, "den-max_fac.dat" using 11:12 title '40' lt 3 pt 3 ps 4, "den-max_fac.dat" using 14:15 title '50' lt 3 pt 14 ps 7
