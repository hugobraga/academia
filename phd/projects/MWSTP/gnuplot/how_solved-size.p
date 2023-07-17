# set terminal canvas  rounded size 600,400 enhanced fsize 10 lw 1.6 fontscale 1 name "histograms_8" jsdir "."
# set output 'histograms.8.js'
set terminal postscript eps
set output "how_solved-size.eps"
#set border 3 front lt black linewidth 1.000 dashtype solid
set boxwidth 0.8 absolute
set style fill   solid 1.00 noborder
set grid nopolar
set grid noxtics nomxtics ytics nomytics noztics nomztics \
 nox2tics nomx2tics noy2tics nomy2tics nocbtics nomcbtics
set grid layerdefault   lt 0 linewidth 0.500,  lt 0 linewidth 0.500
set key bmargin center horizontal Left reverse noenhanced autotitle columnhead nobox
set style histogram rowstacked title textcolor lt -1 offset character 2, 0.25
set datafile missing '-'
set style data histograms
set xtics border in scale 0,0 nomirror rotate by -45  autojustify
set xtics  norangelimit  font ",8"
set xtics   ()
set ytics border in scale 0,0 mirror norotate  autojustify
set ytics  norangelimit autofreq  font ",8"
set ztics border in scale 0,0 nomirror norotate  autojustify
set cbtics border in scale 0,0 mirror norotate  autojustify
set rtics axis in scale 0,0 nomirror norotate  autojustify
set title "How instances were solved x Network Size" 
set xlabel "Network Size" 
set xlabel  offset character 0, -2, 0 font "" textcolor lt -1 norotate
set ylabel "Instances" 
set yrange [ 0 : 10 ] noreverse nowriteback
x = 0
i = 24
## Last datafile plotted: "immigration.dat"
plot newhistogram "10", 'how_solved-size.dat' using 2:xtic(1) t "No solution" lt -1 fs pattern 1, '' u 3 t "Optimal (w/o diameter algorithm)" lt -1 fs pattern 4, '' u 4 t "Optimal (w/ diameter algorithm)" fs pattern 3, '' u 5 t "Not solved" lt -1 fs pattern 5, newhistogram "20", '' using 6:xtic(1) t "" lt -1 fs pattern 1, '' u 7 t "" lt -1 fs pattern 4, '' u 8 t "" fs pattern 3, '' u 9 t "" lt -1 fs pattern 5, newhistogram "30", '' using 10:xtic(1) t "" lt -1 fs pattern 1, '' u 11 t "" lt -1 fs pattern 4, '' u 12 t "" fs pattern 3, '' u 13 t "" lt -1 fs pattern 5, newhistogram "40", '' using 14:xtic(1) t "" lt -1 fs pattern 1, '' u 15 t "" lt -1 fs pattern 4, '' u 16 t "" fs pattern 3, '' u 17 t "" lt -1 fs pattern 5, newhistogram "50", '' using 18:xtic(1) t "" lt -1 fs pattern 1, '' u 19 t "" lt -1 fs pattern 4, '' u 20 t "" fs pattern 3, '' u 21 t "" lt -1 fs pattern 5, newhistogram "60", '' using 22:xtic(1) t "" lt -1 fs pattern 1, '' u 23 t "" lt -1 fs pattern 4, '' u 24 t "" fs pattern 3, '' u 25 t "" lt -1 fs pattern 5
