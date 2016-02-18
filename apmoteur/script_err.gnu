set terminal pngcairo size 525,100 enhanced font 'Verdana,8' background rgb 'black'
set output 'temp_err.png'
set style line 1 lc rgb '#5e9c36' pt 6 ps 1 lt 1 lw 2
set style line 2 lc rgb '#8b1a0e' pt 1 ps 1 lt 1 lw 2
set style line 3 lc rgb '#8B0E6B' pt 1 ps 1 lt 1 lw 2
set style line 4 lc rgb '#619FD2' pt 1 ps 1 lt 1 lw 2
set style line 5 lc rgb '#DFB53E' pt 1 ps 1 lt 1 lw 2
set style line 11 lc rgb '#808080' lt 1
set style line 12 lc rgb '#808080' lt 0 lw 1
set border 3 back ls 11
set rmargin 0
set lmargin 4
set yrange [-250000:250000]
set key inside right horizontal top textcolor rgb "white"
set xrange [0:30]
plot "config/step_error.txt" using 1:2 title "Erreur" with lines linestyle 2