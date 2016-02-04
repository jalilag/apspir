set terminal pngcairo size 1200,100 enhanced font 'Verdana,10' background rgb 'black'
set output 'temp_vel.png'
set key off
set style line 1 lc rgb '#5e9c36' pt 6 ps 1 lt 1 lw 2
set style line 2 lc rgb '#8b1a0e' pt 1 ps 1 lt 1 lw 2
set style line 11 lc rgb '#808080' lt 1
set style line 12 lc rgb '#808080' lt 0 lw 1
set border 3 back ls 11
set lmargin 0
set rmargin 0
set xrange [0:30]
plot "config/velocity.txt" using 1:2 with lines linestyle 1