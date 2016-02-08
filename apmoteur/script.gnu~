set terminal pngcairo size 660,100 enhanced font 'Verdana,8' background rgb 'black'
set output 'temp.png'
set key outside left horizontal top textcolor rgb "white"
set style line 1 lc rgb '#5e9c36' pt 6 ps 1 lt 1 lw 2
set style line 2 lc rgb '#8b1a0e' pt 1 ps 1 lt 1 lw 2
set style line 11 lc rgb '#808080' lt 1
set style line 12 lc rgb '#808080' lt 0 lw 1
set border 3 back ls 11
set lmargin 0
set rmargin 0
set xrange [0:30]
set yrange [-1:1]
set sample 10000
g0(x)=((x>=0 && x<=30) ? 1 : 1/0)
plot sin(2*pi/7*x)*g0(x) with lines linestyle 1title "Courbe théorique", "config/helix_record.txt" using 1:2 title "Courbe réelle" with lines linestyle 2
