set terminal pngcairo size 1000,262 enhanced font 'Verdana,10'
set output 'introduction.png'
set key off
#unset border
set style line 1 lc rgb '#8b1a0e' pt 1 ps 1 lt 1 lw 2 # --- red
set style line 2 lc rgb '#5e9c36' pt 6 ps 1 lt 1 lw 2 # --- green
set style line 11 lc rgb '#808080' lt 1
set style line 12 lc rgb '#808080' lt 0 lw 1
set border 3 back ls 11
set tics nomirror
set grid back ls 12
set xrange [0:30]
set yrange [-1:1]
g1(x)=( (x<=15) ? 1 : 1/0 )
g2(x)=( (x>=15 && x<=30) ? 1 : 1/0 )
plot 0.5*g1(x) with lines linestyle 1 ,cos(2*pi/7*x)*g2(x) with lines linestyle 2
#set size 1,0.5
#set terminal png size 1000,500
#set grid
#set xtics 0,2,30
#set ytics -1,0.2,1
#set size ratio 1
#set output "resultat.png"

