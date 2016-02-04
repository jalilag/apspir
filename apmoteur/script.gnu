set terminal pngcairo size 1200,100 enhanced font 'Verdana,10' background rgb 'black'
set output 'temp.png'
set key off
set style line 1 lc rgb '#5e9c36' pt 6 ps 1 lt 1 lw 2
set style line 2 lc rgb '#8b1a0e' pt 1 ps 1 lt 1 lw 2
set style line 11 lc rgb '#808080' lt 1
set style line 12 lc rgb '#808080' lt 0 lw 1
set border 3 back ls 11
set xrange [0:30]
set yrange [-1:1]
set sample 10000
set lmargin 0
set rmargin 0
g0(x)=((x>=0 && x<=1) ? 1 : 1/0)
g1(x)=((x>=1 && x<=29) ? 1 : 1/0)
g2(x)=((x>=29 && x<=30) ? 1 : 1/0)
 plot sin(0)*g0(x) with lines linestyle 1,sin(2*pi/9*x+0-2*pi/9*1)*g1(x) with lines linestyle 1,sin(2*pi/9*29+0-2*pi/9*1)*g2(x) with lines linestyle 1