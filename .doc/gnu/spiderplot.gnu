# set terminal pngcairo  transparent enhanced font "arial,10" fontscale 1.0 size 600, 400
set terminal svg fontscale 96./72.*1.3
set output 'spiderplot.svg'

$Mydata << EOD
Dataset, Regression, Plotting, Visualization, Metadata, Configuration
90, 30, 70, 80, 95, 80
EOD

set spiderplot
set datafile separator comma
set for [p=1:6] paxis p range [0:100]
set for [p=1:6] paxis p tics format ""
set paxis 4 tics 2000 font ",8" format "%g"
set style spiderplot fillstyle transparent solid 0.3 border lw 1.0
set grid spider lt black lc "grey" lw 1.0 back
plot for [i=1:6] $Mydata using i title columnhead
