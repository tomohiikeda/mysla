
set size square
set xr[-3000:3000]
set yr[-3000:3000]

do for [i=0:168] {
    plot sprintf("pt_%04d.txt", i) 
    pause 0.3
}
