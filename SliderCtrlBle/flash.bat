@mode com5:1200 > nul
@timeout /T 1 > nul
@bossac --port=COM5 -U false -e -w -v -b Debug\SliderCtrlBle.bin -R
