# for each file in src run analysis
for f in src/*; do
  ghdl -a --std=08 $f
done
