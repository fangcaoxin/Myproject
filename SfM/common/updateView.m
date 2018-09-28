function view = updateView(view, R_opm, t_opm, i)
for j = 2 :i
  view(j).rot =  R_opm(:,:,j-1);
  view(j).trans =  t_opm(:,:,j-1);
end