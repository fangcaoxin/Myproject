function view = updateView(view, R_opm, t_opm, i)
% for j = 2 :i
  view(i).rot =  R_opm;
  view(i).trans =  t_opm;
end