function  view = addview1(view, points, label, rot, trans, bearing_vector, id)
view(id).points =  points;
view(id).label = label;
view(id).rot =  rot;
view(id).trans =  trans;
view(id).bearing_vector=  bearing_vector;

end