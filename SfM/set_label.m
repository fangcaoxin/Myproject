function label = set_label(v)
n = size(v, 1);
label = ones(n,1);
for i = 1:n
  if(v(i,1)<0||v(i,2)<0)
     label(i, 1) = 0;
   end
end

end