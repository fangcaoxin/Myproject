function flow = choose_flow(optical_flow)
flow_num = size(optical_flow,1);
flow_base_angle = atand(optical_flow(1,2)/optical_flow(1,1));
count = 1;
for i = 2: flow_num
   flow_i_angle = atand(optical_flow(i,2)/optical_flow(i,1));
   if(abs(flow_i_angle - flow_base_angle)> 5)
      flow(count, 1:2) = optical_flow(i,:);
      flow(count, 3) = i;
      count = count + 1;
      flow_base_angle = flow_i_angle;
    end
end