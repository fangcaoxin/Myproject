function [v1_s, v2_s, i_label] = matchVectors(v1,v2, label_1,label_2)
   i_label = label_1&label_2;
   num = sum(i_label(:)==1);
   v1_s = zeros(num, size(v1,2));
   v2_s = zeros(num, size(v2,2));
   count = 1;
   for i = 1: size(i_label, 1)
      if(i_label(i,1)==1)
        v1_s(count,:)= v1(i,:);
        v2_s(count,:)= v2(i,:);
        count = count + 1;
      end
   end
   
end