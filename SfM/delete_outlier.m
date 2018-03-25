function [image1_new, image2_new] = delete_outlier(image1_pt, image2_pt)
point_num = size(image1_pt,1);
count = 0;
load parameter.mat;
for i= 1:point_num
    if(image1_pt(i,1)>=0&&image1_pt(i,1)<=2*hcx&&image1_pt(i,2)>=0&&image1_pt(i,2)<=2*hcy&&...
           image2_pt(i,1)>=0&&image2_pt(i,1)<=2*hcx&&image2_pt(i,2)>=0&&image2_pt(i,2)<=2*hcy)
       count = count + 1;
       image1_new(count,:) = image1_pt(i,:);
       image2_new(count,:) = image2_pt(i,:);
    end
end
