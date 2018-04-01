function [image1_new, image2_new] = delete_outlier(image1_pt, image2_pt)
point_num = size(image1_pt,1);
count = 0;
load parameter.mat;
load ray1_true.mat;
load ray2_true.mat;
load points.mat;
for i= 1:point_num
    if(image1_pt(i,1)>=0&&image1_pt(i,1)<=2*hcx&&image1_pt(i,2)>=0&&image1_pt(i,2)<=2*hcy&&...
           image2_pt(i,1)>=0&&image2_pt(i,1)<=2*hcx&&image2_pt(i,2)>=0&&image2_pt(i,2)<=2*hcy)
       count = count + 1;
       image1_new(count,:) = image1_pt(i,:);
       image2_new(count,:) = image2_pt(i,:);
       ray1_in_select(count,:) = r1_in_true(i,:);
       ray1_out_select(count,:) = r1_out_true(i,:);
        ray2_in_select(count,:) = r2_in_true(i,:);
       ray2_out_select(count,:) = r2_out_true(i,:);
       point3d_effctive(count,:) = points(i,:);
    end
end
effctive_point = count;
save ray1_select.mat ray1_in_select ray1_out_select;
save ray2_select.mat ray2_in_select ray2_out_select;
save effctive_num.mat effctive_point;
save point3d_effctive.mat point3d_effctive;