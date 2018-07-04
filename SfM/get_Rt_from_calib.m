function [R, t] = get_Rt_from_calib(gg)
r1 = [gg(1) gg(2) gg(3)];
r2 = [gg(4) gg(5) gg(6)];
r3 = cross(r1,r2);
R = [r1;r2;r3];
t = [gg(7); gg(8);gg(9)];
end