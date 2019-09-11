clc;
% syms g mB mT IB x y thB thT dx dy dthB dthT ddx ddy ddthB ddthT LB LT
% 
% q = [x; y; thB; thT];
% dq = [dx; dy; dthB; dthT];
% ddq = [ddx; ddy; ddthB; ddthT];
% mass = [mB mT ];
% inertia = [IB ];

a1 = table2array(log(:,1:3));
g1 = table2array(log(:,4:6));
m1 = table2array(log(:,7:9));
a2 = table2array(log(:,10:12));
g2 = table2array(log(:,13:15));
m2 = table2array(log(:,16:18));

pitch1 = 180 * atan2(a1(:,1), sqrt(a1(:,2).*a1(:,2) + a1(:,3).*a1(:,3)))/pi; % atan2(a1x, sqrt(ay1*ay1 + az1*az1))
roll1 = 180 * atan2(a1(:,2), sqrt(a1(:,1).*a1(:,1) + a1(:,3).*a1(:,3)))/pi; %atan2(accelY, sqrt(accelX*accelX + accelZ*accelZ))
pitch2 = 180 * atan2(a2(:,1), sqrt(a2(:,2).*a2(:,2) + a2(:,3).*a2(:,3)))/pi; % atan2(a1x, sqrt(ay1*ay1 + az1*az1))
roll2 = 180 * atan2(a2(:,2), sqrt(a2(:,1).*a2(:,1) + a2(:,3).*a2(:,3)))/pi; %atan2(accelY, sqrt(accelX*accelX + accelZ*accelZ))

pitchdiff = pitch2-pitch1;
% plot(pitch1)
% hold on
% plot(roll1)
% plot(pitch2-pitch1)

down1 = a1;
east1 = cross(down1, m1);
north1 = cross(east1, down1);

rotmat1 = [];
quat1 = [];
for i=1:size(down1,1) 
   quat1 = vertcat(quat1, rotm2quat([north1(i,:)' east1(i,:)' down1(i,:)']));
end

eul1 = rad2deg(quat2eul(quat1));

down2 = a2;
east2 = cross(down2, m2);
north2 = cross(east2, down2);

rotmat2 = [];
quat2 = [];
for i=1:size(down2,1) 
   quat2 = vertcat(quat2, rotm2quat([north2(i,:)' east2(i,:)' down2(i,:)']));
end

eul2 = rad2deg(quat2eul(quat2));
plot(eul2)
%%
dT = 1/25;
w1 = [0 0 0];
w2 = [0 0 0];
for i = 2:size(g1,1)
   w1 = vertcat(w1, dT*(g1(i,:)) + w1(i-1,:));
   w2 = vertcat(w2, dT*(g2(i,:)) + w2(i-1,:));
end

gangle = w1(:,2)-w2(:,2);
% plot(gangle)

fangle = 0;
for i = 2:size(g1,1)
   fangle = vertcat(fangle, 0.998*(dT*(g1(i,2)-g2(i,2)) + fangle(i-1)) + 0.002 * (pitchdiff(i-1)));
%    w2 = vertcat(w2, dT*(g2(i,:)) + w2(i-1,:));
end

% plot(fangle)