q1pre = table2array(qlog2(:,1:4));
q2pre = table2array(qlog2(:,5:8));

q1 = [quaternion()];
q2 = [quaternion()];
for j = 1:size(q1pre,1)
    if q1pre(j,:) ~= q2pre(j,:)
        q1 = [q1; quaternion(q1pre(j,:))];
        q2 = [q2; quaternion(q2pre(j,:))];
    end
end

%% calibrate so all in line at start
rot1 = quat2rotm(q1(100));
rot2 = quat2rotm(q2(100));

% ---- Get arm orientation ---- %

% arm forward / x
x_arm = rot1(:,1);                                                          %vector of arbitrary length defining direction of arm (find from IMU1)
x_arm_n = x_arm/norm(x_arm);                                                %normalised direction of arm

% arm left / y
y_arm = rot1(:,2);
y_arm_n = y_arm / norm(y_arm);

% arm up / z = x cross y
z_arm = rot1(:,3); 
z_arm_n = z_arm/norm(z_arm); 

% ---- Get hand orientation ---- %

% hand forward / x
x_hand = rot2(:,1);                                                          %vector of arbitrary length defining direction of hand (find from IMU2)
x_hand_n = x_hand/norm(x_hand);                                             %normalised direction of hand

% hand left / y
y_hand = rot2(:,2);
y_hand_n = y_hand/norm(y_hand);

% hand up / z = x cross y
z_hand = rot2(:,3);                                                        %vector of arbitrary length defining orientation of hand (note z component is set below to ensure orientation makes sense)
z_hand_n = z_hand/norm(z_hand);                                             %normalised orientation vector of hand

% ---- Find calibration rotation matrix ---- %

% hand_start_offset = rot1 * rot2'; % undo rot2 to get hand aligned with inertial, then by rot1 to align with arm
hand_start_offset = q1(100) * conj(q2(100));
% 12*v21*
%% do angle calculation

angles = [];
F(size(q1,1)) = struct('cdata',[],'colormap',[]);
plot3(0,0,0,"r*" )                                                 %plot origin
hold off
for i = 100:size(q1,1) - 50
    clf;
    plot3(0,0,0,"r*" );
    xlim([-1.5,3]);
    ylim([-1.5,3]);
    zlim([-1.5,3]);
    pbaspect([1 1 1])

%%
    rot1 = quat2rotm(q1(i));
    rot2 = quat2rotm(q2(i));
%     
% %
%     %plot IMU1 forward vector
%     frame_imu1_for_start = [0,0,0];
%     frame_imu1_for_end = rot1(:,1); % column 1
%     arrow(frame_imu1_for_start,frame_imu1_for_end,'linewidth',2,'Length',6,'color','r');
% %     drawnow
%     
%     %plot IMU1 left vector
%     frame_imu1_left_start = [0,0,0];
%     frame_imu1_left_end = rot1(:,2); % column 2
%     arrow(frame_imu1_left_start,frame_imu1_left_end,'linewidth',2,'Length',6,'color','g');
% %     drawnow
%     
% 	%plot IMU1 up vector
%     frame_imu1_up_start = [0,0,0];
%     frame_imu1_up_end = rot1(:,3); % column 3
%     arrow(frame_imu1_up_start,frame_imu1_up_end,'linewidth',2,'Length',6,'color','b');
% %     drawnow
% 
%      rot2 = quat2rotm((hand_start_offset) * q2(i));% * (hand_start_offset));
% %     rot2 = hand_start_offset * quat2rotm(q2(i,:));    
% 
%     %plot IMU2 forward vector
%     frame_imu2_for_start = 2*frame_imu1_for_end;
%     frame_imu2_for_end = 2*frame_imu1_for_end + rot2(:,1); % column 1
%     arrow(frame_imu2_for_start,frame_imu2_for_end,'linewidth',2,'Length',6,'color','r');
% %     drawnow
%     
%     %plot IMU2 left vector
%     frame_imu2_left_start = 2*frame_imu1_for_end;
%     frame_imu2_left_end = 2*frame_imu1_for_end + rot2(:,2); % column 2
%     arrow(frame_imu2_left_start,frame_imu2_left_end,'linewidth',2,'Length',6,'color','g');
% %     drawnow
%     
% 	%plot IMU2 up vector
%     frame_imu2_up_start = 2*frame_imu1_for_end;
%     frame_imu2_up_end = 2*frame_imu1_for_end + rot2(:,3); % column 3
%     arrow(frame_imu2_up_start,frame_imu2_up_end,'linewidth',2,'Length',6,'color','b');
%     drawnow
% 
% continue
%%

l_arm = 10;                                                                 %length of arm
l_hand = 8;                                                                 %length of hand

l_imu1 = 4;                                                                 %distance along arm of IMU1
l_imu2 = 3;                                                                 %distance along hand of IMU2

O = [0;0;0];                                                                %position of origin

% Arm is IMU 1 (currently 0x68, with ADO low)

% arm forward / x
x_arm = rot1(:,1);                                                          %vector of arbitrary length defining direction of arm (find from IMU1)
x_arm_n = x_arm/norm(x_arm);                                                %normalised direction of arm
P_joint = O + l_arm * x_arm_n;                                              %position of joint centre    

% arm left / y
y_arm = rot1(:,2);
y_arm_n = y_arm / norm(y_arm);

% arm up / z = x cross y
z_arm = rot1(:,3);                                                         %vector of arbitrary length defining orientation of arm (note z component is set below to ensure orientation makes sense)
% u_arm(1) = -(u_arm(2)*v_arm(2)+u_arm(3)*v_arm(3))/v_arm(1);                 %ensure orthog to arm
z_arm_n = z_arm/norm(z_arm);                                                %normalised orientation vector of arm

% x,y,z used to plot arm
arm_x = [O(1);P_joint(1)];                                                  
arm_y = [O(2);P_joint(2)];
arm_z = [O(3);P_joint(3)];

P_imu1 = O + l_imu1*x_arm_n;                                                %position of IMU1
% x,y,z used to plot IMU1
imu1_x = [O(1);P_imu1(1)];
imu1_y = [O(2);P_imu1(2)];
imu1_z = [O(3);P_imu1(3)];

% Hand is IMU 2 (currently 0x69, with ADO high)

% rot2 = hand_start_offset * quat2rotm(q2(i,:));
rot2 = quat2rotm(hand_start_offset * q2(i));

% hand forward / x
x_hand = rot2(:,1);                                                          %vector of arbitrary length defining direction of hand (find from IMU2)
x_hand_n = x_hand/norm(x_hand);                                             %normalised direction of hand

% hand left / y
y_hand = rot2(:,2);
y_hand_n = y_hand/norm(y_hand);

% hand up / z = x cross y
z_hand = rot2(:,3);                                                        %vector of arbitrary length defining orientation of hand (note z component is set below to ensure orientation makes sense)
% u_hand(3) = -(u_hand(1)*v_hand(1)+u_arm(2)*v_hand(2))/v_hand(3);            %ensure orthog to hand
z_hand_n = z_hand/norm(z_hand);                                             %normalised orientation vector of hand

% x,y,z used to plot hand
hand_x = [P_joint(1);P_joint(1)+l_hand*x_hand_n(1)];                        
hand_y = [P_joint(2);P_joint(2)+l_hand*x_hand_n(2)];
hand_z = [P_joint(3);P_joint(3)+l_hand*x_hand_n(3)];

P_imu2 = P_joint+l_imu2*x_hand_n;                                           %position of IMU2
% x,y,z used to plot IMU2
imu2_x = [O(1);P_imu2(1)];
imu2_y = [O(2);P_imu2(2)];
imu2_z = [O(3);P_imu2(3)];

% Plotting
plot3(O(1),O(2),O(3),"r*" )                                                 %plot origin
hold on

%plot arm
plot3(arm_x, arm_y, arm_z)                                                   %plot arm
plot3(imu1_x, imu1_y, imu1_z,"g*")                                           %plot imu1

%plot IMU1 frame
frame_imu1_x_start = P_imu1;
frame_imu1_x_end = P_imu1 + x_arm_n;
arrow(frame_imu1_x_start,frame_imu1_x_end,'linewidth',2,'Length',6,'color','r'); %unsure why linewith = 1 looks dif to later

frame_imu1_y_start = P_imu1;
frame_imu1_y_end = P_imu1 + y_arm_n;
arrow(frame_imu1_y_start,frame_imu1_y_end,'linewidth',2,'Length',6,'color','g');

frame_imu1_z_start = P_imu1;
frame_imu1_z_end = P_imu1 + z_arm_n;
arrow(frame_imu1_z_start,frame_imu1_z_end,'linewidth',2,'Length',6,'color','b');


plot3(hand_x,hand_y, hand_z,"m")                                            %plot hand
plot3(imu2_x,imu2_y, imu2_z,"g*")                                           %plot imu2

%plot IMU2 frame
frame_imu2_x_start = P_imu2;
frame_imu2_x_end = P_imu2 + x_hand_n;
arrow(frame_imu2_x_start,frame_imu2_x_end,'linewidth',1,'Width',1,'Length',6,'color','r');

frame_imu2_y_start = P_imu2;
frame_imu2_y_end = P_imu2 + y_hand_n;
arrow(frame_imu2_y_start,frame_imu2_y_end,'linewidth',1,'Width',1,'Length',6,'color','g');

frame_imu2_z_start = P_imu2;
frame_imu2_z_end = P_imu2 + z_hand_n;
arrow(frame_imu2_z_start,frame_imu2_z_end,'linewidth',1,'Width',1,'Length',6,'color','b');

xlabel("X")
ylabel("Y")
zlabel("Z")

% plot IMU1 plane
v1 = x_arm_n;
v2 = z_arm_n;
n_vec = y_arm_n; % cross(v1,v2); %y_arm
imu1_plane = fsurf(@(x,y) ((n_vec(1)*O(1)+ n_vec(2)*O(2)+n_vec(3)*O(3))- n_vec(1)*x-n_vec(2)*y)/n_vec(3),...
    [min(min(arm_x, hand_x)),max(max(arm_x, hand_x)),min(min(arm_y, hand_y)),max(max(arm_y, hand_y))],'MeshDensity',2);

set(imu1_plane,'FaceColor',[0 1 0], ...
      'FaceAlpha',0.1,'EdgeColor','none')

  % plot IMU2 plane

v1 = x_hand_n;
v2 = n_vec; %y_arm_n;
n_vec2 = cross(v1,v2)/norm(cross(v1,v2));

% n_vec2 = u_hand_n;
% n_vec2 = v_arm_n;

imu2_plane = fsurf(@(x,y) ((n_vec2(1)*P_joint(1)+ n_vec2(2)*P_joint(2)+n_vec2(3)*P_joint(3))- n_vec2(1)*x-n_vec2(2)*y)/n_vec2(3),...
    [min(min(arm_x, hand_x)),max(max(arm_x, hand_x)),min(min(arm_y, hand_y)),max(max(arm_y, hand_y))],'MeshDensity',2,'edgecolor','none');

set(imu2_plane,'FaceColor',[0 1 1], ...
      'FaceAlpha',0.1,'EdgeColor','none')


imu1_plane.FaceAlpha = 0.1 ;                                                % set transparency of planes
imu2_plane.FaceAlpha = 0.1 ;


v_intersection = cross(n_vec,n_vec2);                                       % find vector of intersection of planes
v_intersection_n = v_intersection/norm(v_intersection);                     % normalise

%x,y,z used to plot line of intersection
intersection_x = [P_joint(1)-l_hand*v_intersection_n(1);P_joint(1)+l_hand*v_intersection_n(1)];
intersection_y = [P_joint(2)-l_hand*v_intersection_n(2);P_joint(2)+l_hand*v_intersection_n(2)];
intersection_z = [P_joint(3)-l_hand*v_intersection_n(3);P_joint(3)+l_hand*v_intersection_n(3)];

%X,y,z used to extend arm
arm_ext_x = [P_joint(1);P_joint(1)+l_hand*x_arm_n(1)];
arm_ext_y = [P_joint(2);P_joint(2)+l_hand*x_arm_n(2)];
arm_ext_z = [P_joint(3);P_joint(3)+l_hand*x_arm_n(3)];


plot3(intersection_x, intersection_y, intersection_z,'r--');                %plot line of intersection
plot3(arm_ext_x, arm_ext_y, arm_ext_z,'b--');                               %plot arm extention

%scale plot
% h = get(gca,'DataAspectRatio') ;
% if h(3)==1
%       set(gca,'DataAspectRatio',[1 1 1])
% else
%       set(gca,'DataAspectRatio',[1 1 1])
% end

% zlim([min(min(arm_z, hand_z)),max(max(arm_z, hand_z))])

xlim([-10,15]);
ylim([-10,15]);
zlim([-10,15]);
pbaspect([1 1 1]);

proj_angle = acos(dot(x_arm_n, v_intersection_n))*360/(2*pi);                    %find angle
fprintf('Angle between arm and hand in plane of hand: %4.2f\260 \n', proj_angle)

angles = [angles proj_angle];

drawnow
%%
    
%     F(i) = getframe(gca);
end
% 
% hold off
% clf;
% plot(angles);
% fig = figure;
% movie(fig,F,1);