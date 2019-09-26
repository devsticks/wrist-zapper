q1pre = table2array(qlog2(:,1:4));
q2pre = table2array(qlog2(:,5:8));

q1 = [quaternion()];
q2 = [quaternion()];
for j = 624:size(q1pre,1)
    if q1pre(j,:) ~= q2pre(j,:)
        q1 = [q1; quaternion(q1pre(j,:))];
        q2 = [q2; quaternion(q2pre(j,:))];
    end
end

%% calibrate so all in line at start

% ---- Find calibration rotation matrix ---- %

% hand_start_offset = rot1 * rot2'; % undo rot2 to get hand aligned with inertial, then by rot1 to align with arm
hand_start_offset = q1(100) * conj(q2(100)); % undo q2 to get hand aligned with inertial, then by rotate by q1 to align with arm

%% do angle calculation

angles = [];
for i = 1:size(q1,1) - 50

    % ---- Get arm orientations ---- %
    % Arm is IMU 1 (currently 0x68, with ADO low)

    rot1 = quat2rotm(q1(i));
    
    % arm forward / x
    x_arm = rot1(:,1);                                                          %vector of arbitrary length defining direction of arm (find from IMU1)
    x_arm_n = x_arm/norm(x_arm);                                                %normalised direction of arm

    % arm left / y
    y_arm = rot1(:,2);
    y_arm_n = y_arm / norm(y_arm);

    % arm up / z = x cross y
    z_arm = rot1(:,3);                                                         %vector of arbitrary length defining orientation of arm (note z component is set below to ensure orientation makes sense)
    % u_arm(1) = -(u_arm(2)*v_arm(2)+u_arm(3)*v_arm(3))/v_arm(1);                 %ensure orthog to arm
    z_arm_n = z_arm/norm(z_arm);                                                %normalised orientation vector of arm

    % ---- Get hand orientations ---- %
    % Hand is IMU 2 (currently 0x69, with ADO high)

    % rot2 = hand_start_offset * quat2rotm(q2(i,:));
    rot2 = quat2rotm(hand_start_offset * q2(i));                                % rotate back by calibration amount

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

    % find normal of IMU1 plane
    v1 = x_arm_n;
    v2 = z_arm_n;
    n_vec = y_arm_n; % cross(v1,v2); %y_arm

    % find normal of hand extension plane
    v1 = x_hand_n;
    v2 = n_vec;         %y_arm_n;
    n_vec2 = cross(v1,v2)/norm(cross(v1,v2));

    v_intersection = cross(n_vec,n_vec2);                                       % find vector of intersection of planes
    v_intersection_n = v_intersection/norm(v_intersection);                     % normalise

    proj_angle = acos(dot(x_arm_n, v_intersection_n))*360/(2*pi);                    %find angle
%     fprintf('Angle between arm and hand in plane of hand: %4.2f\260 \n', proj_angle)

    cross_prod = cross(x_arm_n, v_intersection_n);
    if (dot(y_arm_n, cross_prod) > 0) 
        sign = -1;
    else
        sign = 1;
    end
    
    proj_angle = sign * proj_angle;

    angles = [angles proj_angle];
end

plot(angles);