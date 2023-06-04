%% 
% Objective: Simulate two angiographic views of the left coronary artery
% (its skeleton) by choosing the DICOM parameters based on the information
% provided in Appendix B.
clear all; clc;
load("Coronary.mat")

% Parameters
SID =  1050; SOD =  750;
DP = 0.1953;
imsize = [1024 1024];
alpha1  = 30; beta1 = 25; alpha2 = -30; beta2 = -25;
num_seg = 7;

% Build view
[source1, detector1] = BuildViewGeom(SID, SOD, DP, alpha1, beta1, imsize);
[source2, detector2] = BuildViewGeom(SID, SOD, DP, alpha2, beta2, imsize);

% Reconstruct from 2D image from 3D data (Use skeleton data from Coronary)
 M1 = source1.P;
 M2 = source2.P;
 
% Use p = M*P to get the u-v coordinates
% P can be retrieved from the Coranary data

uv1 = {};
for i = 1:num_seg
    mat = [Coronary.skeleton{i}(:,1)'; Coronary.skeleton{i}(:,2)'; Coronary.skeleton{i}(:,3)'; ones(size(Coronary.skeleton{i}(:,3)))'];
    uv1{i} = (M1 * mat)'; 
    uv1{i} = uv1{i} ./ uv1{i}(:, end);
end

uv2 = {};
for i = 1:num_seg
    mat = [Coronary.skeleton{i}(:,1)'; Coronary.skeleton{i}(:,2)'; Coronary.skeleton{i}(:,3)'; ones(size(Coronary.skeleton{i}(:,3)))'];
    uv2{i} = (M2 * mat)'; 
    uv2{i} = uv2{i} ./ uv2{i}(:, end);
end

% Plot the artery in both views
figure(1);
subplot(1, 2, 1);
for i = 1:num_seg
    plot(uv1{i}(:,1), uv1{i}(:,2), 'linewidth', 2)
    hold on;
end
xlabel("u1-values")
ylabel("v1-values")
axis equal
hold off;
title('View 1')

subplot(1, 2, 2);
for j = 1:num_seg
    plot(uv2{j}(:,1), uv2{j}(:,2), 'linewidth', 2)
    hold on;
end
xlabel("u2-values")
ylabel("v2-values")
title('View 2')
axis equal
hold off;

%% 
% Question 1 
A = [source1.R source1.T; [0 0 0 1]];
A_prime = [source2.R source2.T; [0 0 0 1]];
As = A_prime * inv(A);
Rs = As(1:3, 1:3);
Ts = As(1:3, 4);
Tx = [0 -Ts(3) Ts(2); Ts(3) 0 -Ts(1); -Ts(2) Ts(1) 0];
E = Tx * Rs;


% Question a:
F = inv(source2.K)' * E * inv(source1.K);
F = F(:,:)./F(3, end);
disp('F='); disp(F);

% Question b:
uv1_concaneted = [uv1{1}(1,:)', uv1{2}(1,:)', uv1{3}(1,:)', uv1{4}(1,:)', uv1{5}(1,:)', uv1{6}(1,:)', uv1{7}(1,:)', uv1{3}(end, :)'];
uv2_concaneted = [uv2{1}(1,:)', uv2{2}(1,:)', uv2{3}(1,:)', uv2{4}(1,:)', uv2{5}(1,:)', uv2{6}(1,:)', uv2{7}(1,:)', uv2{3}(end, :)'];

[F_norm, E1, E2] = FMatNorm8(uv1_concaneted(:,:), uv2_concaneted(:,:));
F_norm = F_norm(:,:)./F_norm(3,end);
disp('F_norm_8='); disp(F_norm);

% Question c:
uv1_concaneted_all = [uv1{1}', uv1{2}', uv1{3}', uv1{4}', uv1{5}', uv1{6}', uv1{7}'];
uv2_concaneted_all = [uv2{1}', uv2{2}', uv2{3}', uv2{4}', uv2{5}', uv2{6}', uv2{7}'];

[F_norm_all, E1_all, E2_all] = FMatNorm8(uv1_concaneted_all(:,:), uv2_concaneted_all(:,:));
F_norm_all = F_norm_all(:,:)./F_norm_all(3,end);
disp('F_norm_all='); disp(F_norm_all);

% They are all the same!
% By adding noise they would be different

%% Question 2 
% 1)
view_1 = uv1;
view_2 = uv2;

% 2)
uv_interpol = {};
for i = 1:num_seg
    n = size(view_2{i});
    uv_interpol{i} = Interpole_Discretise(view_2{i}, 2 * n(1));
end


% 3)
% The epipolar line is expressed as au + bv + c = 0
% In view 2 : [a, b, c]^T = F * (U_view1, V_view1)^T
% In view 1 : [a, b, c]^T   = F^T * (U_view2, V_view2)^T

l1 = {};
l2 = {};        
for i = 1:num_seg
    l1{i} = (F_norm_all * uv1{i}')';
    l1{i} = l1{i} ./ l1{i}(:,end);

    l2{i} = (F_norm_all' * uv_interpol{i}')';
    l2{i} = l2{i} ./ l2{i}(:,end);
end

% plot
figure(2);
subplot(1, 2, 1);
x = linspace(-10000, 10000, 100);
y1 = (-l2{1}(:,3) - l2{1}(:,1) .* x) ./ l2{1}(:,2);
plot(x, y1, 'LineStyle', '-', 'color', [0.25, 0.25, 0.25])
hold on;
scatter(E1_all(1), E1_all(2),"filled", "red", "o")
plot(uv1{1}(:,1), uv1{1}(:,2), 'LineWidth', 2, 'Color', "red")
axis([-500 6500 0 5000]);
% axis equal
hold off
title('View 1')

subplot(1, 2, 2);
x = linspace(-10000, 10000, 100);
y2 = (-l1{1}(:,3) - l1{1}(:,1) .* x) ./ l1{1}(:,2);
plot(x, y2, 'LineStyle', "-", 'color', [0.25, 0.25, 0.25])
hold on
scatter(E2_all(1), E2_all(2),"filled", "red", "o")
plot(uv2{1}(:,1), uv2{1}(:,2), "LineWidth", 2, 'color', "red")
axis([-5000 1050 -4000 1000]);
% axis equal
hold off
title('View 2')

% Distance: D = |au + bv + c|/sqrt(a^2 + b^2)
% Formula taken from Wiki: https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
index_dot = {};
for i = 1:7
    shape = size(l1{i});
    n = shape(1);
    index_dot{i} = [];
    for i_line = 1:n
        a_line = l1{i}(i_line, :);
        % using uv2 should have the index # as its output
        uv = uv_interpol{i};
%         uv = uv2{i};
        dist = abs(a_line(1) .* uv(:, 1) + a_line(2).*uv(:,2) + a_line(3)) ./ sqrt(a_line(1).^2 + sqrt(a_line(2).^2));   
        [min_value, index_min_value] = min(dist);
        index_dot{i}(i_line) = index_min_value;
    end
end

% D1 = {};
% D2 = {};
% for i = 1:7
%     D1{i} = abs(l2{i}(:,1).*uv_interpol{i}(:, 1) + l2{i}(:,2).*uv_interpol{i}(:,2) + l2{i}(3))./sqrt(l2{i}(:,1).^2 + sqrt(l2{i}(:,2).^2));
%     D2{i} = abs(l1{i}(:,1).*uv1{i}(:, 1) + l1{i}(:,2).*uv1{i}(:,2) + l1{i}(3))./sqrt(l1{i}(:,1).^2 + sqrt(l1{i}(:,2).^2));
% end
% 
% D1_array = [D1{1}; D1{2}; D1{3}; D1{4}; D1{5}; D1{6}; D1{7}];
% D2_array = [D2{1}; D2{2}; D2{3}; D2{4}; D2{5}; D2{6}; D2{7}];
% [minDist1, index] = min(D1_array);
% 
% minDist2 = D2_array(floor(index+1)/2);

% plot a point
index1 = 100;
index2 = 20;
simulated_index1 = round(index_dot{1}(index1));
simulated_index2 = round(index_dot{1}(index2));

figure(3)
subplot(1,2,1)
x = linspace(-10000, 10000, 100);
y1 = (-l2{1}(:,3) - l2{1}(:,1) .* x) ./ l2{1}(:,2);
% plot(x, y1, 'LineStyle', "-", 'color', [0.25, 0.25, 0.25])
hold on
for i = 1:7
    plot(uv1{i}(:,1), uv1{i}(:,2), "LineWidth", 2, 'color', "red")
end
yindex1 = (-l2{1}(simulated_index1,3) - l2{1}(simulated_index1,1) .* x) ./ l2{1}(simulated_index1,2);
plot(x, yindex1, 'LineStyle', "-", 'LineWidth', 2, 'color', 'blue')
yindex2 = (-l2{1}(simulated_index2,3) - l2{1}(simulated_index2,1) .* x) ./ l2{1}(simulated_index2,2);
plot(x, yindex2, 'LineStyle', "-", 'LineWidth', 2, 'color', 'blue')
scatter(uv1{1}(index1, 1), uv1{1}(index1, 2), "filled", "green", "o")
scatter(uv1{1}(index2, 1), uv1{1}(index2, 2), "filled", "green", "o")
axis([300 1100 100 800]);
hold off
title('View 1')

subplot(1,2,2)
x = linspace(-10000, 10000, 100);
y2 = (-l1{1}(:,3) - l1{1}(:,1) .* x) ./ l1{1}(:,2);
% plot(x, y2, 'LineStyle', "-", 'color', [0.25, 0.25, 0.25])
hold on
scatter(E2_all(1), E2_all(2),"filled", "red", "o")
for i = 1:7
    plot(uv2{i}(:,1), uv2{i}(:,2), "LineWidth", 2, 'color', "red")
end
yindex1 = (-l1{1}(index1,3) - l1{1}(index1,1) .* x) ./ l1{1}(index1,2);
plot(x, yindex1, 'LineStyle', "-", 'LineWidth', 2, 'color', 'blue')
yindex2 = (-l1{1}(index2,3) - l1{1}(index2,1) .* x) ./ l1{1}(index2,2);
plot(x, yindex2, 'LineStyle', "-", 'LineWidth', 2, 'color', 'blue')
scatter(uv2{1}(index1, 1), uv2{1}(index1, 2), "filled", "green", "o")
scatter(uv2{1}(index2, 1), uv2{1}(index2, 2), "filled", "green", "o")
scatter(uv_interpol{1}(simulated_index1, 1), uv_interpol{1}(simulated_index1, 2), "yellow", "*", 'linewidth', 1)
scatter(uv_interpol{1}(simulated_index2, 1), uv_interpol{1}(simulated_index2, 2), "yellow", "*", 'linewidth', 1)
axis([300 1100 -100 900]);
% axis([0 1050 -4000 1000]);
% axis equal
hold off
title('View 2')
% Add legend

%% Question 3
% 

% Calculate depth map
% B is distance in 3d
% f we know
% d is distance in 2d as if they were in the same view
Z_no_rec = {};
d_no_rec = {};

uv1_matched = {};
uv2_matched = {};
for i = 1:7
    uv1_matched{i} = uv1{i};
    uv2_matched{i} = uv_interpol{i}(index_dot{i}', :);
end

for i = 1:7
%     If we use indexes calculated in q2, should we use B as the calculated
%     indexes and d?
    B = sqrt((source2.worldPos(1) - source1.worldPos(1))^2 + (source2.worldPos(2) - source1.worldPos(2))^2 + (source2.worldPos(3) - source1.worldPos(3))^2);
    f = source1.f;
    d_no_rec{i} = sqrt((uv2_matched{i}(:, 1) - uv1_matched{i}(:, 1)).^2 + (uv2_matched{i}(:, 2) - uv1_matched{i}(:, 2)).^2 + (uv2_matched{i}(:, 3) - uv1_matched{i}(:, 3)).^2);
    Z_no_rec{i} = (f .* B) ./ d_no_rec{i};
end


% Rectify and calculate depth map again
[T1,T2,Pn1,Pn2] = Rectify(source1.P, source2.P);
uv1_rec = {};
for i = 1:num_seg
    mat = [Coronary.skeleton{i}(:,1)'; Coronary.skeleton{i}(:,2)'; Coronary.skeleton{i}(:,3)'; ones(size(Coronary.skeleton{i}(:,3)))'];
    uv1_rec{i} = (Pn1 * mat)'; 
    uv1_rec{i} = uv1_rec{i} ./ uv1_rec{i}(:, end);
end

uv2_rec = {};
for i = 1:num_seg
    mat = [Coronary.skeleton{i}(:,1)'; Coronary.skeleton{i}(:,2)'; Coronary.skeleton{i}(:,3)'; ones(size(Coronary.skeleton{i}(:,3)))'];
    uv2_rec{i} = (Pn2 * mat)'; 
    uv2_rec{i} = uv2_rec{i} ./ uv2_rec{i}(:, end);
end

% Calculate depth map rectified
Z_rec = {};
d_rec = {};
for i = 1:7
    B = sqrt((source2.worldPos(1) - source1.worldPos(1))^2 + (source2.worldPos(2) - source1.worldPos(2))^2 + (source2.worldPos(3) - source1.worldPos(3))^2);
    f = source1.f;
    d_rec{i} = sqrt((uv2_rec{i}(:, 1) - uv1_rec{i}(:, 1)).^2 + (uv2_rec{i}(:, 2) - uv1_rec{i}(:, 2)).^2 + (uv2_rec{i}(:, 3) - uv1_rec{i}(:, 3)).^2);
    Z_rec{i} = (f .* B) ./ d_rec{i};
end

%% Show disparity maps
% Disparity no rec
figure
subplot(2, 2, 1);
for i = 1:7
    scatter(uv1{i}(:, 1), uv1{i}(:, 2), [], d_no_rec{i})
    colorbar
    hold on
end
hold off
title('Disparity uv1 not rectified')

% Depth no rec
subplot(2, 2, 2);
for i = 1:7
    scatter(uv2{i}(:, 1), uv2{i}(:, 2), [], d_no_rec{i})
    colorbar
    hold on
end
hold off
title('Disparity uv2 not rectified')

% Disparity rec
subplot(2, 2, 3);
for i = 1:7
    scatter(uv1{i}(:, 1), uv1{i}(:, 2), [], d_rec{i})
    colorbar
    hold on
end
hold off
title('Disparity uv1 rectified')

% Depth rec
subplot(2, 2, 4);
for i = 1:7
    scatter(uv2{i}(:, 1), uv2{i}(:, 2), [], d_rec{i})
    colorbar
    hold on
end
hold off
title('Disparity uv2 rectified')
%% Show depth maps
% Disparity no rec
figure
subplot(2, 2, 1);
for i = 1:7
    scatter(uv1{i}(:, 1), uv1{i}(:, 2), [], Z_no_rec{i})
    colorbar
    hold on
end
hold off
title('Depth map uv1 not rectified')

% Depth no rec
subplot(2, 2, 2);
for i = 1:7
    scatter(uv2{i}(:, 1), uv2{i}(:, 2), [], Z_no_rec{i})
    colorbar
    hold on
end
hold off
title('Depth map uv2 not rectified')

% Disparity rec
subplot(2, 2, 3);
for i = 1:7
    scatter(uv1{i}(:, 1), uv1{i}(:, 2), [], Z_rec{i})
    colorbar
    hold on
end
hold off
title('Depth map uv1 rectified')

% Depth rec
subplot(2, 2, 4);
for i = 1:7
    scatter(uv2{i}(:, 1), uv2{i}(:, 2), [], Z_rec{i})
    colorbar
    hold on
end
hold off
title('Depth map uv2 rectified')
%%
% Plot the artery in both views
figure
subplot(1, 2, 1)
axis equal;
for i = 1:num_seg
    plot(uv1_rec{i}(:,1), uv1_rec{i}(:,2), 'linewidth', 2)
    hold on;
end
xlabel("u1-values")
ylabel("v1-values")
hold off;
title('uv1 rectified')

subplot(1, 2, 2)
axis equal
for j = 1:num_seg
    plot(uv2_rec{j}(:,1), uv2_rec{j}(:,2), 'linewidth', 2)
    hold on;
end
xlabel("u2-values")
ylabel("v2-values")
hold off;
title('uv2 rectified')

%% Calculate ground truth depth map
Z_truth = {};
for i = 1:7
    shape = size(Coronary.skeleton{i});
    n = shape(1);
    Z_truth{i} = [];
    for j = 1:n
        coord = source1.extMat * [Coronary.skeleton{i}(j,:) 1]';   
        Z_truth{i}(j) = coord(3);
    end
end

% Depth rec
figure
subplot(1,2,1)
for i = 1:7
    scatter(uv1{i}(:, 1), uv1{i}(:, 2), [], Z_truth{i})
    hold on
end
hold off
colorbar
title('Groung truth depth map uv1')

subplot(1,2,2)
for i = 1:7
    scatter(uv2{i}(:, 1), uv2{i}(:, 2), [], Z_truth{i})
    hold on
end
hold off
colorbar
title('Groung truth depth map uv2')

%%  Bonus
% Calculate quatitatively the differences

diff_no_rec = calculate_dist(Z_truth, Z_no_rec);
diff_rec = calculate_dist(Z_truth, Z_rec);

disp('diff_no_rec:'); disp(diff_no_rec);
disp('diff_rec:'); disp(diff_rec);
%% Question 4
% Use uv to calculate xyz (Use matching found in q2 and source1.P and
% source2.P)
% uv1_mateched = {};
% uv2_matched = {};
% for i = 1:7
%     uv1_matched{i} = uv1{i};
%     uv2_matched{i} = uv_interpol{i}(index_dot{i}', :);
% end
xyzs = reconstruct(source1.P, source2.P, uv1_matched, uv2_matched);

figure
for i = 1:7
    plot3(xyzs{i}(:,1), xyzs{i}(:,2), xyzs{i}(:,3), 'r', 'linewidth', 2)
    hold on
end

for i = 1:7
    plot3(Coronary.skeleton{i}(:,1), Coronary.skeleton{i}(:,2), Coronary.skeleton{i}(:,3), 'b', 'linewidth', 2)
    hold on
end
legend('Red: Reconstructed','','','','','','','Blue: Original')
hold off

%% Backproject
[uv1_bp, uv2_bp] = backproject(xyzs, source1.P, source2.P);
% Plot the artery in both views
figure(1);
subplot(1, 2, 1);
for i = 1:num_seg
    plot(uv1_bp{i}(:,1), uv1_bp{i}(:,2), 'linewidth', 2)
    hold on;
end
xlabel("u1-values backproject")
ylabel("v1-values backproject")
axis equal
hold off;
title('View 1')

subplot(1, 2, 2);
for j = 1:num_seg
    plot(uv2_bp{j}(:,1), uv2_bp{j}(:,2), 'linewidth', 2)
    hold on;
end
xlabel("u2-values backproject")
ylabel("v2-values backproject")
axis equal
hold off;
title('View 2')

error_3d = rms(Coronary.skeleton, xyzs);
error_uv1 = rms(uv1, uv1_bp);
error_uv2 = rms(uv2, uv2_bp);
disp('error_3d:'); disp(error_3d);
disp('error_uv1:'); disp(error_uv1);
disp('error_uv2:'); disp(error_uv2);

%%
clear errors_xyzs; clear errors_uv1; clear errors_uv1;
errors_xyzs = zeros(1, 100);
errors_uv1 = zeros(1, 100);
errors_uv2 = zeros(1, 100);
errors_x = zeros(1, 100);
errors_y = zeros(1, 100);
errors_z = zeros(1, 100);

n_repet = 10;
for level = 1:100
    disp(level)
    repet_xyz0 = 0;
    repet_uv10 = 0;
    repet_uv20 = 0;
    repet_x0 = 0;
    repet_y0 = 0;
    repet_z0 = 0;

    for repet = 1:n_repet
        r = rand; % random number from uniform distribution in range (0,1)
        rnum = r + (sign(r-0.5)-1)/2; % rnum is either in range (-1, -0.5) or (0.5, 1)
        SID_noisy = SID*(1 + rnum*level/100); % noisy value of SID
        % generate new random value for next parameter
        r = rand; rnum = r + (sign(r-0.5)-1)/2;
        SOD_noisy = SOD*(1 + rnum*level/100); % noisy value of SOD
        % generate new random value for next parameter
        r = rand; rnum = r + (sign(r-0.5)-1)/2;
        DP_noisy = DP*(1 + rnum*level/100); % noisy value of DP
        % generate new random value for next parameter
        r = rand; rnum = r + (sign(r-0.5)-1)/2;
        alpha1_noisy = alpha1*(1 + rnum*level/100); % noisy value of alpha1
        % generate new random value for next parameter
        r = rand; rnum = r + (sign(r-0.5)-1)/2;
        beta1_noisy = beta1*(1 + rnum*level/100); % noisy value of beta1
        % generate new random value for next parameter
        r = rand; rnum = r + (sign(r-0.5)-1)/2;
        alpha2_noisy = alpha2*(1 + rnum*level/100); % noisy value of alpha2
        % generate new random value for next parameter
        r = rand; rnum = r + (sign(r-0.5)-1)/2;
        beta2_noisy = beta2*(1 + rnum*level/100); % noisy value of beta2

        % Build view
        [source1_noisy, detector1] = BuildViewGeom(SID_noisy, SOD_noisy, DP_noisy, alpha1_noisy, beta1_noisy, imsize);
        
        r = rand; % random number from uniform distribution in range (0,1)
        rnum = r + (sign(r-0.5)-1)/2; % rnum is either in range (-1, -0.5) or (0.5, 1)
        SID_noisy = SID*(1 + rnum*level/100); % noisy value of SID
        % generate new random value for next parameter
        r = rand; rnum = r + (sign(r-0.5)-1)/2;
        SOD_noisy = SOD*(1 + rnum*level/100); % noisy value of SOD
        % generate new random value for next parameter
        r = rand; rnum = r + (sign(r-0.5)-1)/2;
        DP_noisy = DP*(1 + rnum*level/100); % noisy value of DP
        [source2_noisy, detector2] = BuildViewGeom(SID_noisy, SOD_noisy, DP_noisy, alpha2_noisy, beta2_noisy, imsize);

        xyzs_noisy = reconstruct(source1_noisy.P./source1_noisy.P(3,4), source2_noisy.P./source2_noisy.P(3,4), uv1_matched, uv2_matched);
        [uv1_noisy, uv2_noisy] = backproject(xyzs_noisy, source1_noisy.P./source1_noisy.P(3,4), source2_noisy.P./source2_noisy.P(3,4));
        
        [repet_xyz, repet_x, repet_y, repet_z] = rms3D(xyzs_noisy, Coronary.skeleton);
        repet_xyz = repet_xyz0 + repet_xyz;
        repet_x = repet_x0 + repet_x;
        repet_y = repet_y0 + repet_y;
        repet_z = repet_z0 + repet_z;

        repet_uv1 = repet_uv10 + rms2D(uv1_noisy, uv1);
        repet_uv2 = repet_uv20 + rms2D(uv2_noisy, uv2);

        
%         errors_xyzs(level) = rms(xyzs_noisy, Coronary.skeleton);
%         errors_uv1(level) = rms(uv1_noisy, uv1);
%         errors_uv2(level) = rms(uv2_noisy, uv2);
    end
    errors_xyzs(level) = repet_xyz ./ n_repet;
    errors_uv1(level) = repet_uv1 ./ n_repet;
    errors_uv2(level) = repet_uv2 ./ n_repet;
    errors_x(level) = repet_x ./ n_repet;
    errors_y(level) = repet_y ./ n_repet;
    errors_z(level) = repet_z ./ n_repet;
end

figure
plot(errors_xyzs, 'linewidth', 2)
hold on
title('xyz error')
ylabel('rms')
xlabel('noise %')
ylim([0 300])
% figure
plot(errors_uv1, 'linewidth', 2)
title('uv1 error')
ylabel('rms')
xlabel('noise %')
ylim([0 300])
% figure
plot(errors_uv2, 'linewidth', 2)
hold off
title('uv2 error')
ylabel('rms')
xlabel('noise %')
ylim([0 300])
title('RMSE error for different noise levels')
legend('3d error','View 1 error','View 2 error')

figure;
plot(errors_xyzs, 'linewidth', 2)
hold on
title('xyz error')
ylabel('rms')
xlabel('noise %')
ylim([0 60])
plot(errors_x, 'linewidth', 2)
title('x error')
ylabel('rms')
xlabel('noise %')
ylim([0 60])
plot(errors_y, 'linewidth', 2)
title('y error')
ylabel('rms')
xlabel('noise %')
ylim([0 60])
plot(errors_z, 'linewidth', 2)
title('z error')
ylabel('rms')
xlabel('noise %')
ylim([0 60])
hold off;
legend('3d error','x error','y error', 'z error')
%% Question 5
% x: non-noisy
% X, K, T R: noisy
% iter: 1000
% Don't use d

clear errors_xyzs; clear errors_uv1; clear errors_uv1;
errors_xyzs = zeros(1, 100);
errors_uv1 = zeros(1, 100);
errors_uv2 = zeros(1, 100);
for level = 1:100
    disp(level)
    repet_xyz0 = 0;
    repet_uv01 = 0;
    repet_uv02 = 0;
    repet_x0 = 0;
    repet_y0 = 0;
    repet_z0 = 0;
    disp(level)
    repet_xyz = 0;
    repet_uv1 = 0;
    repet_uv2 = 0;
    repet_x = 0;
    repet_y = 0;
    repet_z = 0;

    for repet = 1:n_repet
        r = rand; % random number from uniform distribution in range (0,1)
        rnum = r + (sign(r-0.5)-1)/2; % rnum is either in range (-1, -0.5) or (0.5, 1)
        SID_noisy = SID*(1 + rnum*level/100); % noisy value of SID
        % generate new random value for next parameter
        r = rand; rnum = r + (sign(r-0.5)-1)/2;
        SOD_noisy = SOD*(1 + rnum*level/100); % noisy value of SOD
        % generate new random value for next parameter
        r = rand; rnum = r + (sign(r-0.5)-1)/2;
        DP_noisy = DP*(1 + rnum*level/100); % noisy value of DP
        % generate new random value for next parameter
        r = rand; rnum = r + (sign(r-0.5)-1)/2;
        alpha1_noisy = alpha1*(1 + rnum*level/100); % noisy value of alpha1
        % generate new random value for next parameter
        r = rand; rnum = r + (sign(r-0.5)-1)/2;
        beta1_noisy = beta1*(1 + rnum*level/100); % noisy value of beta1
        % generate new random value for next parameter
        r = rand; rnum = r + (sign(r-0.5)-1)/2;
        alpha2_noisy = alpha2*(1 + rnum*level/100); % noisy value of alpha2
        % generate new random value for next parameter
        r = rand; rnum = r + (sign(r-0.5)-1)/2;
        beta2_noisy = beta2*(1 + rnum*level/100); % noisy value of beta2

        % Build view
        [source1_noisy, detector1] = BuildViewGeom(SID_noisy, SOD_noisy, DP_noisy, alpha1_noisy, beta1_noisy, imsize);
        
        r = rand; % random number from uniform distribution in range (0,1)
        rnum = r + (sign(r-0.5)-1)/2; % rnum is either in range (-1, -0.5) or (0.5, 1)
        SID_noisy = SID*(1 + rnum*level/100); % noisy value of SID
        % generate new random value for next parameter
        r = rand; rnum = r + (sign(r-0.5)-1)/2;
        SOD_noisy = SOD*(1 + rnum*level/100); % noisy value of SOD
        % generate new random value for next parameter
        r = rand; rnum = r + (sign(r-0.5)-1)/2;
        DP_noisy = DP*(1 + rnum*level/100); % noisy value of DP
        [source2_noisy, detector2] = BuildViewGeom(SID_noisy, SOD_noisy, DP_noisy, alpha2_noisy, beta2_noisy, imsize);
        
        xyzs_noisy = reconstruct(source1_noisy.P, source2_noisy.P, uv1_matched, uv2_matched);
        
        [M1_better, ~, ~, ~, ~] = refine(uv1_matched, xyzs_noisy, source1_noisy);
        [M2_better, ~, ~, ~, ~] = refine(uv2_matched, xyzs_noisy, source2_noisy);
        
        xyzs_noisy_v2 = reconstruct(M1_better, M2_better, uv1_matched, uv2_matched);
        [uv1_noisy, uv2_noisy] = backproject(xyzs_noisy_v2, M1_better, M2_better);
        
        repet_xyz = rms3D(xyzs_noisy_v2, Coronary.skeleton);
        repet_xyz = repet_xyz0 + repet_xyz; 
        repet_uv1 = repet_uv1 + rms(uv1_noisy, uv1);
        repet_uv2 = repet_uv2 + rms(uv2_noisy, uv2);
        
%         errors_xyzs(level) = rms(xyzs_noisy, Coronary.skeleton);
%         errors_uv1(level) = rms(uv1_noisy, uv1);
%         errors_uv2(level) = rms(uv2_noisy, uv2);
    end
    errors_xyzs(level) = repet_xyz ./ n_repet;
    errors_uv1(level) = repet_uv1 ./ n_repet;
    errors_uv2(level) = repet_uv2 ./ n_repet;
end

%
figure
plot(errors_xyzs, 'linewidth', 2)
hold on
title('xyz error refined')
ylabel('rms')
xlabel('noise %')
ylim([0 300])
% figure
plot(errors_uv1, 'linewidth', 2)
title('uv1 error refined')
ylabel('rms')
xlabel('noise %')
ylim([0 300])
% figure
plot(errors_uv2, 'linewidth', 2)
title('uv2 error refined')
ylabel('rms')
xlabel('noise %')
ylim([0 300])
hold off
title('RMSE error after refinement for different noise levels')
legend('3d error','View 1 error','View 2 error')

% The 2d error seems better but the 3d error seems similar if not worse

%% Compare SID, SOD, DP...

clear errors_xyzs; clear errors_uv1; clear errors_uv1;
level = 20;
r = rand; % random number from uniform distribution in range (0,1)
rnum = r + (sign(r-0.5)-1)/2; % rnum is either in range (-1, -0.5) or (0.5, 1)
SID1_noisy = SID*(1 + rnum*level/100); % noisy value of SID
% generate new random value for next parameter
r = rand; rnum = r + (sign(r-0.5)-1)/2;
SOD1_noisy = SOD*(1 + rnum*level/100); % noisy value of SOD
% generate new random value for next parameter
r = rand; rnum = r + (sign(r-0.5)-1)/2;
DP1_noisy = DP*(1 + rnum*level/100); % noisy value of DP
% generate new random value for next parameter
r = rand; rnum = r + (sign(r-0.5)-1)/2;
alpha1_noisy = alpha1*(1 + rnum*level/100); % noisy value of alpha1
% generate new random value for next parameter
r = rand; rnum = r + (sign(r-0.5)-1)/2;
beta1_noisy = beta1*(1 + rnum*level/100); % noisy value of beta1
% generate new random value for next parameter
r = rand; rnum = r + (sign(r-0.5)-1)/2;
alpha2_noisy = alpha2*(1 + rnum*level/100); % noisy value of alpha2
% generate new random value for next parameter
r = rand; rnum = r + (sign(r-0.5)-1)/2;
beta2_noisy = beta2*(1 + rnum*level/100); % noisy value of beta2

% Build view
[source1_noisy, detector1] = BuildViewGeom(SID1_noisy, SOD1_noisy, DP1_noisy, alpha1_noisy, beta1_noisy, imsize);

r = rand; % random number from uniform distribution in range (0,1)
rnum = r + (sign(r-0.5)-1)/2; % rnum is either in range (-1, -0.5) or (0.5, 1)
SID2_noisy = SID*(1 + rnum*level/100); % noisy value of SID
% generate new random value for next parameter
r = rand; rnum = r + (sign(r-0.5)-1)/2;
SOD2_noisy = SOD*(1 + rnum*level/100); % noisy value of SOD
% generate new random value for next parameter
r = rand; rnum = r + (sign(r-0.5)-1)/2;
DP2_noisy = DP*(1 + rnum*level/100); % noisy value of DP
[source2_noisy, detector2] = BuildViewGeom(SID2_noisy, SOD2_noisy, DP2_noisy, alpha2_noisy, beta2_noisy, imsize);

xyzs_noisy = reconstruct(source1_noisy.P, source2_noisy.P, uv1_matched, uv2_matched);

[M1_better, SID1_ref, SOD1_ref, alpha1_ref, beta1_ref] = refine(uv1_matched, xyzs_noisy, source1_noisy);
[M2_better, SID2_ref, SOD2_ref, alpha2_ref, beta2_ref] = refine(uv2_matched, xyzs_noisy, source2_noisy);

xyzs_noisy_v2 = reconstruct(M1_better, M2_better, uv1_matched, uv2_matched);
[uv1_noisy, uv2_noisy] = backproject(xyzs_noisy_v2, M1_better, M2_better);

errors_xyzs(level) = rms(xyzs_noisy_v2, Coronary.skeleton);
errors_uv1(level) = rms(uv1_noisy, uv1);
errors_uv2(level) = rms(uv2_noisy, uv2);

y = [SID1_ref SID1_noisy SID; SID2_ref SID2_noisy SID; SOD1_ref SOD1_noisy SOD; SOD2_ref SOD2_noisy SOD;alpha1_ref alpha1_noisy alpha1; alpha2_ref alpha2_noisy alpha2; beta1_ref beta1_noisy beta1; beta2_ref beta2_noisy beta2];
X = categorical({'SID view1', 'SID view2', 'SOD view1', 'SOD view2', 'alpha view1', 'alpha view2', 'beta view1', 'beta view2'});
X = reordercats(X, {'SID view1', 'SID view2', 'SOD view1', 'SOD view2', 'alpha view1', 'alpha view2', 'beta view1', 'beta view2'});

figure
bar(X, y)
legend('Refined', 'Noisy', 'Ground truth')
% Refining does not suceed in finding the best params, but they are on
% average closer to the ground truth

%% Bonus
% Better matching (If 2 points on the same epipolar line, it's not 100%
% right all the time)
% Use more than 2 views
% Use a calibration tool

%% Question 6

% From the dicoms, we can extract the initial parameters for each view
% (SID, SOD, DP, alpha, beta). We could identify the different arteries in both views to facilitate the net steps.
% We then need to find matching points on the arteries
% in both views so that we can use those to compute the 3d reconstruction.
% To do that, we can compute the epipolar lines (using the initial
% parameters) to find where a perticular point in a view is in the other
% view (it would be beneficial to chose points that are easy to recognize
% (on an edge for example). Since the initial parameters are not perfect,
% the points chosen might not lie exactly on the epipolar line but chosing
% easy to recognize points would help. We could automate this process as
% done in this lab but that could sacrifice accuracy, especially where
% there are multiple arteries on the same epipolar line. With the matched
% points, we can then reconstruct the 3d points from the 2 views. At this
% point we could refine the initial parameters using a non linear solver,
% but this lab has shown that this helps getting better views from 3d
% reconstruction rather than helping improve the initial 3d reconstruction.

%% functions
function [M_x, SID_x, SOD_x, alpha_x, beta_x] = refine(uv_matched_x, xyzs_x, source_x)
    uv_matched_y = [uv_matched_x{1}; uv_matched_x{2}; uv_matched_x{3}; uv_matched_x{4}; uv_matched_x{5}; uv_matched_x{6}; uv_matched_x{7}];
    xyzs_y = [xyzs_x{1}; xyzs_x{2}; xyzs_x{3}; xyzs_x{4}; xyzs_x{5}; xyzs_x{6}; xyzs_x{7}];
    [K, R, t, rperr] = RefineCamParam(uv_matched_y, xyzs_y, source_x.K, source_x.R, source_x.T, 2^(-52), 100);
    
    extMat = [R t];
    M_x = K*extMat;
    
    DP = 0.1953;
    imsize = [1024 1024];
    [SID_x, SOD_x, alpha_x, beta_x] = GetDicomFromCalib(K, R, t, DP);

%     [source, detector] = BuildViewGeom(SID_x, SOD_x, DP, alpha_x, beta_x, imsize);
%     M_x = source.P;
%     disp(M_x)
end

function [uv1, uv2] = backproject(xyzs, M1, M2)

    % Reconstruct from 2D image from 3D data (Use skeleton data from Coronary)
%      M1 = source1.P;
%      M2 = source2.P;

    % Use p = M*P to get the u-v coordinates
    % P can be retrieved from the Coranary data
    num_seg = 7;
    uv1 = {};
    for i = 1:num_seg
        mat = [xyzs{i}(:,1)'; xyzs{i}(:,2)'; xyzs{i}(:,3)'; ones(size(xyzs{i}(:,3)))'];
        uv1{i} = (M1 * mat)'; 
        uv1{i} = uv1{i} ./ uv1{i}(:, end);
    end

    uv2 = {};
    for i = 1:num_seg
        mat = [xyzs{i}(:,1)'; xyzs{i}(:,2)'; xyzs{i}(:,3)'; ones(size(xyzs{i}(:,3)))'];
        uv2{i} = (M2 * mat)'; 
        uv2{i} = uv2{i} ./ uv2{i}(:, end);
    end
end
    
function [pts_3d] = reconstruct(M1, M2, uv1, uv2)
    pts_3d = {};
    for i = 1:7
        shape = size(uv1{i});
        n = shape(1);
        pts_3d{i} = [];
        pts_3d{i} = zeros([n, 3]);
        for j = 1:n
            A = zeros([4 3]);
            A(1, :) = [-uv1{i}(j, 1) .* M1(3, 1) + M1(1, 1), -uv1{i}(j, 1) .* M1(3, 2) + M1(1, 2), -uv1{i}(j, 1) .* M1(3, 3) + M1(1, 3)];
            A(2, :) = [-uv1{i}(j, 2) .* M1(3, 1) + M1(2, 1), -uv1{i}(j, 2) .* M1(3, 2) + M1(2, 2), -uv1{i}(j, 2) .* M1(3, 3) + M1(2, 3)];
            A(3, :) = [-uv2{i}(j, 1) .* M2(3, 1) + M2(1, 1), -uv2{i}(j, 1) .* M2(3, 2) + M2(1, 2), -uv2{i}(j, 1) .* M2(3, 3) + M2(1, 3)];
            A(4, :) = [-uv2{i}(j, 2) .* M2(3, 1) + M2(2, 1), -uv2{i}(j, 2) .* M2(3, 2) + M2(2, 2), -uv2{i}(j, 2) .* M2(3, 3) + M2(2, 3)];
            A_inv = pinv(A);
            B = [uv1{i}(j, 1).*  M1(3, 4) - M1(1, 4), uv1{i}(j, 2).*  M1(3, 4) - M1(2, 4), uv2{i}(j, 1).*  M2(3, 4) - M2(1, 4), uv2{i}(j, 2).*  M2(3, 4) - M2(2, 4)]';
            xyz = A_inv * B;
            pts_3d{i}(j, :) = xyz;
        end
    end
end

function [diff] = calculate_dist(Z_ref, Z_val)
    ref = [Z_ref{1}'; Z_ref{2}'; Z_ref{3}'; Z_ref{4}'; Z_ref{5}'; Z_ref{6}'; Z_ref{7}'];
    val = [Z_val{1}; Z_val{2}; Z_val{3}; Z_val{4}; Z_val{5}; Z_val{6}; Z_val{7}];
    n = size(ref);
    diff = sqrt(sum((((val - ref) .^ 2)) ./ n(1)));
end

function [rms_xyz, rms_x, rms_y, rms_z] = rms3D(ref, pred)

    x0 = [ref{1}(:, 1); ref{2}(:, 1); ref{3}(:, 1); ref{4}(:, 1); ref{5}(:, 1); ref{6}(:, 1); ref{7}(:, 1)];
    x = [pred{1}(:, 1); pred{2}(:, 1); pred{3}(:, 1); pred{4}(:, 1); pred{5}(:, 1); pred{6}(:, 1); pred{7}(:, 1)];
    y0 = [ref{1}(:, 2); ref{2}(:, 2); ref{3}(:, 2); ref{4}(:, 2); ref{5}(:, 2); ref{6}(:, 2); ref{7}(:, 2)];
    y = [pred{1}(:, 2); pred{2}(:, 2); pred{3}(:, 2); pred{4}(:, 2); pred{5}(:, 2); pred{6}(:, 2); pred{7}(:, 2)];
    z0 = [ref{1}(:, 3); ref{2}(:, 3); ref{3}(:, 3); ref{4}(:, 3); ref{5}(:, 3); ref{6}(:, 3); ref{7}(:, 3)];
    z = [pred{1}(:, 3); pred{2}(:, 3); pred{3}(:, 3); pred{4}(:, 3); pred{5}(:, 3); pred{6}(:, 3); pred{7}(:, 3)];
   
    n = size(y); n = n(1);
    
    rms_x = sqrt(sum(((x - x0).^2) ./ n));
    rms_y = sqrt(sum(((y - y0).^2) ./ n));
    rms_z = sqrt(sum(((z - z0).^2) ./ n));

    rms_xyz = sqrt(sum((((x - x0).^2) + ((y - y0).^2) + ((z - z0).^2)) ./ n));
    
end

function [rms_uv] = rms2D(ref, pred)
    u0 = [ref{1}(:, 1); ref{2}(:, 1); ref{3}(:, 1); ref{4}(:, 1); ref{5}(:, 1); ref{6}(:, 1); ref{7}(:, 1)];
    u = [pred{1}(:, 1); pred{2}(:, 1); pred{3}(:, 1); pred{4}(:, 1); pred{5}(:, 1); pred{6}(:, 1); pred{7}(:, 1)];
    v0 = [ref{1}(:, 2); ref{2}(:, 2); ref{3}(:, 2); ref{4}(:, 2); ref{5}(:, 2); ref{6}(:, 2); ref{7}(:, 2)];
    v = [pred{1}(:, 2); pred{2}(:, 2); pred{3}(:, 2); pred{4}(:, 2); pred{5}(:, 2); pred{6}(:, 2); pred{7}(:, 2)];
    n = size(v); n = n(1);
    rms_uv = sqrt(sum(((u - u0).^2) + ((v - v0).^2) ./ n));
end