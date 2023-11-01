clc;close all;clear

%% Simulation setup
% Anchor locations
A_Loc = [0,0;
        2, 0;
        3,1.732;
        2,2*1.732;
        0,2*1.732;
        -1,1.732];

% Active anchor and passive anchor designation
ref_anc_id = 1;
helper_anc_id = 2:size(A_Loc,1);

% Test tag locations
test_tag_x = -4:0.25:6;
test_tag_y = -3:0.25:7;

% Spoofing distance
spoofing_range = [-2,-1,-0.5,0,0.5,1,2];

% Ranging noise std
range_noise_std = 0.05; % 50cm

% Number of noisy ranges for each tag locations
num_noisy_measurements = 100;

detection_metric = zeros([num_noisy_measurements, length(spoofing_range), length(test_tag_x),length(test_tag_y)]);

%% Simulation of range spoofing and detection metric
fsolve_options = optimoptions('fsolve','Display','off','Algorithm','levenberg-marquardt');
for tag_x_idx=1:length(test_tag_x)
    tag_x_idx
    for tag_y_idx=1:length(test_tag_y)
        tag_location = [test_tag_x(tag_x_idx),test_tag_y(tag_y_idx)];
        true_range = vecnorm(tag_location-A_Loc,2,2);
        noisy_range = true_range+range_noise_std*randn([length(true_range),num_noisy_measurements]);

        for spoof_idx = 1:length(spoofing_range)
            d_vec = noisy_range+spoofing_range(spoof_idx);
            a_loc = A_Loc;
            parfor noise_idx = 1:num_noisy_measurements
                fun = @(x)fsolve_toa(x, d_vec(:,noise_idx), a_loc);
                tag_loc = fsolve(fun, [1,1],fsolve_options);
                d_vec_predicted = vecnorm( tag_loc-a_loc,2,2);
                detection_metric(noise_idx, spoof_idx, tag_x_idx,tag_y_idx) = rmse( d_vec_predicted, d_vec(:,noise_idx) );
            end
        end
        
    end
end

%% 

spoofing_range_idx_for_plotting = [4,5,6];

figure;sgtitle("Ranging Noise STD="+num2str(range_noise_std)+"m")
test_threshold = (0.05:0.015:0.095);
for spoofing_idx=1:length(spoofing_range_idx_for_plotting)
    for t=1:length(test_threshold)
        subplot(length(spoofing_range_idx_for_plotting),length(test_threshold),(spoofing_idx-1)*length(test_threshold)+t);hold on
        % Threshold based detection
        fp_map = 100*squeeze( sum(detection_metric(:,spoofing_range_idx_for_plotting(spoofing_idx),:,:)>test_threshold(t),1)/size(detection_metric,1) ); %squeeze(mean(metric2(:,spoof_idx,:,:),1));
        % Heatmap of detection percentage map
        imagesc(test_tag_x, test_tag_y, fp_map,[0,100]);
        %Show anchor locations
        scatter(A_Loc(2:end,1),A_Loc(2:end,2),80,'filled','kd')
        scatter(A_Loc(1,1),A_Loc(1,2),80,'filled','gd')
    
        h=colorbar;h.Label.String="Spoofing detection rate (%)";    
        axis square
        title({"Threshold="+num2str(test_threshold(t)*100)+"cm","Spoofing dist="+num2str(spoofing_range(spoofing_range_idx_for_plotting(spoofing_idx)))+"m"})
    end

end


