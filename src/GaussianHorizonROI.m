clear; close all

v = VideoReader('april21.avi');
 
i = 0;
meanValue = 0;
horCuts = [];

while hasFrame(v)
    frame = readFrame(v);
    
    i = i + 1;
    if (mod(i,1) == 0)
        
         
%%%%%%%%%%%      Gaussian Noise ROI method      %%%%%%%%%%%%%%%
% % 
         [frame_noisy] = white_gaussian_noise(frame,15); 
         [SubFrame,row, horCuts] = cutHorizon(frame_noisy,horCuts);
%              

         imshow(SubFrame)
     end
end

%%%%%%%%%%%%%%%%% FUNCTIONS %%%%%%%%%%%%%%%%%%%%%

function [frame, row, horCuts] = cutHorizon(frame,horCuts)

        [~,Gy] = imgradientxy(im2double(rgb2gray(frame)));
        
        Gy = imadjust(Gy); % den xreazetei
        [~,row] = max(sum(Gy,2));
        horCuts = [horCuts ; row];
        row = ceil(mean(horCuts));
        frame(1:row,:,:) = 0;
end


function [f_noisy] = white_gaussian_noise(f , SNR)
    f = double(f);
    [M,N,K] = size(f);
    mean_f = mean(f(:))^2;
    var_noise = mean_f * 10 ^ ( - SNR / 10);
    
    noise_R = sqrt(var_noise)*(floor(randn(M,N))) + 0;
    noise_G = sqrt(var_noise)*(floor(randn(M,N))) + 0;
    noise_B = sqrt(var_noise)*(floor(randn(M,N))) + 0;
    
    f_noisy(:,:,1) = f(:,:,1) + noise_R;
    f_noisy(:,:,2) = f(:,:,2) + noise_G;
    f_noisy(:,:,3) = f(:,:,3) + noise_B;
     
    f_noisy = uint8(f_noisy);
end


