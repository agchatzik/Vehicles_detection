clear; close all

v = VideoReader('april21.avi');
 
i = 0;
meanValue = 0;
horCuts = [];

while hasFrame(v)
    frame = readFrame(v);
    
    i = i + 1;
    if (mod(i,1) == 0)
        
%%%%%%%%%%%         Slat and Pepper Noise ROI method      %%%%%%%%%%%%%%%
 
            frame_noisy = salt_pepper_noise(frame);

            frame_cleaned = median_filter(frame_noisy);
            
            [SubFrame,row, horCuts] = cutHorizon(frame_cleaned ,horCuts); 
            
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

function [ f_noisy ] = salt_pepper_noise( f )
    f = double(f);
    [M ,N, K] = size(f);
    for i=1:M;
        for j=1:N;
            if (rand<=0.1)
                if (rand>=0.066)
                     f(i,j,1) = 0;
                elseif(rand>=0.33)
                     f(i,j,2) = 0;
                else
                     f(i,j,3) = 0;
                end
            elseif (rand>0.1 && rand<=0.2)
                 if (rand>=0.166)
                     f(i,j,1) = 255;
                elseif(rand>=0.133)
                     f(i,j,2) = 255;
                else
                     f(i,j,3) = 255;
                end      
            end
        end
    end
    f_noisy = f;
    f_noisy = uint8(f_noisy);
end

function [f_clean] = median_filter( f_noisy )
    f = double(f_noisy);
    [M ,N] = size(f);
    for i = 1:M
        for j = 1:N  
            xmin = max(1,i-1);
            xmax = min(M,i+1);
            ymin = max(1,j-1);
            ymax = min(N,j+1);
            temp = f_noisy(xmin:xmax, ymin:ymax);    
            f_noisy(i,j) = median(temp(:));
        end
    end
    f_clean = f_noisy;
    f_clean = uint8(f_clean);
end



