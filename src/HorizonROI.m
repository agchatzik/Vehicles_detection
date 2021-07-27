clear; close all

v = VideoReader('april21.avi');
 
i = 0;
meanValue = 0;
horCuts = [];

while hasFrame(v)
    frame = readFrame(v);
    
    i = i + 1;
    if (mod(i,1) == 0)
        
%%%%%%%%%%%         Horizon ROI method      %%%%%%%%%%%%%%%
% % 
           [SubFrame,row, horCuts] = cutHorizon(frame,horCuts);      

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

