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

%           imshow(SubFrame)
           [meanValue] = VehicleMaskFunctionHor(SubFrame,row,i,meanValue);
        
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

function [meanValue] = computeRoadSectionHor(frame,row)
    [M,N] = size(frame);
    roadSection = [];
    
    for i=row:M
        for j=1:N        
               roadSection = [roadSection; frame(i,j)];         
        end
    end
    
    [m,s] = normfit(roadSection);
    meanValue = m - 7*s ;
end

function [shadowImage, meanValue] = computeShadowImageHor(frame,row,i, meanValue)

        frame = rgb2gray(frame);
        frame = imgaussfilt(frame,0.5);
        [M,N] = size(frame);
         
        if (i == 1)  
            meanValue = computeRoadSectionHor(frame,row);
        end
        if (mod(i,25) == 0)
            meanValue = computeRoadSectionHor(frame,row);
        end
               
        shadowImage = zeros(M,N);
        
        for i= row:M
            for j=1:N
                
                if (frame(i,j) < meanValue)
                    shadowImage(i,j) = 255;
                end
                
            end
        end
end

function [meanValue] = VehicleMaskFunctionHor(frame,row,i, meanValue)

        [shadowImage, meanValue] = computeShadowImageHor(frame,row,i,meanValue);
        
        [~,edgeImage] = imgradientxy(im2double(rgb2gray(frame)),'sobel');
        
        andImage = shadowImage & edgeImage;
        imshow(andImage)
end

