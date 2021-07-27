clear; close all

v = VideoReader('april21.avi');
 
i = 0;
meanValue = 0;
horCuts = [];
Ar = [];
Br = [];
Al = [];
Bl = [];

while hasFrame(v)
    frame = readFrame(v);
    
    i = i + 1;
    if (mod(i,1) == 0)
        
%%%%%%%%%%%         Horizon ROI method      %%%%%%%%%%%%%%%
% 
         %  [cutFrame,row, horCuts] = cutUpperHor(frame,horCuts);
         %  row = 1;   
%           
         %  imshow(cutFrame)
%           [st, meanValue] = shadowImageFunctionHor(cutFrame,row,i,meanValue);
% %         
%%%%%%%%%%%         Lanes ROI method      %%%%%%%%%%%%%%%      
% 
         [cutFrame, ls2, Ar ,Br] = cutRigtLane(frame, Ar, Br);
         [cutFrame, ls1, Al ,Bl] = cutLeftLane(cutFrame, Al, Bl);
%           
%          [st, meanValue] = shadowImageFunctionLines(cutFrame,ls1,ls2,i,meanValue);
%  
%%%%%%%%%%%         Slat and Pepper Noise ROI method      %%%%%%%%%%%%%%%
% 
         % frame_noisy = salt_pepper_noise(frame); 
          %frame_noisy = imnoise(frame,'salt & pepper');
         %frame_cleaned = median_filter(frame_noisy); 
         % [cutFrame,row, horCuts] = cutUpperHor(frame_cleaned ,horCuts);        
%           
   %       imshow(cutFrame)

%%%%%%%%%%%      Gaussian Noise ROI method      %%%%%%%%%%%%%%%
% 
%           [frame_noisy] = white_gaussian_noise(frame,10); 
%           %sig=100; V=(sig/256)^2    % 
%           %frame_noisy = imnoise(frame,'gaussian');
%           
%           %frame = moving_average_filter(frame_noisy); 
%           [cutFrame,row, horCuts] = cutUpperHor(frame_noisy,horCuts);
%           row = 1;   
%           
           imshow(cutFrame)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        

%        colorLineDetector(frame,row);
%        imshowpair(frame,Gy,'montage')

%        st = shadowImageFunction(cutFrame,row,ls);
       
%        hold on
%        for k = 1 :length(st)
%           thisBB = st(k).BoundingBox;
%           rectangle('Position', [thisBB(1),thisBB(2),thisBB(3),thisBB(4)],...
%           'EdgeColor','r','LineWidth',2 )
%        end
%        pause(0.01);
     end
end

function [frame, row, horCuts] = cutUpperHor(frame,horCuts)
        [~,Gy] = imgradientxy(im2double(rgb2gray(frame)));
        Gy = imadjust(Gy); % den xreazetei
        [~,row] = max(sum(Gy,2));
        horCuts = [horCuts ; row];
        row = round(mean(horCuts));
        frame(1:row,:,:) = 0;

end

function [frame, ls, Amat , Bmat] = cutLeftLane(frame,Amat , Bmat)
        [~,ls] = lineDetector(frame);
        
        x1 = ls(2,1);
        y1 = ls(2,2);
        x0 = ls(1,1);
        y0 = ls(1,2);
        

        A = (y1 -y0)/(x1 - x0); 
        B = -(x0) * (y1 -y0)/(x1 - x0) + y0;
        Amat = [Amat ; A];
        Bmat = [Bmat ; B];
        Acut = mean(Amat); 
        Bcut = mean(Bmat);
       [M,N] = size(rgb2gray(frame));
        for i=1:M
            for j=1:N
                if (Acut*j - i + Bcut >= 0)
                    frame(i,j,:) = 0;
                end
            end
        end
end

function [frame, ls, Amat , Bmat] = cutRigtLane(frame, Amat , Bmat)
        [lines,~] = lineDetector(frame);
        
        if(size(Amat) == [0 0])
           Acut = 0 ;
           Bcut = 0 ;
        end   
        
        max_len = 0;     
        exists = 0;
        ls = [0 0 ; 0 0 ];  
        
        for k = 1:length(lines)
           points = [lines(k).point1; lines(k).point2];
           A = (points(2,2) - points(1,2))/(points(2,1) - points(1,1));
           
           if (A > 0)
               exists = 1;
               len = norm(lines(k).point1 - lines(k).point2);
               if ( len > max_len)                  
                  max_len = len;
                  ls = points ;
               end
           end          
        end
        
        if (exists == 1)
            x1 = ls(2,1);
            y1 = ls(2,2);
            x0 = ls(1,1);
            y0 = ls(1,2);
       
            A = (y1 -y0)/(x1 - x0); 
            B = -(x0) * (y1 -y0)/(x1 - x0) + y0;
            
            Amat = [Amat ; A];
            Bmat = [Bmat ; B];
            Acut = mean(Amat); 
            Bcut = mean(Bmat);
        else 
            Acut = mean(Amat); 
            Bcut = mean(Bmat);
        end
       
        [M,N] = size(rgb2gray(frame));
        for i=1:M
            for j=1:N
                if (Acut*j - i + Bcut >= 0)
                    frame(i,j,:) = 0;
                end
            end
         end
end

function meanValue = computeRoadSection(frame,row)
    [M,N] = size(frame);
    roadSection = [];
    for i=1:M
        for j=1:N
            if(i > row)
                roadSection = [roadSection; frame(i,j)];
            end
        end
    end
    [m,s] = normfit(roadSection);
    meanValue = m - 6*s;
end

function meanValue = computeRoadSectionLines(frame,A1,B1,A2,B2)
    [M,N] = size(frame);
    roadSection = [];
    for i=1:M
        for j=1:N
            if((A2*j - i + B2<= 0)&& (A1*j - i + B1 <= 0))
                roadSection = [roadSection; frame(i,j)];
            end
        end
    end
    [m,s] = normfit(roadSection);
    meanValue = m - 4*s;
end

function meanValue = computeRoadSectionHor(frame,row)
    [M,N] = size(frame);
    roadSection = [];
    
    for i=row:M
        for j=1:N        
               roadSection = [roadSection; frame(i,j)];         
        end
    end
    
    [m,s] = normfit(roadSection);
    meanValue = m - s ;
end

function [shadowImage, meanValue] = computeShadowImageHor(frame,row,i, meanValue)

        frame = rgb2gray(frame);
        frame = imgaussfilt(frame,1);
        [M,N] = size(frame);
         
        if (i == 1)  
            meanValue = computeRoadSectionHor(frame,row);
        end
        if (mod(i,30) == 0)
            meanValue = computeRoadSectionHor(frame,row);
        end
               
        shadowImage = zeros(M,N);
        
        for i=row:M
            for j=1:N
                
                if (frame(i,j) < meanValue)
                    shadowImage(i,j) = 255;
                end
                
            end
        end
end

function [shadowImage, meanValue] = computeShadowImageLines(frame,ls1,ls2,i,meanValue)

        frame = rgb2gray(frame);
        frame = imgaussfilt(frame,1);
        [M,N] = size(frame);
        
        x1 = ls1(2,1);
        y1 = ls1(2,2);
        x0 = ls1(1,1);
        y0 = ls1(1,2);
              
        x3 = ls2(2,1);
        y3 = ls2(2,2);
        x2 = ls2(1,1);
        y2 = ls2(1,2);
        
        if (ls1 ~= -1)           
            A1 = (y1 -y0)/(x1 - x0); 
            B1 = -(x0) * (y1 -y0)/(x1 - x0) + y0;
        else 
            ls1 = -1;
            A1 = 0;
            B1 = 0;
        end
        
        if (ls2(1,1) == 0)
            %ls2 = -1;
            A2 = 0;
            B2 = 0;
        else  
            A2 = (y3 -y2)/(x3 - x2); 
            B2 = -(x2) * (y3 -y2)/(x3 - x2) + y2;
        end
        
       %meanValue = computeRoadSectionLines(frame,A1,B1,A2,B2);
        if (i == 1)  
           meanValue = computeRoadSectionLines(frame,A1,B1,A2,B2);
        end
        if (mod(i,10) == 0)
           meanValue = computeRoadSectionLines(frame,A1,B1,A2,B2);
        end
%         meanValue = mean(roadSection) - 29;
        
%         roadSection = frame(row:end,:);
%         meanValue = mean(roadSection(:));

        shadowImage = zeros(M,N);
        for i=1:M
            for j=1:N
     
               if((A2*j - i + B2 <= 5) && (A1*j - i + B1 <= -5))
                    if (frame(i,j) < meanValue)
                        shadowImage(i,j) = 255;
                    end
                end
            end
        end
end

function [shadowImage] = computeShadowImage(frame,row,ls)
        frame = rgb2gray(frame);
        frame = imgaussfilt(frame,1);
        [M,N] = size(frame);
        
        if (ls ~= -1)
            A = (ls(2,2) - ls(1,2))/(ls(2,1) - ls(1,1)); 
            B = -(ls(1,1) * (ls(2,2) - ls(1,2))/(ls(2,1) - ls(1,1))) + ls(1,2);
        else 
            ls = -1;
            A = 0;
            B = 0;
        end
        
        meanValue = computeRoadSection(frame,row,A,B);
%         meanValue = mean(roadSection) - 29;
        
%         roadSection = frame(row:end,:);
%         meanValue = mean(roadSection(:));

        shadowImage = zeros(M,N);
        for i=1:M
            for j=1:N
               if(i > row && (A*j - i + B <= -5))
                    if (frame(i,j) < meanValue)
                        shadowImage(i,j) = 255;
                    else
                        shadowImage(i,j) = 0;
                    end
                else 
                    shadowImage(i,j) = 0;
               end
            end
        end
end

function [st, meanValue] = shadowImageFunctionHor(frame,row,i, meanValue)
        [shadowImage, meanValue] = computeShadowImageHor(frame,row,i,meanValue);
        [~,edgeImage] = imgradientxy(im2double(rgb2gray(frame)),'prewitt');
         andImage = shadowImage & edgeImage;
%        st =1;

        imshow(andImage)
        st = regionprops(andImage , 'BoundingBox' );
end

function [st, meanValue] = shadowImageFunctionLines(frame,ls1,ls2,i,meanValue)

        [shadowImage,meanValue] = computeShadowImageLines(frame,ls1,ls2,i,meanValue);
        [~,edgeImage] = imgradientxy((rgb2gray(frame)),'prewitt');
         andImage = shadowImage & edgeImage;
%        st =1;
        imshow(andImage)
        st = regionprops(andImage , 'BoundingBox' );
end

function [st] = shadowImageFunction(frame,row,ls)
        shadowImage = computeShadowImage(frame,row,ls);
        [~,edgeImage] = imgradientxy(im2double(rgb2gray(frame)));
         andImage = shadowImage & edgeImage;
%        st =1;

        imshow(andImage)
        st = regionprops(andImage , 'BoundingBox' );
end

function colorLineDetector(frame,row)
    ycbcr = rgb2ycbcr(frame);
    [M,N] = size(rgb2gray(frame));
    ycbcr(1:row,:,:) = 0;
    ycbcr(row,:,3) = 255;
    ycbcr(row,:,1) = 200;
    for i=row:M
        for j=1:N
            if(ycbcr(i,j,1)> 180 && ycbcr(i,j,3)> 150)
                ycbcr(i,j,:) = 255;
            else
                ycbcr(i,j,:) = 0;
            end
        end
    end
    imshow(ycbcr);
end

function [lines, xy_long] = lineDetector(frame)
    if (length(size(frame)) == 3)
        frame = rgb2gray(frame);
    end
    BW = edge(frame,'canny');
    [H,T,R] = hough(BW);
    % imshow(H,[],'XData',T,'YData',R,...
    %             'InitialMagnification','fit');
    % xlabel('\theta'), ylabel('\rho');
    % axis on, axis normal, hold on;
    P  = houghpeaks(H,5,'threshold',ceil(0.3*max(H(:))));
    lines = houghlines(BW,T,R,P,'FillGap',5,'MinLength',7);
%     imshow(rotI), hold on
    max_len = 0;
    for k = 1:length(lines)
       xy = [lines(k).point1; lines(k).point2];
%        plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');
% 
%        % Plot beginnings and ends of lines
%        plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
%        plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');

       len = norm(lines(k).point1 - lines(k).point2);
       if ( len > max_len)
          max_len = len;
          xy_long = xy;
       end
    end
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

function [f_clean] = moving_average_filter(f_noisy)
    f = double(f_noisy);
    [m ,n] = size(f);
    for i = 1:m
        for j = 1:n  
            xmin = max(1,i-1);
            xmax = min(m,i+1);
            ymin = max(1,j-1);
            ymax = min(n,j+1);
            temp = f_noisy(xmin:xmax, ymin:ymax);    
            f_noisy(i,j) = mean(temp(:));
        end
    end
    f_clean = f_noisy;
    f_clean = uint8(f_clean);
end

function [imgSPN] = SPNoiseAddition(img, saltPercent, pepperPercent)
    [N,M] = size(img);
    img = reshape(img,N*M,1);
    imgSPN = img;
    for i=1:N*M
            random = rand(1);
            if (random < saltPercent)
                imgSPN(i) = 0;
            elseif (random >= saltPercent && random < saltPercent + pepperPercent)
                imgSPN(i) = 1;
            else 
                imgSPN(i) = img(i);
            end
    end
    imgSPN = reshape(imgSPN,N,M);
end

