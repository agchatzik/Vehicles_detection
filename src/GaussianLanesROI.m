
clear; close all

v = VideoReader('april21.avi');
 
i = 0;
meanValue = 0;

Ar = [];
Br = [];
Al = [];
Bl = [];

while hasFrame(v)
    frame = readFrame(v);
    
    i = i + 1;
    if (mod(i,1) == 0)

%%%%%%%%%%%      Gaussian Noise ROI method      %%%%%%%%%%%%%%%
% % 
            [frame_noisy] = white_gaussian_noise(frame,15); 
             
%           frame_cleaned = moving_average_filter(frame_noisy); 
   
            [SubFrame, ls2, Ar ,Br] = cutRigtLane(frame_noisy, Ar, Br);
            [SubFrame, ls1, Al ,Bl] = cutLeftLane(SubFrame, Al, Bl);
        
            imshow(SubFrame)
     end
end
    
%%%%%%%%%%%%%%%%%%%%%%%%  FUNCTIONS  %%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    
function [frame, ls, Amat , Bmat] = cutRigtLane(frame, Amat , Bmat)

        [lines] = lineDetector(frame);
        
        if(size(Amat) == [0 0])
           Acut = 0 ;
           Bcut = 0 ;
        end   
        
        Amax = 50000;
                exists = 0;
        ls = [0 0 ; 0 0 ];  
        
        for k = 1:length(lines)
           points = [lines(k).point1; lines(k).point2];
           A = (points(2,2) - points(1,2))/(points(2,1) - points(1,1));
           
           if (A > 0)
               exists = 1;
%               len = norm(lines(k).point1 - lines(k).point2);
                if (A < Amax)
            %   if ( len > max_len)                  
            %      max_len = len;
                  Amax = A;
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

function [frame, ls, Amat , Bmat] = cutLeftLane(frame, Amat , Bmat)

        [lines] = lineDetector(frame);
        
        if(size(Amat) == [0 0])
           Acut = 0 ;
           Bcut = 0 ;
        end
        
        Amin = 0;
        exists = 0;
        ls = [0 0 ; 0 0 ];  
        
        for k = 1:length(lines)
           points = [lines(k).point1; lines(k).point2];
           A = (points(2,2) - points(1,2))/(points(2,1) - points(1,1));
           
           if (A < 0)
               exists = 1;
%       
                if (A < Amin)
            %   if ( len > max_len)                  
            %      max_len = len;
                  Amin = A;
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

function [lines] = lineDetector(frame)

    if (length(size(frame)) == 3)
        frame = rgb2gray(frame);
    end
    
    %[ycbcr] = colorLineDetector(frame,row)
      
    BW = edge(frame,'canny');
    [H,T,R] = hough(BW);
  
    P  = houghpeaks(H,5,'threshold',ceil(0.3*max(H(:))));
    lines = houghlines(BW,T,R,P,'FillGap',5,'MinLength',7);
    
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
