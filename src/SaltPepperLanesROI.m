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
        
  
%%%%%%%%%%%         Slat and Pepper Noise ROI method      %%%%%%%%%%%%%%%
% 
             frame_noisy = salt_pepper_noise(frame); 
%           
             [SubFrame, ls2, Ar ,Br] = cutRigtLane(frame_noisy, Ar, Br);
             [SubFrame, ls1, Al ,Bl] = cutLeftLane(SubFrame, Al, Bl);
%             
%            
%            frame_cleaned = median_filter(frame_noisy); 
%            [SubFrame, ls2, Ar ,Br] = cutRigtLane(frame_cleaned, Ar, Br);
%            [SubFrame, ls1, Al ,Bl] = cutLeftLane(SubFrame, Al, Bl);
        
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
