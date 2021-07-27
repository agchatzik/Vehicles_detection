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
        
%%%%%%%%%%%         Lanes ROI method      %%%%%%%%%%%%%%%                
      
            [SubFrame, ls2, Ar ,Br] = cutRigtLane(frame, Ar, Br);
            [SubFrame, ls1, Al ,Bl] = cutLeftLane(SubFrame, Al, Bl);
          
            [meanValue] = VehicleMaskFunctionLines(SubFrame,ls1,ls2,i,meanValue);
   
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

function meanValue = computeRoadSectionLines(frame,A1,B1,A2,B2)
    [M,N] = size(frame);
    roadSection = [];
    for i= 1:M 
        for j=1:N
            if((A2*j - i + B2 <= 0) && (A1*j - i + B1 <= 0))
                roadSection = [roadSection; frame(i,j)];
            end
        end
    end
    [m,s] = normfit(roadSection);
    meanValue = m - 3*s;
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
            A2 = 0;
            B2 = 0;
        else  
            A2 = (y3 -y2)/(x3 - x2); 
            B2 = -(x2) * (y3 -y2)/(x3 - x2) + y2;
        end
        
        if (i == 1)  
           meanValue = computeRoadSectionLines(frame,A1,B1,A2,B2);
        end
        if (mod(i,10) == 0)
           meanValue = computeRoadSectionLines(frame,A1,B1,A2,B2);
        end
        
        shadowImage = zeros(M,N);
        for i= 1:M 
            for j=1:N
     
               if((A2*j - i + B2 <= 5) && (A1*j - i + B1 <= -5))
                    if (frame(i,j) < meanValue)
                        shadowImage(i,j) = 255;
                    end
                end
            end
        end
end

function [meanValue] = VehicleMaskFunctionLines(frame,ls1,ls2,i,meanValue)

        [shadowImage,meanValue] = computeShadowImageLines(frame,ls1,ls2,i,meanValue);
        [~,edgeImage] = imgradientxy((rgb2gray(frame)),'prewitt');
        andImage = shadowImage & edgeImage;

        imshow(andImage)  
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

function [ycbcr] = LaneDetector(frame,row)
    ycbcr = rgb2ycbcr(frame);
    
    [M,N] = size(rgb2gray(frame));
    
    ycbcr(1:row,:,:) = 0;
    ycbcr(row,:,3) = 255;
    ycbcr(row,:,1) = 200;
    
    ThreshY = 180;
    ThreshCr = 150;
    
    for i=row:M
        for j=1:N
            if(ycbcr(i,j,1)> ThreshY && ycbcr(i,j,3)> ThreshCr)
                ycbcr(i,j,:) = 255;
            else
                ycbcr(i,j,:) = 0;
            end
        end
    end
    
    imshow(ycbcr);
end


