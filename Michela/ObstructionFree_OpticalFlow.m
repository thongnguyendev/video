% OBSTRUCTION FREE PHOTOGRAPHY
%Implementation by Michela Testolina
%Data Hiding course
%2016

clear all;
close all;
clc;

%-------------Initialization-------------%

video=VideoReader('dorm1.mp4');
numTotFrames=get(video,'numberOfFrames');


%extract the reference frame (a frame in the middel, to have a shorter motion vector -> more precise)
refFrame = read(video, 45);
[rows, columns, numberOfColorChannels] = size(refFrame); %Get dimentions
I1=rgb2gray(refFrame);
I1_edge=double(edge(I1,'Canny', 0.1));

%Initialize the background
numFrame=0;
for x=1:rows
    for y=1:columns
        I_background(x,y)=255;
    end
end



numFrame=0;
for k = 1 : 15 : numTotFrames  %Extract a frame every 30
    
    if k~=46
        
        numFrame=numFrame+1;
        
        I2 = rgb2gray(read(video, k));
        
        filename=['./frames/' num2str(numFrame) '.png'];
        imwrite(I2,filename);
        
        I2_edge=double(edge(I2,'Canny', 0.1));
        
        %-------------Optical Flow (Lucas Kanade)-------------%
        
        %Find Corners
        ww = 40;
        w = round(ww/2);
        
        % Reduce the size of the image
        sc = 2;
        im2c = imresize(I2_edge, 1/sc);
        C1 = corner(im2c);
        C1 = C1*sc;
        
        % Discard coners near the margin of the image
        k = 1;
        for i = 1:size(C1,1)
            x_i = C1(i, 2);
            y_i = C1(i, 1);
            if x_i-w>=1 && y_i-w>=1 && x_i+w<=size(I1_edge,1)-1 && y_i+w<=size(I1_edge,2)-1
                C(k,:) = C1(i,:);
                k = k+1;
            end
        end
        
        
        % Plot corners on the image
        
        %         figure();
        %         imshow(I2_edge);
        %         hold on
        %         plot(C(:,1), C(:,2), 'r*');
        
        
        %Apply LUCAS KANADE Method
        Ix_m = conv2(I1_edge,[-1 1; -1 1], 'valid'); % partial on x
        Iy_m = conv2(I1_edge, [-1 -1; 1 1], 'valid'); % partial on y
        It_m = conv2(I1_edge, ones(2), 'valid') + conv2(I2_edge, -ones(2), 'valid'); % partial on t
        u = zeros(length(C),1);
        v = zeros(length(C),1);
        
        % within window ww * ww
        for k = 1:length(C(:,2))
            i = C(k,2);
            j = C(k,1);
            Ix = Ix_m(i-w:i+w, j-w:j+w);
            Iy = Iy_m(i-w:i+w, j-w:j+w);
            It = It_m(i-w:i+w, j-w:j+w);
            
            Ix = Ix(:);
            Iy = Iy(:);
            b = -It(:); % get b here
            
            A = [Ix Iy]; % get A here
            nu = pinv(A)*b;
            
            u(k)=nu(1);
            v(k)=nu(2);
        end;
        
        %Visualize the optical flow vectors
        figure();
        imshow(I2_edge);
        hold on;
        quiver(C(:,1), C(:,2), u,v, 1,'r')
        
        
        %-------------Find the minimum motion vector-------------%
        
        transition = [u,v];
        [row, col]=size(transition);
        
        distance_eucl=[];
        for i=1:row
            distance_eucl=[ distance_eucl ; sqrt((transition(i,1))^2+(transition(i,2))^2)];
        end
        
        [distance_min,row]=min(distance_eucl);
        [distance_max,row_max]=max(distance_eucl);
        
        Vmin=[transition(row,1) transition(row,2)];
        Vobstr=[transition(row_max,1) transition(row_max,2)];
        
        I2_translate=imtranslate(I2,-Vmin);
        I2_translate_obstr=imtranslate(I2,-Vobstr);
        figure(); imshow(I2_translate);
        
        
        % Correlation method -> TOO SLOW BUT PRECISE
        
        %         V = [u,v]
        %         [row_V,col_V]=size(V);
        %
        %         NCC_min = 1;
        %
        %         for x=1:row_V
        %                 vector_to_translate=-(V(x,:));
        %                 I2_translated=imtranslate(I2,vector_to_translate);
        %                 NCC=mean(mean(normxcorr2(I1,I2_translated),2));
        %
        %                 if NCC < NCC_min
        %                     NCC_min=NCC;
        %                     Vmin=V(x,:)
        %                 end
        %
        %         end
        %
        %         I2_translate=imtranslate(I2,-Vmin);
        %         figure(); imshow(I2_translate);
        
        
        
        %-------------Extract Background-------------%
        
        for a=1:rows
            for b=1:columns
                I_background(a,b)=min(I_background(a,b),I2_translate(a,b));
                I_obstruction(a,b)=I1(a,b)-I_background(a,b);
                
                if I_obstruction(a,b)<100 %treshold
                    I_obstruction(a,b)=0;
                    %else I_obstruction(a,b)=1;
                end
                
            end
        end
        
        figure; imshowpair(uint8(I_background),I_obstruction,'montage');
        
        
    end
    
    
end










