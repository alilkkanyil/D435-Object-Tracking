function depth_example()
    % Make Pipeline object to manage streaming
    pipe = realsense.pipeline();
    % Make Colorizer object to prettify depth output
    %colorizer = realsense.colorizer();

    % Start streaming on an arbitrary camera with default settings
    profile = pipe.start();

    % Get streaming device's name
    dev = profile.get_device();
    name = dev.get_info(realsense.camera_info.name);

    % Get frames. We discard the first couple to allow
    % the camera time to settle
    while true
        %for i = 1:5
        fs = pipe.wait_for_frames();
        %end
    
    % Stop streaming
    % pipe.stop();

    % Select depth frame
        depth = fs.get_depth_frame();
        
        color = fs.get_color_frame();
        colordata=color.get_data();
        img = permute(reshape(colordata',[3,color.get_width(),color.get_height()]),[3 2 1]);
        
    % Get actual data and convert into a format imshow can use
    % (Color data arrives as [R, G, B, R, G, B, ...] vector)
        data = depth.get_data();
        dIm = reshape(data,[640 480]);
        dIm = imrotate(dIm,270);
        I2 = flipdim(dIm ,2);
        I3 = img;

        % Define thresholds for channel 1 based on histogram settings
        channel1Min = 205.000;
        channel1Max = 255.000;

        % Define thresholds for channel 2 based on histogram settings
        channel2Min = 89.000;
        channel2Max = 255.000;

        % Define thresholds for channel 3 based on histogram settings
        channel3Min = 0.000;
        channel3Max = 255.000;

        % Create mask based on chosen histogram thresholds
        sliderBW = (I3(:,:,1) >= channel1Min ) & (I3(:,:,1) <= channel1Max) & ...
            (I3(:,:,2) >= channel2Min ) & (I3(:,:,2) <= channel2Max) & ...
            (I3(:,:,3) >= channel3Min ) & (I3(:,:,3) <= channel3Max);
        BW = sliderBW;

        % Initialize output masked image based on input image.
        maskedRGBImage = img;

        % Set background pixels where BW is false to zero.
        maskedRGBImage(repmat(~BW,[1 1 3])) = 0;
        
        
%         I3=rgb2hsv(img);
%         img2 = I3 > 0.05 .* I3 < 0.2;
        
        subplot(2,1,1);imagesc(I2);axis equal;
        title('Depth image');
        subplot(2,1,2);
        imshow(BW);axis equal;
        [centers,radii] = imfindcircles(I3,[20 60],'ObjectPolarity', 'bright', 'Sensitivity', 0.8)
        h = viscircles(centers,radii);
        title('Color image');
        drawnow;
    % Display image
%         imshow(img);
%         title(sprintf("Colorized depth frame from %s", name));
    end
    pipe.stop();
end