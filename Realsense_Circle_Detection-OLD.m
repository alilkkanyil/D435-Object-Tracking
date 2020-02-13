% OUTDATED, GO TO CAMERA-CODE FOLDER
function Realsense_Circle_Detection()
    % Make Pipeline object to manage streaming
    pipe = realsense.pipeline();

    % Start streaming on an arbitrary camera with default settings
    profile = pipe.start();

    % Get streaming device's name
    dev = profile.get_device();
    name = dev.get_info(realsense.camera_info.name);

    % Get frames.
    while true
        fs = pipe.wait_for_frames();

        % Select depth frame
        depth = fs.get_depth_frame();
        % Select color frame
        color = fs.get_color_frame();
        colordata=color.get_data();
        img = permute(reshape(colordata',[3,color.get_width(),color.get_height()]),[3 2 1]);
        
        % Get actual data and convert into a format imshow can use
        % (Color data arrives as [R, G, B, R, G, B, ...] vector)
        data = depth.get_data();
        dIm = reshape(data,[640 480]);

        % Fix orientation of the image
        dIm = imrotate(dIm,270);
        I2 = flipdim(dIm ,2);
        
        % Create mask to get a specific color of object
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
        
        
        % Display results
        
        % Display depth image
        subplot(2,1,1);imagesc(I2);axis equal;
        title('Depth image');
        % Display color image
        subplot(2,1,2);imagesc(img);axis equal;
        % Detect circles and display detected circles, currently finds
        % Orange circles only due to mask
        [centers,radii] = imfindcircles(I3,[20 60],'ObjectPolarity', 'bright', 'Sensitivity', 0.8)
        h = viscircles(centers,radii);
        title('Color image');
        % Refreshes the imagescs
        drawnow;

    end
    pipe.stop();
end
