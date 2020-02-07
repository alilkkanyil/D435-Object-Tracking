function Realsense_Circle_Detection()
    % TODO: Add keyboard interrupt to break the loop and quit

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
        % Creates colorized frame
        img = permute(reshape(colordata',[3,color.get_width(),color.get_height()]),[3 2 1]);
        
        % Get actual data and convert into a format imshow can use
        % (Color data arrives as [R, G, B, R, G, B, ...] vector)
        data = depth.get_data();
        dIm = reshape(data,[1280 720]);

        % Fix orientation of the image
        % I2 is the correct orientation of video feed.
        dIm = imrotate(dIm,270);
        I2 = flipdim(dIm ,2);
        
        % Create mask to get orange color of object
        % values can be edited to create a rgb mask
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

        % Crops the depth image to match the size of color image
        I4 = imcrop(I2,[294 112 664 503]);
        
        % Display results
        
        % Display depth image
        subplot(2,1,1);imagesc(I4);axis equal;
        title('Depth image');
        
        % Display color image
        subplot(2,1,2);imagesc(img);axis equal;
        % Detect circles and display detected circles, currently finds
        % Orange circles only due to mask
        [centers,radii] = imfindcircles(I3,[20 60],'ObjectPolarity', 'bright', 'Sensitivity', 0.8);
        h = viscircles(centers,radii);
        title('Color image');
        
        % centers is a matrix of circle center coordinates.
        % matrix is n by 2, n being number of circles detected
        % ordered based on detection certainty.
        % if size(centers) is 0, no circles are found.
        check_centers = size(centers);
        
        % Checks if a circle is detected
        if check_centers(1) ~= 0
            % displays the x,y coordiantes of the highest certainty center
            disp("Center x,y coordinates:");
            disp(centers(1,[1,2]));
            
            % x coordiante of center
            xval = centers(1,1);
            % y coordinate of center
            yval = centers(1,2);
            
            % Gets the index of the pixel at (y,x) of the depth frame.
            % x and y are rounded to an integer from float.
            depth_pixel = I4(ceil(yval), ceil(xval));
            
            % Camera has a minimum and maximum range. If depth pixel is 0,
            % the object is out of range. Refer to realsense manual 
            % for range values.
            if depth_pixel == 0
                disp("Camera is too close or too far to the object. Increase distance.");
            else
                % Depth value is in milimeters.
                disp("Depth value (in milimeters):");
                disp(depth_pixel);
            end
        else
            disp("No circle detected.");
        end
        
        % Refreshes the imagescs
        drawnow;

    end
    pipe.stop();
end