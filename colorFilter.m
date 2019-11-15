clear;clc;

cam = webcam(1);

while(1)
    
    rgbImage = snapshot(cam);
    subplot(1,3,1);imagesc(rgbImage);
    
    hsv = rgb2hsv(rgbImage);
    h = hsv(:,:,1);
    subplot(1,3,2);imagesc(h); % Display the hue channel 
    
    orangePixels = h < 0.2;
    subplot(1,3,3);imagesc(orangePixels);

end