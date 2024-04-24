% Prepare the new file.
vidObj = VideoWriter('C:\Users\yancy_000\Desktop\BraidTrack_M80N5_Zoomed.avi');
vidObj.FrameRate = 100;
vidObj.Quality = 100;
open(vidObj);
% Create an animation.
for k = 2:1:NumFrames
    % Write each frame to the file.
    writeVideo(vidObj,Frames(k));
end
% Close the file.
close(vidObj);