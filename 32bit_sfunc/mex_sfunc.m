% mex -I.\3rdparty\ffmpeg\include...
%     -I.\3rdparty\kijineko\include...
%     -L.\3rdparty\ffmpeg\lib...
%     -lavcodec -lavdevice -lavfilter -lavformat -lavutil -lswresample -lswscale...
%     -I.\ardrone\ardrone.h...
%      sfun_ardrone_video.cpp ...
%     .\ardrone\udp.cpp 
%     

%making change to master branch, how does it propagate?
%github test, 2nd branch?

mex -I.\3rdparty\ffmpeg\include...
    -I.\3rdparty\kijineko\include...
    -L.\3rdparty\ffmpeg\lib...
    -lavcodec -lavdevice -lavfilter -lavformat -lavutil -lswresample -lswscale...
    -I.\ardrone\ardrone.h...
     multi_inputs.cpp ...
    .\ardrone\udp.cpp 
    