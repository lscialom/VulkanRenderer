set filepath=%1

%VK_SDK_PATH%/Bin/glslc.exe -I include %filepath% -o ..\spv\%filepath%.spv
