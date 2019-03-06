FOR /f %%a in ('dir /b /A:-D "*.vert"') do (
	%VK_SDK_PATH%/Bin/glslc.exe -I include %%a -o ../spv/%%a.spv
)

FOR /f %%a in ('dir /b /A:-D "*.frag"') do (
	%VK_SDK_PATH%/Bin/glslc.exe -I include %%a -o ../spv/%%a.spv
)

pause