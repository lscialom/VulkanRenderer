FOR /f %%a in ('dir /b "..\resources\shaders\hr\*"') do (
	glslangValidator.exe -t -V ../resources/shaders/hr/%%a -o ../resources/shaders/spv/%%a.spv
)

pause