FILE=$1

$VK_SDK_PATH/Bin/glslc -I include $FILE -o ../spv/"$(basename "$FILE")".spv
