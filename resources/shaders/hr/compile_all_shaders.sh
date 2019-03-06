#!/bin/bash

for file in *.vert
do
   $VK_SDK_PATH/Bin/glslc -I include $file -o ../spv/"$(basename "$file")".spv
done

for file in *.frag
do
   $VK_SDK_PATH/Bin/glslc -I include $file -o ../spv/"$(basename "$file")".spv
done
