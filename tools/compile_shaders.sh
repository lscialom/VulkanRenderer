#!/bin/bash

for file in ../resources/shaders/hr/*
do
   glslangValidator -t -V $file -o ../resources/shaders/spv/"$(basename "$file")".spv
done