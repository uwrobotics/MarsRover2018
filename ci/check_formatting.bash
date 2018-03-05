#!/bin/bash
find $CI_SOURCE_PATH/Workspace/src -name '*.cpp' -name '*.h' -name '*.hpp' | xargs -n1 clang-format -style=file -output-replacements-xml | grep "<replacement " > /dev/null
if [ $? -ne 1 ]
then
    echo "Incorrect formatting in some C++ files, run clang-format to fix"
    exit 1;
fi

sudo pip install -r requirements.txt
find $CI_SOURCE_PATH/Workspace/src -name '*.py' -exec pycodestyle {} \; | grep ".*" > /dev/null
if [ $? -ne 1 ]
then
    echo "Incorrect formatting in some C++ files, run clang-format to fix"
    exit 1;
fi
