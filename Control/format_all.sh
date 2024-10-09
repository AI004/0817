#!/bin/bash
find . -iname '*.cpp' -o -iname '*.h' | xargs clang-format -i