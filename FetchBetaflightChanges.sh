#!/bin/bash

# HELP: https://github.com/betaflight/betaflight/blob/master/docs/development/Development.md
# Next line only once necessary
# git remote add betaflight https://github.com/betaflight/betaflight.git

git checkout master
git fetch betaflight
git merge betaflight/master
git push origin master

