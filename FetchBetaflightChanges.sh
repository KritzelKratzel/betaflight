#!/bin/bash

# HELP: https://github.com/betaflight/betaflight/blob/master/docs/development/Development.md
# Next line only once necessary
# git remote add upstream https://github.com/betaflight/betaflight.git


# see also:
# https://help.github.com/en/github/getting-started-with-github/fork-a-repo
# https://help.github.com/en/github/collaborating-with-issues-and-pull-requests/syncing-a-fork

git pull origin master
git fetch upstream
git checkout master
git merge upstream/master
git push origin master

