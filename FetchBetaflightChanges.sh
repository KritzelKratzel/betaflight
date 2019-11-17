#!/bin/bash

# HELP: https://github.com/betaflight/betaflight/blob/master/docs/development/Development.md
# Next line only once necessary
# git remote add upstream https://github.com/betaflight/betaflight.git


# see also:
# https://help.github.com/en/github/getting-started-with-github/fork-a-repo
# https://help.github.com/en/github/collaborating-with-issues-and-pull-requests/syncing-a-fork

git fetch upstream
git checkout 4.0.x-maintenance
git merge upstream/4.0.x-maintenance
git push origin 4.0.x-maintenance

