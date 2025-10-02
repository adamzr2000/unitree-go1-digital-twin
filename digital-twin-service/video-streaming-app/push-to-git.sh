#!/bin/bash

# Check if commit message is provided
if [ -z "$1" ]; then
  echo "Usage: $0 <commit_message>"
  exit 1
fi

# Commit message
COMMIT_MESSAGE="$1"

# Add all changes
git add --all

# Commit with the provided message
git commit -m "$COMMIT_MESSAGE"

# Set the branch to main
git branch -M main

# Push to the origin
git push -u origin main

echo "Changes have been pushed to the repository."

