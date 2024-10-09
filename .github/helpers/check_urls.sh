#!/bin/bash

set -e

IGNORE_FILES=""
IGNORE_PATTERNS=""

while getopts ":f:d:p:" opt; do
    case "${opt}" in
        f)
            IGNORE_FILES="${OPTARG}";;
        d)
            IGNORE_DIRS="${OPTARG}";;
        p)
            IGNORE_PATTERNS="${OPTARG}";;
        \?)
            echo "Invalid option -$OPTARG"
            exit;;
    esac
done

read -r -a ignore_files <<<"$IGNORE_FILES"
read -r -a ignore_dirs <<<"$IGNORE_DIRS"
read -r -a ignore_patterns <<<"$IGNORE_PATTERNS"

IGNORE_FILES_ARG=""
for item in "${ignore_files[@]}"; do
  IGNORE_FILES_ARG="$IGNORE_FILES_ARG --exclude=$item"
done
IGNORE_DIRS_ARG=""
for item in "${ignore_dirs[@]}"; do
  IGNORE_DIRS_ARG="$IGNORE_DIRS_ARG --exclude-dir=$item"
done

#Find URLs in code:
urls=$(grep -oP '(http|ftp|https):\/\/([a-zA-Z0-9_-]+(?:(?:\.[a-zA-Z0-9_-]+)+))([a-zA-Z0-9_.,@?^=%&:\/~+#-]*[a-zA-Z0-9_@?^=%&\/~+#-])?' -rI $IGNORE_FILES_ARG $IGNORE_DIRS_ARG)

fail_counter=0

FAILED_LINKS=()
for item in $urls; do
#     echo $item
    skip=0
    for pattern in "${ignore_patterns[@]}"; do
      [[ "$item" =~ $pattern ]] && skip=1
    done

    if [[ $skip == 1 ]]; then
      echo "SKIPPING $item"
      continue
    fi

    filename=$(echo "$item" | cut -d':' -f1)
    url=$(echo "$item" | cut -d':' -f2-)
    echo -n "Checking $url from file $filename"
    if ! curl --head --silent --fail "$url" 2>&1 > /dev/null; then
        echo -e " \033[0;31mNOT FOUND\033[32m\n"
        FAILED_LINKS+=("$url from file $filename")
        ((fail_counter=fail_counter+1))
    else
        printf " \033[32mok\033[0m\n"
    fi
done

echo "Failed files:"
printf '%s\n' "${FAILED_LINKS[@]}"
exit $fail_counter
