#!/bin/bash

set -e

#Find URLs in code:
urls=$(grep -oP "(http|ftp|https):\/\/([a-zA-Z0-9_-]+(?:(?:\.[a-zA-Z0-9_-]+)+))([a-zA-Z0-9_.,@?^=%&:\/~+#-]*[a-zA-Z0-9_@?^=%&\/~+#-])?" "$@")

fail_counter=0

FAILED_LINKS=()
for item in $urls; do
#     echo $item
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
