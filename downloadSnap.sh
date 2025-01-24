#!/bin/bash

TARGET_DIR="data/snap/"
mkdir -p "$TARGET_DIR"

SNAP_LINKS=(
  "https://snap.stanford.edu/data/wiki-Talk.txt.gz"
  "https://snap.stanford.edu/data/web-NotreDame.txt.gz"
  "https://snap.stanford.edu/data/soc-Slashdot0902.txt.gz"
  "https://snap.stanford.edu/data/soc-Epinions1.txt.gz"
  "https://snap.stanford.edu/data/as-skitter.txt.gz"
  "https://snap.stanford.edu/data/web-BerkStan.txt.gz"
  "https://snap.stanford.edu/data/soc-LiveJournal1.txt.gz"
  "https://snap.stanford.edu/data/bigdata/communities/com-youtube.ungraph.txt.gz"
)

for URL in "${SNAP_LINKS[@]}"; do
  FILE_NAME=$(basename "$URL")
  FILE_PATH="$TARGET_DIR$FILE_NAME"

  curl -o "$FILE_PATH" "$URL"

  if [[ -f "$FILE_PATH" ]]; then
    gunzip -d "$FILE_PATH"
  else
    echo "Error: Failed to download $URL."
  fi
done

echo "All files downloaded and decompressed in $TARGET_DIR."
