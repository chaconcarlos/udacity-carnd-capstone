#!/usr/bin/env bash

if [ -z "$1" ]
then
    echo "file argument required"
    exit 1
fi

python -m tf.label_image --graph=../src/tl_detector/tf_files/retrained_graph.pb --labels=../src/tl_detector/tf_files/retrained_labels.txt --image="$1" --reader="$2" --input_layer="Mul" --input_width=299 --input_height=299

