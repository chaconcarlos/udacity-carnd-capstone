#!/usr/bin/env bash


training_steps=4000
ARCHITECTURE=${ARCHITECTURE:-"mobilenet_0.50_224"}

export IMAGE_SIZE=224

python -m tf.retrain \
--bottleneck_dir=tf_files/bottlenecks \
--how_many_training_steps="$training_steps" \
--model_dir=tf_files/models/ \
--summaries_dir=tf_files/training_summaries/"${ARCHITECTURE}" \
--output_graph=../src/tl_detector/tf_files/retrained_graph.pb \
--output_labels=../src/tl_detector/tf_files/retrained_labels.txt \
--image_dir=../src/tl_detector/data \
--architecture="${ARCHITECTURE}"
