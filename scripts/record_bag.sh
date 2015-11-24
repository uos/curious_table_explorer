#!/bin/sh

rosbag record "$@" /nav_map /tf /tf_static /table_view /finalize_table
