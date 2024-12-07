#!/bin/sh
gdb -iex "set pagination off" -q -ex cont -p $1