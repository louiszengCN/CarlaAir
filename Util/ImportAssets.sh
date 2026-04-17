#!/usr/bin/env bash

# ==============================================================================
# -- Parse arguments -----------------------------------------------------------
# ==============================================================================

DOC_STRING="Unpack and copy over CarlaUE4's Exported Assets"

USAGE_STRING="Usage: $0 [-h|--help] [-d|--dir] <outdir>"

OUTPUT_DIRECTORY=""

OPTS=$(getopt -o h,d:: --long help,dir:: -n 'parse-options' -- "$@")

if [ $? != 0 ] ; then echo "$USAGE_STRING" ; exit 2; fi  # SC2181: getopt exit code check is intentional here

eval set -- "$OPTS"

while true; do
  case "$1" in
    --dir )
      OUTPUT_DIRECTORY="$2"
      shift ;;
    -h | --help )
      echo "$DOC_STRING"
      echo "$USAGE_STRING"
      exit 1
      ;;
    * )
      break ;;
  esac
done

#Tar.gz the stuff
while IFS= read -r filepath; do
  tar --keep-newer-files -xvf "${filepath}"
done < <(find Import/ -type f -name "*.tar.gz")

