#!/bin/bash

outputdir="tmp"
dataset="$outputdir/dataset.txt"
histgram="$outputdir/hists.svm.txt"
svmmodel="$outputdir/model.libsvm"
svm=""

if [ $# -ne 1 ]; then 
    echo "ARGUMENT ERROR"
    exit 1
else
    if [ "$1" = "--clean" ]; then
        rm -rf $outputdir
        exit 0
    elif [ ! -e $1 ]; then
        echo "no such directory \"$1\""
        exit 1
    fi
fi

echo "Finding svm-train"
svm=($(find $HOME -type d -name ".*" -prune -o -type f -name "svm-train" -print))
if [ "$svm" = "" ]; then
    echo "!!!ERROR!!!    No such file svm-train: Install LibSVM"
    exit 1
fi

if [ ! -e $outputdir ]; then
    mkdir $outputdir
fi

if [ ! -e $dataset ]; then 
    touch $dataset
else
    echo -n "" > $dataset
fi

if [ -e $histgram ]; then
    rm $histgram
fi

labels=($(find $1 -maxdepth 1 -mindepth 1 -type d | sed -n "s|^$1/||p"))
for label in ${labels[@]}
do
    files=`find $1/$label \( -name *.jpg -o -name *.png \)`
    for file in ${files[@]}
    do
        echo "$label $file" >> $dataset
    done
done

sort $dataset -o $dataset

# Extract features from $dataset
python alexnet.py $dataset --extract --output_file $histgram

# Train svm
$svm -s 0 -t 0 -b 1 $histgram $svmmodel

