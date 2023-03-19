#!/bin/bash
mv $1.ioc $2.ioc
rm *launch 
rm -rf .git

sed -i.bak -e "s/$1/$2/g" .mxproject
sed -i.bak -e "s/$1/$2/g" .cproject
sed -i.bak -e "s/$1/$2/g" .project
