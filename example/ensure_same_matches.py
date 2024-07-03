import os
import sys
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("file1", help="The name of the file with point matches")
parser.add_argument("file2", help="The name of the file with point matches")
args = parser.parse_args()

matches1 = set()
matches2 = set()

with open(args.file1) as m1:
    for line in m1:
        matches1.add(tuple(tuple(map(float, c.split(','))) for c in line.split('|')))

with open(args.file2) as m2:
    for line in m2:
        matches2.add(tuple(tuple(map(float, c.split(','))) for c in line.split('|')))

if matches2 == matches1:
    print("yay")
else:
    print("1-2")
    print(matches1.difference(matches2))
    print("2-1")
    print(matches2.difference(matches1))