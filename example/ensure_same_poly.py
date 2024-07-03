import os
import sys
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("file1", help="The name of the file with polygons")
parser.add_argument("file2", help="The name of the file with polygons")
args = parser.parse_args()

poly1 = set()
poly2 = set()

with open(args.file1) as m1:
    for line in m1:
        poly1.add(tuple((int(c.split(',')[0][1:]), int(c.split(',')[1][:-1])) for c in line.split(' ')[1:-1]))

with open(args.file2) as m2:
    for line in m2:
        poly2.add(tuple((int(c.split(',')[0][1:]), int(c.split(',')[1][:-1])) for c in line.split(' ')[1:-1]))

if poly2 == poly1:
    print("yay")
else:
    print("1-2")
    for t in poly1.difference(poly2): print(t)
    # print(poly1.difference(poly2))
    print("2-1")
    for t in poly2.difference(poly1): print(t)
    # print(poly2.difference(poly1))