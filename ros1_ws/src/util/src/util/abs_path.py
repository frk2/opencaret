#!/usr/bin/python
import sys
import os.path
import glob

def main():
    print(os.path.abspath(sys.argv[1]))

if __name__ == '__main__':
    main()
