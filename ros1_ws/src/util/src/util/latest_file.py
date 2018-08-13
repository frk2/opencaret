#!/usr/bin/python
import sys
import os.path
import glob

def latest_file_in_glob_path(path):
    list_of_files = glob.glob(path)
    latest_file = max(list_of_files, key=os.path.getctime)
    return latest_file

def main():
    print(os.path.abspath(latest_file_in_glob_path(sys.argv[1] + "/*")))

if __name__ == '__main__':
    main()
