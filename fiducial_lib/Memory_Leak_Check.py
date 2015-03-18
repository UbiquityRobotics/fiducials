#!/usr/bin/env python

def main():
    # Read in the allocation records:
    allocate_file = open("/tmp/memory_allocate.log", "r")
    allocate_lines = allocate_file.readlines()
    allocate_file.close()

    # Read in the free records:
    free_file = open("/tmp/memory_free.log", "r")
    free_lines = free_file.readlines()
    free_file.close()

    free_table = {}
    for free_line in free_lines:
	free_fields = free_line.split()
	free_address = free_fields[0]
	#print("free_fields={0}".format(free_fields))
	free_table[free_address] = None

    leaks = []
    for allocate_line in allocate_lines:
	allocate_fields = allocate_line.split()
	allocate_address = allocate_fields[0]
	allocate_from = allocate_fields[1]
	leak = (allocate_address, allocate_from)
	if not allocate_address in free_table:
	    leaks.append(leak)

    leaks.sort()
    for leak in leaks:
	print("{0}: {1}".format(leak[0], leak[1]))

main()
