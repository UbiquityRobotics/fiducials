#!/bin/sh
sort /tmp/memory_allocate.log > /tmp/memory_allocate.sorted
sort /tmp/memory_check.log > /tmp/memory_check.sorted
diff /tmp/memory_allocate.sorted /tmp/memory_check.sorted | head -10

