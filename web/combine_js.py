#!/usr/bin/env python3
"""
Combine JavaScript parts into a single file.
"""

import os

# Define the parts to combine
parts = [
    'web/static/js/app.js',
    'web/static/js/app_part2.js',
    'web/static/js/app_part3.js',
    'web/static/js/app_part4.js'
]

# Output file
output_file = 'web/static/js/app_combined.js'

# Combine the files
with open(os.path.normpath(output_file), 'w') as outfile:
    for part in parts:
        with open(os.path.normpath(part), 'r') as infile:
            outfile.write(infile.read())
            outfile.write('\n')

print(f"Combined JavaScript files into {output_file}")
