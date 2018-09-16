##############################################################################
# 
# Copyright (c) 2005 Greg Green <ggreen@bit-builder.com>
# 
# File: 
'''

'''


mask_plus_filter = [
    0x0fc1,
    0x0080,
    0x0ff9,
    0x0080,
    0x0000,
    0x00c0,
    ]

def check(cid):
    return (cid & m1) == f1

def pass_through(cid):
    i = 0
    j = 0
    while i < len(mask_plus_filter):
        if (cid == mask_plus_filter[i+1]) | ~(mask_plus_filter[i]) :
            return j
        i += 2
        j += 1
    return -1

for i in range(2**11):
    pt = pass_through(i)
    if pt >= 0:
        print "%03x:%d" % (i, pt)
