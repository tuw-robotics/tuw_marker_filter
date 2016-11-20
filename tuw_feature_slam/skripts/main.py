#!/usr/bin/env python

print "Hello"

from tuw.DemoNode import *

if __name__ == '__main__':
    node = DemoNode()
    node.init_node()
    node.loop()
    print "Good-by"