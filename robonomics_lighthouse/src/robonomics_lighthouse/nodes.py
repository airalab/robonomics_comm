# -*- coding: utf-8 -*-
#
# Robonomics market related nodes.
#

from . import infochan
from . import signer
from . import matcher
from . import reporter
from . import generator

def infochan_node():
    infochan.InfoChan().spin()

def signer_node():
    signer.Signer().spin()

def generator_node():
    generator.Generator().spin()

def matcher_node():
    matcher.Matcher().spin()

def reporter_node():
    reporter.Reporter().spin()
