""" Main entry of the program."""

from __future__ import division

from legoassembler.main import run
from legoassembler.utils import load_config


if __name__ == '__main__':

    cfg = load_config()
    run(cfg)


