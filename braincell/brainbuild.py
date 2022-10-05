#!/usr/bin/env python3
# Brainbuild is the build script tool for building and flashing Braincell targets.

# Usage:
# ./brainbuild.py [flash] [OPTIONS]
#
# OPTIONS:
#   -h, --help
#      Outputs the help menu
#   (no option)
#      Builds the braincell app
#   -e, --example [example]
#      Builds the specified example

# From: Steven James Walker <swalker1@emerald.tufts.edu>

#        _---~~(~~-_.
#      _{        )   )
#    ,   ) -~~- ( ,-' )_
#   (  `-,_..`., )-- '_,)
#  ( ` _)  (  -~( -_ `,  }
#  (_-  _  ~_-~~~~`,  ,' )
#    `~ -^(    __;-,((()))
#          ~~~~ {_ -_(())
#                 `\  }
#                   { }

# ------------------------------------------------
# Thank you for visiting https://asciiart.website/
# This ASCII pic can be found at
# https://asciiart.website/index.php?art=people/body%20parts/brains

import os, sys, getopt, subprocess

usage = '''Usage:
./brainbuild.py [OPTIONS] [flash]

OPTIONS:
   -h, --help
      Outputs the help menu
   (no option)
      Builds the braincell app
   -e, --example [example]
      Builds the specified example
'''

DEBUG_DIR = os.path.join('target', 'thumbv7em-none-eabihf', 'debug')
EXAMPLES_DIR = os.path.join(DEBUG_DIR, 'examples')


def run_shell_cmd(cmd):
    process = subprocess.Popen(cmd.split(), stdout=subprocess.PIPE, shell=False)
    while True:
        output = process.stdout.readline()
        if process.poll() is not None:
            break
        if output:
            print(output.strip())
    rc = process.poll()
    return rc


def main(argv):
    build_target = 'braincell'

    try:
        opts, args = getopt.getopt(argv,"he:", ['help', 'example='])
    except getopt.GetoptError:
        print(usage)
        sys.exit(2)

    flash = 'flash' in args

    for opt, arg in opts:
        if opt in ('-h', '--help'):
            print(usage)
            sys.exit(0)
        elif opt in ('-e', '--example'):
            build_target = arg

    # build app
    if build_target == 'braincell':
        run_shell_cmd('cargo build')
        os.chdir(DEBUG_DIR)

    # build example
    else:
        run_shell_cmd('cargo build --example ' + build_target)
        os.chdir(EXAMPLES_DIR)

    # objcopy
    run_shell_cmd('arm-none-eabi-objcopy -O binary {} {}.bin'.format(build_target, build_target))

    # flash board
    if flash:
        print('flashing')
        run_shell_cmd('st-flash --reset write {}.bin 0x8000000'.format(build_target))


if __name__ == '__main__':
    main(sys.argv[1:])
