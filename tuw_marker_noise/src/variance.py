#!/usr/bin/env python

import sys
import getopt
import math
from oct2py import octave

def angle_difference(alpha, beta):
    return math.atan2(math.sin(alpha-beta), math.cos(alpha-beta));

def main(argv):
    prog = argv[0]

    recordfile = ''
    precision = -1

    # parse input
    try:
        (opts, args) = getopt.getopt(argv[1:],"hr:p:",["recordfile=","precision="])
    except getopt.GetoptError:
        print prog + ' -r <recordfile> -p <precision>'
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print prog + ' -r <recordfile> -p <precision>'
            sys.exit()
        elif opt in ("-r", "--recordfile"):
            recordfile = arg
        elif opt in ("-p", "--precision"):
            precision = abs(float(arg))

    # verify input
    if recordfile == '' or precision == -1 or len(args) != 0:
        print prog + ' -r <recordfile> -p <precision>'
        sys.exit(2)

    pos = recordfile.rfind("/")
    variancefile = recordfile[:pos+1] + 'variance.csv'
    parameterfile = recordfile[:pos+1] + 'parameter.csv'

    # read in expectations and samples from record file and prepare variance estimation
    boxes = {}
    with open(recordfile, 'r') as f:
        for line in f:
            if line == 'exp_length;exp_angle;exp_orientation;length;angle;orientation\n':
                continue

            tmp = line.split(';')
            if len(tmp) != 6:
                raise ValueError('Not a valid line: ' + line)

            # get expected values
            el = float(tmp[0])
            ea = float(tmp[1])
            eo = float(tmp[2])

            # get observed values
            ol = float(tmp[3])
            oa = float(tmp[4])
            oo = float(tmp[5])

            # calculate local squared deviation
            dl2 = (ol - el)**2
            da2 = angle_difference(oa, ea)**2
            do2 = angle_difference(oo, eo)**2

            # build id based on precision
            id = '{}:{}:{}'.format(int(math.floor(abs(el/precision))), int(math.floor(abs(10*ea/precision))), int(math.floor(abs(10*eo/precision))))

            # update global squared deviation
            if id in boxes.keys():
                (n, sl2, sa2, so2) = boxes[id]
                boxes[id] = (n + 1, sl2 + dl2, sa2 + da2, so2 + do2)
            else:
                boxes[id] = (1, dl2, da2, do2)

    # estimate variance based on samples and their expectation
    variance = {}
    for id in boxes.keys():
        (n, sl2, sa2, so2) = boxes[id]
        variance[id] = (sl2/n, sa2/n, so2/n)

    # protocol expectations and variances
    with open(recordfile, 'r') as rf:
        with open(variancefile, 'w') as vf:
            for line in rf:
                if line == 'exp_length;exp_angle;exp_orientation;length;angle;orientation\n':
                    vf.write('exp_length;exp_angle;exp_orientation;var_length;var_angle;var_orientation\n')
                    continue

                tmp = line.split(';')
                if len(tmp) != 6:
                    raise ValueError('Not a valid line: ' + line)

                # get expected values
                el = float(tmp[0])
                ea = float(tmp[1])
                eo = float(tmp[2])

                # build id based on precision
                id = '{}:{}:{}'.format(int(math.floor(abs(el/precision))), int(math.floor(abs(10*ea/precision))), int(math.floor(abs(10*eo/precision))))

                # get estimated variance
                (vl, va, vo) = variance[id]
                assert vl > 0 and va > 0 and vo > 0

                # write out expectation and estimated variance
                vf.write('{};{};{};{};{};{}\n'.format(el, ea, eo, vl, va, vo))

    print 'Successfully created file \"' + variancefile + '\"'

    # estimate parameters
    p = octave.parameter(variancefile)
    print p

    with open(parameterfile, 'w') as f:
        f.write(';length;angle\n')
        f.write('length;{};{}\n'.format(p[0][0], p[0][1]))
        f.write('angle;{};{}\n'.format(p[1][0], p[1][1]))
        f.write('orientation;{};{}\n'.format(p[2][0], p[2][1]))

    print 'Successfully created file \"' + parameterfile + '\"'

if __name__ == '__main__':
    main(sys.argv)

