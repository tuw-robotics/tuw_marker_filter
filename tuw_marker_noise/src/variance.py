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
    count_all = 0
    variance_global = (0, 0, 0, 0)
    with open(recordfile, 'r') as f:
        for line in f:
            if line == 'exp_length;exp_angle;exp_orientation;length;angle;orientation\n':
                continue

            tmp = line.split(';')
            if len(tmp) != 6:
                raise ValueError('Not a valid line: ' + line)
            count_all = count_all + 1

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
            variance_global = (variance_global[0] + 1, variance_global[1] + dl2, variance_global[2] + da2, variance_global[3] + do2)
            if id in boxes.keys():
                (n, sl2, sa2, so2) = boxes[id]
                boxes[id] = (n + 1, sl2 + dl2, sa2 + da2, so2 + do2)
            else:
                boxes[id] = (1, dl2, da2, do2)

    # estimate variance based on samples and their expectation
    variance_global = (variance_global[1]/variance_global[0], variance_global[2]/variance_global[0], variance_global[3]/variance_global[0])
    print variance_global
    variance = {}
    suspicious = []
    for id in boxes.keys():
        (n, sl2, sa2, so2) = boxes[id]
        variance[id] = (sl2/n, sa2/n, so2/n)

        if variance[id][0] > 2*variance_global[0] or variance[id][1] > 2*variance_global[1] or variance[id][2] > 2*variance_global[2]:
          suspicious.append(id)

    # look for supsicous data
    if len(suspicious) > 0:
      count_susp = 0
      print "Suspicious:"
    for id in suspicious:
      n = boxes[id][0]
      count_susp = count_susp + n
      print id + " (" + str(n) + "): " + str(variance[id])
    if len(suspicious) > 0:
      print "At all " + str(count_susp) + " from " + str(count_all) + " measurements are suspicious"

    # protocol expectations and variances
    with open(recordfile, 'r') as rf:
        with open(variancefile, 'w') as vf:
            for line in rf:
                if line == 'exp_length;exp_angle;exp_orientation;length;angle;orientation\n':
                    vf.write('exp_length;exp_angle;exp_orientation;var_length;var_angle;var_orientation;length;angle;orientation\n')
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
                if id not in suspicious:
                  vf.write('{};{};{};{};{};{};{};{};{}\n'.format(el, ea, eo, vl, va, vo, tmp[3], tmp[4], tmp[5]))

    print 'Successfully created file \"' + variancefile + '\"'

    # estimate parameters
    p = octave.parameter(variancefile)
    print p

    with open(parameterfile, 'w') as f:
        f.write(';par_length_x;par_length_c;par_angle_x;par_angle_c;par_orientation_x;par_orientation_c\n')
        f.write('var_length;{};{};{};{};{};{}\n'.format(p[0][0], p[0][1], p[0][2], p[0][3], p[0][4], p[0][5]))
        f.write('var_angle;{};{};{};{};{};{}\n'.format(p[1][0], p[1][1], p[1][2], p[1][3], p[1][4], p[1][5]))
        f.write('var_orientation;{};{};{};{};{};{}\n'.format(p[2][0], p[2][1], p[2][2], p[2][3], p[2][4], p[2][5]))

    print 'Successfully created file \"' + parameterfile + '\"'

if __name__ == '__main__':
    main(sys.argv)

