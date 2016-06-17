In order to obtain the parameters for a given measurement noise model
(see "src/parameter.m") one needs first to record samples:
  roslaunch tuw_marker_noise record.launch
The record is formated written into a file "record.csv".

This file is parsed again by "src/variance.py" which then estimates the
variances for boxes of given precision parameterized by the expected
measurement length and the expected measurement angle (2D only, orientation
is skipped because lack of meaningful data):
  ./variance.py -r ../output/record.csv -p -0.2
The variances are written into a file "variance.csv" in the same directory
as "record.csv".

Finally "src/variance.py" calls the M-File "src/parameter.m" in order
to estimate the parameters. These parameters are written into the file
"parameter.csv"

Dependencies for "src/variance.py"
apt-get install liboctave-dev
pip install numpy
pip install scipy
pip install oct2py

Dependencies for "src/parameter.m"
octave> pkg install struct.tar.gz (http://octave.sourceforge.net/optim/index.html)
octave> pkg install optim.tar.gz (http://octave.sourceforge.net/struct/index.html)
