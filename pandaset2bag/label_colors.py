# MIT License
#
# Copyright (c) 2023 b-plus technologies GmbH
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

# numpydoc ignore=GL08
from __future__ import annotations

LABEL_COLORMAP: dict[str, tuple[float, float, float]] = {
    'Smoke': (0.8187488187488188, 1.0, 0.0),
    'Exhaust': (0.7492634551458082, 1.0, 0.0),
    'Spray or rain': (0.6797780915427977, 1.0, 0.0),
    'Reflection': (0.6102927279397868, 1.0, 0.0),
    'Vegetation': (0.5408073643367761, 1.0, 0.0),
    'Ground': (0.4713220007337658, 1.0, 0.0),
    'Road': (0.4018366371307548, 1.0, 0.0),
    'Lane Line Marking': (0.309189485660074, 1.0, 0.0),
    'Stop Line Marking': (0.23970412205706326, 1.0, 0.0),
    'Other Road Marking': (0.17021875845405265, 1.0, 0.0),
    'Sidewalk': (0.10073339485104227, 1.0, 0.0),
    'Driveway': (0.03124934374934376, 1.0, 1.3125013124790507e-06),
    'Car': (0.008087555881673585, 1.0, 0.04632488823665283),
    'Pickup Truck': (0.0, 1.0, 0.10772241105651532),
    'Medium-sized Truck': (0.0, 1.0, 0.1772073369040554),
    'Semi-truck': (0.0, 1.0, 0.2698539047007752),
    'Towed Object': (0.0, 1.0, 0.33933883054831565),
    'Motorcycle': (0.0, 1.0, 0.4088237563958557),
    'Other Vehicle - Construction Vehicle': (0.0, 1.0, 0.47830868224339584),
    'Other Vehicle - Uncommon': (0.0, 1.0, 0.547793608090936),
    'Other Vehicle - Pedicab': (0.0, 1.0, 0.617278533938476),
    'Emergency Vehicle': (0.0, 1.0, 0.6867634597860162),
    'Bus': (0.0, 1.0, 0.7562483856335561),
    'Personal Mobility Device': (0.0, 1.0, 0.8488949534302763),
    'Motorized Scooter': (0.0, 1.0, 0.9183798792778164),
    'Bicycle': (0.0, 1.0, 0.9878648051253566),
    'Train': (0.0, 0.9426499077234377, 1.0),
    'Trolley': (0.0, 0.8731645441204264, 1.0),
    'Tram / Subway': (0.0, 0.8036791805174157, 1.0),
    'Pedestrian': (0.0, 0.7341938169144051, 1.0),
    'Pedestrian with Object': (0.0, 0.6647084533113945, 1.0),
    'Animals - Bird': (0.0, 0.5952230897083839, 1.0),
    'Animals - Other': (0.0, 0.5025759382377031, 1.0),
    'Pylons': (0.0, 0.43309057463469247, 1.0),
    'Road Barriers': (0.0, 0.36360521103168175, 1.0),
    'Signs': (0.0, 0.29411984742867114, 1.0),
    'Cones': (0.0, 0.22463448382566054, 1.0),
    'Construction Signs': (0.0, 0.15514912022264993, 1.0),
    'Temporary Construction Barriers': (0.0, 0.08566375661963932, 1.0),
    'Rolling Containers': (0.015440535661123769, 0.03161892867775246, 1.0),
    'Building': (0.07646875845405214, 0.0, 1.0),
    'Other Static Object': (0.14595412205706287, 0.0, 1.0),
}
