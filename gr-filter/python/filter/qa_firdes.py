#!/usr/bin/env python
#
# Copyright 2012 Free Software Foundation, Inc.
#
# This file is part of GNU Radio
#
# GNU Radio is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
#
# GNU Radio is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with GNU Radio; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.
#

from gnuradio import gr, gr_unittest, filter
import sys

class test_firdes(gr_unittest.TestCase):

    def setUp(self):
	pass

    def tearDown(self):
	pass

    def test_low_pass(self):
        known_taps = (0.0024871660862118006, -4.403502608370943e-18,
                      -0.014456653036177158, 0.0543283149600029,
                      -0.116202212870121, 0.17504146695137024,
                      0.7976038455963135, 0.17504146695137024,
                      -0.116202212870121, 0.0543283149600029,
                      -0.014456653036177158, -4.403502608370943e-18,
                      0.0024871660862118006)

        new_taps = filter.firdes.low_pass(1, 1, 0.4, 0.2)
        self.assertFloatTuplesAlmostEqual(known_taps, new_taps, 5)

    def test_low_pass_2(self):
        known_taps = (0.0024871660862118006, -4.403502608370943e-18,
                      -0.014456653036177158, 0.0543283149600029,
                      -0.116202212870121, 0.17504146695137024,
                      0.7976038455963135, 0.17504146695137024,
                      -0.116202212870121, 0.0543283149600029,
                      -0.014456653036177158, -4.403502608370943e-18,
                      0.0024871660862118006)
        new_taps = filter.firdes.low_pass_2(1, 1, 0.4, 0.2, 60)
        self.assertFloatTuplesAlmostEqual(known_taps, new_taps, 5)

    def test_high_pass(self):
        known_taps = (-0.0027197482995688915, 4.815287179370254e-18,
                       0.01580853760242462, -0.05940871313214302,
                       0.1270686239004135, -0.1914101094007492,
                       0.21804752945899963, -0.1914101094007492,
                       0.1270686239004135, -0.05940871313214302,
                       0.01580853760242462, 4.815287179370254e-18,
                      -0.0027197482995688915)
        new_taps = filter.firdes.high_pass(1, 1, 0.4, 0.2)
        self.assertFloatTuplesAlmostEqual(known_taps, new_taps, 5)

    def test_high_pass_2(self):
        known_taps = (-0.0027197482995688915, 4.815287179370254e-18,
                       0.01580853760242462, -0.05940871313214302,
                       0.1270686239004135, -0.1914101094007492,
                       0.21804752945899963, -0.1914101094007492,
                       0.1270686239004135, -0.05940871313214302,
                       0.01580853760242462, 4.815287179370254e-18,
                       -0.0027197482995688915)
        new_taps = filter.firdes.high_pass_2(1, 1, 0.4, 0.2, 60)
        self.assertFloatTuplesAlmostEqual(known_taps, new_taps, 5)

    def test_band_pass(self):
        known_taps = (-0.001676854444667697, -2.4018533253972557e-18,
                       0.009746716357767582, 0.09589414298534393,
                      -0.20510689914226532, -0.11801345646381378,
                       0.4350462853908539, -0.11801345646381378,
                      -0.20510689914226532, 0.09589414298534393,
                       0.009746716357767582, -2.4018533253972557e-18,
                      -0.001676854444667697)
        new_taps = filter.firdes.band_pass(1, 1, 0.2, 0.4, 0.2)
        self.assertFloatTuplesAlmostEqual(known_taps, new_taps, 5)

    def test_band_pass_2(self):
        known_taps = (-0.001676854444667697, -2.4018533253972557e-18,
                       0.009746716357767582, 0.09589414298534393,
                       -0.20510689914226532, -0.11801345646381378,
                       0.4350462853908539, -0.11801345646381378,
                       -0.20510689914226532, 0.09589414298534393,
                       0.009746716357767582, -2.4018533253972557e-18,
                       -0.001676854444667697)
        new_taps = filter.firdes.band_pass_2(1, 1, 0.2, 0.4, 0.2, 60)
        self.assertFloatTuplesAlmostEqual(known_taps, new_taps, 5)

    def test_complex_band_pass(self):
        known_taps = ((-0.0008404505206272006-0.0025866336654871702j),
                      (-1.2038217948425635e-18+1.1767648157397848e-24j),
                      (0.0048850891180336475-0.015034818090498447j),
                      (0.048062704503536224+0.03491950035095215j),
                      (-0.10280057787895203+0.07468919456005096j),
                      (-0.05914920195937157-0.18204176425933838j),
                      (0.21804752945899963-2.5993290364567656e-07j),
                      (-0.059148769825696945+0.18204189836978912j),
                      (-0.10280075669288635-0.07468894869089127j),
                      (0.04806262254714966-0.0349196158349514j),
                      (0.004885117989033461+0.015034808777272701j),
                      (-1.2038217948425635e-18+1.1193430388030685e-24j),
                      (-0.000840445572976023+0.002586635295301676j))
        new_taps = filter.firdes.complex_band_pass(1, 1, 0.2, 0.4, 0.2)
        self.assertComplexTuplesAlmostEqual(known_taps, new_taps, 5)

    def test_complex_band_pass_2(self):
        known_taps = ((-0.0008404505206272006-0.0025866336654871702j),
                      (-1.2038217948425635e-18+1.1767648157397848e-24j),
                      (0.0048850891180336475-0.015034818090498447j),
                      (0.048062704503536224+0.03491950035095215j),
                      (-0.10280057787895203+0.07468919456005096j),
                      (-0.05914920195937157-0.18204176425933838j),
                      (0.21804752945899963-2.5993290364567656e-07j),
                      (-0.059148769825696945+0.18204189836978912j),
                      (-0.10280075669288635-0.07468894869089127j),
                      (0.04806262254714966-0.0349196158349514j),
                      (0.004885117989033461+0.015034808777272701j),
                      (-1.2038217948425635e-18+1.1193430388030685e-24j),
                      (-0.000840445572976023+0.002586635295301676j))
        new_taps = filter.firdes.complex_band_pass_2(1, 1, 0.2, 0.4, 0.2, 60)
        self.assertComplexTuplesAlmostEqual(known_taps, new_taps, 5)

    def test_band_reject(self):
        known_taps = (0.0015371545450761914, 2.201753372137003e-18,
                      -0.00893471110612154, -0.08790513873100281,
                      0.1880193054676056, 0.1081816703081131,
                      0.5982034206390381, 0.1081816703081131,
                      0.1880193054676056, -0.08790513873100281,
                      -0.00893471110612154, 2.201753372137003e-18,
                      0.0015371545450761914)
        new_taps = filter.firdes.band_reject(1, 1, 0.2, 0.4, 0.2)
        self.assertFloatTuplesAlmostEqual(known_taps, new_taps, 5)

    def test_band_reject_2(self):
        known_taps = (0.0015371545450761914, 2.201753372137003e-18,
                      -0.00893471110612154, -0.08790513873100281,
                      0.1880193054676056, 0.1081816703081131,
                      0.5982034206390381, 0.1081816703081131,
                      0.1880193054676056, -0.08790513873100281,
                      -0.00893471110612154, 2.201753372137003e-18,
                      0.0015371545450761914)
        new_taps = filter.firdes.band_reject_2(1, 1, 0.2, 0.4, 0.2, 60)
        self.assertFloatTuplesAlmostEqual(known_taps, new_taps, 5)

    def test_hilbert(self):
        known_taps = (-0.010056184604763985, 0.0,
                       -0.08335155993700027, 0.0,
                       -0.5732954144477844, 0.0,
                       0.5732954144477844, 0.0,
                       0.08335155993700027, 0.0,
                       0.010056184604763985)
        new_taps = filter.firdes.hilbert(11, filter.firdes.WIN_HAMMING)
        self.assertFloatTuplesAlmostEqual(known_taps, new_taps, 5)

    def test_root_raised_cosine(self):
        known_taps = (-0.04609205573797226, -0.02069387212395668,
                       0.050548505038022995, 0.14850808680057526,
                       0.23387153446674347, 0.2677156329154968,
                       0.23387153446674347, 0.14850808680057526,
                       0.050548505038022995, -0.02069387212395668,
                       -0.04609205573797226)
        new_taps = filter.firdes.root_raised_cosine(1, 4, 1, 0.35, 11)
        self.assertFloatTuplesAlmostEqual(known_taps, new_taps, 5)

    def test_root_raised_cosine_gain(self):
        """Confirm DC gain is as expected"""
        taps = filter.firdes.root_raised_cosine(1, 4, 1, 0.35, 11)
        self.assertAlmostEqual(sum(taps), 1.0)
        taps = filter.firdes.root_raised_cosine(1, 4, 1, 1.0, 11)
        self.assertAlmostEqual(sum(taps), 1.0)

    def test_gaussian(self):
        known_taps = (0.0003600157215259969, 0.0031858310103416443,
                      0.0182281993329525, 0.06743486225605011,
                      0.16130395233631134, 0.24947398900985718,
                      0.24947398900985718, 0.16130395233631134,
                      0.06743486225605011, 0.0182281993329525,
                      0.0031858310103416443, 0.0003600157215259969,
                      2.630509879963938e-05)
        new_taps = filter.firdes.gaussian(1, 4, 0.35, 13)
        self.assertFloatTuplesAlmostEqual(known_taps, new_taps, 5)

if __name__ == '__main__':
    gr_unittest.run(test_firdes, "test_firdes.xml")

