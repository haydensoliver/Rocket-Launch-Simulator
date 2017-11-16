import pytest
import unittest

def test_Vacuum_dV():
    """Tests the function to find vacuum dV"""

    from sounding_rocket_launch import Vacuum_dV

    assert 0 == Vacuum_dV(300,100,100)
    print("pass 1")
    assert .1 <= abs(Vacuum_dV(300,100,50) - 1885.33)
    print("pass2")
    assert 0 < Vacuum_dV(300,100,150)
    print("pass3")

    #assert float('Inf') == Vacuum_dV(300,0,1)
    #assert float('NaN') == Vacuum_dV(300,1,0)
test_Vacuum_dV()
