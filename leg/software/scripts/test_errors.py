#!/usr/bin/env python3
"""Test catching errors."""

import odrive
from odrive.utils import dump_errors

def flag_error(status_string):
    """raise runtime error flag errors instead of just printing them."""
    if "Error" in status_string:
        raise RuntimeError(string)

skip_print = lambda string : pass

odrv0 = odrv0.find_any()

print(f"found odrive: {odrv0}")
dump_errors(odrv0, flag_error) # only catches the first error if many are present.

