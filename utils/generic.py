# -*- coding: utf-8 -*-
"""
Created on Thu Jan 25 09:43:05 2024

@author: arnau
"""
import re


#%%
def findFirstNumber(s: str):
    """
    Finds the first number (with any number of digits) in the string.
    Returns the number as a int or None if no number is found.

    Parameters
    ----------
    s : str
        Input string.

    Returns
    -------
    int or None
        The number found.

    """
    match = re.search(r'\d+', s)
    if match:
        return int(match.group())
    else:
        return None