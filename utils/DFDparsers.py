# -*- coding: utf-8 -*-
"""
Created on Wed Jan 24 18:56:38 2024

@author: arnau
"""
from widgets.DFDGUIobjects import ProcessItem
from utils.globals import ACCEPTED_DFD_ITEM_TYPES
from utils.generic import findFirstNumber

#%%
class WrongDFDitemType(Exception):
    """Handles a custom exception.
    
    Inherits from Exception.
    This is supposed to be raised when the DFD item type is wrong.
    """
    pass

class DFDitem:
    """Handles a signle DFD item"""
    def __init__(self, itemType: str, string: str, strId: str):
        """DFDitem constructor
        
        Sets the item string, type and id.

        Parameters
        ----------
        itemType : str
            Type of DFD item. See globals.ACCEPTED_DFD_ITEM_TYPES.
        string : str
            String of the item (e.g. 'Proccess 1').
        strId : str
            The string ID (e.g. 'P1'). The id will be the number in the string.

        Returns
        -------
        None.

        """
        self.string = string
        self.strId = strId
        self.id = findFirstNumber(strId)
        
        self.itemType = itemType
    
    @property
    def itemType(self):
        """
        Getter method for property: itemType.

        Returns
        -------
        str
            The type of DFD item. See globals.ACCEPTED_DFD_ITEM_TYPES.

        """
        return self._itemType
    
    @itemType.setter
    def itemType(self, newType):
        """
        Setter method for property: itemType.

        Parameters
        ----------
        newType : str
            The new type of DFD item. See globals.ACCEPTED_DFD_ITEM_TYPES.

        Returns
        -------
        None.

        """
        if newType in ACCEPTED_DFD_ITEM_TYPES:
            self._itemType = newType
        else:
            raise(WrongDFDitemType(f"DFD item '{newType}' type not implemented."))

class DFD:
    """Handles a DFD.
    
    Has a list of items. Further functionality could be added.
    """
    def __init__(self, itemList: list[DFDitem]):
        """DFD constructor
        
        Sets the list of items.
        

        Parameters
        ----------
        itemList : list
            List of DFD items.

        Returns
        -------
        None.

        """
        self.itemList = itemList


def parseSingleLine(line: str) -> DFDitem:
    """
    Parses one line or instruction to a DFDitem instance.
    
    For example:
        The line 'process P1 Phase 1' will be parsed to a DFDitem with:
            string = 'Phase 1'
            itemType = 'process'
            strId = 'P1'
            id = 1

    Parameters
    ----------
    line : str
        Instruction to be parsed.

    Returns
    -------
    DFDitem
        The DFDitem object.

    """
    lineList = line.split(maxsplit=2)  
    return DFDitem(lineList[0], lineList[2].rstrip(), lineList[1])

def parseMultipleLines(data: list[str]) -> DFD:
    """
    Parses multiple lines, each one to a DFDitem. The whole list is returned as
    a DFD object.

    Parameters
    ----------
    data : list[str]
        List of lines to be parsed.

    Returns
    -------
    DFD
        DFD object.

    """
    DFDitemList = []
    for line in data:
        DFDitemList.append(parseSingleLine(line))
    return DFD(DFDitemList)



if __name__=='__main__':
    x = parseMultipleLines(['process	P1	Fase 1', 'process	P2	Fase 2'])
    print(x.itemList[0].itemType)
    print(x.itemList[0].string)
    print(x.itemList[1].itemType)
    print(x.itemList[1].string)