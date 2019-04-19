# -*- coding: utf-8 -*-

class StaticObjective:
    _topics = list()

    def __init__(self):
        return NotImplemented
    
    def listen(self, names: list):
        """Listen to topics with given names, lock until all names will have a value
           Returns a dict with names as a key and values from topics messages
        """
        return self.get_all()

    def put(name, val, val_type):
        """Puts given type value to objective topic with given name
        """
        return NotImplemented

    def get(name):
        """Takes
        """
        return NotImplemented

    def get_all(self):
        """Returns a dict with names as a key and values from topics messages
        """
        return NotImplemented

    def compose(path):
        """Writes an objective file with static data
        """
        return NotImplemented
