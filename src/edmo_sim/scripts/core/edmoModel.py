import json
from json import JSONEncoder, JSONDecoder

import numpy as np


class Structure:

    def __init__(self, n_modules=1, name=''):
        self.name = name
        self.frequency = 0
        self.weight = 0.025
        self.phase_bias_matrix = np.zeros([n_modules, n_modules])

        self.modules = []
        for i in range(n_modules):
            self.modules.append(Module())

    def toJSON(self):
        return json.dumps(dict(
            name = self.name,
            frequency = self.frequency,
            weight = self.weight,
            phase_bias_matrix = self.phase_bias_matrix.tolist(),
            modules = self.modules
        ), cls=ModuleEncoder)


    @staticmethod
    def fromJSON(json_str):
        json_obj = json.loads(json_str, cls=ModuleDecoder)
        s = Structure()
        s.name = json_obj["name"]
        s.frequency = json_obj["frequency"]
        s.weight = json_obj["weight"]
        s.phase_bias_matrix = np.array(json_obj["phase_bias_matrix"])
        s.modules = json_obj["modules"]
        return s


class Module:

    def __init__(self, parameters=np.array([0, 0])):
        self.amplitude = parameters[0]
        self.offset = parameters[1]

    def set_amplitude(self, amplitude):
        self.amplitude = amplitude

    def set_offset(self, offset):
        self.offset = offset


class ModuleEncoder(JSONEncoder):
    def default(self, module: Module):
        return dict(
            amplitude=int(module.amplitude),
            offset=int(module.offset)
        )

class ModuleDecoder(JSONDecoder):
    def default(self, json_str: str) -> Module:
        json_obj = json.loads(json_str)
        m = Module()
        m.amplitude = json_obj["amplitude"]
        m.offset = json_obj["offset"]
        return m