# Copyright 2026 Giorgio Simonini
# Licensed under the Apache License, Version 2.0.

"""
Call the generated Thunder regressors from Python, without bindings.

`thunder gen` was run with `gen_python: false`, so there is no Python module —
but the shared library exports the CasADi entry points with C linkage, and
those are perfectly callable through ctypes. That keeps the analysis a plain
Python script with no build step, and guarantees it evaluates exactly the same
code the controller ran.

The signatures are the ones in the generated header:

    franka_Yr_fun(q[7], dq[7], dqr[7], ddqr[7], KIN_base[6], KIN_EE[6]) -> [7x100]
    franka_Y_fun (q[7], dq[7], ddq[7],          KIN_base[6], KIN_EE[6]) -> [7x100]
    franka_reg_G_fun(q[7],                      KIN_base[6], KIN_EE[6]) -> [7x100]

CasADi writes dense outputs column-major, which is why the result is reshaped
as (columns, rows) and transposed.
"""

import ctypes
import os

import numpy as np
import yaml

_DOUBLE_P = ctypes.POINTER(ctypes.c_double)
_CASADI_INT = ctypes.c_longlong


class ThunderModel:
    """The regressors of the generated model, evaluated through ctypes."""

    NUM_JOINTS = 7
    NUM_PARAMETERS = 100

    def __init__(self, library, parameter_file=None):
        if not os.path.exists(library):
            raise FileNotFoundError(
                f"{library} not found. Build the workspace, source install/setup.bash, "
                f"or pass --library explicitly.")
        self.lib = ctypes.CDLL(library)

        # The kinematic frames are symbolic inputs of every regressor. They are
        # zero in the shipped parameters, but read them rather than assume.
        self.kin_base = np.zeros(6)
        self.kin_ee = np.zeros(6)
        if parameter_file and os.path.exists(parameter_file):
            with open(parameter_file) as handle:
                parameters = yaml.safe_load(handle)
            self.kin_base = np.asarray(
                parameters.get('KIN_base_xyzrpy', self.kin_base), dtype=np.float64)
            self.kin_ee = np.asarray(
                parameters.get('KIN_EE_xyzrpy', self.kin_ee), dtype=np.float64)

        self._functions = {}
        for name in ('franka_Yr_fun', 'franka_Y_fun', 'franka_reg_G_fun'):
            self._functions[name] = self._prepare(name)

    def _prepare(self, name):
        """Bind one entry point and ask it how much scratch space it needs."""
        try:
            function = getattr(self.lib, name)
        except AttributeError as error:
            raise RuntimeError(f"'{name}' is not exported by the library") from error
        function.argtypes = [
            ctypes.POINTER(_DOUBLE_P),      # arg
            ctypes.POINTER(_DOUBLE_P),      # res
            ctypes.POINTER(_CASADI_INT),    # iw
            _DOUBLE_P,                      # w
            ctypes.c_int,                   # mem
        ]
        function.restype = ctypes.c_int

        # <name>_work reports the working-set sizes; allocating from it rather
        # than guessing keeps this correct if the model is regenerated.
        size_arg = _CASADI_INT(0)
        size_res = _CASADI_INT(0)
        size_iw = _CASADI_INT(0)
        size_w = _CASADI_INT(0)
        work = getattr(self.lib, name + '_work', None)
        if work is not None:
            work.argtypes = [ctypes.POINTER(_CASADI_INT)] * 4
            work.restype = ctypes.c_int
            work(
                ctypes.byref(size_arg), ctypes.byref(size_res),
                ctypes.byref(size_iw), ctypes.byref(size_w))
        integer_work = max(int(size_iw.value), 1)
        real_work = max(int(size_w.value), 1)
        return function, (_CASADI_INT * integer_work)(), np.zeros(real_work)

    def _evaluate(self, name, inputs, rows, columns):
        function, integer_work, real_work = self._functions[name]

        buffers = [np.ascontiguousarray(value, dtype=np.float64) for value in inputs]
        argument_array = (_DOUBLE_P * len(buffers))(
            *[buffer.ctypes.data_as(_DOUBLE_P) for buffer in buffers])

        output = np.zeros(rows * columns, dtype=np.float64)
        result_array = (_DOUBLE_P * 1)(output.ctypes.data_as(_DOUBLE_P))

        status = function(
            argument_array, result_array, integer_work,
            real_work.ctypes.data_as(_DOUBLE_P), 0)
        if status != 0:
            raise RuntimeError(f"{name} returned {status}")

        # CasADi dense output is column-major.
        return output.reshape((columns, rows)).T

    def regressor_r(self, q, dq, dqr, ddqr):
        """Slotine-Li regressor Y_r(q, dq, dqr, ddqr), 7 x 100."""
        return self._evaluate(
            'franka_Yr_fun', (q, dq, dqr, ddqr, self.kin_base, self.kin_ee),
            self.NUM_JOINTS, self.NUM_PARAMETERS)

    def regressor(self, q, dq, ddq):
        """Standard regressor Y(q, dq, ddq), 7 x 100, with tau = Y * pi."""
        return self._evaluate(
            'franka_Y_fun', (q, dq, ddq, self.kin_base, self.kin_ee),
            self.NUM_JOINTS, self.NUM_PARAMETERS)

    def regressor_gravity(self, q):
        """Gravity regressor reg_G(q), 7 x 100."""
        return self._evaluate(
            'franka_reg_G_fun', (q, self.kin_base, self.kin_ee),
            self.NUM_JOINTS, self.NUM_PARAMETERS)
