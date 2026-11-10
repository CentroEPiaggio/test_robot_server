import os, sys, numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'controller_tests', 'scripts'))
from plot_abstract_figure import read_run, default_library, default_parameter_file, default_gains_file, load_gains
from thunder_ctypes import ThunderModel
print(default_library(), default_parameter_file(), default_gains_file(), load_gains(default_gains_file()))
m = ThunderModel(default_library(), default_parameter_file())
r = read_run(sys.argv[1])
idx = np.flatnonzero((r['time'] > 6) & (r['time'] < 34))[::5]
res = np.array([r['tau_model'][i] - m.regressor_r(r['q'][i], r['dq'][i], r['dq_d'][i], r['ddq_d'][i]) @ r['pi'][i] for i in idx])
for j in range(7):
    A = np.column_stack([r['e'][idx, j], r['de'][idx, j], np.ones(len(idx))])
    c, *_ = np.linalg.lstsq(A, res[:, j], rcond=None)
    print(j, 'kp %.3f kv %.3f off %.3f  resid rms %.4f' % (c[0], c[1], c[2], np.sqrt(np.mean((A @ c - res[:, j])**2))))
