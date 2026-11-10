import os, sys, numpy as np, yaml
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'controller_tests', 'scripts'))
from plot_abstract_figure import read_run
for bag in sys.argv[1:]:
    r = read_run(bag)
    first = np.flatnonzero(r['time'] > 0)[0]
    pi0 = r['pi'][first]
    print(bag, 'pi0 norm', np.linalg.norm(pi0), 'n', len(pi0), 'time span', r['time'][first], r['time'][-1])
    np.save('/tmp/pi0.npy', pi0)
