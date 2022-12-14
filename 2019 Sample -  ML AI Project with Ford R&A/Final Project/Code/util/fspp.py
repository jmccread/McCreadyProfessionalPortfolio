#  Created by Luis Alejandro (alejand@umich.edu)

import numpy as np
import numpy.random as rnd

def get_fspp (mdl,X):
    m,d = X.shape
    temp = np.zeros(m)
    rank = np.zeros(d)
    actual_posterior = mdl.predict_proba(X)
    for j in range(d):
        i = rnd.permutation(m)
        temp[:] = X[:,j]
        X[:,j] = X[i,j]
        affected_posterior = mdl.predict_proba(X)
        X[:,j] = temp
        rank[j] = np.abs(actual_posterior - affected_posterior).sum()/m
    return rank