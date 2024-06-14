# ransac_prosac.py
import random
from abc import ABC, abstractmethod
import numpy as np
from scipy import stats
from scipy.stats.distributions import chi2

# Abstract Model class
class Model(ABC):
    @abstractmethod
    def fit(self, pts):
        pass

    @abstractmethod
    def error(self, data):
        pass

    @abstractmethod
    def predict(self, data):
        pass

    @staticmethod
    @abstractmethod
    def get_complexity():
        pass

# RANSAC implementation
def ransac(data, model_type, tolerance, prob_inlier, p=0.99):
    m = model_type.get_complexity()
    best_num_inliers = 0
    n = data.shape[0]
    max_times = int(np.ceil(np.log(1 - p) / np.log(1 - prob_inlier ** m)))
    satisfactory_inlier_ratio = prob_inlier * n

    inliers = []
    for _ in range(max_times):
        pts = data[random.sample(range(n), m)]
        model = model_type()
        model.fit(pts)
        error = model.error(data)
        num_inliers = (error < tolerance).sum()
        if num_inliers / n > satisfactory_inlier_ratio:
            inliers = data[error < tolerance]
            break
        if num_inliers > best_num_inliers:
            best_num_inliers = num_inliers
            inliers = data[error < tolerance]

    model = model_type()
    model.fit(inliers)
    return model

# PROSAC implementation
def prosac(data, quality, model_type, tolerance, beta, eta0, psi,
           max_outlier_proportion, p_good_sample, max_number_of_draws,
           enable_n_star_optimization=True):
    indexes = np.argsort(quality)
    data = data[indexes[::-1]]

    num_points = data.shape[0]
    num_points_to_sample = model_type.get_complexity()
    chi2_value = chi2.isf(2 * psi, 1)

    def niter_ransac(p, epsilon, s, n_max):
        if n_max == -1:
            n_max = np.iinfo(np.int32).max
        if not (n_max >= 1):
            raise ValueError('n_max must be positive')
        if epsilon <= 0:
            return 1
        logarg = - np.exp(s * np.log(1 - epsilon))
        logval = np.log(1 + logarg)
        n = np.log(1 - p) / logval
        if logval < 0 and n < n_max:
            return np.ceil(n)
        return n_max

    def i_min(m, n, beta):
        mu = n * beta
        sigma = np.sqrt(n * beta * (1 - beta))
        return np.ceil(m + mu + sigma * np.sqrt(chi2_value))

    N = num_points
    m = num_points_to_sample
    T_N = niter_ransac(p_good_sample, max_outlier_proportion, num_points_to_sample, -1)
    I_N_min = (1 - max_outlier_proportion) * N

    n_star = N
    I_n_star = 0
    I_N_best = 0
    t = 0
    n = m
    T_n = T_N

    for i in range(m):
        T_n = T_n * (n - i) / (N - i)

    T_n_prime = 1
    k_n_star = T_N

    while ((I_N_best < I_N_min) or t <= k_n_star) and t < T_N and t <= max_number_of_draws:
        t = t + 1

        if (t > T_n_prime) and (n < n_star):
            T_nplus1 = (T_n * (n + 1)) / (n + 1 - m)
            n = n + 1
            T_n_prime = T_n_prime + np.ceil(T_nplus1 - T_n)
            T_n = T_nplus1

        if t > T_n_prime:
            pts_idx = np.random.choice(n, m, replace=False)
        else:
            pts_idx = np.append(np.random.choice(n - 1, m - 1, replace=False), n)

        sample = data[pts_idx]

        model = model_type()
        model.fit(sample)

        error = model.error(data)
        is_inlier = (error < tolerance)
        I_N = is_inlier.sum()

        if I_N > I_N_best:
            I_N_best = I_N
            n_best = N
            I_n_best = I_N
            best_model = model

            if enable_n_star_optimization:
                epsilon_n_best = I_n_best / n_best
                I_n_test = I_N
                for n_test in range(N, m, -1):
                    if not (n_test >= I_n_test):
                        raise RuntimeError('Loop invariant broken: n_test >= I_n_test')
                    if ((I_n_test * n_best > I_n_best * n_test) and (I_n_test > epsilon_n_best * n_test + np.sqrt(
                            n_test * epsilon_n_best * (1 - epsilon_n_best) * chi2_value))):
                        if I_n_test < i_min(m, n_test, beta):
                            break
                        n_best = n_test
                        I_n_best = I_n_test
                        epsilon_n_best = I_n_best / n_best
                    I_n_test = I_n_test - is_inlier[n_test - 1]

            if I_n_best * n_star > I_n_star * n_best:
                if not (n_best >= I_n_best):
                    raise RuntimeError('Assertion not respected: n_best >= I_n_best')
                n_star = n_best
                I_n_star = I_n_best
                k_n_star = niter_ransac(1 - eta0, 1 - I_n_star / n_star, m, T_N)

    return best_model

# LinearModel class
class LinearModel(Model):

    def __init__(self):
        self.m = 0
        self.b = 0

    def fit(self, pts):
        xs = pts[:, 0]
        ys = pts[:, 1]
        self.m, self.b = stats.linregress(xs, ys)[:2]

    def predict(self, x):
        return self.m * x + self.b

    def error(self, data):
        prediction = self.predict(data[:, 0])
        true_value = data[:, 1]
        return np.sqrt(np.square(true_value - prediction))

    @staticmethod
    def get_complexity():
        return 2
