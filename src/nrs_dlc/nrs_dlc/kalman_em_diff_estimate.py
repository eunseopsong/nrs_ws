import numpy as np

def kalman_em_diff_estimate(Z, max_iter=100, tol=1e-4):
    """
    Implements the EM algorithm to estimate the parameters of a Kalman filter model,
    then applies the filter to estimate the hidden states and their derivatives.
    
    Parameters:
    - Z: numpy.ndarray, shape (T, n_z)
        Observed data (e.g., force or moment readings).
    - max_iter: int
        Maximum number of EM iterations.
    - tol: float
        Convergence tolerance.

    Returns:
    - X_hat: numpy.ndarray, shape (T, n_z)
        Estimated states (filtered signal).
    - V: numpy.ndarray, shape (T, n_z, n_z)
        State covariance matrices.
    - A: numpy.ndarray
        Estimated state transition matrix.
    - C: numpy.ndarray
        Estimated observation matrix.
    - Q: numpy.ndarray
        Estimated process noise covariance.
    - R: numpy.ndarray
        Estimated measurement noise covariance.
    """
    T, n_z = Z.shape
    n_x = n_z  # State dimension equals observation dimension

    # Initialize parameters
    A = np.eye(n_x)  # Initial guess for state transition
    C = np.eye(n_z)  # Observation matrix
    Q = np.eye(n_x) * 0.01
    R = np.eye(n_z) * 1.0
    mu_0 = np.zeros((n_x,))
    V_0 = np.eye(n_x)

    ll_old = -np.inf  # Log-likelihood

    for iteration in range(max_iter):
        # === E-step ===
        x_pred = np.zeros((T, n_x))
        V_pred = np.zeros((T, n_x, n_x))
        x_filt = np.zeros((T, n_x))
        V_filt = np.zeros((T, n_x, n_x))
        J = np.zeros((T - 1, n_x, n_x))  # Smoother gain
        x_smooth = np.zeros((T, n_x))
        V_smooth = np.zeros((T, n_x, n_x))
        V_pair = np.zeros((T - 1, n_x, n_x))

        # Forward filtering (Kalman filter)
        x_pred[0] = mu_0
        V_pred[0] = V_0
        for t in range(T):
            if t > 0:
                x_pred[t] = A @ x_filt[t-1]
                V_pred[t] = A @ V_filt[t-1] @ A.T + Q
            K = V_pred[t] @ C.T @ np.linalg.inv(C @ V_pred[t] @ C.T + R)
            x_filt[t] = x_pred[t] + K @ (Z[t] - C @ x_pred[t])
            V_filt[t] = V_pred[t] - K @ C @ V_pred[t]

        # Backward smoothing
        x_smooth[-1] = x_filt[-1]
        V_smooth[-1] = V_filt[-1]
        for t in reversed(range(T - 1)):
            J[t] = V_filt[t] @ A.T @ np.linalg.inv(V_pred[t+1])
            x_smooth[t] = x_filt[t] + J[t] @ (x_smooth[t+1] - x_pred[t+1])
            V_smooth[t] = V_filt[t] + J[t] @ (V_smooth[t+1] - V_pred[t+1]) @ J[t].T
            V_pair[t] = J[t] @ V_smooth[t+1]

        # === M-step ===
        sum_xx = np.zeros((n_x, n_x))
        sum_xx1 = np.zeros((n_x, n_x))
        sum_x1x1 = np.zeros((n_x, n_x))
        sum_zz = np.zeros((n_z, n_z))
        sum_zx = np.zeros((n_z, n_x))
        for t in range(T):
            sum_xx += V_smooth[t] + np.outer(x_smooth[t], x_smooth[t])
            sum_zz += np.outer(Z[t], Z[t])
            sum_zx += np.outer(Z[t], x_smooth[t])
        for t in range(1, T):
            sum_xx1 += V_pair[t-1] + np.outer(x_smooth[t], x_smooth[t-1])
            sum_x1x1 += V_smooth[t-1] + np.outer(x_smooth[t-1], x_smooth[t-1])

        A_new = sum_xx1 @ np.linalg.inv(sum_x1x1)
        C_new = sum_zx @ np.linalg.inv(sum_xx)
        Q_new = (sum_xx - A_new @ sum_xx1.T - sum_xx1 @ A_new.T + A_new @ sum_x1x1 @ A_new.T) / (T - 1)
        R_new = (sum_zz - C_new @ sum_zx.T - sum_zx @ C_new.T + C_new @ sum_xx @ C_new.T) / T

        # Check for convergence
        ll = -0.5 * T * np.log(np.linalg.det(R_new))  # Rough log-likelihood
        if np.abs(ll - ll_old) < tol:
            break
        ll_old = ll

        A, C, Q, R = A_new, C_new, Q_new, R_new

    return x_smooth, V_smooth, A, C, Q, R
