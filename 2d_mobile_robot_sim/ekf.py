import numpy as np

class ExtendedKalmanFilter:
    def __init__(self, initial_state, initial_P, Q, R):
        """
        EKF Sınıfı Başlatıcı
        :param initial_state: [x, y, theta] başlangıç konumu
        :param initial_P: Başlangıç Kovaryans Matrisi
        :param Q: Süreç Gürültüsü (Enkoder Hatası)
        :param R: Ölçüm Gürültüsü (IMU Hatası)
        """
        self.X = np.array(initial_state, dtype=float)
        self.P = np.array(initial_P, dtype=float)
        self.Q = np.array(Q, dtype=float)
        self.R = np.array(R, dtype=float)

    def predict(self, v_enc, w_enc, dt):
        """1. ADIM: Enkoder verisiyle durum tahmini (Prediction)"""
        # Jacobian Matrisi (F)
        F = np.array([
            [1.0, 0.0, -v_enc * np.sin(self.X[2]) * dt],
            [0.0, 1.0,  v_enc * np.cos(self.X[2]) * dt],
            [0.0, 0.0,  1.0]
        ])
        
        # Durum Tahmini (State Prediction)
        self.X[0] += v_enc * np.cos(self.X[2]) * dt
        self.X[1] += v_enc * np.sin(self.X[2]) * dt
        self.X[2] += w_enc * dt
        
        # Açıyı [-pi, pi] aralığında tut
        self.X[2] = (self.X[2] + np.pi) % (2 * np.pi) - np.pi
        
        # Kovaryans Tahmini (Covariance Prediction)
        self.P = F @ self.P @ F.T + self.Q

    def update(self, theta_imu):
        """2. ADIM: IMU verisiyle durumu güncelleme (Update)"""
        # Ölçüm Matrisi (H)
        H = np.array([[0.0, 0.0, 1.0]])
        Z = np.array([theta_imu]) 
        
        # Yenilik / Hata Matrisi (y)
        y = Z - (H @ self.X)
        y[0] = (y[0] + np.pi) % (2 * np.pi) - np.pi 
        
        # Kalman Kazancı (K)
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # Durum Güncellemesi (State Update)
        self.X = self.X + (K @ y).flatten()
        self.X[2] = (self.X[2] + np.pi) % (2 * np.pi) - np.pi
        
        # Kovaryans Güncellemesi (Covariance Update)
        self.P = (np.eye(3) - K @ H) @ self.P