import numpy as np

class NonHolonomicRobot:
    def __init__(self, x=1.0, y=1.0, theta=np.pi/4, v_max=0.5, w_max=1.0):
        self.x = x
        self.y = y
        self.theta = theta
        self.v_max = v_max
        self.w_max = w_max
        self.radius = 0.2
        
    def move(self, v, w, dt):
        # 1. Hız limitlerini uygula (Sadece ileri hareket ve maksimum dönüş hızı)
        v = np.clip(v, 0, self.v_max) 
        w = np.clip(w, -self.w_max, self.w_max)
        
        # 2. Kinematik durumu güncelle (Zaman adımı dt ile)
        self.theta += w * dt
        
        # Açıyı [-pi, pi] aralığında tut
        self.theta = (self.theta + np.pi) % (2 * np.pi) - np.pi 
        
        self.x += v * np.cos(self.theta) * dt
        self.y += v * np.sin(self.theta) * dt
        
        # Aracın "Gerçek" konumunu döndür
        return self.x, self.y, self.theta