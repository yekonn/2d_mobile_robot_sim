import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

class Encoder:
    def __init__(self, v_noise_std=0.05, w_noise_std=0.05):
        self.v_noise_std = v_noise_std
        self.w_noise_std = w_noise_std
        
    def read(self, v_true, w_true):
        v_meas = v_true + np.random.normal(0, self.v_noise_std)
        w_meas = w_true + np.random.normal(0, self.w_noise_std)
        return v_meas, w_meas

class IMU:
    def __init__(self, w_noise_std=0.02, drift_rate=0.002):
        self.w_noise_std = w_noise_std
        self.drift_rate = drift_rate
        self.accumulated_drift = 0.0
        
    def read(self, theta_true, w_true, dt):
        self.accumulated_drift += self.drift_rate * dt
        theta_meas = theta_true + self.accumulated_drift + np.random.normal(0, 0.01)
        w_meas = w_true + np.random.normal(0, self.w_noise_std)
        return theta_meas, w_meas

class Lidar:
    def __init__(self, num_rays=36, max_range=4.0):
        self.num_rays = num_rays
        self.max_range = max_range
        self.angles = np.linspace(0, 2 * np.pi, num_rays, endpoint=False)
        
        # Görselleştirme nesneleri
        self.fig_viz = None
        self.ax_viz = None
        self.viz_lines = []
        self.viz_points = None
        self.viz_robot = None
        self.start_marker = None
        self.goal_marker = None
        self.viz_obstacles = []

    def start_viz(self, start_point, goal_point, obstacles):
        """Lidar görselleştirme figürünü ve sabit nesneleri oluşturur (Lejant dahil)."""
        self.fig_viz, self.ax_viz = plt.subplots(figsize=(6, 6))
        self.fig_viz.canvas.manager.set_window_title('6.3 Sensör Görselleştirmesi (LiDAR)')
        
        self.ax_viz.set_aspect('equal')
        # Görselleştirmeyi image_11.png'deki gibi ayarlamak için eksenleri ayarla
        self.ax_viz.set_xlim(1, 10.5)
        self.ax_viz.set_ylim(-1.5, 9)
        self.ax_viz.grid(True, linestyle='--', alpha=0.5)
        self.ax_viz.set_title("6.3 Sensör Görselleştirmesi (LiDAR)")

        # Engelleri ekle (Gri dikdörtgenler)
        for (x, y, w, h) in obstacles:
            rect = patches.Rectangle((x, y), w, h, linewidth=1, edgecolor='black', facecolor='gray')
            self.ax_viz.add_patch(rect)
            self.viz_obstacles.append(rect)

        # Başlangıç/Hedef (Sabit)
        self.start_marker, = self.ax_viz.plot(start_point[0], start_point[1], 'go', markersize=10, label="Start", zorder=2)
        self.goal_marker, = self.ax_viz.plot(goal_point[0], goal_point[1], 'ro', markersize=10, label="Goal", zorder=2)

        # Robot (Hafıza tutucusu - Mor üçgen)
        self.viz_robot, = self.ax_viz.plot([], [], 'm^', markersize=12, label="Robot (Anlık)", zorder=3)

        # Lidar Işınları (Hafıza tutucusu - Sarı çizgiler)
        for _ in range(self.num_rays):
            line, = self.ax_viz.plot([], [], color='khaki', alpha=0.7, linewidth=1.5, zorder=1)
            self.viz_lines.append(line)

        # Çarpışma Noktaları (Hafıza tutucusu - Kırmızı noktalar)
        self.viz_points, = self.ax_viz.plot([], [], 'ro', markersize=4, label="LiDAR Noktaları", zorder=2)

        # Lejantı ekle (image_11.png'deki gibi sağ alt)
        self.ax_viz.legend(loc='lower right')
        
        # Figürü çiz
        self.fig_viz.canvas.draw()
        plt.tight_layout()

    def update_viz(self, rx, ry, r_theta, measurements):
        """Lidar görselleştirmesini robotun konumuna ve ölçümlerine göre günceller."""
        if self.fig_viz is None:
            return

        # Robot konumunu güncelle
        self.viz_robot.set_data([rx], [ry])

        # Işınları ve noktaları güncelle
        point_xs = []
        point_ys = []
        for i, (local_angle, dist) in enumerate(measurements):
            global_angle = local_angle + r_theta
            end_x = rx + dist * np.cos(global_angle)
            end_y = ry + dist * np.sin(global_angle)
            
            # Işın çizgisini güncelle
            self.viz_lines[i].set_data([rx, end_x], [ry, end_y])

            # Çarpışma noktasını kaydet (Menzil içindeyse)
            if dist < self.max_range:
                point_xs.append(end_x)
                point_ys.append(end_y)

        # Tüm çarpışma noktalarını tek seferde güncelle
        self.viz_points.set_data(point_xs, point_ys)

        # Figürü çiz (Ana animasyonu dondurmadan canlı güncelleme için)
        self.fig_viz.canvas.draw()
        self.fig_viz.canvas.flush_events()

    def scan(self, rx, ry, r_theta, obstacles):
        measurements = []
        for angle in self.angles:
            global_angle = angle + r_theta
            min_dist = self.max_range
            
            for ray_t in np.linspace(0, self.max_range, 40):
                ray_x = rx + ray_t * np.cos(global_angle)
                ray_y = ry + ray_t * np.sin(global_angle)
                
                hit = False
                for (ox, oy, w, h) in obstacles:
                    if ox <= ray_x <= ox + w and oy <= ray_y <= oy + h:
                        min_dist = ray_t
                        hit = True
                        break
                if hit: break
                
            noisy_dist = min_dist + np.random.normal(0, 0.02)
            if noisy_dist > self.max_range: noisy_dist = self.max_range
            measurements.append((angle, noisy_dist))
            
        return measurements