import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation
import numpy as np

class Environment:
    def __init__(self, start_point, goal_point, obstacles):
        self.start_point = start_point
        self.goal_point = goal_point
        self.obstacles = obstacles

    def animate(self, true_states, est_states, lidar_history, raw_lidar_history, dt=0.1):
        # YENİ: İki Panelli (1x2) Dashboard Oluşturuluyor
        fig, (ax_main, ax_lidar) = plt.subplots(1, 2, figsize=(14, 7))
        fig.canvas.manager.set_window_title('Otonom Navigasyon ve Canlı LiDAR Paneli')

        # ==========================================
        # SOL PANEL: ANA HARİTA (Sensör Füzyonu)
        # ==========================================
        ax_main.set_aspect('equal')
        ax_main.set_xlim(0, 15) # 10 yerine 15 oldu
        ax_main.set_ylim(0, 15) # 10 yerine 15 oldu
        ax_main.grid(True)
        ax_main.set_title("Sensör Füzyonlu Rota Takibi", fontweight='bold')

        for (x, y, w, h) in self.obstacles:
            rect = patches.Rectangle((x, y), w, h, linewidth=1, edgecolor='black', facecolor='gray')
            ax_main.add_patch(rect)
        
        ax_main.plot(self.start_point[0], self.start_point[1], 'go', markersize=10, label="Başlangıç", zorder=2)
        ax_main.plot(self.goal_point[0], self.goal_point[1], 'ro', markersize=10, label="Hedef")

        true_trail, = ax_main.plot([], [], color='green', linewidth=2, alpha=0.7, label="Gerçek Rota")
        est_trail, = ax_main.plot([], [], color='magenta', linewidth=2, linestyle='--', label="Füzyon Tahmini")
        
        robot_radius = 0.2
        robot_main = patches.Circle((self.start_point[0], self.start_point[1]), robot_radius, facecolor='blue', label="Robot", zorder=3)
        ax_main.add_patch(robot_main)
        robot_heading, = ax_main.plot([], [], color='white', linewidth=2, zorder=4)

        # ==========================================
        # SAĞ PANEL: CANLI LİDAR GÖRSELLEŞTİRMESİ
        # ==========================================
        ax_lidar.set_aspect('equal')
        ax_lidar.set_xlim(0, 15) # 10 yerine 15 oldu
        ax_lidar.set_ylim(0, 15) # 10 yerine 15 oldu
        ax_lidar.grid(True, linestyle='--', alpha=0.5)
        ax_lidar.set_title("Lidar Görselleştirmesi", fontweight='bold')

        for (x, y, w, h) in self.obstacles:
            rect = patches.Rectangle((x, y), w, h, linewidth=1, edgecolor='black', facecolor='gray')
            ax_lidar.add_patch(rect)

        ax_lidar.plot(self.start_point[0], self.start_point[1], 'go', markersize=10, label="Start")
        ax_lidar.plot(self.goal_point[0], self.goal_point[1], 'ro', markersize=10, label="Goal")

        robot_lidar, = ax_lidar.plot([], [], 'm^', markersize=12, label="Robot (Anlık)", zorder=3)
        
        num_rays = len(raw_lidar_history[0]) if raw_lidar_history else 36
        lidar_lines = [ax_lidar.plot([], [], color='khaki', alpha=0.7, linewidth=1.5, zorder=1)[0] for _ in range(num_rays)]
        lidar_points, = ax_lidar.plot([], [], 'ro', markersize=4, label="Ham LiDAR Noktaları", zorder=2)

        ax_main.legend(loc='lower left', fontsize=9)
        ax_lidar.legend(loc='lower right', fontsize=9)

        # ==========================================
        # ANİMASYON DÖNGÜSÜ (İkisini aynı anda saniye saniye günceller)
        # ==========================================
        def update(frame):
            tx, ty, t_theta = true_states[frame]
            ex, ey, e_theta = est_states[frame]

            # 1. Sol Paneli Güncelle
            robot_main.set_center((tx, ty))
            hx = tx + robot_radius * np.cos(t_theta)
            hy = ty + robot_radius * np.sin(t_theta)
            robot_heading.set_data([tx, hx], [ty, hy])

            t_xs, t_ys, _ = zip(*true_states[:frame+1])
            e_xs, e_ys, _ = zip(*est_states[:frame+1])
            true_trail.set_data(t_xs, t_ys)
            est_trail.set_data(e_xs, e_ys)

            # 2. Sağ Paneli (LiDAR) Güncelle
            robot_lidar.set_data([tx], [ty])
            
            measurements = raw_lidar_history[frame]
            point_xs = []
            point_ys = []
            
            for i, (local_angle, dist) in enumerate(measurements):
                global_angle = local_angle + t_theta
                end_x = tx + dist * np.cos(global_angle)
                end_y = ty + dist * np.sin(global_angle)
                
                lidar_lines[i].set_data([tx, end_x], [ty, end_y])
                
                # Duvara çarpan (max menzilden kısa olan) noktaları kırmızı işaretle
                if dist < 3.99: 
                    point_xs.append(end_x)
                    point_ys.append(end_y)
            
            lidar_points.set_data(point_xs, point_ys)
            
            return [robot_main, robot_heading, true_trail, est_trail, robot_lidar, lidar_points] + lidar_lines

        #import os
        ani = animation.FuncAnimation(fig, update, frames=len(true_states), interval=50, blit=True, repeat=False)
        
        #if not os.path.exists('assets'):
        #    os.makedirs('assets')
            
        #print("\nAnimasyon 'assets/simulasyon.gif' olarak kaydediliyor...")
        #print("Lütfen bekleyin (Parkur uzun olduğu için 1-2 dakika sürebilir)...")
        
        # GIF olarak kaydet (Pillow kütüphanesini kullanır)
        #ani.save('assets/simulasyon.gif', writer='pillow', fps=20)
        
        #print("Animasyon başarıyla kaydedildi! Şimdi ekranda izleyebilirsiniz.")
        # ---------------------------------

        plt.tight_layout()
        plt.show()