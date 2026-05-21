import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

def plot_accumulated_lidar(true_states, raw_lidar_history, obstacles, start_point, goal_point):
    """Simülasyon sonunda tüm LiDAR çarpmalarını gösteren Nokta Bulutu Haritası"""
    fig, ax = plt.subplots(figsize=(8, 8))
    fig.canvas.manager.set_window_title('6.3 Sensör Görselleştirmesi (Birikmiş Nokta Bulutu)')
    ax.set_aspect('equal')
    ax.set_xlim(0, 15)
    ax.set_ylim(0, 15)
    ax.grid(True, linestyle='--', alpha=0.5)
    ax.set_title("Sensör Görselleştirmesi (Tüm LiDAR Çarpışma Noktaları)", fontweight='bold')

    for (x, y, w, h) in obstacles:
        rect = patches.Rectangle((x, y), w, h, linewidth=1, edgecolor='black', facecolor='gray')
        ax.add_patch(rect)

    ax.plot(start_point[0], start_point[1], 'go', markersize=10, label="Başlangıç", zorder=4)
    ax.plot(goal_point[0], goal_point[1], 'ro', markersize=10, label="Hedef", zorder=4)

    t_xs = [s[0] for s in true_states]
    t_ys = [s[1] for s in true_states]
    ax.plot(t_xs, t_ys, 'g-', alpha=0.3, linewidth=2, label="Gerçek Rota", zorder=2)

    all_hit_xs = []
    all_hit_ys = []
    
    for frame in range(len(true_states)):
        tx, ty, t_theta = true_states[frame]
        measurements = raw_lidar_history[frame]
        
        for local_angle, dist in measurements:
            if dist < 3.95: 
                global_angle = local_angle + t_theta
                end_x = tx + dist * np.cos(global_angle)
                end_y = ty + dist * np.sin(global_angle)
                all_hit_xs.append(end_x)
                all_hit_ys.append(end_y)

    ax.plot(all_hit_xs, all_hit_ys, 'r.', markersize=3, label="LiDAR Çarpışma Noktaları", zorder=3)

    ax.legend(loc='lower right')
    plt.tight_layout()
    plt.show()

def plot_error_analysis(true_states, est_states, dt):
    """Lokasyon ve Navigasyon Hata Analizi (RMSE) ve Zaman Serisi"""
    times = [i * dt for i in range(len(true_states))]
    
    t_x = [state[0] for state in true_states]
    t_y = [state[1] for state in true_states]
    t_theta = [np.degrees(state[2]) for state in true_states]
    
    e_x = [state[0] for state in est_states]
    e_y = [state[1] for state in est_states]
    e_theta = [np.degrees(state[2]) for state in est_states]

    pos_errors = [np.hypot(tx - ex, ty - ey) for tx, ty, ex, ey in zip(t_x, t_y, e_x, e_y)]
    theta_errors = [np.degrees(abs((np.radians(tt) - np.radians(et) + np.pi) % (2 * np.pi) - np.pi)) 
                    for tt, et in zip(t_theta, e_theta)]

    rmse_pos = np.sqrt(np.mean(np.array(pos_errors)**2))
    rmse_theta = np.sqrt(np.mean(np.array(theta_errors)**2))
    
    fig1, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 10))
    fig1.canvas.manager.set_window_title('Zaman Ekseninde Durum Karşılaştırması')
    
    ax1.plot(times, t_x, 'g-', label='Gerçek $x(t)$', linewidth=2)
    ax1.plot(times, e_x, 'm--', label='EKF Tahmini $x(t)$', linewidth=2)
    ax1.set_ylabel("X Konumu (m)")
    ax1.set_title("Zaman Serisi Grafiği: Gerçek vs Tahmin Edilen $x(t)$")
    ax1.grid(True); ax1.legend()

    ax2.plot(times, t_y, 'g-', label='Gerçek $y(t)$', linewidth=2)
    ax2.plot(times, e_y, 'm--', label='EKF Tahmini $y(t)$', linewidth=2)
    ax2.set_ylabel("Y Konumu (m)")
    ax2.set_title("Zaman Serisi Grafiği: Gerçek vs Tahmin Edilen $y(t)$")
    ax2.grid(True); ax2.legend()

    ax3.plot(times, t_theta, 'g-', label=r'Gerçek $\theta(t)$', linewidth=2)
    ax3.plot(times, e_theta, 'm--', label=r'EKF Tahmini $\theta(t)$', linewidth=2)
    ax3.set_xlabel("Zaman (saniye)")
    ax3.set_ylabel("Yönelim (Derece)")
    ax3.set_title(r"Zaman Serisi Grafiği: Gerçek vs Tahmin Edilen $\theta(t)$")
    ax3.grid(True); ax3.legend()

    plt.tight_layout()
    plt.show()

    fig2, (ax4, ax5) = plt.subplots(2, 1, figsize=(10, 8))
    fig2.canvas.manager.set_window_title('Hata Analizi (RMSE)')

    ax4.plot(times, pos_errors, 'b-', linewidth=2)
    ax4.fill_between(times, pos_errors, color='blue', alpha=0.1)
    ax4.set_title(f"Zaman Boyunca Konum Hatası (RMSE: {rmse_pos:.3f} m)", fontsize=13, fontweight='bold')
    ax4.set_xlabel("Zaman (saniye)")
    ax4.set_ylabel("Hata (Metre)")
    ax4.grid(True, linestyle='--', alpha=0.7)

    ax5.plot(times, theta_errors, 'r-', linewidth=2)
    ax5.fill_between(times, theta_errors, color='red', alpha=0.1)
    ax5.set_title(f"Zaman Boyunca Yönelim Hatası (RMSE: {rmse_theta:.3f}°)", fontsize=13, fontweight='bold')
    ax5.set_xlabel("Zaman (saniye)")
    ax5.set_ylabel("Hata (Derece)")
    ax5.grid(True, linestyle='--', alpha=0.7)

    plt.tight_layout()
    plt.show()