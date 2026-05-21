import numpy as np
from robot import NonHolonomicRobot
from sensors import Encoder, IMU, Lidar
from environment import Environment
from visualizer import plot_error_analysis, plot_accumulated_lidar
from ekf import ExtendedKalmanFilter  # YENİ: EKF sınıfımızı import ettik

def moving_average_filter(data, window_size=3):
    filtered = np.copy(data)
    for i in range(len(data)):
        window = [data[(i + j - window_size//2) % len(data)] for j in range(window_size)]
        filtered[i] = np.mean(window)
    return filtered

def main():
    # --- 1. SİMÜLASYON KURULUMU ---
    start_point = (1.0, 1.5)
    goal_point = (14.0, 14.0)
    
    obstacles = [
        (2, 2, 2, 2), (5, 1, 1, 3), (8, 2, 3, 1),
        (2, 7, 2, 1), (6, 6, 2, 2), (10, 5, 1, 3),
        (1, 10, 3, 1), (5, 11, 2, 2), (9, 9, 2, 2),
        (12, 1, 1, 4), (13, 8, 2, 1), (11, 12, 3, 1),
        (2, 13, 2, 1)
    ]

    robot = NonHolonomicRobot(x=start_point[0], y=start_point[1], theta=np.pi/4)
    encoder = Encoder(v_noise_std=0.01, w_noise_std=0.01)
    imu = IMU(w_noise_std=0.005, drift_rate=0.001)
    lidar = Lidar(num_rays=36, max_range=4.0) 
    env = Environment(start_point, goal_point, obstacles)

    # YENİ: EKF Nesnesini Başlatıyoruz
    ekf = ExtendedKalmanFilter(
        initial_state=[start_point[0], start_point[1], np.pi/4],
        initial_P=np.eye(3) * 0.1,
        Q=np.diag([0.005, 0.005, 0.01]),
        R=np.array([[0.05]])
    )

    # --- 2. DEĞİŞKENLER VE LİSTELER ---
    true_states = []
    est_states = []
    lidar_history = []
    raw_lidar_history = [] 

    dt = 0.1
    k_att = 4.0
    k_rep = 2.0
    d0 = 0.8
    max_steps = 4000 

    position_history = []  
    recovery_timer = 0     
    escape_angle = 0.0     
    angles = np.linspace(0, 2 * np.pi, lidar.num_rays, endpoint=False)

    # --- 3. ANA KONTROL DÖNGÜSÜ ---
    for step in range(max_steps):
        true_x, true_y, true_theta = robot.x, robot.y, robot.theta
        true_states.append((true_x, true_y, true_theta))
        est_states.append((ekf.X[0], ekf.X[1], ekf.X[2]))
        
        # A) Sensör Okumaları
        raw_lidar_meas = lidar.scan(true_x, true_y, true_theta, obstacles)
        raw_distances = [d for a, d in raw_lidar_meas]
        raw_lidar_history.append(raw_lidar_meas) 
        
        filtered_distances = moving_average_filter(raw_distances, window_size=3)
        filtered_lidar_meas = [(angles[i], filtered_distances[i]) for i in range(len(angles))]
        lidar_history.append(filtered_lidar_meas)

        # Hedef Kontrolü
        if np.hypot(goal_point[0] - ekf.X[0], goal_point[1] - ekf.X[1]) < 0.2:
            print(f"Adım {step}: Hedefe başarıyla ulaşıldı!")
            break

        # B) Sıkışma Algılayıcı ve Kaçış Manevrası
        position_history.append((ekf.X[0], ekf.X[1]))
        if len(position_history) > 40:
            position_history.pop(0)

        if len(position_history) == 40 and recovery_timer == 0:
            past_x, past_y = position_history[0]
            if np.hypot(ekf.X[0] - past_x, ekf.X[1] - past_y) < 0.15:
                recovery_timer = 25 
                max_dist = -1
                best_local_angle = 0
                for local_angle, dist in filtered_lidar_meas:
                    if dist > max_dist:
                        max_dist = dist
                        best_local_angle = local_angle
                
                escape_angle = ekf.X[2] + best_local_angle
                escape_angle = (escape_angle + np.pi) % (2 * np.pi) - np.pi
                position_history.clear()

        # C) Yörünge Planlama (APF)
        if recovery_timer > 0:
            recovery_timer -= 1
            error_theta = escape_angle - ekf.X[2]
            error_theta = (error_theta + np.pi) % (2 * np.pi) - np.pi
            
            v_cmd = robot.v_max * 0.8 
            if abs(error_theta) > np.pi/4:
                v_cmd = 0.0 
            w_cmd = 2.0 * error_theta

        else:
            dist_to_goal = np.hypot(goal_point[0] - ekf.X[0], goal_point[1] - ekf.X[1])
            fx_att = k_att * (goal_point[0] - ekf.X[0]) / dist_to_goal
            fy_att = k_att * (goal_point[1] - ekf.X[1]) / dist_to_goal

            fx_rep, fy_rep = 0.0, 0.0
            for local_angle, dist in filtered_lidar_meas:
                if 0.05 < dist < d0:
                    global_angle = ekf.X[2] + local_angle
                    force = k_rep * (1.0 / dist - 1.0 / d0) * (1.0 / (dist**2))
                    fx_rep += force * (-np.cos(global_angle))
                    fy_rep += force * (-np.sin(global_angle))

            max_rep_force = 10.0
            rep_magnitude = np.hypot(fx_rep, fy_rep)
            if rep_magnitude > max_rep_force:
                fx_rep = (fx_rep / rep_magnitude) * max_rep_force
                fy_rep = (fy_rep / rep_magnitude) * max_rep_force

            fx = fx_att + fx_rep
            fy = fy_att + fy_rep

            desired_theta = np.arctan2(fy, fx)
            error_theta = desired_theta - ekf.X[2]
            error_theta = (error_theta + np.pi) % (2 * np.pi) - np.pi

            v_cmd = robot.v_max * np.cos(error_theta)
            if v_cmd < 0: v_cmd = 0.0 
            w_cmd = 1.5 * error_theta

        # D) Fiziksel Hareket ve Sensör Gürültüsü
        robot.move(v_cmd, w_cmd, dt)
        v_true = np.clip(v_cmd, 0, robot.v_max)
        w_true = np.clip(w_cmd, -robot.w_max, robot.w_max)
        
        v_enc, w_enc = encoder.read(v_true, w_true)
        theta_imu, w_imu = imu.read(robot.theta, w_true, dt)

        # E) LOKALİZASYON (EKF Modülü Çağrısı)
        # YENİ: Karmaşık matrisler tek satıra indi!
        ekf.predict(v_enc, w_enc, dt)
        ekf.update(theta_imu)

    # --- 4. GÖRSELLEŞTİRME ÇIKTILARI ---
    print("Simülasyon bitti, pencereler sırayla açılıyor...")
    env.animate(true_states, est_states, lidar_history, raw_lidar_history, dt)
    plot_error_analysis(true_states, est_states, dt)
    plot_accumulated_lidar(true_states, raw_lidar_history, obstacles, start_point, goal_point)

if __name__ == '__main__':
    main()