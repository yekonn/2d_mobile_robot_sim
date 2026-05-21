# 2B Otonom Mobil Robot Navigasyonu ve Sensör Füzyonu

![Simülasyon Animasyonu](assets/simulasyon.gif)

Bu proje, 15x15 metrelik karmaşık ve engellerle dolu bir ortamda...
Bu proje, 15x15 metrelik karmaşık ve engellerle dolu bir ortamda, hedef noktasına güvenle ulaşmaya çalışan otonom bir mobil robotun Python tabanlı simülasyonudur. Sistem, gerçek dünya kısıtlarını (sensör gürültüleri ve sürüklenme hataları) modelleyerek, Genişletilmiş Kalman Filtresi (EKF) ve Yapay Potansiyel Alanlar (APF) algoritmaları ile bu zorlukların üstesinden gelmektedir.

## Proje Özellikleri

* **Modüler Mimari:** Temiz kod prensiplerine (OOP) uygun olarak ayrılmış robot, sensör, EKF, görselleştirme ve çevre modülleri.
* **Genişletilmiş Kalman Filtresi (EKF):** Tekerlek enkoderi (Dead Reckoning) ve IMU verilerindeki Gauss gürültüsü ile kümülatif kaymaları (drift) filtreleyen sensör füzyonu mimarisi.
* **Yapay Potansiyel Alanlar (APF):** Lidar verileriyle anlık olarak çalışan, dinamik itme ve çekme kuvvetlerine dayalı reaktif navigasyon.
* **Durum Makinesi (Kaçış Manevrası):** APF'nin kronik "yerel minimum" kilitlenmelerini algılayan ve Lidar'ın gösterdiği en derin boşluğa doğru otonom kurtarma manevrası yapan akıllı algoritma.
* **Canlı Sensör Görselleştirmesi:** Simülasyon anında 36 ışınlı LiDAR'ın ortamı taramasını eşzamanlı gösteren Dashboard ve simülasyon sonu Nokta Bulutu (Point Cloud) haritalaması.

## Kurulum ve Çalıştırma

Projeyi kendi bilgisayarınızda çalıştırmak için Python 3.x yüklü olmalıdır.

1. Depoyu bilgisayarınıza klonlayın:
`git clone https://github.com/KULLANICI_ADINIZ/mobile_robot_sim.git`

2. Gerekli kütüphaneleri yükleyin:
`pip install numpy matplotlib`

3. Ana simülasyonu başlatın:
`python main.py`

## Proje Çıktıları ve Analiz

Simülasyon tamamlandığında sistem otomatik olarak şu analiz çıktılarını üretecektir:
1. **Sensör Füzyonu Rota Takibi:** Gerçek konum ile EKF tahmininin 15x15 harita üzerinde karşılaştırılması.
2. **LiDAR Nokta Bulutu:** Robotun tüm rotası boyunca algıladığı engellerin birikmiş silüet haritası.
3. **Zaman Serisi ve RMSE Analizi:** $x(t)$, $y(t)$ ve $\theta(t)$ durumları için zaman eksenli sapma (drift) grafikleri.

---
*Geliştirici: Yasin Efe KUL*
