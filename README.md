# 2B Otonom Mobil Robot Navigasyonu ve Sensör Füzyonu

![Simülasyon Animasyonu](assets/simulasyon.gif)

Bu proje, 15x15 metrelik karmaşık ve engellerle dolu bir ortamda, hedef noktasına güvenle ulaşmaya çalışan otonom bir mobil robotun Python tabanlı simülasyonudur. Sistem, gerçek dünya kısıtlarını (sensör gürültüleri ve sürüklenme hataları) modelleyerek, Genişletilmiş Kalman Filtresi (EKF) ve Yapay Potansiyel Alanlar (APF) algoritmaları ile bu zorlukların üstesinden gelmektedir.

## Gereksinimler

* Python 3.8 veya üzeri
* Matplotlib
* NumPy

## Proje Özellikleri

* **Modüler Mimari:** Temiz kod prensiplerine (OOP) uygun olarak ayrılmış robot, sensör, EKF, görselleştirme ve çevre modülleri.
* **Genişletilmiş Kalman Filtresi (EKF):** Tekerlek enkoderi (Dead Reckoning) ve IMU verilerindeki Gauss gürültüsü ile kümülatif kaymaları (drift) filtreleyen sensör füzyonu mimarisi.
* **Yapay Potansiyel Alanlar (APF):** Lidar verileriyle anlık olarak çalışan, dinamik itme ve çekme kuvvetlerine dayalı reaktif navigasyon.
* **Durum Makinesi (Kaçış Manevrası):** APF'nin kronik "yerel minimum" kilitlenmelerini algılayan ve Lidar'ın gösterdiği en derin boşluğa doğru otonom kurtarma manevrası yapan akıllı algoritma.
* **Canlı Sensör Görselleştirmesi:** Simülasyon anında 36 ışınlı LiDAR'ın ortamı taramasını eşzamanlı gösteren Dashboard ve simülasyon sonu Nokta Bulutu (Point Cloud) haritalaması.

## Kurulum ve Kullanım

Projeyi kendi bilgisayarınızda çalıştırmak için Python 3.8 veya üzeri bir sürüm yüklü olmalıdır.

1. Repoyu klonlayın ve proje klasörüne gidin:
```bash
git clone https://github.com/yekonn/2d_mobile_robot_sim.git
cd 2d_mobile_robot_sim
```

2. Gerekli kütüphaneleri yükleyin:
```bash
pip install numpy matplotlib
```

3. Ana simülasyonu başlatın:
```bash
python main.py
```
## Proje Çıktıları ve Analiz

Simülasyon tamamlandığında sistem otomatik olarak şu analiz çıktılarını üretecektir:
1. **1x2 Canlı Dashboard:** Sol panelde robotun harita üzerindeki rotasını (Sensör Füzyonu), sağ panelde ise anlık LiDAR ışınlarını ve engellerle çarpışmalarını saniyesi saniyesine gösteren interaktif animasyon penceresi.
2. **Sensör Füzyonu Rota Takibi:** Gerçek konum ile EKF tahmininin 15x15 harita üzerinde karşılaştırılması.
3. **LiDAR Nokta Bulutu:** Robotun tüm rotası boyunca algıladığı engellerin birikmiş silüet haritası.
4. **Zaman Serisi ve RMSE Analizi:** $x(t)$, $y(t)$ ve $\theta(t)$ durumları için zaman eksenli sapma (drift) grafikleri.

## Dosya Yapısı ve Mimari
* `assets` : Projeye ait görsel çıktılar.
* `main.py` : Ana simülasyon döngüsü ve sistem yöneticisi.
* `ekf.py` : Sensör füzyonu için Genişletilmiş Kalman Filtresi (Extended Kalman Filter) sınıfı.
* `robot.py` : Non-holonomic (diferansiyel sürüş) kinematik robot modeli.
* `environment.py` : 15x15 ortam, engeller, başlangıç/hedef tanımları ve canlı animasyon (Dashboard) modülü.
* `sensors.py` : LiDAR, IMU ve Tekerlek Enkoderi için gürültü simülasyon modelleri.
* `visualizer.py` : Simülasyon sonu RMSE hata analizleri ve LiDAR nokta bulutu haritalandırma fonksiyonları.


---
*Geliştirici: Yasin Efe KUL*
