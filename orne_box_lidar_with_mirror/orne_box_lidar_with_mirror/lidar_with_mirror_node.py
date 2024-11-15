import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np
from ament_index_python.packages import get_package_share_directory

# モデルのクラス定義
class LidarAutoencoder(nn.Module):
    def __init__(self, input_dim):
        super(LidarAutoencoder, self).__init__()
        # ネットワークサイズを増やして表現力を強化
        self.encoder = nn.Sequential(
            nn.Linear(input_dim, 128),
            nn.ReLU(),
            nn.Linear(128, 64),
            nn.ReLU(),
            nn.Linear(64, 32),
            nn.ReLU(),
            nn.Linear(32, 16),
            nn.ReLU()
        )
        self.decoder = nn.Sequential(
            nn.Linear(16, 32),
            nn.ReLU(),
            nn.Linear(32, 64),
            nn.ReLU(),
            nn.Linear(64, 128),
            nn.ReLU(),
            nn.Linear(128, input_dim)
        )

    def forward(self, x):
        x = self.encoder(x)
        x = self.decoder(x)
        return x

# モデルのトレーニング関数
def train_autoencoder(input_dim, model_path):
    model = LidarAutoencoder(input_dim=input_dim)
    criterion = nn.MSELoss()
    optimizer = torch.optim.Adam(model.parameters(), lr=0.001)

    # ダミーのトレーニングデータ
    normal_data = torch.rand(1000, input_dim)  # 例としてランダムデータを使用
    epochs = 50

    for epoch in range(epochs):
        optimizer.zero_grad()
        output = model(normal_data)
        loss = criterion(output, normal_data)
        loss.backward()
        optimizer.step()
        print(f'Epoch [{epoch+1}/{epochs}], Loss: {loss.item():.4f}')

    # モデルの保存
    os.makedirs(os.path.dirname(model_path), exist_ok=True)
    torch.save(model.state_dict(), model_path)
    print(f'Model saved to {model_path}')

# ROS2ノードの定義
class AnomalyDetectionNode(Node):
    def __init__(self):
        super().__init__('anomaly_detection_node')
        self.subscription = self.create_subscription(
            LaserScan,
            '/mirror_scan',
            self.lidar_callback,
            10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        input_dim = 57  # LiDARデータのビーム数に合わせる

        # モデルの保存・ロードパスを設定
        package_dir = get_package_share_directory('orne_box_lidar_with_mirror')
        model_path = os.path.join(package_dir, 'models/cit3f/lidar_autoencoder.pth')

        # モデル保存・読み込みのフラグ
        SAVE_MODEL = True  # Trueにするとモデルを保存
        LOAD_MODEL = True   # Trueにするとモデルを読み込む

        # モデル初期化
        self.model = LidarAutoencoder(input_dim=input_dim)
        
        # モデルの保存
        if SAVE_MODEL:
            print("Training and saving the model...")
            train_autoencoder(input_dim=input_dim, model_path=model_path)  # モデルを訓練し、保存します

        # モデルの読み込み
        if LOAD_MODEL:
            if os.path.exists(model_path):
                self.model.load_state_dict(torch.load(model_path, map_location=torch.device('cpu'), weights_only=True))
                self.model.eval()
                print(f"Model loaded from {model_path}")
            else:
                print(f"Model file not found at {model_path}. Please train and save the model first.")
        
        # 異常判定のしきい値
        self.threshold = 0.08  # 適切に設定（再構築誤差を基に調整可能）

    def lidar_callback(self, msg):
        # LiDARデータをPyTorchテンソルに変換
        scan_data = np.array(msg.ranges, dtype=np.float32)

        # NaNやInfが含まれている場合、異常として処理をスキップ
        if np.isnan(scan_data).any() or np.isinf(scan_data).any():
            self.get_logger().warn("Invalid LiDAR data detected: contains NaN or Inf. Skipping this data.")
            return  # 処理をスキップして次のデータへ

        scan_tensor = torch.tensor(scan_data)

        # オートエンコーダで再構築
        with torch.no_grad():
            reconstructed = self.model(scan_tensor)
            loss = F.mse_loss(reconstructed, scan_tensor).item()

        # 再構築誤差の確認
        self.get_logger().info(f'Reconstruction Loss: {loss}')

        # 異常かどうかを判定
        if loss < self.threshold:
            self.get_logger().info('Anomaly detected! Stopping the robot.')
            self.stop_robot()
        else:
            self.get_logger().info('No anomaly detected.')

    def stop_robot(self):
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.publisher.publish(stop_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AnomalyDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
