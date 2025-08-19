# Hệ thống Điều khiển Giao thông Thích ứng dựa trên Chu vi

Dự án này triển khai một hệ thống điều khiển đèn giao thông thích ứng sử dụng thuật toán Perimeter Control. Hệ thống sử dụng SUMO (Simulation of Urban MObility) để mô phỏng và bộ điều khiển được viết bằng Python để tối ưu hóa luồng giao thông trong một khu vực được xác định.

## Cấu trúc thư mục

```
PC_Algorithms/
├── src/
│   ├── algorithm/
│   │   └── algo.py             # Logic chính của thuật toán Perimeter Control
│   ├── config/
│   │   ├── application.yml     # Cấu hình kết nối database (nếu có)
│   │   ├── detector_config.json # Cấu hình các cảm biến (detector)
│   │   └── simulation.yml      # Cấu hình cho mô phỏng SUMO
│   ├── data/
│   │   └── ...                 # Các script quản lý dữ liệu
│   ├── PhuQuoc/                # Thư mục chứa các file mô phỏng SUMO
│   ├── intersection_config.json # File cấu hình chính cho các nút giao
│   └── main.py                 # File thực thi chính của chương trình
├── tools/
│   └── ...                     # Các công cụ hỗ trợ
└── README.md                   # File hướng dẫn này
```

## Hướng dẫn áp dụng cho một khu vực/nút giao mới

Để áp dụng thuật toán cho một mạng lưới giao thông khác, bạn cần thực hiện một chuỗi các bước cấu hình, từ mô hình SUMO đến các file JSON của thuật toán.

### Bước 1: Chuẩn bị các file mô phỏng SUMO

Đây là bước nền tảng. Bạn cần có một mô hình giao thông hoàn chỉnh trong SUMO cho khu vực bạn muốn điều khiển.

1.  **File Mạng lưới (`.net.xml`):** Tạo hoặc chỉnh sửa file mạng lưới đường, bao gồm các nút (junctions), các tuyến (edges), và đặc biệt là các đèn giao thông (`traffic_light`).
2.  **File Cảm biến (`.add.xml`):** Định nghĩa các cảm biến (detectors) tại các vị trí chiến lược trong mạng lưới. Các cảm biến này là nguồn dữ liệu đầu vào cho thuật toán. Bạn sẽ cần hai loại cảm biến chính:
    *   **Cảm biến đo lường tích lũy (Accumulation):** Đặt tại các làn đường bên trong khu vực điều khiển để đo tổng số xe (`e2 detector`).
    *   **Cảm biến đo hàng đợi (Queue):** Đặt ở các làn đường dẫn vào các nút giao chính để đo chiều dài hàng đợi.
3.  **File Luồng giao thông (`.rou.xml`):** Định nghĩa luồng di chuyển của các phương tiện.
4.  **File Cấu hình SUMO (`.sumocfg`):** Kết hợp tất cả các file trên (`.net.xml`, `.rou.xml`, `.add.xml`) thành một kịch bản mô phỏng hoàn chỉnh.

### Bước 2: Cập nhật Cấu hình Mô phỏng

Mở file `src/config/simulation.yml` và trỏ đường dẫn đến file `.sumocfg` mới của bạn.

**Ví dụ:**
```yaml
type: "sumo"
config:
  # Thay đổi đường dẫn này tới file cấu hình SUMO của bạn
  config_file: "DuongMoi/khuvucmoi.sumocfg"
  net_file: "DuongMoi/khuvucmoi.net.xml"
  route_file: "DuongMoi/khuvucmoi.rou.xml"
  # ... các thông số khác
  gui: true
```

### Bước 3: Cấu hình Cảm biến (Detectors)

Đây là bước quan trọng để kết nối dữ liệu từ SUMO với thuật toán. Mở file `src/config/detector_config.json`.

1.  **`algorithm_input_detectors`**: Trong phần `detector_ids`, liệt kê ID của tất cả các cảm biến (`e2 detector`) dùng để đo tổng số xe trong khu vực điều khiển.
    ```json
    "algorithm_input_detectors": {
      "description": "Các cảm biến dùng để tính toán tổng số xe trong khu vực.",
      "detector_ids": [
        "cam_bien_duong_A",
        "cam_bien_duong_B",
        "..."
      ]
    }
    ```
2.  **`solver_input_detectors`**: Trong phần `intersections`, định nghĩa các cảm biến đo hàng đợi cho từng nút giao sẽ được tối ưu.
    *   Mỗi key là một `junction_id` (bạn sẽ định nghĩa ở bước 4).
    *   `main_queue_detector`: ID của cảm biến đo hàng đợi cho luồng chính.
    *   `secondary_queue_detector`: Danh sách ID các cảm biến đo hàng đợi cho luồng phụ.

    ```json
    "solver_input_detectors": {
      "intersections": {
        "nut_giao_A": {
          "main_queue_detector": "cam_bien_hang_doi_A_chinh",
          "secondary_queue_detector": ["cam_bien_hang_doi_A_phu_1", "cam_bien_hang_doi_A_phu_2"]
        },
        "nut_giao_B": {
          "main_queue_detector": "cam_bien_hang_doi_B_chinh",
          "secondary_queue_detector": ["cam_bien_hang_doi_B_phu"]
        }
      }
    }
    ```

### Bước 4: Cấu hình các Nút giao và Thuật toán

Đây là file cấu hình trung tâm, `src/intersection_config.json`.

1.  **`intersections`**: Liệt kê các nút giao của bạn. `id` ở đây phải khớp với `junction_id` bạn đã dùng trong `detector_config.json`. `traffic_light_id` là ID của đèn giao thông tương ứng trong file `.net.xml` của SUMO.
    ```json
    "intersections": {
      "nut_giao_A": {
        "id": "nut_giao_A",
        "traffic_light_id": "TL1",
        "type": "traffic_light",
        "x": 0.0, "y": 0.0
      },
      "nut_giao_B": {
        "id": "nut_giao_B",
        "traffic_light_id": "TL2",
        "type": "traffic_light",
        "x": 0.0, "y": 0.0
      }
    }
    ```

2.  **`optimization_parameters`**:
    *   **`intersection_ids`**: Liệt kê lại tất cả các `id` của nút giao sẽ tham gia vào quá trình tối ưu.
    *   **`intersection_data`**: Cung cấp các tham số chi tiết cho từng nút giao.
        *   `cycle_length`: Tổng thời gian một chu kỳ đèn (giây).
        *   `main_phases`: Index (thứ tự) của các pha đèn thuộc luồng chính.
        *   `secondary_phases`: Index của các pha đèn thuộc luồng phụ.
        *   `saturation_flows`: Lưu lượng bão hòa (xe/giây) cho luồng chính và phụ.
        *   `turn_in_ratios`: Tỷ lệ các dòng xe rẽ vào khu vực từ các luồng.
        *   `queue_lengths`: Chiều dài hàng đợi mặc định/khởi tạo.

    ```json
    "intersection_data": {
      "nut_giao_A": {
        "cycle_length": 90,
        "main_phases": [0],
        "secondary_phases": [2],
        "saturation_flows": {"main": 0.5, "secondary": 0.4},
        "turn_in_ratios": {"main": 0.8, "secondary": 0.5},
        "queue_lengths": {"main": 15, "secondary": 10}
      },
      "..."
    }
    ```

### Bước 5: Điều chỉnh Tham số Thuật toán

Các tham số cốt lõi của bộ điều khiển PI và mục tiêu của hệ thống được định nghĩa ở đầu file `src/algorithm/algo.py`.

```python
# src/algorithm/algo.py

# === CONSTANTS ===
KP_H = 20.0        # Hệ số khuếch đại tỉ lệ (Proportional gain)
KI_H = 5.0         # Hệ số khuếch đại tích phân (Integral gain)
N_HAT = 150.0      # Mật độ xe mục tiêu trong khu vực (xe)
CONTROL_INTERVAL_S = 90  # Khoảng thời gian giữa 2 lần điều khiển (giây)
```

Bạn có thể cần tinh chỉnh các giá trị `KP_H`, `KI_H`, và đặc biệt là `N_HAT` (số lượng xe mục tiêu) để phù hợp với đặc điểm của khu vực mới.

### Bước 6: Chạy Chương trình

Sau khi đã hoàn tất các bước cấu hình, bạn có thể chạy mô phỏng từ thư mục `src`:

```bash
python main.py
```

Nếu bạn muốn chạy thử nghiệm với dữ liệu giả lập (không cần SUMO) để kiểm tra logic, sử dụng lệnh:

```bash
python main.py mock
```

Bằng cách tuân theo các bước trên, bạn có thể cấu hình và áp dụng hệ thống điều khiển này cho bất kỳ mạng lưới giao thông nào được mô hình hóa trong SUMO.

# Lệnh tạo route (cần có 2 file: phuquoc.rou.xml và phuquoc.conf.xml)

python 'C:\Program Files (x86)\Eclipse\Sumo\tools\randomTrips.py' -c .\phuquoc.conf.xml
=> Tạo ra file phuquoc-demo.rou.xml