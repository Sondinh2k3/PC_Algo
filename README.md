# Hệ thống Điều khiển Giao thông Thích ứng dựa trên Chu vi (Perimeter Control)

## 1. Tổng quan

Dự án này triển khai một hệ thống điều khiển đèn giao thông thích ứng trong khu vực đô thị, sử dụng thuật toán **Perimeter Control**. Mục tiêu chính là tối ưu hóa luồng giao thông, giảm thiểu tắc nghẽn và thời gian di chuyển bằng cách điều chỉnh thời gian xanh của các đèn tín hiệu tại biên của một khu vực được bảo vệ.

Hệ thống sử dụng **SUMO (Simulation of Urban MObility)** để mô phỏng môi trường giao thông thực tế và một bộ điều khiển trung tâm được viết bằng **Python** để thực thi thuật toán.

### Công nghệ sử dụng
- **Mô phỏng:** SUMO
- **Ngôn ngữ chính:** Python
- **Thư viện Python:** Traci (SUMO API), Pandas, Matplotlib, YAML
- **Database (Tùy chọn):** Neo4j, SQL (thông qua các collector)
- **Containerization:** Docker (qua `docker-compose.yml`)

## 2. Tính năng chính

- **Thuật toán Perimeter Control:** Tự động điều chỉnh chu kỳ đèn tín hiệu dựa trên số lượng xe bên trong khu vực (accumulation) và hàng đợi (queue) tại các lối vào.
- **Tích hợp SUMO:** Kết nối trực tiếp với SUMO thông qua Traci để lấy dữ liệu thời gian thực và gửi lệnh điều khiển.
- **Cấu hình linh hoạt:** Dễ dàng tùy chỉnh các tham số mô phỏng, cấu hình mạng lưới, và thuật toán thông qua các file YAML và JSON.
- **Phân tích và Trực quan hóa:** Cung cấp các công cụ để vẽ biểu đồ MFD (Macroscopic Fundamental Diagram), so sánh kết quả và phân tích hiệu quả của thuật toán.
- **Thu thập dữ liệu:** Có khả năng lưu trữ dữ liệu mô phỏng vào database để phân tích sâu hơn.

## 3. Cấu trúc thư mục

```
PC_Algorithms/
├── docker-compose.yml            # Cấu hình Docker cho các dịch vụ phụ trợ (ví dụ: database)
├── requirements.txt              # Danh sách các thư viện Python cần thiết
├── routes.rou.xml                # File luồng giao thông ví dụ
├── output/                       # Thư mục chứa kết quả đầu ra (log, biểu đồ, dữ liệu)
├── scripts/                      # Chứa các script tự động hóa (nếu có)
├── src/                          # Toàn bộ mã nguồn của dự án
│   ├── algorithm/                # Lõi của thuật toán điều khiển
│   │   ├── algo.py               # Logic chính của thuật toán Perimeter Control (PI controller)
│   │   ├── solver.py             # Bộ giải tối ưu hóa thời gian xanh cho các nút giao
│   │   └── common.py             # Các hàm và lớp dùng chung cho thuật toán
│   ├── config/                   # Các file cấu hình cho hệ thống
│   │   ├── application.yml       # Cấu hình chung của ứng dụng (ví dụ: kết nối DB)
│   │   ├── simulation.yml        # Cấu hình kịch bản mô phỏng SUMO
│   │   ├── detector_config.json  # Định nghĩa các cảm biến (detectors) trong SUMO
│   │   └── intersection_config.json # Cấu hình chi tiết cho từng nút giao
│   ├── data/                     # Các module liên quan đến xử lý và quản lý dữ liệu
│   │   ├── collector/            # Các lớp thu thập dữ liệu (ví dụ: SQLCollector)
│   │   ├── sql/                  # Các file script SQL
│   │   └── neo4j.cypher          # Các truy vấn cho cơ sở dữ liệu đồ thị Neo4j
│   ├── PhuQuoc/                  # Thư mục ví dụ chứa một kịch bản mô phỏng SUMO hoàn chỉnh
│   ├── main.py                   # Điểm khởi chạy chính của chương trình
│   └── sumosim.py                # Lớp quản lý và tương tác với mô phỏng SUMO
└── tools/                        # Các công cụ hỗ trợ phát triển và phân tích
    ├── generate_intersection_config.py # Script tạo file intersection_config.json từ file .net.xml
    ├── mfd_graph.py              # Script vẽ biểu đồ MFD từ dữ liệu đầu ra
    └── visual_comparator.py      # Script so sánh trực quan các kịch bản khác nhau
```

## 4. Hướng dẫn Cài đặt và Chuẩn bị

### a. Yêu cầu
- **Python 3.8+**
- **SUMO:** Cài đặt SUMO và đảm bảo biến môi trường `SUMO_HOME` đã được thiết lập.
- **Docker (Tùy chọn):** Nếu bạn muốn sử dụng các dịch vụ như database.

### b. Các bước cài đặt
1.  **Clone repository:**
    ```bash
    git clone <your-repository-url>
    cd PC_Algorithms
    ```
2.  **Cài đặt thư viện Python:**
    ```bash
    pip install -r requirements.txt
    ```
3.  **Cấu hình SUMO:**
    - Tải và cài đặt SUMO từ [trang chủ Eclipse SUMO](https://www.eclipse.org/sumo/).
    - Thêm đường dẫn đến thư mục `tools` của SUMO vào biến môi trường `PATH` của hệ thống.
    - Thiết lập biến môi trường `SUMO_HOME` trỏ đến thư mục cài đặt gốc của SUMO.

## 5. Hướng dẫn Cấu hình

Để áp dụng thuật toán cho một mạng lưới giao thông mới, bạn cần cấu hình các file sau trong `src/config/`:

### a. `simulation.yml`
Trỏ đến các file mô phỏng SUMO của bạn.
```yaml
type: "sumo"
config:
  config_file: "PhuQuoc/phuquoc.sumocfg" # Đường dẫn tới file .sumocfg
  net_file: "PhuQuoc/phuquoc.net.xml"
  route_file: "PhuQuoc/phuquoc.rou.xml"
  gui: true # true để hiển thị giao diện đồ họa SUMO, false để chạy nền
```

### b. `detector_config.json`
Định nghĩa ID của các cảm biến (detectors) mà thuật toán sẽ sử dụng để thu thập dữ liệu từ SUMO.
- **`algorithm_input_detectors`**: Các cảm biến `e2` dùng để đo tổng số xe (accumulation) trong khu vực.
- **`solver_input_detectors`**: Các cảm biến `e1` (lane area detectors) dùng để đo chiều dài hàng đợi tại các nút giao.

### c. `intersection_config.json`
File cấu hình quan trọng nhất, định nghĩa các tham số cho từng nút giao và cho bộ giải tối ưu.
- **`intersections`**: Danh sách các nút giao, ID của chúng và ID đèn tín hiệu tương ứng trong SUMO.
- **`optimization_parameters`**:
    - `intersection_ids`: Các nút giao sẽ được tối ưu.
    - `intersection_data`: Các thông số kỹ thuật cho từng nút giao như `cycle_length`, `main_phases`, `saturation_flows`, v.v.

### d. `src/algorithm/algo.py`
Tinh chỉnh các hằng số của bộ điều khiển PI và mật độ xe mục tiêu.
```python
KP_H = 20.0        # Hệ số tỉ lệ (Proportional gain)
KI_H = 5.0         # Hệ số tích phân (Integral gain)
N_HAT = 150.0      # Mật độ xe mục tiêu (xe)
CONTROL_INTERVAL_S = 90 # Tần suất điều khiển (giây)
```

## 6. Cách sử dụng

### a. Chạy mô phỏng
Để bắt đầu, chạy file `main.py` từ thư mục `src`.
```bash
cd src
python main.py
```
Giao diện đồ họa của SUMO sẽ khởi chạy và bộ điều khiển sẽ bắt đầu hoạt động.

### b. Chạy với dữ liệu giả lập (Mock Mode)
Để kiểm tra logic thuật toán mà không cần chạy SUMO, sử dụng tham số `mock`.
```bash
cd src
python main.py mock
```

### c. Sử dụng các công cụ
Các script trong thư mục `tools/` giúp tự động hóa và phân tích:
- **Tạo config nút giao:**
  ```bash
  python tools/generate_intersection_config.py --net-file src/PhuQuoc/phuquoc.net.xml --output src/intersection_config.json
  ```
- **Vẽ biểu đồ MFD:**
  ```bash
  python tools/mfd_graph.py --file output/edgedata.xml --output output/mfd_graph.png
  ```

## 7. Phân tích Kết quả

Kết quả mô phỏng và phân tích được lưu trong thư mục `output/`, bao gồm:
- `edgedata.xml`: Dữ liệu chi tiết từ các cảm biến.
- `tripinfo.xml`: Thông tin về hành trình của các xe.
- `mfd_graph.png`: Biểu đồ MFD thể hiện mối quan hệ giữa mật độ và lưu lượng.
- `*.csv`: Các file dữ liệu đã được xử lý.
