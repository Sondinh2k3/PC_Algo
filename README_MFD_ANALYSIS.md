# MFD Analysis Tool - Hướng dẫn sử dụng

## Mô tả
Công cụ phân tích MFD (Macroscopic Fundamental Diagram) được cải tiến để tạo ra biểu đồ mượt mà từ dữ liệu cảm biến e1 và e2 trong SUMO simulation.

## Tính năng chính

### 1. Thu thập dữ liệu thông minh
- **Cảm biến e2**: Thu thập số lượng xe (vehicle count) từ `algorithm_input_detectors`
- **Cảm biến e1**: Thu thập lưu lượng (flow) từ `mfd_input_flow_detectors`
- Tần suất thu thập: Mỗi 5 giây (thay vì 10 giây như trước)
- Thời gian mô phỏng: 10 phút (600 giây) để có nhiều điểm dữ liệu hơn

### 2. Xử lý dữ liệu mượt mà
- **Loại bỏ outliers**: Sử dụng phương pháp IQR (Interquartile Range)
- **Làm mượt dữ liệu**: Gaussian filter, moving average, spline interpolation
- **Binning**: Nhóm dữ liệu thành 25 bins để giảm nhiễu
- **Curve fitting**: Sử dụng spline interpolation để tạo đường cong mượt

### 3. Trực quan hóa nâng cao
- **Biểu đồ kép**: Hiển thị quá trình xử lý và kết quả cuối cùng
- **Confidence intervals**: Hiển thị khoảng tin cậy ±1σ và ±2σ
- **Thống kê**: Hiển thị các thông số quan trọng (max flow, optimal density)
- **Độ phân giải cao**: 300 DPI cho chất lượng in ấn

## Cài đặt

### 1. Cài đặt dependencies
```bash
pip install -r requirements_mfd.txt
```

### 2. Đảm bảo SUMO đã được cài đặt
```bash
# Kiểm tra SUMO_HOME environment variable
echo $SUMO_HOME

# Nếu chưa có, cài đặt SUMO và set environment variable
export SUMO_HOME=/path/to/sumo
```

## Sử dụng

### 1. Chạy phân tích MFD
```bash
cd tools
python mfd_graph.py
```

### 2. Kết quả
- **Biểu đồ**: `output/smooth_mfd_vehicles_vs_flow.png`
- **Dữ liệu đã xử lý**: `output/mfd_processed_data.csv`

## Cấu hình

### 1. Detector Configuration (`src/config/detector_config.json`)
```json
{
  "algorithm_input_detectors": {
    "detector_ids": ["e2_34", "e2_35", ...]  // Cảm biến e2 cho vehicle count
  },
  "mfd_input_flow_detectors": {
    "detector_ids": ["e1_33", "e1_25", ...]  // Cảm biến e1 cho flow
  }
}
```

### 2. Simulation Configuration (`src/config/simulation.yml`)
```yaml
type: "sumo"
config:
  config_file: "PhuQuoc/phuquoc.sumocfg"
  step_length: 1  # seconds
  gui: true
```

## Tùy chỉnh

### 1. Thay đổi thời gian mô phỏng
```python
# Trong mfd_graph.py, dòng 108
end_time = 600  # Thay đổi từ 600 (10 phút) thành giá trị mong muốn
```

### 2. Thay đổi tần suất thu thập dữ liệu
```python
# Trong mfd_graph.py, dòng 113
measurement_interval = 5  # Thay đổi từ 5 giây thành giá trị mong muốn
```

### 3. Thay đổi phương pháp làm mượt
```python
# Trong mfd_graph.py, dòng 175-176
vehicles_smooth = smooth_data(vehicles, method='gaussian', sigma=1.0)
flows_smooth = smooth_data(flows, method='gaussian', sigma=1.0)

# Các phương pháp có sẵn:
# - 'gaussian': Gaussian filter (mặc định)
# - 'moving_average': Moving average
# - 'spline': Spline interpolation
```

### 4. Thay đổi số lượng bins
```python
# Trong mfd_graph.py, dòng 179
binned_vehicles, binned_flows, binned_std = create_mfd_bins(vehicles_smooth, flows_smooth, n_bins=25)
```

## Giải thích kết quả

### 1. Biểu đồ MFD
- **Trục X**: Tổng số xe từ cảm biến e2
- **Trục Y**: Lưu lượng (vehicles/hour) từ cảm biến e1
- **Đường cong**: MFD curve được làm mượt
- **Vùng tô màu**: Khoảng tin cậy ±1σ

### 2. Thống kê quan trọng
- **Max Flow**: Lưu lượng tối đa có thể đạt được
- **Optimal Density**: Mật độ xe tối ưu (tương ứng với max flow)
- **Vehicle Range**: Phạm vi số lượng xe trong mạng
- **Flow Range**: Phạm vi lưu lượng đo được

### 3. Ý nghĩa của MFD
- **Vùng tự do (Free-flow)**: Lưu lượng tăng theo số lượng xe
- **Vùng tối ưu (Optimal)**: Đạt lưu lượng tối đa
- **Vùng tắc nghẽn (Congested)**: Lưu lượng giảm khi số xe tăng

## Xử lý lỗi

### 1. Lỗi SUMO_HOME
```
Please declare environment variable 'SUMO_HOME'
```
**Giải pháp**: Cài đặt SUMO và set environment variable

### 2. Lỗi detector không tìm thấy
```
Error: No detectors found in 'algorithm_input_detectors' in detector_config.json
```
**Giải pháp**: Kiểm tra file `detector_config.json` và đảm bảo detector IDs đúng

### 3. Lỗi import thư viện
```
ModuleNotFoundError: No module named 'scipy'
```
**Giải pháp**: Cài đặt dependencies: `pip install -r requirements_mfd.txt`

## Tối ưu hóa

### 1. Để có biểu đồ mượt hơn
- Tăng thời gian mô phỏng (`end_time`)
- Giảm khoảng thời gian thu thập (`measurement_interval`)
- Tăng số lượng bins (`n_bins`)
- Điều chỉnh sigma của Gaussian filter

### 2. Để có kết quả ổn định hơn
- Chạy nhiều lần mô phỏng và lấy trung bình
- Sử dụng seed cố định cho SUMO
- Tăng thời gian warm-up trước khi thu thập dữ liệu

## Ví dụ kết quả

Biểu đồ MFD sẽ hiển thị:
- Đường cong mượt mà thể hiện mối quan hệ giữa density và flow
- Khoảng tin cậy cho thấy độ ổn định của dữ liệu
- Điểm tối ưu (optimal density) cho traffic management
- Thông tin thống kê chi tiết về hiệu suất mạng giao thông
