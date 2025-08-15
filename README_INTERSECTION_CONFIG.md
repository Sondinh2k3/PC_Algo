# 🚦 Hệ thống Cấu hình Intersection từ SUMO

Hệ thống này cho phép tự động đọc thông tin nút giao từ SUMO network và lưu trữ trong file JSON, thay thế cho việc hardcode dữ liệu trong code.

## 🎯 Tính năng chính

### ✅ **Tự động phát hiện**
- Đọc traffic lights từ file `.net.xml`
- Phân tích pha đèn chính/phụ
- Ước tính saturation flow và capacity
- Phát hiện controlled lanes

### ✅ **Linh hoạt**
- Cấu hình từ file JSON thay vì hardcode
- Dễ dàng thay đổi tham số
- Hỗ trợ nhiều mạng lưới khác nhau
- Validation tự động

### ✅ **Tích hợp**
- Tự động tích hợp với PerimeterController
- Hỗ trợ bài toán tối ưu hóa MIQP
- Backup và restore cấu hình

## 📁 Cấu trúc file

```
src/
├── data/
│   ├── intersection_analyzer.py      # Phân tích SUMO network
│   ├── intersection_config_manager.py # Quản lý cấu hình JSON
│   └── scripts/
│       └── generate_intersection_config.py # Script tạo cấu hình
├── algorithm/
│   └── algo.py                       # PerimeterController (đã cập nhật)
└── intersection_config.json          # File cấu hình (tự tạo)
```

## 🚀 Cách sử dụng

### **1. Tạo cấu hình từ SUMO network**

```bash
# Tạo cấu hình mặc định
python src/scripts/generate_intersection_config.py

# Tạo cấu hình từ network cụ thể
python src/scripts/generate_intersection_config.py PhuQuoc/phuquoc.net.xml

# Chỉ phân tích, không tạo cấu hình
python src/scripts/generate_intersection_config.py --analyze-only

# Validate cấu hình sau khi tạo
python src/scripts/generate_intersection_config.py --validate
```

### **2. Sử dụng trong code**

```python
from data.intersection_config_manager import IntersectionConfigManager
from algorithm.algo import PerimeterController

# Load cấu hình
config_manager = IntersectionConfigManager("intersection_config.json")

# Khởi tạo controller với cấu hình
controller = PerimeterController(
    kp=20.0, 
    ki=5.0, 
    n_hat=150.0,
    config_file="intersection_config.json"
)
```

## 📊 Cấu trúc file JSON

```json
{
  "metadata": {
    "network_file": "PhuQuoc/phuquoc.net.xml",
    "generated_at": "2024-01-15 10:30:00",
    "total_intersections": 3,
    "total_traffic_lights": 3
  },
  "traffic_lights": {
    "1166230678": {
      "type": "static",
      "phases": [
        {"duration": 45, "state": "GGGG"},
        {"duration": 45, "state": "rrrr"}
      ],
      "total_cycle": 90
    }
  },
  "intersections": {
    "1166230678": {
      "id": "1166230678",
      "type": "traffic_light",
      "x": 1234.56,
      "y": 789.01
    }
  },
  "optimization_parameters": {
    "intersection_ids": ["1166230678", "1677153107", "357410392"],
    "theta_1": 1.0,
    "theta_2": 0.5,
    "default_cycle_length": 90,
    "min_green_time": 15,
    "max_green_time": 75,
    "max_change": 5,
    "intersection_data": {
      "1166230678": {
        "cycle_length": 90,
        "main_phases": [0],
        "secondary_phases": [1],
        "saturation_flows": {
          "main": 0.45,
          "secondary": 0.35
        },
        "turn_in_ratios": {
          "main": 0.7,
          "secondary": 0.5
        },
        "queue_lengths": {
          "main": 15,
          "secondary": 8
        }
      }
    }
  }
}
```

## 🔧 API của IntersectionConfigManager

### **Khởi tạo**
```python
config_manager = IntersectionConfigManager("intersection_config.json")
```

### **Lấy thông tin cơ bản**
```python
# Danh sách intersection IDs
intersection_ids = config_manager.get_intersection_ids()

# Tham số toàn cục
global_params = config_manager.get_global_params()

# Dữ liệu intersection cụ thể
intersection_data = config_manager.get_intersection_data("1166230678")
```

### **Lấy tham số tối ưu hóa**
```python
# Saturation flows
saturation_flows = config_manager.get_saturation_flows("1166230678")
# Returns: {'main': 0.45, 'secondary': 0.35}

# Turn-in ratios
turn_in_ratios = config_manager.get_turn_in_ratios("1166230678")
# Returns: {'main': 0.7, 'secondary': 0.5}

# Queue lengths
queue_lengths = config_manager.get_queue_lengths("1166230678")
# Returns: {'main': 15, 'secondary': 8}

# Cycle length
cycle_length = config_manager.get_cycle_length("1166230678")
# Returns: 90

# Phase info
phase_info = config_manager.get_phase_info("1166230678")
# Returns: {'main_phases': [0], 'secondary_phases': [1]}
```

### **Cập nhật dữ liệu**
```python
new_data = {
    'cycle_length': 100,
    'saturation_flows': {'main': 0.5, 'secondary': 0.4},
    'turn_in_ratios': {'main': 0.8, 'secondary': 0.6},
    'queue_lengths': {'main': 20, 'secondary': 10}
}
config_manager.update_intersection_data("1166230678", new_data)
config_manager.save_config()
```

### **Validation**
```python
if config_manager.validate_config():
    print("✅ Cấu hình hợp lệ")
else:
    print("❌ Cấu hình không hợp lệ")
```

## 🔍 API của IntersectionAnalyzer

### **Phân tích network**
```python
analyzer = IntersectionAnalyzer("PhuQuoc/phuquoc.net.xml")

# Phân tích từ file network
network_data = analyzer.analyze_network()

# Phân tích từ simulation (cần TraCI)
simulation_data = analyzer.analyze_from_simulation()

# Tạo file cấu hình
config_data = analyzer.generate_intersection_config("output.json")
```

## 🎯 Tích hợp với PerimeterController

### **Trước (hardcode)**
```python
# Cũ - hardcode dữ liệu
self.intersection_data = {
    'M': [1, 2, 3],
    'theta_1': 1.0,
    'theta_2': 0.5,
    'C_o': 90,
    'S_prime_p': {1: 0.45, 2: 0.40, 3: 0.50},
    # ...
}
```

### **Sau (từ JSON)**
```python
# Mới - đọc từ JSON
self.config_manager = IntersectionConfigManager(config_file)
self.intersection_ids = self.config_manager.get_intersection_ids()
self.global_params = self.config_manager.get_global_params()

# Lấy dữ liệu động
saturation_flows = self.config_manager.get_saturation_flows(intersection_id)
turn_in_ratios = self.config_manager.get_turn_in_ratios(intersection_id)
```

## 📈 Lợi ích

### **1. Linh hoạt**
- Dễ dàng thay đổi tham số mà không cần sửa code
- Hỗ trợ nhiều mạng lưới khác nhau
- Cấu hình có thể được chia sẻ và version control

### **2. Tự động hóa**
- Tự động phát hiện intersection từ SUMO
- Ước tính tham số dựa trên network
- Validation tự động

### **3. Bảo trì**
- Tách biệt dữ liệu và logic
- Dễ dàng debug và test
- Backup/restore cấu hình

### **4. Mở rộng**
- Dễ dàng thêm intersection mới
- Hỗ trợ các loại pha phức tạp
- Tích hợp với các thuật toán khác

## 🐛 Xử lý sự cố

### **File cấu hình không tồn tại**
```bash
# Tạo cấu hình mặc định
python src/scripts/generate_intersection_config.py
```

### **Network file không tìm thấy**
```bash
# Kiểm tra đường dẫn
ls PhuQuoc/phuquoc.net.xml

# Sử dụng đường dẫn tuyệt đối
python src/scripts/generate_intersection_config.py /path/to/network.net.xml
```

### **Cấu hình không hợp lệ**
```bash
# Validate cấu hình
python src/scripts/generate_intersection_config.py --validate

# Tạo lại cấu hình
python src/scripts/generate_intersection_config.py --force
```

## 🔄 Workflow đề xuất

1. **Phân tích network**: `python src/scripts/generate_intersection_config.py --analyze-only`
2. **Tạo cấu hình**: `python src/scripts/generate_intersection_config.py --validate`
3. **Chỉnh sửa tham số**: Sửa file JSON theo nhu cầu
4. **Test**: Chạy simulation với cấu hình mới
5. **Tối ưu**: Điều chỉnh tham số dựa trên kết quả

## 📝 Ghi chú

- File cấu hình được tạo tự động từ SUMO network
- Có thể chỉnh sửa thủ công các tham số
- Backup cấu hình trước khi thay đổi lớn
- Validate cấu hình sau khi chỉnh sửa
