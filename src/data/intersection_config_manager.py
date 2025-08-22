"""
Intersection Config Manager - Quản lý cấu hình nút giao từ file JSON
Đọc và cung cấp dữ liệu cho bài toán tối ưu hóa.
MODIFIED: Hỗ trợ cấu trúc pha linh hoạt (1 pha chính, nhiều pha phụ).
"""

import json
import os
from typing import Dict, List, Optional, Any

class IntersectionConfigManager:
    """
    Quản lý cấu hình intersection từ file JSON với cấu trúc pha linh hoạt.
    """
    
    def __init__(self, config_file: str = "src/intersection_config.json"):
        """
        Khởi tạo config manager.
        
        Args:
            config_file: Đường dẫn đến file cấu hình JSON.
        """
        self.config_file = config_file
        self.config_data = {}
        self.load_config()
    
    def load_config(self) -> bool:
        """
        Load cấu hình từ file JSON.
        """
        try:
            if not os.path.exists(self.config_file):
                print(f"Lỗi: Không tìm thấy file cấu hình tại '{self.config_file}'")
                return False
            
            with open(self.config_file, 'r', encoding='utf-8') as f:
                self.config_data = json.load(f)
            
            print(f"Đã load cấu hình từ: {self.config_file}")
            return True
            
        except Exception as e:
            print(f"Lỗi khi load cấu hình: {e}")
            return False

    def save_config(self, output_file: Optional[str] = None):
        """
        Lưu cấu hình vào file JSON.
        """
        if output_file is None:
            output_file = self.config_file
        
        try:
            with open(output_file, 'w', encoding='utf-8') as f:
                json.dump(self.config_data, f, indent=2, ensure_ascii=False)
            print(f"✅ Đã lưu cấu hình vào: {output_file}")
        except Exception as e:
            print(f"❌ Lỗi khi lưu cấu hình: {e}")
    
    def get_intersection_ids(self) -> List[str]:
        """
        Lấy danh sách ID của các intersection được định nghĩa trong 'optimization_parameters'.
        """
        return self.config_data.get('optimization_parameters', {}).get('intersection_ids', [])
    
    def get_global_params(self) -> Dict[str, Any]:
        """
        Lấy các tham số toàn cục cho bài toán tối ưu hóa.
        """
        params = self.config_data.get('optimization_parameters', {})
        return {
            'theta_1': params.get('theta_1', 1.0),
            'theta_2': params.get('theta_2', 0.5),
            'default_cycle_length': params.get('default_cycle_length', 90),
            'min_green_time': params.get('min_green_time', 15),
            'max_green_time': params.get('max_green_time', 75),
            'max_change': params.get('max_change', 5)
        }

    def get_intersection_data(self, intersection_id: str) -> Optional[Dict]:
        """
        Lấy toàn bộ dữ liệu của một intersection cụ thể.
        """
        return self.config_data.get('optimization_parameters', {}).get('intersection_data', {}).get(intersection_id)

    def get_cycle_length(self, intersection_id: str) -> int:
        """
        Lấy chu kỳ đèn của intersection.
        """
        intersection_data = self.get_intersection_data(intersection_id)
        if intersection_data:
            return intersection_data.get('cycle_length', 90)
        return 90

    def get_traffic_light_id(self, intersection_id: str) -> Optional[str]:
        """
        Lấy ID đèn giao thông của một intersection.
        """
        intersection_data = self.config_data.get('intersections', {}).get(intersection_id)
        if intersection_data:
            return intersection_data.get('traffic_light_id')
        return None

    def get_phase_info(self, intersection_id: str) -> Optional[Dict]:
        """
        Lấy thông tin về các pha (chính và phụ) của một intersection.
        
        Returns:
            Dict: Một dict chứa thông tin về pha chính ('p') và danh sách các pha phụ ('s').
                  Ví dụ: {'p': {...}, 's': [{...}, {...}]}
        """
        intersection_data = self.get_intersection_data(intersection_id)
        if intersection_data:
            return intersection_data.get('phases')
        return None
