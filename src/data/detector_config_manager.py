"""
Detector Config Manager - Quản lý cấu hình detector từ file JSON
"""

import json
import os
from typing import Dict, List, Optional

class DetectorConfigManager:
    """
    Quản lý cấu hình detector từ file JSON.
    """
    
    def __init__(self, config_file: str):
        """
        Khởi tạo config manager.
        
        Args:
            config_file: Đường dẫn đến file cấu hình JSON.
        """
        if not os.path.exists(config_file):
            raise FileNotFoundError(f"Detector config file not found at: {config_file}")
            
        self.config_file = config_file
        self.config_data = {}
        self.load_config()
    
    def load_config(self):
        """
        Load cấu hình từ file JSON.
        """
        try:
            with open(self.config_file, 'r', encoding='utf-8') as f:
                self.config_data = json.load(f)
            print(f"[INFO] Đã load cấu hình detector từ: {self.config_file}")
        except Exception as e:
            print(f"[ERROR] Lỗi khi load cấu hình detector: {e}")
            self.config_data = {}

    def get_algorithm_input_detectors(self) -> List[str]:
        """
        Lấy danh sách ID của các detector dùng cho đầu vào của thuật toán chính.
        
        Returns:
            List[str]: Danh sách detector IDs.
        """
        return self.config_data.get('algorithm_input_detectors', {}).get('detector_ids', [])

    def get_solver_input_detectors(self) -> Dict:
        """
        Lấy danh sách các detector dùng cho đầu vào của bộ giải (solver).
        
        Returns:
            Dict: Cấu trúc các detector cho từng intersection.
        """
        return self.config_data.get('solver_input_detectors', {}).get('intersections', {})
