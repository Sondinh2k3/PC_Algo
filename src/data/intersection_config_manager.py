"""
Intersection Config Manager - Quản lý cấu hình nút giao từ file JSON
Đọc và cung cấp dữ liệu cho bài toán tối ưu hóa
"""

import json
import os
from typing import Dict, List, Optional, Any
import logging

class IntersectionConfigManager:
    """
    Quản lý cấu hình intersection từ file JSON
    """
    
    def __init__(self, config_file: str = "intersection_config.json"):
        """
        Khởi tạo config manager
        
        Args:
            config_file: Đường dẫn đến file cấu hình JSON
        """
        self.config_file = config_file
        self.config_data = {}
        self.load_config()
    
    def load_config(self) -> bool:
        """
        Load cấu hình từ file JSON
        
        Returns:
            bool: True nếu load thành công
        """
        try:
            if not os.path.exists(self.config_file):
                print(f" Không tìm thấy file cấu hình: {self.config_file}")
                print(" Tạo cấu hình mặc định...")
                self._create_default_config()
                return True
            
            with open(self.config_file, 'r', encoding='utf-8') as f:
                self.config_data = json.load(f)
            
            print(f" Đã load cấu hình từ: {self.config_file}")
            print(f"   - Số intersection: {len(self.config_data.get('intersections', {}))}")
            print(f"   - Số traffic light: {len(self.config_data.get('traffic_lights', {}))}")
            
            return True
            
        except Exception as e:
            print(f" Lỗi khi load cấu hình: {e}")
            self._create_default_config()
            return False
    
    def _create_default_config(self):
        """
        Tạo cấu hình mặc định nếu không có file
        """
        self.config_data = {
            'metadata': {
                'network_file': 'unknown',
                'generated_at': 'default',
                'total_intersections': 3,
                'total_traffic_lights': 3
            },
            'traffic_lights': {
                '1166230678': {
                    'type': 'static',
                    'phases': [
                        {'duration': 45, 'state': 'GGGG'},
                        {'duration': 45, 'state': 'rrrr'}
                    ],
                    'total_cycle': 90
                },
                '1677153107': {
                    'type': 'static',
                    'phases': [
                        {'duration': 40, 'state': 'GGGG'},
                        {'duration': 50, 'state': 'rrrr'}
                    ],
                    'total_cycle': 90
                },
                '357410392': {
                    'type': 'static',
                    'phases': [
                        {'duration': 50, 'state': 'GGGG'},
                        {'duration': 40, 'state': 'rrrr'}
                    ],
                    'total_cycle': 90
                }
            },
            'intersections': {
                '1166230678': {
                    'id': '1166230678',
                    'type': 'traffic_light',
                    'x': 0.0,
                    'y': 0.0
                },
                '1677153107': {
                    'id': '1677153107',
                    'type': 'traffic_light',
                    'x': 0.0,
                    'y': 0.0
                },
                '357410392': {
                    'id': '357410392',
                    'type': 'traffic_light',
                    'x': 0.0,
                    'y': 0.0
                }
            },
            'optimization_parameters': {
                'intersection_ids': ['1166230678', '1677153107', '357410392'],
                'theta_1': 1.0,
                'theta_2': 0.5,
                'default_cycle_length': 90,
                'min_green_time': 15,
                'max_green_time': 75,
                'max_change': 5,
                'intersection_data': {
                    '1166230678': {
                        'cycle_length': 90,
                        'main_phases': [0],
                        'secondary_phases': [1],
                        'saturation_flows': {'main': 0.45, 'secondary': 0.35},
                        'turn_in_ratios': {'main': 0.7, 'secondary': 0.5},
                        'queue_lengths': {'main': 15, 'secondary': 8}
                    },
                    '1677153107': {
                        'cycle_length': 90,
                        'main_phases': [0],
                        'secondary_phases': [1],
                        'saturation_flows': {'main': 0.40, 'secondary': 0.38},
                        'turn_in_ratios': {'main': 0.8, 'secondary': 0.5},
                        'queue_lengths': {'main': 20, 'secondary': 10}
                    },
                    '357410392': {
                        'cycle_length': 90,
                        'main_phases': [0],
                        'secondary_phases': [1],
                        'saturation_flows': {'main': 0.50, 'secondary': 0.42},
                        'turn_in_ratios': {'main': 0.6, 'secondary': 0.5},
                        'queue_lengths': {'main': 12, 'secondary': 6}
                    }
                }
            }
        }
        
        # Lưu cấu hình mặc định
        self.save_config()
    
    def save_config(self, output_file: Optional[str] = None):
        """
        Lưu cấu hình vào file JSON
        
        Args:
            output_file: File output (nếu None thì dùng self.config_file)
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
        Lấy danh sách ID của các intersection
        
        Returns:
            List[str]: Danh sách intersection IDs
        """
        return self.config_data.get('optimization_parameters', {}).get('intersection_ids', [])
    
    def get_optimization_params(self) -> Dict:
        """
        Lấy tham số tối ưu hóa
        
        Returns:
            Dict: Tham số tối ưu hóa
        """
        return self.config_data.get('optimization_parameters', {})
    
    def get_intersection_data(self, intersection_id: str) -> Optional[Dict]:
        """
        Lấy dữ liệu của một intersection cụ thể
        
        Args:
            intersection_id: ID của intersection
            
        Returns:
            Dict: Dữ liệu intersection hoặc None nếu không tìm thấy
        """
        intersection_data = self.config_data.get('optimization_parameters', {}).get('intersection_data', {})
        return intersection_data.get(intersection_id)

    def get_traffic_light_id(self, intersection_id: str) -> Optional[str]:
        """
        Lấy ID của đèn giao thông tương ứng với ID của nút giao.
        
        Args:
            intersection_id: ID của nút giao (e.g., "junction01")
            
        Returns:
            str: ID của đèn giao thông (e.g., "1166230678") hoặc None nếu không tìm thấy.
        """
        intersection = self.config_data.get('intersections', {}).get(intersection_id)
        if intersection:
            return intersection.get('traffic_light_id')
        return None
    
    def get_traffic_light_data(self, tl_id: str) -> Optional[Dict]:
        """
        Lấy dữ liệu traffic light
        
        Args:
            tl_id: ID của traffic light
            
        Returns:
            Dict: Dữ liệu traffic light hoặc None nếu không tìm thấy
        """
        return self.config_data.get('traffic_lights', {}).get(tl_id)
    
    def get_saturation_flows(self, intersection_id: str) -> Dict[str, float]:
        """
        Lấy saturation flows của intersection
        
        Args:
            intersection_id: ID của intersection
            
        Returns:
            Dict[str, float]: Saturation flows cho pha chính và phụ
        """
        intersection_data = self.get_intersection_data(intersection_id)
        if intersection_data:
            return intersection_data.get('saturation_flows', {'main': 0.45, 'secondary': 0.35})
        return {'main': 0.45, 'secondary': 0.35}
    
    def get_turn_in_ratios(self, intersection_id: str) -> Dict[str, float]:
        """
        Lấy turn-in ratios của intersection
        
        Args:
            intersection_id: ID của intersection
            
        Returns:
            Dict[str, float]: Turn-in ratios cho pha chính và phụ
        """
        intersection_data = self.get_intersection_data(intersection_id)
        if intersection_data:
            return intersection_data.get('turn_in_ratios', {'main': 0.7, 'secondary': 0.5})
        return {'main': 0.7, 'secondary': 0.5}
    
    def get_queue_lengths(self, intersection_id: str) -> Dict[str, int]:
        """
        Lấy queue lengths của intersection
        
        Args:
            intersection_id: ID của intersection
            
        Returns:
            Dict[str, int]: Queue lengths cho pha chính và phụ
        """
        intersection_data = self.get_intersection_data(intersection_id)
        if intersection_data:
            return intersection_data.get('queue_lengths', {'main': 15, 'secondary': 8})
        return {'main': 15, 'secondary': 8}
    
    def get_cycle_length(self, intersection_id: str) -> int:
        """
        Lấy chu kỳ đèn của intersection
        
        Args:
            intersection_id: ID của intersection
            
        Returns:
            int: Chu kỳ đèn (giây)
        """
        intersection_data = self.get_intersection_data(intersection_id)
        if intersection_data:
            return intersection_data.get('cycle_length', 90)
        return 90
    
    def get_phase_info(self, intersection_id: str) -> Dict[str, List[int]]:
        """
        Lấy thông tin pha chính và phụ
        
        Args:
            intersection_id: ID của intersection
            
        Returns:
            Dict[str, List[int]]: Danh sách pha chính và phụ
        """
        intersection_data = self.get_intersection_data(intersection_id)
        if intersection_data:
            return {
                'main_phases': intersection_data.get('main_phases', [0]),
                'secondary_phases': intersection_data.get('secondary_phases', [1])
            }
        return {'main_phases': [0], 'secondary_phases': [1]}
    
    def get_global_params(self) -> Dict[str, Any]:
        """
        Lấy tham số toàn cục cho bài toán tối ưu hóa
        
        Returns:
            Dict[str, Any]: Tham số toàn cục
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
    
    def update_intersection_data(self, intersection_id: str, new_data: Dict):
        """
        Cập nhật dữ liệu của một intersection
        
        Args:
            intersection_id: ID của intersection
            new_data: Dữ liệu mới
        """
        if 'optimization_parameters' not in self.config_data:
            self.config_data['optimization_parameters'] = {}
        
        if 'intersection_data' not in self.config_data['optimization_parameters']:
            self.config_data['optimization_parameters']['intersection_data'] = {}
        
        self.config_data['optimization_parameters']['intersection_data'][intersection_id] = new_data
        print(f"✅ Đã cập nhật dữ liệu cho intersection: {intersection_id}")
    
    def validate_config(self) -> bool:
        """
        Kiểm tra tính hợp lệ của cấu hình
        
        Returns:
            bool: True nếu cấu hình hợp lệ
        """
        try:
            # Kiểm tra cấu trúc cơ bản
            required_keys = ['metadata', 'traffic_lights', 'intersections', 'optimization_parameters']
            for key in required_keys:
                if key not in self.config_data:
                    print(f"❌ Thiếu key: {key}")
                    return False
            
            # Kiểm tra optimization parameters
            opt_params = self.config_data['optimization_parameters']
            required_opt_keys = ['intersection_ids', 'theta_1', 'theta_2', 'intersection_data']
            for key in required_opt_keys:
                if key not in opt_params:
                    print(f"❌ Thiếu optimization parameter: {key}")
                    return False
            
            # Kiểm tra dữ liệu intersection
            intersection_ids = opt_params['intersection_ids']
            intersection_data = opt_params['intersection_data']
            
            for intersection_id in intersection_ids:
                if intersection_id not in intersection_data:
                    print(f"❌ Thiếu dữ liệu cho intersection: {intersection_id}")
                    return False
            
            print("✅ Cấu hình hợp lệ")
            return True
            
        except Exception as e:
            print(f"❌ Lỗi khi validate cấu hình: {e}")
            return False
    
    def print_summary(self):
        """
        In tóm tắt cấu hình
        """
        print("\n" + "="*60)
        print("📊 TÓM TẮT CẤU HÌNH INTERSECTION")
        print("="*60)
        
        metadata = self.config_data.get('metadata', {})
        print(f"Network file: {metadata.get('network_file', 'unknown')}")
        print(f"Generated at: {metadata.get('generated_at', 'unknown')}")
        print(f"Total intersections: {metadata.get('total_intersections', 0)}")
        print(f"Total traffic lights: {metadata.get('total_traffic_lights', 0)}")
        
        print("\n🔧 Tham số tối ưu hóa:")
        opt_params = self.get_global_params()
        for key, value in opt_params.items():
            print(f"  {key}: {value}")
        
        print("\n🚦 Intersections:")
        for intersection_id in self.get_intersection_ids():
            data = self.get_intersection_data(intersection_id)
            if data:
                print(f"  {intersection_id}:")
                print(f"    - Cycle length: {data.get('cycle_length', 90)}s")
                print(f"    - Main phases: {data.get('main_phases', [])}")
                print(f"    - Secondary phases: {data.get('secondary_phases', [])}")
                print(f"    - Saturation flows: {data.get('saturation_flows', {})}")
        
        print("="*60)


def create_default_config():
    """
    Tạo file cấu hình mặc định
    """
    manager = IntersectionConfigManager()
    manager.save_config("intersection_config.json")
    manager.print_summary()
    return manager


if __name__ == "__main__":
    # Tạo cấu hình mặc định
    create_default_config()
