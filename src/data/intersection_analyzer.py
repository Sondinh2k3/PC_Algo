"""
Intersection Analyzer - Phân tích và đọc thông tin nút giao từ SUMO
Tự động phát hiện các nút giao, pha đèn và thông số liên quan
"""

import traci
# import libsumo as traci
import json
import xml.etree.ElementTree as ET
from typing import Dict, List, Tuple, Optional
import os
import logging

class IntersectionAnalyzer:
    """
    Phân tích thông tin nút giao từ SUMO network và simulation
    """
    
    def __init__(self, net_file_path: str):
        """
        Khởi tạo analyzer
        
        Args:
            net_file_path: Đường dẫn đến file .net.xml
        """
        self.net_file_path = net_file_path
        self.intersections = {}
        self.traffic_lights = {}
        
    def analyze_network(self) -> Dict:
        """
        Phân tích file network để lấy thông tin nút giao
        """
        print("🔍 Đang phân tích file network...")
        
        try:
            tree = ET.parse(self.net_file_path)
            root = tree.getroot()
            
            # Tìm tất cả traffic lights
            for tl_logic in root.findall('.//tlLogic'):
                tl_id = tl_logic.get('id')
                tl_type = tl_logic.get('type', 'static')
                
                phases = []
                for phase in tl_logic.findall('phase'):
                    duration = int(phase.get('duration', 0))
                    state = phase.get('state', '')
                    phases.append({
                        'duration': duration,
                        'state': state
                    })
                
                self.traffic_lights[tl_id] = {
                    'type': tl_type,
                    'phases': phases,
                    'total_cycle': sum(p['duration'] for p in phases)
                }
            
            # Tìm các junction có traffic light
            for junction in root.findall('.//junction'):
                junction_id = junction.get('id')
                junction_type = junction.get('type')
                
                # Chỉ quan tâm đến junction có traffic light
                if junction_type in ['traffic_light', 'traffic_light_right_on_red']:
                    self.intersections[junction_id] = {
                        'id': junction_id,
                        'type': junction_type,
                        'x': float(junction.get('x', 0)),
                        'y': float(junction.get('y', 0)),
                        'incoming_lanes': [],
                        'outgoing_lanes': [],
                        'internal_lanes': []
                    }
            
            print(f"✅ Tìm thấy {len(self.traffic_lights)} traffic lights và {len(self.intersections)} intersections")
            return self.intersections
            
        except Exception as e:
            print(f"❌ Lỗi khi phân tích network: {e}")
            return {}
    
    def analyze_from_simulation(self) -> Dict:
        """
        Phân tích từ simulation đang chạy (TraCI)
        """
        print("🔍 Đang phân tích từ simulation...")
        
        try:
            # Lấy danh sách traffic light IDs
            tl_ids = traci.trafficlight.getIDList()
            
            intersection_data = {}
            
            for tl_id in tl_ids:
                print(f"  Phân tích traffic light: {tl_id}")
                
                # Lấy thông tin cơ bản
                program_id = traci.trafficlight.getProgram(tl_id)
                phase_count = traci.trafficlight.getPhaseNumber(tl_id)
                current_phase = traci.trafficlight.getPhase(tl_id)
                
                # Lấy thông tin các pha
                phases = []
                for i in range(phase_count):
                    duration = traci.trafficlight.getPhaseDuration(tl_id, i)
                    state = traci.trafficlight.getRedYellowGreenState(tl_id, i)
                    phases.append({
                        'index': i,
                        'duration': duration,
                        'state': state
                    })
                
                # Lấy controlled lanes
                controlled_lanes = traci.trafficlight.getControlledLanes(tl_id)
                
                # Phân tích các pha chính và phụ
                main_phases, secondary_phases = self._classify_phases(phases, controlled_lanes)
                
                intersection_data[tl_id] = {
                    'id': tl_id,
                    'program_id': program_id,
                    'total_phases': phase_count,
                    'current_phase': current_phase,
                    'controlled_lanes': controlled_lanes,
                    'phases': phases,
                    'main_phases': main_phases,
                    'secondary_phases': secondary_phases,
                    'cycle_length': sum(p['duration'] for p in phases),
                    'estimated_capacity': self._estimate_capacity(controlled_lanes)
                }
            
            return intersection_data
            
        except Exception as e:
            print(f"❌ Lỗi khi phân tích từ simulation: {e}")
            return {}
    
    def _classify_phases(self, phases: List[Dict], controlled_lanes: List[str]) -> Tuple[List[int], List[int]]:
        """
        Phân loại pha chính và phụ dựa trên số lượng lane được điều khiển
        """
        main_phases = []
        secondary_phases = []
        
        for i, phase in enumerate(phases):
            # Đếm số lane có đèn xanh trong pha này
            green_lanes = sum(1 for char in phase['state'] if char == 'G')
            
            if green_lanes >= 2:  # Pha chính có ít nhất 2 lane xanh
                main_phases.append(i)
            else:
                secondary_phases.append(i)
        
        return main_phases, secondary_phases
    
    def _estimate_capacity(self, controlled_lanes: List[str]) -> Dict:
        """
        Ước tính capacity cho các lane được điều khiển
        """
        capacity_data = {}
        
        for lane_id in controlled_lanes:
            try:
                # Lấy thông tin lane
                max_speed = traci.lane.getMaxSpeed(lane_id)
                length = traci.lane.getLength(lane_id)
                
                # Ước tính saturation flow (xe/giờ)
                # Giả sử khoảng cách trung bình giữa các xe là 7.5m
                avg_vehicle_gap = 7.5  # meters
                saturation_flow = (3600 * max_speed) / (avg_vehicle_gap + 5)  # 5m là chiều dài xe trung bình
                
                capacity_data[lane_id] = {
                    'max_speed': max_speed,
                    'length': length,
                    'estimated_saturation_flow': saturation_flow,
                    'estimated_saturation_flow_per_second': saturation_flow / 3600
                }
                
            except traci.TraCIException:
                # Lane không tồn tại hoặc không thể truy cập
                capacity_data[lane_id] = {
                    'max_speed': 13.89,  # 50 km/h mặc định
                    'length': 100,
                    'estimated_saturation_flow': 1800,  # xe/giờ mặc định
                    'estimated_saturation_flow_per_second': 0.5
                }
        
        return capacity_data
    
    def generate_intersection_config(self, output_file: str = "intersection_config.json"):
        """
        Tạo file cấu hình JSON cho các nút giao
        """
        print("📝 Đang tạo file cấu hình intersection...")
        
        # Phân tích từ network trước
        network_data = self.analyze_network()
        
        # Nếu có simulation đang chạy, lấy thêm dữ liệu
        simulation_data = {}
        try:
            simulation_data = self.analyze_from_simulation()
        except:
            print("⚠️ Không thể kết nối với simulation, chỉ sử dụng dữ liệu network")
        
        # Kết hợp dữ liệu
        combined_data = {
            'metadata': {
                'network_file': self.net_file_path,
                'generated_at': str(traci.simulation.getTime()) if 'traci' in globals() else 'unknown',
                'total_intersections': len(network_data),
                'total_traffic_lights': len(self.traffic_lights)
            },
            'traffic_lights': self.traffic_lights,
            'intersections': network_data,
            'simulation_data': simulation_data,
            'optimization_parameters': self._generate_optimization_params(network_data, simulation_data)
        }
        
        # Lưu vào file JSON
        with open(output_file, 'w', encoding='utf-8') as f:
            json.dump(combined_data, f, indent=2, ensure_ascii=False)
        
        print(f"✅ Đã tạo file cấu hình: {output_file}")
        return combined_data
    
    def _generate_optimization_params(self, network_data: Dict, simulation_data: Dict) -> Dict:
        """
        Tạo tham số cho bài toán tối ưu hóa
        """
        optimization_params = {
            'intersection_ids': list(network_data.keys()),
            'theta_1': 1.0,  # Trọng số độ lệch
            'theta_2': 0.5,  # Trọng số capacity
            'default_cycle_length': 90,  # Chu kỳ mặc định (giây)
            'min_green_time': 15,  # Thời gian xanh tối thiểu
            'max_green_time': 75,  # Thời gian xanh tối đa
            'max_change': 5,  # Thay đổi tối đa giữa các chu kỳ
            'intersection_data': {}
        }
        
        # Tạo dữ liệu cho từng intersection
        for intersection_id in network_data.keys():
            tl_data = self.traffic_lights.get(intersection_id, {})
            sim_data = simulation_data.get(intersection_id, {})
            
            # Lấy thông tin pha
            phases = tl_data.get('phases', [])
            if not phases and sim_data:
                phases = sim_data.get('phases', [])
            
            # Phân loại pha chính/phụ
            main_phases = []
            secondary_phases = []
            if sim_data:
                main_phases = sim_data.get('main_phases', [])
                secondary_phases = sim_data.get('secondary_phases', [])
            else:
                # Phân loại dựa trên network data
                for i, phase in enumerate(phases):
                    if len(phase.get('state', '')) >= 2:
                        main_phases.append(i)
                    else:
                        secondary_phases.append(i)
            
            # Ước tính saturation flow
            saturation_flows = {}
            if sim_data and 'estimated_capacity' in sim_data:
                for lane_id, capacity in sim_data['estimated_capacity'].items():
                    saturation_flows[lane_id] = capacity['estimated_saturation_flow_per_second']
            
            # Tạo dữ liệu mặc định nếu không có
            if not saturation_flows:
                saturation_flows = {
                    'main': 0.45,  # xe/giây-xanh
                    'secondary': 0.35
                }
            
            optimization_params['intersection_data'][intersection_id] = {
                'cycle_length': tl_data.get('total_cycle', 90),
                'main_phases': main_phases,
                'secondary_phases': secondary_phases,
                'saturation_flows': saturation_flows,
                'turn_in_ratios': {
                    'main': 0.7,  # Tỷ lệ rẽ vào mặc định
                    'secondary': 0.5
                },
                'queue_lengths': {
                    'main': 15,  # Hàng đợi mặc định (xe)
                    'secondary': 8
                }
            }
        
        return optimization_params


def analyze_phuquoc_network():
    """
    Hàm tiện ích để phân tích mạng lưới Phú Quốc
    """
    # Đường dẫn đến file network
    net_file = "../PhuQuoc/phuquoc.net.xml"
    
    if not os.path.exists(net_file):
        print(f"❌ Không tìm thấy file network: {net_file}")
        return None
    
    analyzer = IntersectionAnalyzer(net_file)
    return analyzer.generate_intersection_config("intersection_config.json")


if __name__ == "__main__":
    # Chạy phân tích mạng lưới Phú Quốc
    analyze_phuquoc_network()
