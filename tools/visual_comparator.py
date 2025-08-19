import os
import json
import xml.etree.ElementTree as ET
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np
import logging

# Thiết lập logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

def load_target_edges(config_path):
    """Load target edges from analysis_config.json"""
    if not os.path.exists(config_path):
        logger.error(f"Không tìm thấy file cấu hình: {config_path}")
        return None
        
    try:
        with open(config_path, 'r', encoding='utf-8') as f:
            config_data = json.load(f)
            target_edges = config_data.get('target_edges')
            
            if not target_edges:
                logger.error("Không tìm thấy 'target_edges' trong file cấu hình")
                return None
                
            logger.info(f"Đã tải {len(target_edges)} target edges từ config: {target_edges}")
            return target_edges
            
    except (json.JSONDecodeError, IOError) as e:
        logger.error(f"Lỗi đọc file cấu hình {config_path}: {e}")
        return None

def parse_vehroutes(file_path):
    """Parse vehroutes XML to get a map of trip_id to edges string."""
    if not os.path.exists(file_path):
        logger.error(f"Không tìm thấy file tuyến đường: {file_path}")
        return {}
    
    try:
        tree = ET.parse(file_path)
        root = tree.getroot()
    except ET.ParseError as e:
        logger.error(f"Lỗi parsing XML {file_path}: {e}")
        return {}
        
    routes_map = {}
    for vehicle in root.findall('vehicle'):
        trip_id = vehicle.get('id')
        route = vehicle.find('route')
        if trip_id and route is not None:
            edges = route.get('edges', '')
            routes_map[trip_id] = edges
            
    logger.info(f"Đã đọc {len(routes_map)} tuyến đường từ {file_path}")
    return routes_map

def parse_tripinfo_by_edges(file_path, target_edges, routes_map):
    """Parse tripinfo XML và chỉ giữ trips đi qua target edges, sử dụng routes_map."""
    if not os.path.exists(file_path):
        logger.error(f"Không tìm thấy file: {file_path}")
        return pd.DataFrame()
    
    if not routes_map:
        logger.error(f"Không có dữ liệu tuyến đường (routes_map) để xử lý {file_path}")
        return pd.DataFrame()

    try:
        tree = ET.parse(file_path)
        root = tree.getroot()
    except ET.ParseError as e:
        logger.error(f"Lỗi parsing XML {file_path}: {e}")
        return pd.DataFrame()
    
    trips = []
    total_trips = 0
    filtered_trips = 0
    target_edges_set = set(target_edges)
    
    for trip in root.findall('tripinfo'):
        total_trips += 1
        try:
            trip_id = trip.get('id')
            if not trip_id:
                continue

            # Lấy route edges của trip từ map đã đọc
            route_edges = routes_map.get(trip_id, '')
            if not route_edges:
                continue  # Bỏ qua trips không có thông tin route trong map

            # Kiểm tra xem trip có đi qua target edges không
            trip_edges = set(route_edges.split())
            if not trip_edges.intersection(target_edges_set):
                continue  # Bỏ qua trip không đi qua target edges
            
            # Trip hợp lệ - lưu dữ liệu
            trips.append({
                'id': trip_id,
                'depart': float(trip.get('depart', 0)),
                'arrival': float(trip.get('arrival', 0)),
                'duration': float(trip.get('duration', 0)),
                'timeLoss': float(trip.get('timeLoss', 0)),
                'routeLength': float(trip.get('routeLength', 0))
            })
            filtered_trips += 1
            
        except (TypeError, ValueError) as e:
            logger.warning(f"Bỏ qua trip lỗi: {e}")
            continue
    
    logger.info(f"Đã lọc {filtered_trips}/{total_trips} trips đi qua target edges")
    return pd.DataFrame(trips)

def parse_edgedata_by_edges(file_path, target_edges):
    """Parse edgedata XML và chỉ tính lưu lượng của target edges"""
    if not os.path.exists(file_path):
        logger.error(f"Không tìm thấy file: {file_path}")
        return pd.DataFrame()
        
    try:
        tree = ET.parse(file_path)
        root = tree.getroot()
    except ET.ParseError as e:
        logger.error(f"Lỗi parsing XML {file_path}: {e}")
        return pd.DataFrame()
    
    intervals_data = []
    target_edges_set = set(target_edges)
    
    for interval in root.findall('interval'):
        try:
            interval_begin = float(interval.get('begin', 0))
            interval_end = float(interval.get('end', 0))
            total_flow = 0
            
            # Chỉ tính lưu lượng của target edges
            for edge in interval.findall('edge'):
                edge_id = edge.get('id')
                if edge_id in target_edges_set:
                    try:
                        entered_value = edge.get('entered', '0')
                        total_flow += float(entered_value)
                    except (TypeError, ValueError):
                        logger.warning(f"Giá trị 'entered' không hợp lệ cho edge {edge_id}")
                        continue
            
            intervals_data.append({
                'time_begin': interval_begin,
                'time_end': interval_end,
                'total_flow': total_flow
            })
            
        except (TypeError, ValueError) as e:
            logger.warning(f"Bỏ qua interval lỗi: {e}")
            continue
        
    return pd.DataFrame(intervals_data)

def ensure_output_dir(output_dir):
    """Đảm bảo thư mục output tồn tại và có thể ghi"""
    try:
        os.makedirs(output_dir, exist_ok=True)
        test_file = os.path.join(output_dir, '.test_write')
        with open(test_file, 'w') as f:
            f.write('test')
        os.remove(test_file)
        return True
    except (OSError, PermissionError) as e:
        logger.error(f"Không thể tạo/ghi thư mục {output_dir}: {e}")
        return False

def plot_delay_comparison(df_algo, df_baseline, output_dir):
    """Vẽ biểu đồ so sánh độ trễ trung bình"""
    if df_algo.empty and df_baseline.empty:
        logger.warning("Không có dữ liệu trip, bỏ qua biểu đồ độ trễ")
        return

    avg_delay_algo = df_algo['timeLoss'].mean() if not df_algo.empty else 0
    avg_delay_baseline = df_baseline['timeLoss'].mean() if not df_baseline.empty else 0
    
    plt.figure(figsize=(10, 6))
    
    labels = ['Baseline', 'Với Thuật Toán']
    values = [avg_delay_baseline, avg_delay_algo]
    colors = ['#3498db', '#e74c3c']
    
    # Đánh dấu nếu không có dữ liệu
    if df_baseline.empty:
        colors[0] = 'lightgray'
        labels[0] += ' (Không có dữ liệu)'
    if df_algo.empty:
        colors[1] = 'lightgray' 
        labels[1] += ' (Không có dữ liệu)'
    
    bars = plt.bar(labels, values, color=colors, alpha=0.8)
    
    plt.title('So sánh Độ trễ Trung bình (Các Edge được chỉ định)', fontsize=16, pad=20)
    plt.ylabel('Độ trễ trung bình (giây)', fontsize=12)
    plt.grid(axis='y', linestyle='--', alpha=0.7)
    
    # Hiển thị giá trị
    for i, (bar, value) in enumerate(zip(bars, values)):
        if (i == 0 and not df_baseline.empty) or (i == 1 and not df_algo.empty):
            plt.text(bar.get_x() + bar.get_width()/2, bar.get_height() + max(values)*0.01, 
                    f"{value:.2f}s", ha='center', va='bottom', fontweight='bold', fontsize=11)

    plt.tight_layout()
    path = os.path.join(output_dir, 'regional_delay_comparison.png')
    try:
        plt.savefig(path, dpi=300, bbox_inches='tight')
        plt.close()
        logger.info(f"Đã lưu biểu đồ độ trễ: {path}")
    except Exception as e:
        logger.error(f"Lỗi lưu biểu đồ: {e}")
        plt.close()

def plot_travel_time_distribution(df_algo, df_baseline, output_dir):
    """Vẽ biểu đồ phân phối thời gian di chuyển"""
    if df_algo.empty and df_baseline.empty:
        logger.warning("Không có dữ liệu trip, bỏ qua biểu đồ phân phối")
        return

    combined_data = []
    
    if not df_baseline.empty:
        baseline_data = df_baseline.copy()
        baseline_data['Case'] = 'Baseline'
        combined_data.append(baseline_data)
    
    if not df_algo.empty:
        algo_data = df_algo.copy()
        algo_data['Case'] = 'Với Thuật Toán'
        combined_data.append(algo_data)
    
    if not combined_data:
        return
        
    combined_df = pd.concat(combined_data, ignore_index=True)

    plt.figure(figsize=(12, 7))
    sns.boxplot(x='Case', y='duration', data=combined_df, 
                order=['Baseline', 'Với Thuật Toán'],
                palette=['#3498db', '#e74c3c'])
    
    plt.title('Phân phối Thời gian di chuyển (Các Edge được chỉ định)', fontsize=16, pad=20)
    plt.ylabel('Thời gian di chuyển (giây)', fontsize=12)
    plt.xlabel('')
    plt.grid(axis='y', linestyle='--', alpha=0.7)

    plt.tight_layout()
    path = os.path.join(output_dir, 'regional_travel_time_distribution.png')
    try:
        plt.savefig(path, dpi=300, bbox_inches='tight')
        plt.close()
        logger.info(f"Đã lưu biểu đồ phân phối: {path}")
    except Exception as e:
        logger.error(f"Lỗi lưu biểu đồ: {e}")
        plt.close()

def plot_throughput_over_time(df_algo, df_baseline, output_dir):
    """Vẽ biểu đồ lưu lượng theo thời gian"""
    if df_algo.empty and df_baseline.empty:
        logger.warning("Không có dữ liệu edge, bỏ qua biểu đồ lưu lượng")
        return

    plt.figure(figsize=(14, 7))
    
    if not df_baseline.empty:
        plt.plot(df_baseline['time_end'], df_baseline['total_flow'], 
                label='Baseline', marker='o', linestyle='--',
                color='#3498db', alpha=0.8, linewidth=2, markersize=4)
    
    if not df_algo.empty:
        plt.plot(df_algo['time_end'], df_algo['total_flow'], 
                label='Với Thuật Toán', marker='s', linestyle='-',
                color='#e74c3c', alpha=0.8, linewidth=2, markersize=4)
    
    plt.title('So sánh Lưu lượng theo Thời gian (Các Edge được chỉ định)', fontsize=16, pad=20)
    plt.xlabel('Thời gian mô phỏng (giây)', fontsize=12)
    plt.ylabel('Tổng lưu lượng (xe/khoảng thời gian)', fontsize=12)
    plt.legend(fontsize=11)
    plt.grid(True, linestyle='--', alpha=0.7)

    plt.tight_layout()
    path = os.path.join(output_dir, 'regional_throughput_over_time.png')
    try:
        plt.savefig(path, dpi=300, bbox_inches='tight')
        plt.close()
        logger.info(f"Đã lưu biểu đồ lưu lượng: {path}")
    except Exception as e:
        logger.error(f"Lỗi lưu biểu đồ: {e}")
        plt.close()

def main():
    """Hàm chính"""
    print("\n" + "="*70)
    print("🚗 PHÂN TÍCH HIỆU NĂNG GIAO THÔNG THEO VÙNG")
    print("="*70)

    # Thiết lập đường dẫn
    project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    output_dir = os.path.join(project_root, 'output')
    config_path = os.path.join(project_root, 'src', 'config', 'analysis_config.json')
    
    # Kiểm tra thư mục output
    if not ensure_output_dir(output_dir):
        logger.error("❌ Không thể tạo thư mục output")
        return

    # Load target edges từ config
    target_edges = load_target_edges(config_path)
    if not target_edges:
        logger.error("❌ Không thể load target edges từ config")
        return

    # Đường dẫn files dữ liệu
    files = {
        'trip_algo': os.path.join(output_dir, 'tripinfo.xml'),
        'trip_baseline': os.path.join(output_dir, 'tripinfo_baseline.xml'),
        'edge_algo': os.path.join(output_dir, 'edgedata.xml'),
        'edge_baseline': os.path.join(output_dir, 'edgedata_baseline.xml'),
        'route_algo': os.path.join(output_dir, 'vehroutes.xml'),
        'route_baseline': os.path.join(output_dir, 'vehroutes_baseline.xml')
    }

    logger.info("📊 Đang đọc và lọc dữ liệu...")
    
    # Đọc dữ liệu tuyến đường trước
    routes_algo_map = parse_vehroutes(files['route_algo'])
    routes_baseline_map = parse_vehroutes(files['route_baseline'])
    
    # Parse dữ liệu trip - truyền vào map tuyến đường
    df_trip_algo = parse_tripinfo_by_edges(files['trip_algo'], target_edges, routes_algo_map)
    df_trip_baseline = parse_tripinfo_by_edges(files['trip_baseline'], target_edges, routes_baseline_map)
    
    # Parse dữ liệu edge
    df_edge_algo = parse_edgedata_by_edges(files['edge_algo'], target_edges)
    df_edge_baseline = parse_edgedata_by_edges(files['edge_baseline'], target_edges)
    
    # Thống kê dữ liệu
    logger.info(f"📈 Dữ liệu đã lọc:")
    logger.info(f"  • Trips algorithm: {len(df_trip_algo)} records")
    logger.info(f"  • Trips baseline: {len(df_trip_baseline)} records")
    logger.info(f"  • Edge intervals algorithm: {len(df_edge_algo)} records")
    logger.info(f"  • Edge intervals baseline: {len(df_edge_baseline)} records")
    
    # Tạo biểu đồ
    logger.info("🎨 Đang tạo biểu đồ so sánh...")
    plot_delay_comparison(df_trip_algo, df_trip_baseline, output_dir)
    plot_travel_time_distribution(df_trip_algo, df_trip_baseline, output_dir)
    plot_throughput_over_time(df_edge_algo, df_edge_baseline, output_dir)
    
    print("\n" + "="*70)
    print("✅ HOÀN TẤT! Các biểu đồ đã được lưu trong thư mục output/")
    print("📁 Files được tạo:")
    print("  • regional_delay_comparison.png")
    print("  • regional_travel_time_distribution.png") 
    print("  • regional_throughput_over_time.png")
    print("="*70)

if __name__ == '__main__':
    main()