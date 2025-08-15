#!/usr/bin/env python3
"""
Script tự động phân tích SUMO network và tạo file cấu hình intersection
Sử dụng: python src/scripts/generate_intersection_config.py [network_file] [output_file]
"""

import sys
import os
import argparse
from pathlib import Path

# Thêm đường dẫn src vào sys.path
sys.path.append(str(Path(__file__).parent.parent))

from data.intersection_analyzer import IntersectionAnalyzer
from data.intersection_config_manager import IntersectionConfigManager

def main():
    parser = argparse.ArgumentParser(description='Tạo cấu hình intersection từ SUMO network')
    parser.add_argument('network_file', nargs='?', default='PhuQuoc/phuquoc.net.xml',
                       help='Đường dẫn đến file .net.xml (mặc định: PhuQuoc/phuquoc.net.xml)')
    parser.add_argument('output_file', nargs='?', default='intersection_config.json',
                       help='File output JSON (mặc định: intersection_config.json)')
    parser.add_argument('--analyze-only', action='store_true',
                       help='Chỉ phân tích network, không tạo cấu hình mặc định')
    parser.add_argument('--validate', action='store_true',
                       help='Validate cấu hình sau khi tạo')
    
    args = parser.parse_args()
    
    print("🚦 TẠO CẤU HÌNH INTERSECTION TỪ SUMO NETWORK")
    print("="*60)
    
    # Kiểm tra file network
    if not os.path.exists(args.network_file):
        print(f"❌ Không tìm thấy file network: {args.network_file}")
        print("💡 Sử dụng: python src/scripts/generate_intersection_config.py [network_file] [output_file]")
        return False
    
    try:
        # Tạo analyzer
        analyzer = IntersectionAnalyzer(args.network_file)
        
        # Phân tích network
        print(f"📁 Đang phân tích network: {args.network_file}")
        network_data = analyzer.analyze_network()
        
        if not network_data:
            print("❌ Không tìm thấy intersection nào trong network")
            return False
        
        # Tạo cấu hình
        if args.analyze_only:
            print("🔍 Chế độ chỉ phân tích - không tạo cấu hình")
            print(f"Tìm thấy {len(network_data)} intersections:")
            for intersection_id, data in network_data.items():
                print(f"  - {intersection_id}: {data.get('type', 'unknown')}")
        else:
            print(f"📝 Đang tạo cấu hình: {args.output_file}")
            config_data = analyzer.generate_intersection_config(args.output_file)
            
            if config_data:
                print("✅ Tạo cấu hình thành công!")
                
                # Validate nếu được yêu cầu
                if args.validate:
                    print("🔍 Đang validate cấu hình...")
                    config_manager = IntersectionConfigManager(args.output_file)
                    if config_manager.validate_config():
                        print("✅ Cấu hình hợp lệ!")
                        config_manager.print_summary()
                    else:
                        print("❌ Cấu hình không hợp lệ!")
                        return False
                
                return True
            else:
                print("❌ Lỗi khi tạo cấu hình")
                return False
    
    except Exception as e:
        print(f"❌ Lỗi: {e}")
        return False

def create_default_config():
    """
    Tạo cấu hình mặc định
    """
    print("🔄 Tạo cấu hình mặc định...")
    
    try:
        config_manager = IntersectionConfigManager()
        config_manager.save_config("intersection_config.json")
        config_manager.print_summary()
        print("✅ Đã tạo cấu hình mặc định thành công!")
        return True
    except Exception as e:
        print(f"❌ Lỗi khi tạo cấu hình mặc định: {e}")
        return False

if __name__ == "__main__":
    if len(sys.argv) == 1:
        # Không có argument, tạo cấu hình mặc định
        success = create_default_config()
    else:
        # Có argument, chạy phân tích
        success = main()
    
    sys.exit(0 if success else 1)
