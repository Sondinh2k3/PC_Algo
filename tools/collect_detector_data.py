import os
import json
import xml.etree.ElementTree as ET
from glob import glob

CONFIG_PATH = os.path.join('..', 'src', 'config', 'evaluation_detectors.json')
PHUQUOC_DIR = os.path.join('..', 'src', 'PhuQuoc')
OUTPUT_PATH = os.path.join('..', 'output', 'output.xml')

# Đọc cấu hình detector
with open(CONFIG_PATH, 'r', encoding='utf-8') as f:
    config = json.load(f)

detectors = config['detectors']

# Tìm tất cả file xml detector trong thư mục PhuQuoc
xml_files = glob(os.path.join(PHUQUOC_DIR, '*.xml'))

# Map detector id -> file xml
detector_file_map = {}
for det in detectors:
    # Tìm file xml phù hợp với id detector
    for xml_file in xml_files:
        if det['id'] in os.path.basename(xml_file):
            detector_file_map[det['id']] = xml_file
            break

# Thu thập dữ liệu từ các file xml
collected_data = []
for det in detectors:
    det_id = det['id']
    xml_file = detector_file_map.get(det_id)
    if not xml_file or not os.path.exists(xml_file):
        print(f"[WARNING] Không tìm thấy file cho detector {det_id}")
        continue
    try:
        tree = ET.parse(xml_file)
        root = tree.getroot()
        for interval in root.findall('interval'):
            entry = {
                'detector_id': det_id,
                'type': det['type'],
                'begin': interval.get('begin'),
                'end': interval.get('end')
            }
            # E1: flow, speed, occupancy
            if det['type'] == 'e1':
                entry['flow'] = interval.get('flow')
                entry['speed'] = interval.get('speed')
                entry['occupancy'] = interval.get('occupancy')
            # E2: jamLengthVeh, jamLengthMeters, meanSpeed
            elif det['type'] == 'e2':
                entry['jamLengthVeh'] = interval.get('jamLengthVeh')
                entry['jamLengthMeters'] = interval.get('jamLengthMeters')
                entry['meanSpeed'] = interval.get('meanSpeed')
            collected_data.append(entry)
    except Exception as e:
        print(f"[ERROR] Lỗi đọc file {xml_file}: {e}")

# Ghi dữ liệu tổng hợp ra file output.xml
import xml.dom.minidom

doc = xml.dom.minidom.Document()
root_elem = doc.createElement('detector_data')
doc.appendChild(root_elem)

for entry in collected_data:
    det_elem = doc.createElement('detector')
    for k, v in entry.items():
        child = doc.createElement(k)
        child.appendChild(doc.createTextNode(str(v) if v is not None else ''))
        det_elem.appendChild(child)
    root_elem.appendChild(det_elem)

with open(OUTPUT_PATH, 'w', encoding='utf-8') as f:
    f.write(doc.toprettyxml(indent='  '))

print(f"✅ Đã tổng hợp dữ liệu detector vào {OUTPUT_PATH}")
