import argparse,json
from pathlib import Path
import openpyxl

ROOT=Path(__file__).resolve().parents[1]

def records(path,sheet=None):
    workbook=openpyxl.load_workbook(path,read_only=True,data_only=True)
    try:
        output=[]
        for worksheet in [workbook[sheet]] if sheet else workbook.worksheets:
            rows=worksheet.iter_rows(values_only=True)
            header=next(rows)
            output.extend(dict(zip(header,row)) for row in rows if row and row[0] is not None)
        return output
    finally:workbook.close()

def load_instances(directory):
    directory=Path(directory)
    metadata=records(directory/'Benchmark_Metadata.xlsx')
    cases={r['case_id']:dict(r,suppliers=[],items=[],arcs=[],stack_pairs=[]) for r in metadata}
    for r in records(directory/'Benchmark_Nodes.xlsx'):
        cases[r.pop('case_id')]['suppliers'].append(r)
    for r in records(directory/'Benchmark_Parts.xlsx'):
        cases[r.pop('case_id')]['items'].append(r)
    for r in records(directory/'Benchmark_Distances.xlsx'):
        cases[r.pop('case_id')]['arcs'].append(r)
    for r in records(directory/'Benchmark_Compatibility.xlsx'):
        cases[r.pop('case_id')]['stack_pairs'].append(r)
    return cases

def protocol(case):
    fmt=lambda v:format(float(v),'.17g')
    pairs=[]
    for pair in case['stack_pairs']:
        pairs.extend([(pair['item_m'],pair['item_b'],pair['q_mb']),(pair['item_b'],pair['item_m'],pair['q_bm'])])
    pairs.sort()
    rows=[[*[case[k] for k in ('vehicle_length_mm','vehicle_width_mm','vehicle_height_mm')],fmt(case['unit_distance_cost']),fmt(case['fixed_vehicle_cost']),case['supplier_count'],case['vehicle_count'],len(case['items']),len(case['arcs']),len(pairs)]]
    rows.extend([r['node_id'],fmt(r['service_time_min']),fmt(r['delivery_deadline_min'])] for r in sorted(case['suppliers'],key=lambda r:r['node_id']))
    rows.extend([r['item_id'],r['supplier_id'],r['length_mm'],r['width_mm'],r['height_mm'],int(r['is_pallet']),int(r['is_optional']),r['priority'],r['dock_id'],r['size_class'],r['stack_mode']] for r in sorted(case['items'],key=lambda r:r['item_id']))
    rows.extend([r['node_i'],r['node_j'],fmt(r['distance_km']),fmt(r['travel_time_min'])] for r in case['arcs'])
    rows.extend(pairs)
    return '\n'.join(' '.join(map(str,r)) for r in rows)+'\n'

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--case',required=True)
    parser.add_argument('--instances',type=Path,default=ROOT/'Instances')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    cases=load_instances(args.instances)
    if args.case not in cases:parser.error('Unknown case ID')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    if args.output.exists():parser.error('Output exists; choose a new path')
    args.output.write_text(json.dumps(cases[args.case],ensure_ascii=True,indent=2),encoding='utf8')
    print(args.case)

if __name__=='__main__':main()
