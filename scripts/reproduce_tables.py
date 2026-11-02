import argparse,csv,json,statistics
from bisect import bisect_right
from collections import defaultdict
from decimal import Decimal
from pathlib import Path
import openpyxl
from convert_instances import records

ROOT=Path(__file__).resolve().parents[1]
METHODS=['ILS','ALNS','HGS','G60','G3600','Gseq']
SCALES=['small','medium','large','overall']
EPS=Decimal('0.00001');F_EPS=Decimal('0.001')

def compare(a,b):
    for key,sense,tol in [('C',1,EPS),('P',-1,EPS),('F',-1,F_EPS)]:
        difference=(Decimal(str(a[key]))-Decimal(str(b[key])))*sense
        if difference < -tol:return -1
        if difference > tol:return 1
    return 0

def best(rows):
    winner=rows[0]
    for row in rows[1:]:
        if compare(row,winner)<0:winner=row
    return winner

def mean(values):return statistics.fmean(values) if values else None

def close(a,b,tol=EPS):return abs(Decimal(str(a))-Decimal(str(b)))<=tol

def cost_gap(actual,reference):
    return 0.0 if close(actual,reference) else 100*(float(actual)-float(reference))/abs(float(reference))

def loss(actual,reference,tol=EPS):
    if close(actual,reference,tol):return 0.0
    if not reference:raise ValueError('Undefined gap against zero reference')
    return 100*(float(reference)-float(actual))/abs(float(reference))

def write_csv(folder,name,rows):
    if not rows:return
    with (folder/(name+'.csv')).open('w',encoding='utf-8-sig',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)

def convergence_grid(events,seconds=600):
    groups=defaultdict(list)
    for event in events:groups[event['case_id'],event['seed']].append(event)
    for (case_id,seed),values in sorted(groups.items()):
        values.sort(key=lambda r:(r['display_t_s'],r['event_id']))
        times=[r['display_t_s'] for r in values]
        for t in range(seconds+1):
            index=bisect_right(times,t)-1
            event=values[index] if index>=0 else {}
            yield dict(case_id=case_id,scale=values[0]['scale'],seed=seed,t_s=t,**{k:event.get(k) for k in ('C','P','F','K','event_id','phase')})

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--results',type=Path,default=ROOT/'Experiment_Results/Experiment_Results.xlsx')
    parser.add_argument('--instances',type=Path,default=ROOT/'Instances')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--figures',action='store_true')
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    workbook=openpyxl.load_workbook(args.results,read_only=True,data_only=True)
    def read(name):
        rows=workbook[name].iter_rows(values_only=True);header=next(rows)
        result=[dict(zip(header,r)) for r in rows if r and r[0] is not None]
        for row in result:
            if 'case_id' in row:
                row.setdefault('scale',row['case_id'].split('_')[2])
            if name=='Algorithm_Comparison':
                row['algorithm']=row['algorithm'].removeprefix('HRLH-')
            if name=='Efficiency_C':
                row.setdefault('Scale',row['Case'].split('_')[2])
        return result
    algorithms=read('Algorithm_Comparison');exact=read('Gurobi');ablation=read('Ablation')
    assert len(algorithms)==9000 and len(exact)==900 and len(ablation)==9000
    groups={m:defaultdict(list) for m in METHODS}
    scales={}
    for row in algorithms:
        groups[row['algorithm']][row['case_id']].append(row);scales[row['case_id']]=row['scale']
    for row in exact:
        if row['feasible'] and all(isinstance(row[k],(int,float)) for k in ['C','P','F']):groups[row['method']][row['case_id']].append(row)
    for method in METHODS[:3]:
        assert len(groups[method])==300
        assert all(len(rs)==10 and len({r['seed'] for r in rs})==10 for rs in groups[method].values())
    selected={m:{cid:best(rs) for cid,rs in groups[m].items()} for m in METHODS}
    bks={cid:best([r for m in METHODS for r in groups[m].get(cid,[])]) for cid in scales}
    table6=[];table7=[];proofs=[]
    for scale in SCALES:
        cases=[cid for cid in sorted(scales) if scale=='overall' or scales[cid]==scale]
        for method in METHODS:
            available=[cid for cid in cases if cid in selected[method]]
            rs=[selected[method][cid] for cid in available]
            means=[mean([r['C'] for r in groups[method][cid]]) for cid in available]
            table6.append({'Scale':scale,'Method':method,'Feasible':len(available),'Avg':mean(means) if method in METHODS[:3] else None,
              'Best':mean([r['C'] for r in rs]),'Gap_Avg_pct':mean([cost_gap(v,bks[cid]['C']) for cid,v in zip(available,means)]) if method in METHODS[:3] else None,
              'Gap_Best_pct':mean([cost_gap(selected[method][cid]['C'],bks[cid]['C']) for cid in available]),
              'Std_pct':mean([100*statistics.stdev([r['C'] for r in groups[method][cid]])/bks[cid]['C'] for cid in available]) if method in METHODS[:3] else None})
            c_matches=[cid for cid in available if close(selected[method][cid]['C'],bks[cid]['C'])]
            cp_matches=[cid for cid in c_matches if close(selected[method][cid]['P'],bks[cid]['P'])]
            hits=[cid for cid in cp_matches if close(selected[method][cid]['F'],bks[cid]['F'],F_EPS)]
            table7.append({'Scale':scale,'Method':method,'N_C':len(c_matches),'Gap_P_given_C_pct':mean([loss(selected[method][cid]['P'],bks[cid]['P']) for cid in c_matches]),
              'N_CP':len(cp_matches),'Gap_F_given_CP_pct':mean([loss(selected[method][cid]['F'],bks[cid]['F'],F_EPS) for cid in cp_matches]),'BKS_hits':len(hits)})
            if method in METHODS[3:]:
                subset=[r for r in exact if r['method']==method and r['case_id'] in cases and r['feasible']]
                proven=lambda r,p:bool(r[f'pass{p}_executed']) and bool(r[f'pass{p}_optimal']) and r[f'pass{p}_status']=='OPTIMAL'
                proofs.append({'Scale':scale,'Method':method,'Cost_optimal':sum(proven(r,1) for r in subset),'Lexicographic_optimal':sum(all(proven(r,p) for p in (1,2,3)) for r in subset)})
    write_csv(args.output,'Table6',table6);write_csv(args.output,'Table7',table7);write_csv(args.output,'Gurobi_optimality',proofs)
    metadata=records(args.instances/'Benchmark_Metadata.xlsx');by_scale=defaultdict(list)
    for row in metadata:by_scale[row['scale']].append(row)
    table4=[]
    for scale in SCALES[:3]:
        rs=by_scale[scale]
        span=lambda key:str(min(r[key] for r in rs))+'-'+str(max(r[key] for r in rs))
        table4.append({'Scale':scale,'Instances':len(rs),'Suppliers':span('supplier_count'),'Total_items':span('item_count'),'Mandatory_items':span('mandatory_item_count'),'Optional_racks':span('optional_rack_count')})
    write_csv(args.output,'Table4',table4);write_csv(args.output,'Table5',read('Parameter_Settings'))
    variants=defaultdict(lambda:defaultdict(list))
    for row in ablation:variants[row['variant']][row['case_id']].append(row)
    variants['Full']=groups['ILS'];panel_a=[];panel_b=[]
    for scale in SCALES:
        cases=[cid for cid in sorted(scales) if scale=='overall' or scales[cid]==scale]
        for variant in ['Full','NoInitialDescent','NoRouteRepair']:
            averages=[];best_gaps=[];deviations=[]
            for cid in cases:
                values=variants[variant][cid];averages.append(cost_gap(mean([r['C'] for r in values]),bks[cid]['C']))
                best_gaps.append(cost_gap(best(values)['C'],bks[cid]['C']));deviations.append(100*statistics.stdev([r['C'] for r in values])/bks[cid]['C'])
            differences=[mean([r['C'] for r in variants[variant][cid]])-mean([r['C'] for r in variants['Full'][cid]]) for cid in cases]
            panel_a.append({'Scale':scale,'Variant':variant,'Gap_Avg_pct':mean(averages),'Gap_Best_pct':mean(best_gaps),'Std_pct':mean(deviations),
              'Wins_vs_Full':sum(v < -float(EPS) for v in differences),'Ties_vs_Full':sum(abs(v)<=float(EPS) for v in differences),'Losses_vs_Full':sum(v > float(EPS) for v in differences)})
        matched=[cid for cid in cases if close(best(variants['Full'][cid])['C'],best(variants['NoProfitIntensification'][cid])['C'])]
        profits=[(best(variants['Full'][cid])['P'],best(variants['NoProfitIntensification'][cid])['P']) for cid in matched]
        panel_b.append({'Scale':scale,'Matched_cases':len(matched),'Mean_profit_Full':mean([p for p,q in profits]),
          'Mean_profit_NoProfitIntensification':mean([q for p,q in profits]),
          'Full_wins':sum(p>q+float(EPS) for p,q in profits),'Ties':sum(close(p,q) for p,q in profits),'Full_losses':sum(p<q-float(EPS) for p,q in profits)})
    write_csv(args.output,'Table8_A',panel_a);write_csv(args.output,'Table8_B',panel_b)
    if args.figures:
        convergence=convergence_grid(read('Convergence_Events'));acc=defaultdict(list)
        for r in convergence:acc[(r['scale'],r['t_s'])].append(r)
        trajectories=[{'scale':scale,'t_s':t,'runs':len(rs),'C':mean([r['C'] for r in rs if r['C'] is not None]),'P':mean([r['P'] for r in rs if r['P'] is not None]),'F':mean([r['F'] for r in rs if r['F'] is not None])} for (scale,t),rs in sorted(acc.items())]
        write_csv(args.output,'Figures4_5_6',trajectories)
        timings=read('Efficiency_B');samples=defaultdict(list)
        for r in timings:samples[(r['n'],r['Instance'])].append(r['Construction time (ms)'])
        per_n=defaultdict(list)
        for (n,i),values in samples.items():assert len(values)==10;per_n[n].append(statistics.median(values))
        write_csv(args.output,'Figure7',[{'n':n,'instances':len(v),'mean_construction_ms':mean(v)} for n,v in sorted(per_n.items())])
        efficiency=read('Efficiency_C')
        write_csv(args.output,'Skyline_time_ratio',[{'Scale':s,'Runs':len(rs),'Mean_Skyline_ratio_pct':mean([r['Skyline ratio (%)'] for r in rs])} for s in SCALES[:3] for rs in [[r for r in efficiency if str(r['Scale']).lower()==s]]])
        sensitivity=read('Sensitivity');sg=defaultdict(list)
        for r in sensitivity:
            sg[(r['case_id'],r['scenario'])].append(r)
        data=[]
        for factor,levels,metric in [('SC',[60,70,80,90,100],'C'),('OD',[60,80,100,120,140],'P'),('DT',[60,80,100,120,140],'F')]:
            for scale in SCALES[:3]:
                ids=sorted({r['case_id'] for r in sensitivity if r['scale']==scale})
                for level in levels:
                    values=[]
                    for cid in ids:
                        scenario='BASE' if level==100 else factor+str(level)
                        v=mean([r[metric] for r in sg[(cid,scenario)]]);base=mean([r[metric] for r in sg[(cid,'BASE')]])
                        values.append(v if factor=='OD' else 100*(v-base)/base)
                    data.append({'factor':factor,'scale':scale,'level':level,'metric':metric,'mean':mean(values),'cases':len(values)})
        write_csv(args.output,'Figure8',data)
    workbook.close()
    summary={'comparison_runs':len(algorithms),'ablation_runs':len(ablation),'gurobi_records':len(exact),'overall_ILS':next(r for r in table6 if r['Scale']=='overall' and r['Method']=='ILS'),
      'overall_ILS_lexicographic':next(r for r in table7 if r['Scale']=='overall' and r['Method']=='ILS'),'profit_intensification':panel_b[-1]}
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf8');print(json.dumps(summary,indent=2))

if __name__=='__main__':main()
