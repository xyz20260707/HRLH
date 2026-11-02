import argparse,json,subprocess,sys,tempfile
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--build',type=Path,default=ROOT/'build')
    parser.add_argument('--seconds',type=float,default=3)
    parser.add_argument('--exact',action='store_true')
    args=parser.parse_args()
    with tempfile.TemporaryDirectory(prefix='hrlh_smoke_') as directory:
        for algorithm in ['ILS','ALNS','HGS']+(['Gurobi'] if args.exact else []):
            output=Path(directory)/(algorithm+'.json')
            command=[sys.executable,'-B','-X','utf8',str(ROOT/'scripts/run_solver.py'),'--instance',str(ROOT/'Instances/Example_Instance.json'),'--algorithm',algorithm,'--seconds',str(args.seconds),'--seed','205062346','--build',str(args.build.resolve()),'--output',str(output)]
            subprocess.run(command,check=True,timeout=max(150,args.seconds+90))
            result=json.loads(output.read_text(encoding='utf8'))
            assert all(k in result for k in ['feasible','objective','runtime','stop_reason','cost_components'])
            assert result['runtime']>=0
            if algorithm!='Gurobi':assert result['feasible'] and result['validation_ok']
            if result['feasible']:
                assert abs(sum(result['cost_components'].values())-result['objective']['C'])<1e-5
            print(algorithm,'smoke passed')

if __name__=='__main__':main()
