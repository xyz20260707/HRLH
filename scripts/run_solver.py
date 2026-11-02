import argparse,configparser,json,math,os,subprocess,tempfile
from pathlib import Path
from convert_instances import protocol

ROOT=Path(__file__).resolve().parents[1]

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--instance',type=Path,required=True)
    parser.add_argument('--algorithm',choices=['ILS','ALNS','HGS','Gurobi'],default='ILS')
    parser.add_argument('--config',type=Path,default=ROOT/'config/hrlh.ini')
    parser.add_argument('--build',type=Path,default=ROOT/'build')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--seed',type=int)
    parser.add_argument('--seconds',type=float)
    parser.add_argument('--pass-limits',nargs=3,type=float)
    args=parser.parse_args()
    if args.output.exists():parser.error('Output exists; choose a new path')
    suffix='.exe' if os.name=='nt' else ''
    stem='hrlh_exact' if args.algorithm=='Gurobi' else 'hrlh_'+args.algorithm.lower()
    candidates=[args.build/'Release'/(stem+suffix),args.build/(stem+suffix)]
    executable=next((p.resolve() for p in candidates if p.is_file()),None)
    if executable is None:parser.error('Executable not found; compile first')
    config=configparser.ConfigParser();config.read(args.config,encoding='utf-8-sig')
    if args.seed is not None:config.set('global','seed',str(args.seed))
    if args.seconds is not None:
        if not math.isfinite(args.seconds) or args.seconds<=0:parser.error('Time budget must be finite and positive')
        config.set('budget','max_wall_seconds',str(args.seconds))
    seconds=float(config.get('budget','max_wall_seconds'))
    environment=os.environ.copy()
    environment.pop('HRLH_EXACT_PASS_LIMITS',None)
    environment.update(OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1')
    sdk=environment.get('GUROBI_HOME')
    if sdk:environment['PATH']=str(Path(sdk)/'bin')+os.pathsep+environment.get('PATH','')
    if args.pass_limits:
        if args.algorithm!='Gurobi' or any(not math.isfinite(v) or v<=0 for v in args.pass_limits):parser.error('Pass limits require Gurobi and three finite positive values')
        environment['HRLH_EXACT_PASS_LIMITS']=' '.join(map(str,args.pass_limits))
    case=json.loads(args.instance.read_text(encoding='utf8'))
    with tempfile.TemporaryDirectory(prefix='hrlh_') as directory:
        folder=Path(directory)
        (folder/'instance.dat').write_text(protocol(case),encoding='ascii')
        with (folder/'solver.ini').open('w',encoding='utf8') as stream:config.write(stream)
        cli=[str(executable),'instance.dat',str(seconds) if args.algorithm=='Gurobi' else 'solver.ini','result.json']
        process=subprocess.run(cli,cwd=folder,env=environment,capture_output=True,text=True,errors='replace',timeout=max(120,seconds+60))
        if process.returncode:
            raise RuntimeError('Native solver failed: '+process.stderr.strip())
        result=json.loads((folder/'result.json').read_text(encoding='utf8'))
    for field in ['feasible','objective','runtime','stop_reason']:
        if field not in result:raise RuntimeError('Missing output field: '+field)
    result['case_id']=case['case_id']
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,ensure_ascii=True,indent=2,allow_nan=False),encoding='utf8')
    print(json.dumps({k:result[k] for k in ('case_id','feasible','objective','runtime','stop_reason')}))

if __name__=='__main__':main()
