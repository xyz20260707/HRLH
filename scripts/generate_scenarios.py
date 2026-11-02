import argparse,collections,copy,hashlib,json,math,random
from pathlib import Path
from convert_instances import load_instances,protocol

LEVELS=("BASE","SC60","SC70","SC80","SC90","OD60","OD80","OD120","OD140","DT60","DT80","DT120","DT140")

def digest(data):
    return hashlib.sha256(data).hexdigest()

def seed_for(master, *parts):
    return int(digest(json.dumps([master, *parts], ensure_ascii=False).encode())[:16], 16)

def rounded(n, percent):
    return (int(n) * int(percent) + 50) // 100

def canonical(it):
    return (min(it[2], it[3]), max(it[2], it[3]))

def candidate(a, b):
    return a[0] != b[0] and (not a[5]) and (not b[5]) and (a[1] == b[1]) and (canonical(a) == canonical(b))

def parse_protocol(data):
    lines = data.decode('utf-8-sig').splitlines()
    h = lines[0].split()
    assert len(h) == 10
    p = {'vehicle': [int(x) for x in h[:3]] + [float(x) for x in h[3:5]], 'vehicle_count': int(h[6])}
    ns, ni, na, nq = map(int, (h[5], h[7], h[8], h[9]))
    pos = 1
    p['suppliers'] = [[int(a), float(b), float(c)] for a, b, c in (x.split() for x in lines[pos:pos + ns])]
    pos += ns
    p['items'] = [list(map(int, x.split())) for x in lines[pos:pos + ni]]
    pos += ni
    p['arcs'] = [[int(a), int(b), float(c), float(d)] for a, b, c, d in (x.split() for x in lines[pos:pos + na])]
    pos += na
    p['q'] = [list(map(int, x.split())) for x in lines[pos:pos + nq]]
    pos += nq
    assert all((not x.strip() for x in lines[pos:])) and all((len(i) == 11 for i in p['items']))
    validate_protocol(p)
    return p

def validate_protocol(p):
    ids = [i[0] for i in p['items']]
    assert len(ids) == len(set(ids)), 'duplicate item ID'
    by = {i[0]: i for i in p['items']}
    ss = {s[0] for s in p['suppliers']}
    assert p['vehicle_count'] >= 1 and all((x > 0 for x in p['vehicle'][:3]))
    assert all((len(i) == 11 and i[1] in ss and (min(i[2:5]) > 0) and (i[7] >= 0) and (not (i[5] and i[6])) for i in by.values()))
    q = {(a, b): v for a, b, v in p['q']}
    assert len(q) == len(p['q'])
    for (a, b), v in q.items():
        assert a != b and a in by and (b in by) and (v in (0, 1))
        assert q.get((b, a), 0) == v, 'asymmetric compatibility'
        if v:
            assert candidate(by[a], by[b]), 'illegal q=1'
    assert all((math.isfinite(v) and v >= 0 for s in p['suppliers'] for v in s[1:]))

def balanced_order(values, key, rng):
    groups = collections.defaultdict(list)
    for value in values:
        groups[key(value)].append(value)
    ranked = []
    for group in sorted(groups, key=str):
        members = groups[group]
        rng.shuffle(members)
        for n, x in enumerate(members):
            ranked.append(((n + 0.5) / len(members), rng.random(), x))
    return [x for _, __, x in sorted(ranked, key=lambda t: t[:2])]

def extend_optional(p, wb, master, case):
    original = [i for i in p['items'] if i[6]]
    n = len(original)
    if not n:
        raise ValueError('No optional-rack templates; OD expansion cannot be constructed')
    raw = {int(x['item_id']): x for x in wb['items']}
    groups = collections.defaultdict(list)
    for i in original:
        groups[i[1]].append(i)
    rng = random.Random(seed_for(master, case, 'OD-templates'))
    for g in sorted(groups):
        rng.shuffle(groups[g])
    ties = {g: rng.random() for g in groups}
    used = collections.Counter()
    added = []
    mapping = {}
    next_id = max((i[0] for i in p['items'])) + 1
    for k in range(rounded(n, 140) - n):
        supplier = max(groups, key=lambda g: ((k + 1) * len(groups[g]) / n - used[g], ties[g]))
        template = groups[supplier][used[supplier] % len(groups[supplier])]
        used[supplier] += 1
        clone = list(template)
        clone[0] = next_id
        next_id += 1
        raw_clone = copy.deepcopy(raw[template[0]])
        raw_clone['item_id'] = clone[0]
        raw[clone[0]] = raw_clone
        mapping[clone[0]] = template[0]
        added.append(clone)
    all_items = p['items'] + added
    q = {(a, b): v for a, b, v in p['q']}
    prob = recover_rule(wb, p)['probability']
    additions = []
    added_ids = {a[0] for a in added}
    for index, a in enumerate(all_items):
        for b in all_items[index + 1:]:
            if not (a[0] in added_ids or b[0] in added_ids) or not candidate(a, b):
                continue
            draw = random.Random(seed_for(master, case, 'OD-new-pair', *sorted((a[0], b[0])))).random()
            value = int(int(raw[a[0]]['max_stack_layers']) > 1 and int(raw[b[0]]['max_stack_layers']) > 1 and (draw < prob))
            q[a[0], b[0]] = q[b[0], a[0]] = value
            additions.append({'item_m': a[0], 'item_b': b[0], 'draw': draw, 'probability': prob, 'q': value, 'max_layers_m': int(raw[a[0]]['max_stack_layers']), 'max_layers_b': int(raw[b[0]]['max_stack_layers'])})
    return (added, raw, mapping, q, additions)

def generate(p, wb, master, case):
    original_q = {(a, b): v for a, b, v in p['q']}
    by = {i[0]: i for i in p['items']}
    positive = [(a, b) for (a, b), v in original_q.items() if a < b and v]
    order = balanced_order(positive, lambda ab: (by[ab[0]][1], canonical(by[ab[0]])), random.Random(seed_for(master, case, 'SC')))
    optional = [i for i in p['items'] if i[6]]
    opt_order = balanced_order(optional, lambda i: (i[1], canonical(i), i[4], i[7]), random.Random(seed_for(master, case, 'OD-subset')))
    added, raw, templates, expanded_q, new_pair_evidence = extend_optional(p, wb, master, case)
    scenarios = {'BASE': copy.deepcopy(p)}
    meta = {}
    for percent in (60, 70, 80, 90):
        x = copy.deepcopy(p)
        keep = set(order[:rounded(len(order), percent)])
        x['q'] = [[a, b, v if tuple(sorted((a, b))) in keep else 0] for a, b, v in p['q']]
        scenarios[f'SC{percent}'] = x
        meta[f'SC{percent}'] = {'original_positive_pairs': len(positive), 'retained_positive_pairs': len(keep), 'actual_retention_rate': len(keep) / len(positive) if positive else None}
    for percent in (60, 80, 120, 140):
        x = copy.deepcopy(p)
        wanted = rounded(len(optional), percent)
        if percent < 100:
            selected = {i[0] for i in opt_order[:wanted]}
            x['items'] = [i for i in p['items'] if not i[6] or i[0] in selected]
            q = original_q
        else:
            x['items'] += copy.deepcopy(added[:wanted - len(optional)])
            q = expanded_q
        ids = {i[0] for i in x['items']}
        x['q'] = [[a, b, v] for (a, b), v in sorted(q.items()) if a in ids and b in ids]
        scenarios[f'OD{percent}'] = x
        counts = collections.Counter((i[1] for i in x['items'] if i[6]))
        base_counts = collections.Counter((i[1] for i in optional))
        meta[f'OD{percent}'] = {'original_optional_count': len(optional), 'actual_optional_count': wanted, 'actual_count_ratio': wanted / len(optional), 'supplier_original_counts': dict(base_counts), 'supplier_actual_counts': dict(counts)}
    arcs = {(a, b): t for a, b, _, t in p['arcs']}
    end = len(p['suppliers']) + 1
    for percent in (60, 80, 120, 140):
        x = copy.deepcopy(p)
        solo = {}
        for s in x['suppliers']:
            i, service, deadline = s
            minimum = arcs[0, i] + service + arcs[i, end]
            if deadline < minimum:
                raise ValueError(f'Negative original solo slack for supplier {i}')
            s[2] = minimum + percent / 100 * (deadline - minimum)
            solo[i] = minimum
        scenarios[f'DT{percent}'] = x
        meta[f'DT{percent}'] = {'solo_return_min': solo, 'earliest_departure': 0, 'time_unit': 'minute'}
    evidence = {'rule': recover_rule(wb, p), 'templates': templates, 'new_pair_evidence': new_pair_evidence, 'expanded_raw_items': [raw[i[0]] for i in added], 'scenario_metrics': meta, 'optional_selection_order': [i[0] for i in opt_order], 'compatibility_edge_order': order}
    validate_family(p, scenarios, evidence)
    return (scenarios, evidence)

def validate_family(base, scenarios, evidence):
    assert set(scenarios) == set(LEVELS)
    mandatory = [i for i in base['items'] if not i[6]]
    original = {i[0]: i for i in base['items']}
    original_q = {(a, b): v for a, b, v in base['q']}
    original_optional = sum((bool(i[6]) for i in base['items']))
    for name, p in scenarios.items():
        validate_protocol(p)
        assert p['vehicle'] == base['vehicle'] and p['vehicle_count'] == base['vehicle_count'] and (p['arcs'] == base['arcs'])
        assert [i for i in p['items'] if not i[6]] == mandatory
        assert [s[:2] for s in p['suppliers']] == [s[:2] for s in base['suppliers']]
        if not name.startswith('DT'):
            assert p['suppliers'] == base['suppliers']
        if not name.startswith('OD'):
            assert p['items'] == base['items']
        else:
            assert sum((bool(i[6]) for i in p['items'])) == rounded(original_optional, int(name[2:]))
        for i in p['items']:
            if i[0] in original:
                assert i == original[i[0]]
            else:
                mapping = evidence['templates']
                template = original[int(mapping.get(i[0], mapping.get(str(i[0]))))]
                assert i[1:] == template[1:] and i[6] and (not i[5])
        if not name.startswith('SC'):
            q = {(a, b): v for a, b, v in p['q']}
            ids = {i[0] for i in p['items']}
            assert {(a, b): v for (a, b), v in q.items() if a in original and b in original} == {(a, b): v for (a, b), v in original_q.items() if a in ids and b in ids}
    edge = lambda p: {(a, b) for a, b, v in p['q'] if a < b and v}
    family = [edge(scenarios[n]) for n in ('SC60', 'SC70', 'SC80', 'SC90', 'BASE')]
    assert all((a <= b for a, b in zip(family, family[1:])))
    family = [{i[0] for i in scenarios[n]['items'] if i[6]} for n in ('OD60', 'OD80', 'BASE', 'OD120', 'OD140')]
    assert all((a <= b for a, b in zip(family, family[1:])))
    lower = scenarios['OD120']
    upper = scenarios['OD140']
    ids = {i[0] for i in lower['items']}
    assert lower['q'] == [[a, b, v] for a, b, v in upper['q'] if a in ids and b in ids]
    added = {i[0]: i for i in upper['items'] if i[0] not in original}
    raw = {int(i['item_id']): i for i in evidence['expanded_raw_items']}
    assert set(raw) == set(added)
    expected = {tuple(sorted((a[0], b[0]))) for n, a in enumerate(upper['items']) for b in upper['items'][n + 1:] if (a[0] in added or b[0] in added) and candidate(a, b)}
    seen = set()
    q = {(a, b): v for a, b, v in upper['q']}
    probability = evidence['rule']['probability']
    assert 0 <= probability <= 1
    for pair in evidence['new_pair_evidence']:
        a, b = (int(pair['item_m']), int(pair['item_b']))
        key = tuple(sorted((a, b)))
        assert key in expected and key not in seen
        seen.add(key)
        draw = pair['draw']
        assert 0 <= draw < 1 and pair['probability'] == probability
        for ident, field in ((a, 'max_layers_m'), (b, 'max_layers_b')):
            if ident in raw:
                assert pair[field] == int(raw[ident]['max_stack_layers'])
        value = int(pair['max_layers_m'] > 1 and pair['max_layers_b'] > 1 and (draw < probability))
        assert pair['q'] == q[a, b] == q[b, a] == value
    assert seen == expected
    deadlines = [[s[2] for s in scenarios[n]['suppliers']] for n in ('DT60', 'DT80', 'BASE', 'DT120', 'DT140')]
    assert all((all((x <= y for x, y in zip(a, b))) for a, b in zip(deadlines, deadlines[1:])))
    return True

def recover_rule(workbook,problem):
    probability=workbook['probability']
    assert isinstance(probability,(int,float)) and 0<=probability<=1
    raw={i['item_id']:i for i in workbook['items']}
    items={i[0]:i for i in problem['items']}
    for a,b,value in problem['q']:
        if value:
            assert candidate(items[a],items[b])
            assert raw[a]['max_stack_layers']>1 and raw[b]['max_stack_layers']>1
    return {'probability':probability}

def scenario_cases(base,master=2026102501):
    problem=parse_protocol(protocol(base).encode())
    recorded={'probability':base['compatibility_probability'],'items':[{'item_id':i['item_id'],'max_stack_layers':int(i['max_stack_layers'])} for i in base['items']]}
    family,evidence=generate(problem,recorded,master,base['case_id'])
    originals={i['item_id']:i for i in base['items']}
    output={}
    for name,value in family.items():
        case=copy.deepcopy(base)
        case['scenario']=name
        case['items']=[]
        for item in value['items']:
            template=item[0] if item[0] in originals else evidence['templates'][item[0]]
            clone=copy.deepcopy(originals[template]);clone['item_id']=item[0]
            case['items'].append(clone)
        for source,target in zip(value['suppliers'],case['suppliers']):
            assert source[0]==target['node_id']
            target['delivery_deadline_min']=source[2]
        q={(a,b):v for a,b,v in value['q']}
        case['stack_pairs']=[{'item_m':a,'item_b':b,'q_mb':q[a,b],'q_bm':q[b,a]} for a,b in sorted(q) if a<b]
        case['item_count']=len(case['items'])
        case['optional_rack_count']=sum(i['is_optional'] for i in case['items'])
        assert parse_protocol(protocol(case).encode())==value
        output[name]=case
    return output

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--instances',type=Path,default=Path(__file__).resolve().parents[1]/'Instances')
    parser.add_argument('--case',required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():parser.error('Choose a new output directory')
    base=load_instances(args.instances)[args.case]
    family=scenario_cases(base)
    args.output.mkdir(parents=True)
    for name,case in family.items():(args.output/(name+'.json')).write_text(json.dumps(case,indent=2,ensure_ascii=True),encoding='utf8')
    print('Generated 13 nested scenarios for',args.case)

if __name__=='__main__':main()
