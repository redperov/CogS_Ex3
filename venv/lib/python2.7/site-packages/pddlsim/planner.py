
try:
    import urllib.request as urllib2
except Exception:
    import urllib2
import os, json, sys

def online(domain_path, problem_path):
    data = {'domain': open(domain_path, 'r').read(),
            'problem': open(problem_path, 'r').read()}

    req = urllib2.Request('http://solver.planning.domains/solve')
    req.add_header('Content-Type', 'application/json')
    resp = json.loads(urllib2.urlopen(req, json.dumps(data).encode('utf-8')).read().decode('utf-8'))
    return [act['name'] for act in resp['result']['plan']]

def local(domain_path, problem_path,out_path = 'tmp.ipc'):    
    planner_path = "\"" + os.path.join(os.path.dirname( sys.modules[__name__].__file__),'external/siw-then-bfsf') + "\""
    
    print planner_path
    os.system(planner_path +' --domain ' + domain_path + ' --problem ' + problem_path + ' --output ' + out_path)
    with open(out_path) as f:
        return [line for line in f.read().split('\n') if line.rstrip()]

use_local = True

if use_local:
    make_plan = local
else:
    make_plan = online


if __name__ == '__main__':
    plan = make_plan(sys.argv[1],sys.argv[2])
    with open(sys.argv[3], 'w') as f:
        f.write('\n'.join(plan))
    
 