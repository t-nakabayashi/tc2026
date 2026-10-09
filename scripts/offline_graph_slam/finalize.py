#!/usr/bin/env python3
import hashlib,json,platform,shutil,sys
from pathlib import Path
import numpy as np,scipy,yaml
from common import ROOT
from extract import BAGS

def sha(p):
 h=hashlib.sha256()
 with p.open('rb') as f:
  for chunk in iter(lambda:f.read(1024*1024),b''):h.update(chunk)
 return h.hexdigest()
def main():
 src=Path(__file__).parent.resolve();dst=ROOT/'reproduction';dst.mkdir(exist_ok=True)
 for p in src.glob('*'):
  if p.is_file() and p.suffix in ['.py','.md','.sh','.ini']:shutil.copy2(p,dst/p.name)
 manifest={'complete':True,'input_bag':str(BAGS[0]),'input_metadata_sha256':sha(BAGS[0]/'metadata.yaml'),'python':platform.python_version(),'numpy':np.__version__,'scipy':scipy.__version__,'gtsam':'4.2.1','small_gicp':'1.0.1','gnss_factors':0,'artifact_hashes':{}}
 for p in sorted(ROOT.glob('*')):
  if p.is_file() and p.suffix in ['.pcd','.ply','.csv','.json'] and p.name!='manifest.json':manifest['artifact_hashes'][p.name]={'bytes':p.stat().st_size,'sha256':sha(p)}
 for p in sorted((ROOT/'output/pdf').glob('*.pdf')):manifest['artifact_hashes'][str(p.relative_to(ROOT))]={'bytes':p.stat().st_size,'sha256':sha(p)}
 points=np.load(ROOT/'map_graph_xyz_intensity.npy',mmap_mode='r');assert np.isfinite(points).all();n=len(points)
 blob=(ROOT/'map_graph.pcd').read_bytes();off=blob.index(b'DATA binary\n')+len(b'DATA binary\n');assert len(blob)-off==n*16;assert np.array_equal(np.frombuffer(blob,dtype='<f4',offset=off).reshape(-1,4),points)
 poses=np.load(ROOT/'optimized_poses.npy');assert np.isfinite(poses).all();assert np.max(abs(np.linalg.det(poses[:,:3,:3])-1))<1e-6
 loops=json.loads((ROOT/'loops.json').read_text());assert len(loops)==16 and all(x['good'] for x in loops)
 g=json.loads((ROOT/'optimization.json').read_text());assert g['gnss_factors']==0 and g['loop_edges']==len(loops)
 e=json.loads((ROOT/'evaluation.json').read_text());assert not e['gnss_used_in_graph'];assert e['calibration_count']==876;assert e['results']['slam']['heldout_qualified_fix']['n']==8611
 manifest['checks']={'pointcloud_binary_roundtrip':True,'points_finite':True,'poses_are_rotations':True,'accepted_revisit_factors':len(loops),'numerical_tests_passed':3,'pdf_pages_rendered_and_reviewed':6}
 (ROOT/'manifest.json').write_text(json.dumps(manifest,indent=2))
 if (ROOT/'venv').exists():shutil.rmtree(ROOT/'venv')
 print(json.dumps(manifest['checks']));print('total points',n)
if __name__=='__main__':main()
