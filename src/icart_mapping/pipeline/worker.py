"""Run every mapping stage in the isolated job; never start ROS nodes."""
import hashlib
import json
import os
from pathlib import Path
import socket
import shutil
import subprocess
import sys
import time

from icart_mapping.job import atomic_json


def main():
    config = json.loads(Path(os.environ['ICART_MAPPING_CONFIG']).read_text())
    root = Path(config['directory'])
    started = time.time()
    def status(state, phase, percent, **more):
        atomic_json(root/'status.json', dict(state=state, phase=phase, percent=percent,
                                           elapsed_s=time.time()-started, **more))
    try:
        assert {n for _, n in socket.if_nameindex()} == {'lo'}, 'Network isolation missing'
        status('running', '入力と依存ライブラリの確認', 1)
        metadata = Path(config['bag']['directory'])/'metadata.yaml'
        if hashlib.sha256(metadata.read_bytes()).hexdigest() != config['bag']['metadata_sha256']:
            raise ValueError('入力bagのmetadataが選択後に変更されました')
        sys.path.insert(0, config['backend'])
        import gtsam, small_gicp, numpy, scipy
        scripts = root/'reproduction'
        scripts.mkdir()
        for path in Path(__file__).parent.glob('*.py'):
            shutil.copyfile(path, scripts/path.name)
        atomic_json(root/'code_hashes.json', {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                                           for p in scripts.glob('*.py')})
        hashes = {}
        for item in config['bag']['files']:
            path = Path(item['path'])
            stat = path.stat()
            if stat.st_size != item['size'] or stat.st_mtime_ns != item['mtime_ns']:
                raise ValueError('入力bagが選択後に変更されました')
            digest = hashlib.sha256()
            with path.open('rb') as stream:
                for block in iter(lambda: stream.read(8*1024*1024), b''):
                    digest.update(block)
            hashes[str(path)] = digest.hexdigest()
            print('INPUT', path.name, flush=True)
        atomic_json(root/'input_hashes.json', hashes)
        (root/'work').mkdir()
        stages = [('extract.py', 'bagから姿勢・FIX・IMUを抽出', 8),
                  ('build_keyframes.py', '点群の歪み補正・キーフレーム生成', 20),
                  ('solve_graph.py', 'ICP区間接続・ループ閉合', 40),
                  ('refine_loops.py', 'ループ候補の再確認', 65),
                  ('constrain_fix.py', 'FIX拘束を加えて最適化・地図生成', 80),
                  ('export.py', 'ICP用地図と俯瞰画像の出力', 95)]
        for filename, phase, percent in stages:
            status('running', phase, percent)
            print('STAGE', phase, flush=True)
            subprocess.run([sys.executable, '-u', str(scripts/filename)], check=True)
        for item in config['bag']['files']:
            stat = Path(item['path']).stat()
            if stat.st_size != item['size'] or stat.st_mtime_ns != item['mtime_ns']:
                raise ValueError('処理中に入力bagが変更されました。完成地図として扱いません')
        if hashlib.sha256(metadata.read_bytes()).hexdigest() != config['bag']['metadata_sha256']:
            raise ValueError('処理中にbagのmetadataが変更されました')
        summary = json.loads((root/'map_summary.json').read_text())
        status('complete', '地図作成が完了しました', 100,
               manifest=str(root/'map_manifest.json'), overview=str(root/'overview.png'), summary=summary)
    except Exception as error:
        status('failed', '処理に失敗しました', 0, error=str(error))
        raise


if __name__ == '__main__':
    main()
