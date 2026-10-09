"""Export a localization-compatible manifest and a readable overview."""
from common import *
from icp_localization.registration import sha256, load_manifest
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    poses = np.load(OUT/'optimized_poses_enu.npy')
    base_imu = np.eye(4)
    base_imu[:3, :3] = BASE_R
    base_imu[:3, 3] = GEO['imu']
    anchors = poses @ np.linalg.inv(base_imu)
    np.save(OUT/'base_anchors.npy', anchors)
    manifest = dict(schema=1, projection=CONFIG['projection'],
        points=dict(path='map_enu_xyz_intensity.npy', sha256=sha256(OUT/'map_enu_xyz_intensity.npy')),
        base_anchors=dict(path='base_anchors.npy', sha256=sha256(OUT/'base_anchors.npy')),
        source=dict(imu_poses='optimized_poses_enu.npy', imu_poses_sha256=sha256(OUT/'optimized_poses_enu.npy'),
                    hardware='hardware.yaml', hardware_sha256=sha256(OUT/'hardware.yaml'),
                    bag=CONFIG['bag']['directory']),
        height_reference='Relative LiDAR height, first base height zero; not GNSS altitude')
    (OUT/'map_manifest.json').write_text(json.dumps(manifest, indent=2))
    load_manifest(OUT/'map_manifest.json', CONFIG['projection'])
    points = np.load(OUT/'map_enu_xyz_intensity.npy', mmap_mode='r')
    factors = json.loads((OUT/'gnss_factors.json').read_text())
    summary = json.loads((OUT/'map_summary.json').read_text())
    optimization = json.loads((OUT/'optimization.json').read_text())[str(CONFIG['sigma_xy_m'])]
    summary.update(nodes=len(poses), join_factors=optimization['join_edges'],
                   fit_residual_m=optimization['GNSS_factor_difference_after_m'],
                   independent_evaluation=False)
    (OUT/'map_summary.json').write_text(json.dumps(summary, indent=2))
    sample = points[::max(1, len(points)//200000)]
    fig, ax = plt.subplots(figsize=(10, 7), layout='constrained')
    ax.scatter(sample[:, 0], sample[:, 1], c=sample[:, 2], cmap='Greys', s=.15, alpha=.35, rasterized=True)
    ax.plot(anchors[:, 0, 3], anchors[:, 1, 3], color='#d73450', lw=1.1, label='FIX + ICP graph trajectory', zorder=3)
    fixes = np.array([x['gnss_enu'] for x in factors])
    ax.scatter(fixes[:, 0], fixes[:, 1], s=8, color='#00a17c', label='Selected FIX constraints', zorder=4)
    ax.set(xlabel='East [m]', ylabel='North [m]', title=f"FIX + ICP map | {len(poses):,} poses | {len(factors)} FIX | {summary['loop_factors']} loops")
    ax.set_aspect('equal'); ax.grid(alpha=.2); ax.legend(loc='best')
    fig.savefig(OUT/'overview.png', dpi=150)
    plt.close(fig)


if __name__ == '__main__':
    main()
