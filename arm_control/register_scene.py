#!/usr/bin/env python3
"""Register CAD into base_link from measured landmarks; never approve hardware."""
import argparse
import json
import os
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def register(scene, measurements):
    cad = np.asarray(measurements['cad_points'], dtype=float)
    base = np.asarray(measurements['base_points_m'], dtype=float)
    units = measurements['cad_units']
    if units not in ('mm', 'm'):
        raise ValueError('Unités CAD mm ou m requises')
    if (cad.ndim != 2 or cad.shape[1] != 3 or cad.shape != base.shape or len(cad) < 3
            or not np.all(np.isfinite(cad)) or not np.all(np.isfinite(base))):
        raise ValueError('Au moins trois correspondances XYZ finies requises')
    cad = cad*(0.001 if units == 'mm' else 1)
    cad_mean, base_mean = np.mean(cad, axis=0), np.mean(base, axis=0)
    centered_cad, centered_base = cad-cad_mean, base-base_mean
    if (np.linalg.matrix_rank(centered_cad, tol=0.001) < 2
            or np.linalg.matrix_rank(centered_base, tol=0.001) < 2):
        raise ValueError('Les repères doivent être distincts et non alignés')
    rotation, _ = Rotation.align_vectors(centered_base, centered_cad)
    translation = base_mean-rotation.apply(cad_mean)
    residual = np.linalg.norm(rotation.apply(cad)+translation-base, axis=1)
    if np.max(residual) > 0.005:
        raise ValueError('Correspondances incohérentes : résidu supérieur à 5 mm')
    result = json.loads(json.dumps(scene))
    result['hardware_verified'] = False
    result['cad']['xyz_m'] = translation.tolist()
    result['cad']['rpy_rad'] = rotation.as_euler('xyz').tolist()
    result['cad']['registration_max_error_m'] = float(np.max(residual))
    return result


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scene', type=Path, required=True)
    p.add_argument('--measurements', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    try:
        result = register(json.loads(a.scene.read_text()), json.loads(a.measurements.read_text()))
        manifest = (a.scene.resolve().parent/result['cad']['manifest']).resolve()
        result['cad']['manifest'] = os.path.relpath(manifest, a.output.resolve().parent)
        with a.output.open('x') as f:
            f.write(json.dumps(result, indent=2)+'\n')
        print(f"Scène enregistrée ; résidu maximal {result['cad']['registration_max_error_m']*1000:.3f} mm ; validation matérielle à faire")
    except (ValueError, OSError, KeyError) as exc:
        p.exit(1, f'Erreur : {exc}\n')
