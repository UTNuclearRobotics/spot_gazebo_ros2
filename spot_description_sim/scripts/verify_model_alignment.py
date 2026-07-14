#!/usr/bin/env python3
"""verify_model_alignment.py

Consistency checks between the artifacts that define Spot in simulation:

  1. URDF vs SDF: for every link present in both the expanded URDF and the
     expanded SDF, compare the model-frame pose at zero joint configuration
     (URDF pose computed by forward kinematics through the joint chain; SDF
     link poses read directly, honoring relative_to).
  2. Optionally, old-vs-new SDF: structural comparison of two SDF files
     (links, joints, sensors, plugins and their numeric content) to audit a
     migration, printing every semantic difference.

Usage:
  verify_model_alignment.py --urdf model.urdf --sdf model.sdf [--tol 1e-4]
  verify_model_alignment.py --diff-sdf old_model.sdf new_model.sdf

Exit code 0 iff all comparisons pass within tolerance.
"""
import argparse
import math
import sys
import xml.etree.ElementTree as ET

import numpy as np


def rpy_to_mat(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def make_tf(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = rpy_to_mat(*rpy)
    T[:3, 3] = xyz
    return T


def parse_floats(s, n, default=0.0):
    vals = [float(v) for v in s.split()] if s else []
    while len(vals) < n:
        vals.append(default)
    return vals[:n]


def urdf_link_poses(urdf_path):
    """Model-frame pose of every URDF link at zero configuration."""
    root = ET.parse(urdf_path).getroot()
    joints = []
    children = set()
    for j in root.findall('joint'):
        parent = j.find('parent').get('link')
        child = j.find('child').get('link')
        origin = j.find('origin')
        xyz = parse_floats(origin.get('xyz') if origin is not None else None, 3)
        rpy = parse_floats(origin.get('rpy') if origin is not None else None, 3)
        joints.append((parent, child, make_tf(xyz, rpy)))
        children.add(child)
    links = [l.get('name') for l in root.findall('link')]
    roots = [l for l in links if l not in children]
    if len(roots) != 1:
        print(f'URDF root ambiguity: {roots}')
    poses = {roots[0]: np.eye(4)}
    pending = list(joints)
    while pending:
        progressed = False
        rest = []
        for parent, child, T in pending:
            if parent in poses:
                poses[child] = poses[parent] @ T
                progressed = True
            else:
                rest.append((parent, child, T))
        pending = rest
        if not progressed:
            print(f'URDF: unreachable joints: {[(p, c) for p, c, _ in pending]}')
            break
    return poses


def sdf_link_poses(sdf_path):
    """Model-frame pose of every SDF link (poses are model-frame or relative_to)."""
    model = ET.parse(sdf_path).getroot().find('model')
    raw = {}
    for l in model.findall('link'):
        pose_el = l.find('pose')
        vals = parse_floats(pose_el.text if pose_el is not None else None, 6)
        rel = pose_el.get('relative_to') if pose_el is not None else None
        raw[l.get('name')] = (make_tf(vals[:3], vals[3:]), rel)
    poses = {}
    def resolve(name, stack=()):
        if name in poses:
            return poses[name]
        T, rel = raw[name]
        if rel and rel not in ('__model__',):
            if name in stack:
                raise RuntimeError(f'relative_to cycle at {name}')
            T = resolve(rel, stack + (name,)) @ T
        poses[name] = T
        return T
    for name in raw:
        resolve(name)
    return poses


def compare_poses(urdf_path, sdf_path, tol):
    up = urdf_link_poses(urdf_path)
    sp = sdf_link_poses(sdf_path)
    shared = sorted(set(up) & set(sp))
    only_urdf = sorted(set(up) - set(sp))
    only_sdf = sorted(set(sp) - set(up))
    print(f'links: urdf={len(up)} sdf={len(sp)} shared={len(shared)}')
    if only_urdf:
        print(f'  URDF-only (TF-only frames, expected): {only_urdf}')
    if only_sdf:
        print(f'  SDF-only  (physics-only bodies, expected): {only_sdf}')
    failures = 0
    for name in shared:
        dp = np.linalg.norm(up[name][:3, 3] - sp[name][:3, 3])
        dR = np.linalg.norm(up[name][:3, :3] - sp[name][:3, :3])
        status = 'OK ' if (dp <= tol and dR <= 10 * tol) else 'FAIL'
        if status == 'FAIL':
            failures += 1
            print(f'  {status} {name}: |dpos|={dp:.6g} m, |dR|={dR:.6g}')
    print(f'pose comparison: {len(shared) - failures}/{len(shared)} within {tol} m')
    return failures == 0


def canon(el):
    """Canonical representation of an XML element for structural diff."""
    def norm_text(t):
        t = (t or '').strip()
        if not t:
            return ''
        try:
            parts = t.split()
            return ' '.join(f'{float(p) + 0.0:.6g}' for p in parts)
        except ValueError:
            return ' '.join(t.split())
    kids = sorted(canon(c) for c in el)
    attrs = sorted((k, v) for k, v in el.attrib.items())
    return (el.tag, tuple(attrs), norm_text(el.text), tuple(kids))


def diff_sdf(old_path, new_path):
    old = ET.parse(old_path).getroot().find('model')
    new = ET.parse(new_path).getroot().find('model')
    ok = True
    for tag in ('link', 'joint', 'plugin'):
        o = {e.get('name'): e for e in old.findall(tag)}
        n = {e.get('name'): e for e in new.findall(tag)}
        for name in sorted(set(o) | set(n)):
            if name not in n:
                print(f'- {tag} "{name}" removed'); ok = False
            elif name not in o:
                print(f'+ {tag} "{name}" added'); ok = False
            elif canon(o[name]) != canon(n[name]):
                ok = False
                print(f'~ {tag} "{name}" changed:')
                ok_children = describe_child_diff(o[name], n[name])
    print('structural diff:', 'IDENTICAL' if ok else 'differences above')
    return ok


def describe_child_diff(o, n, indent='    '):
    o_kids = {}
    n_kids = {}
    for e in o:
        o_kids.setdefault((e.tag, e.get('name')), []).append(e)
    for e in n:
        n_kids.setdefault((e.tag, e.get('name')), []).append(e)
    for key in sorted(set(o_kids) | set(n_kids), key=str):
        a = [canon(x) for x in o_kids.get(key, [])]
        b = [canon(x) for x in n_kids.get(key, [])]
        if a != b:
            print(f'{indent}{key}:')
            print(f'{indent}  old: {a}')
            print(f'{indent}  new: {b}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--urdf')
    ap.add_argument('--sdf')
    ap.add_argument('--tol', type=float, default=1e-4)
    ap.add_argument('--diff-sdf', nargs=2, metavar=('OLD', 'NEW'))
    args = ap.parse_args()
    ok = True
    if args.urdf and args.sdf:
        ok &= compare_poses(args.urdf, args.sdf, args.tol)
    if args.diff_sdf:
        ok &= diff_sdf(*args.diff_sdf)
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
