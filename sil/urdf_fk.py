"""
urdf_fk.py -- forward kinematics straight from an expanded URDF, numpy only.

Used on the host (cycle and pose generation) and inside the Isaac container (planning the
pose a scenario starts from). It reads joint origins, axes and types and composes 4x4
transforms; mimic joints are honoured as q = multiplier * q_master + offset.
"""
import math
import xml.etree.ElementTree as ET

import numpy as np


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _axis_angle(a, t):
    a = np.asarray(a, float)
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(t) * K + (1 - math.cos(t)) * K @ K


class UrdfModel:
    def __init__(self, urdf_path):
        root = ET.parse(str(urdf_path)).getroot()
        self.joints = {}
        self.parent_joint = {}
        for j in root.findall("joint"):
            o = j.find("origin")
            xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
            rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
            ax = j.find("axis")
            lim = j.find("limit")
            mim = j.find("mimic")
            T = np.eye(4)
            T[:3, :3] = _rpy(*rpy)
            T[:3, 3] = xyz
            rec = dict(name=j.get("name"), type=j.get("type"), parent=j.find("parent").get("link"),
                       child=j.find("child").get("link"), T=T,
                       axis=[float(v) for v in ax.get("xyz").split()] if ax is not None else [1, 0, 0],
                       lower=float(lim.get("lower")) if lim is not None and lim.get("lower") else None,
                       upper=float(lim.get("upper")) if lim is not None and lim.get("upper") else None,
                       mimic=(mim.get("joint"), float(mim.get("multiplier", 1)), float(mim.get("offset", 0)))
                       if mim is not None else None)
            self.joints[rec["name"]] = rec
            self.parent_joint[rec["child"]] = rec["name"]

    def limits_deg(self, joint):
        j = self.joints[joint]
        return math.degrees(j["lower"]), math.degrees(j["upper"])

    def fk(self, link, q):
        """World (base) transform of `link` for joint positions q {joint: rad}."""
        chain = []
        while link in self.parent_joint:
            j = self.joints[self.parent_joint[link]]
            chain.append(j)
            link = j["parent"]
        M = np.eye(4)
        for j in reversed(chain):
            M = M @ j["T"]
            if j["type"] in ("revolute", "continuous", "prismatic"):
                if j["mimic"]:
                    master, mult, off = j["mimic"]
                    v = mult * q.get(master, 0.0) + off
                else:
                    v = q.get(j["name"], 0.0)
                R = np.eye(4)
                if j["type"] == "prismatic":
                    R[:3, 3] = np.asarray(j["axis"]) * v
                else:
                    R[:3, :3] = _axis_angle(j["axis"], v)
                M = M @ R
        return M
