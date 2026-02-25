#!/usr/bin/env python3
"""
ROS2 service node that fits spheres to a robot link's collision mesh.

Wraps curobo.geom.sphere_fit.fit_spheres_to_mesh via the
curobo_robot_setup/srv/GenerateSpheres service.

Usage:
  ros2 run curobo_robot_setup sphere_fit_service.py
"""

import math
import os
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from rclpy.node import Node

try:
    import trimesh
    from curobo.geom.sphere_fit import SphereFitType, fit_spheres_to_mesh
    _CUROBO_AVAILABLE = True
except ImportError as _exc:
    _CUROBO_AVAILABLE = False
    _IMPORT_ERROR = str(_exc)

from curobo_robot_setup.srv import GenerateSpheres

try:
    from ament_index_python.packages import get_package_share_directory
    _AMENT_INDEX_AVAILABLE = True
except ImportError:
    _AMENT_INDEX_AVAILABLE = False


# ──────────────────────────────────────────────────────────────────────────────
_FIT_TYPE_MAP = {
    "VOXEL_VOLUME_SAMPLE_SURFACE": SphereFitType.VOXEL_VOLUME_SAMPLE_SURFACE
        if _CUROBO_AVAILABLE else None,
    "VOXEL_SURFACE": SphereFitType.VOXEL_SURFACE
        if _CUROBO_AVAILABLE else None,
    "VOXEL_VOLUME": SphereFitType.VOXEL_VOLUME
        if _CUROBO_AVAILABLE else None,
    "VOXEL_VOLUME_INSIDE": SphereFitType.VOXEL_VOLUME_INSIDE
        if _CUROBO_AVAILABLE else None,
    "SAMPLE_SURFACE": SphereFitType.SAMPLE_SURFACE
        if _CUROBO_AVAILABLE else None,
}


def _rpy_to_matrix(xyz, rpy):
    """4×4 homogeneous transform from xyz translation and rpy rotation."""
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll),  math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw),   math.sin(yaw)

    m = np.array([
        [cy * cp,  cy * sp * sr - sy * cr,  cy * sp * cr + sy * sr,  xyz[0]],
        [sy * cp,  sy * sp * sr + cy * cr,  sy * sp * cr - cy * sr,  xyz[1]],
        [-sp,      cp * sr,                 cp * cr,                  xyz[2]],
        [0.0,      0.0,                     0.0,                      1.0],
    ], dtype=float)
    return m


def _resolve_mesh_path(filename: str, urdf_dir: str) -> str | None:
    """Resolve a mesh filename (package:// or file:// or relative) to an
    absolute file path that exists on disk.  Returns None if not found."""
    if filename.startswith("package://"):
        rest = filename[len("package://"):]
        pkg_name, _, rel = rest.partition("/")
        if _AMENT_INDEX_AVAILABLE:
            try:
                share = get_package_share_directory(pkg_name)
                candidate = os.path.join(share, rel)
                if os.path.isfile(candidate):
                    return candidate
            except Exception:
                pass
        # Fall back: walk up from urdf_dir to find <pkg_name>/...
        candidate = os.path.normpath(os.path.join(urdf_dir, "..", rel))
        if os.path.isfile(candidate):
            return candidate
        # Last resort: absolute path join
        candidate = os.path.join("/", rel)
        if os.path.isfile(candidate):
            return candidate
        return None

    if filename.startswith("file://"):
        path = filename[len("file://"):]
        return path if os.path.isfile(path) else None

    # Relative path
    candidate = os.path.join(urdf_dir, filename)
    return candidate if os.path.isfile(candidate) else None


def _get_collision_mesh(urdf_path: str, link_name: str):
    """Parse the URDF and return (mesh_path, transform_4x4) for the
    collision geometry of *link_name*.  Returns (None, None) if the link
    has no mesh collision (e.g. primitive shapes, no collision element)."""
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    urdf_dir = os.path.dirname(os.path.abspath(urdf_path))

    for link_elem in root.iter("link"):
        if link_elem.get("name") != link_name:
            continue

        collision = link_elem.find("collision")
        if collision is None:
            return None, None

        geometry = collision.find("geometry")
        if geometry is None:
            return None, None

        mesh_elem = geometry.find("mesh")
        if mesh_elem is None:
            # Primitive shape (box / cylinder / sphere) – skip silently
            return None, None

        filename = mesh_elem.get("filename", "")
        if not filename:
            return None, None

        mesh_path = _resolve_mesh_path(filename, urdf_dir)
        if mesh_path is None:
            return None, None

        # Build origin transform
        origin_elem = collision.find("origin")
        if origin_elem is not None:
            xyz_str = origin_elem.get("xyz", "0 0 0").split()
            rpy_str = origin_elem.get("rpy", "0 0 0").split()
            xyz = [float(v) for v in xyz_str]
            rpy = [float(v) for v in rpy_str]
            transform = _rpy_to_matrix(xyz, rpy)
        else:
            transform = np.eye(4)

        return mesh_path, transform

    return None, None


# ──────────────────────────────────────────────────────────────────────────────
class SphereFitService(Node):

    def __init__(self):
        super().__init__("sphere_fit_service")

        if not _CUROBO_AVAILABLE:
            self.get_logger().error(
                f"cuRobo / trimesh not available: {_IMPORT_ERROR}\n"
                "Install them before using this service.")

        self._srv = self.create_service(
            GenerateSpheres,
            "generate_spheres",
            self._handle_request)

        self.get_logger().info("sphere_fit_service ready on /generate_spheres")

    # ──────────────────────────────────────────────────────────────────────────
    def _handle_request(self, request, response):
        if not _CUROBO_AVAILABLE:
            response.success = False
            response.message = f"cuRobo unavailable: {_IMPORT_ERROR}"
            return response

        link = request.link_name
        urdf  = request.urdf_path

        # Resolve collision mesh
        mesh_path, transform = _get_collision_mesh(urdf, link)
        if mesh_path is None:
            # No mesh → silently succeed with 0 spheres
            response.success = True
            response.message = f"No collision mesh for link '{link}' (skipped)"
            return response

        try:
            mesh = trimesh.load(mesh_path, force="mesh")
        except Exception as exc:
            response.success = False
            response.message = f"Failed to load mesh '{mesh_path}': {exc}"
            return response

        # Apply collision-element origin transform
        mesh.apply_transform(transform)

        # Sphere fit type
        fit_type = _FIT_TYPE_MAP.get(
            request.fit_type,
            SphereFitType.VOXEL_VOLUME_SAMPLE_SURFACE)

        try:
            pts, radii = fit_spheres_to_mesh(
                mesh,
                request.n_spheres,
                surface_sphere_radius=request.surface_sphere_radius,
                fit_type=fit_type,
            )
        except Exception as exc:
            response.success = False
            response.message = f"sphere_fit failed for '{link}': {exc}"
            return response

        if pts is None or len(pts) == 0:
            response.success = False
            response.message = f"sphere_fit returned no points for '{link}'"
            return response

        pts = np.asarray(pts, dtype=float)
        radii_arr = np.ravel(radii).astype(float)

        response.positions_x = pts[:, 0].tolist()
        response.positions_y = pts[:, 1].tolist()
        response.positions_z = pts[:, 2].tolist()
        response.radii = radii_arr.tolist()
        response.success = True
        response.message = (
            f"Generated {len(pts)} spheres for '{link}' "
            f"(method={request.fit_type})")

        self.get_logger().info(response.message)
        return response


# ──────────────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = SphereFitService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
