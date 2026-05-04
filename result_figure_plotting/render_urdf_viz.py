#!/usr/bin/env python3
"""Blender headless script: render four camera angles by N visualization modes from a URDF.

Run via:
    /Applications/Blender.app/Contents/MacOS/blender --background --python plots/render_urdf_viz.py -- \
        --urdf /path/to/model.urdf --output /path/to/plots/<record_id> [--samples 128] [--resolution 1920x1080]
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path

import bpy
import bmesh
from mathutils import Euler, Quaternion, Vector

# ── palette ───────────────────────────────────────────────────────────────────

SEG_PALETTE = [
    (0.89, 0.15, 0.21), (0.20, 0.50, 0.85), (0.15, 0.70, 0.40), (0.95, 0.60, 0.10),
    (0.60, 0.25, 0.80), (0.95, 0.85, 0.10), (0.10, 0.75, 0.85), (0.90, 0.40, 0.65),
    (0.40, 0.65, 0.25), (0.55, 0.35, 0.20), (0.75, 0.75, 0.75), (0.45, 0.80, 0.70),
]

COL_PALETTE = [
    (1.00, 0.35, 0.21),  # #ff5a36
    (0.00, 0.70, 1.00),  # #00b3ff
    (1.00, 0.83, 0.00),  # #ffd400
    (0.09, 0.77, 0.50),  # #16c47f
    (1.00, 0.18, 0.57),  # #ff2f92
    (0.49, 0.36, 1.00),  # #7c5cff
    (1.00, 0.54, 0.00),  # #ff8a00
    (0.00, 0.76, 0.66),  # #00c2a8
    (1.00, 0.42, 0.42),  # #ff6b6b
    (0.36, 0.84, 0.17),  # #5dd62c
]

# Warm amber — visible against both light and dark surfaces
_JOINT_COLOR = (1.0, 0.52, 0.07)
JOINT_COLORS = {
    "revolute":   _JOINT_COLOR,
    "continuous": _JOINT_COLOR,
    "prismatic":  _JOINT_COLOR,
}

# Camera: 35mm lens on 36mm sensor — vertical half-FOV ≈ 16.1°
# Safe distance to fully contain a bounding sphere of radius r (with 20% margin):
#   dist = r / tan(half_vfov * 0.8)  ≈  r * 4.5
_LENS_MM = 35
_CAM_DIST_MULT = 4.5

# ── PBR material inference ────────────────────────────────────────────────────

def _infer_pbr(mat_name: str, rgba: list) -> dict:
    """Return Principled BSDF kwargs from material name keywords."""
    n = (mat_name or "").lower()
    a = (rgba + [1.0])[3]

    # glass / tinted glass / transparent
    if any(k in n for k in ("glass", "transparent", "window", "lens")):
        return dict(metallic=0.0, roughness=0.05, transmission=0.95, ior=1.45, alpha=min(a, 0.4))

    # polished / chrome / mirror metal
    if any(k in n for k in ("chrome", "mirror", "polished_metal", "shiny")):
        return dict(metallic=1.0, roughness=0.05, transmission=0.0, ior=1.45, alpha=1.0)

    # steel / iron / metal / hub / rim / axle / bolt / bearing / shaft
    if any(k in n for k in ("steel", "iron", "metal", "hub", "rim", "axle", "bolt",
                             "bearing", "shaft", "frame", "strut", "spoke", "truss")):
        roughness = 0.25 if "polished" in n else (0.5 if "painted" in n else 0.35)
        metallic = 0.9 if not any(k in n for k in ("painted", "coated")) else 0.5
        return dict(metallic=metallic, roughness=roughness, transmission=0.0, ior=1.45, alpha=1.0)

    # rubber / tire / grip / gasket / seal
    if any(k in n for k in ("rubber", "tire", "grip", "gasket", "seal", "pad")):
        return dict(metallic=0.0, roughness=0.95, transmission=0.0, ior=1.45, alpha=1.0)

    # concrete / stone / brick / rock / masonry / cement / ground / floor / base
    if any(k in n for k in ("concrete", "stone", "brick", "rock", "masonry",
                             "cement", "ground", "floor", "base", "pavement")):
        return dict(metallic=0.0, roughness=0.92, transmission=0.0, ior=1.45, alpha=1.0)

    # wood / plywood / lumber
    if any(k in n for k in ("wood", "plywood", "lumber", "timber")):
        return dict(metallic=0.0, roughness=0.85, transmission=0.0, ior=1.45, alpha=1.0)

    # plastic / nylon / abs / pvc / acrylic
    if any(k in n for k in ("plastic", "nylon", "abs", "pvc", "acrylic", "resin")):
        return dict(metallic=0.0, roughness=0.6, transmission=0.0, ior=1.45, alpha=1.0)

    # paint / coat / lacquer / enamel
    if any(k in n for k in ("paint", "coat", "lacquer", "enamel", "primer")):
        return dict(metallic=0.0, roughness=0.55, transmission=0.0, ior=1.45, alpha=1.0)

    # fabric / cloth / canvas / foam / cushion
    if any(k in n for k in ("fabric", "cloth", "canvas", "foam", "cushion", "seat")):
        return dict(metallic=0.0, roughness=0.97, transmission=0.0, ior=1.45, alpha=1.0)

    # if RGBA alpha < 0.95 → treat as semi-transparent material
    if a < 0.95:
        return dict(metallic=0.0, roughness=0.1, transmission=0.85, ior=1.45, alpha=a)

    # default: matte non-metal
    return dict(metallic=0.0, roughness=0.7, transmission=0.0, ior=1.45, alpha=1.0)


def make_material_visual(name: str, rgba: list, mat_name: str = "") -> bpy.types.Material:
    pbr = _infer_pbr(mat_name, rgba)
    mat = bpy.data.materials.new(name)
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    links = mat.node_tree.links
    nodes.clear()
    out = nodes.new("ShaderNodeOutputMaterial")
    bsdf = nodes.new("ShaderNodeBsdfPrincipled")

    r, g, b, a = (rgba + [1.0])[:4]
    bsdf.inputs["Base Color"].default_value = (r, g, b, 1.0)
    bsdf.inputs["Roughness"].default_value = pbr["roughness"]
    bsdf.inputs["Metallic"].default_value = pbr["metallic"]
    bsdf.inputs["IOR"].default_value = pbr["ior"]

    effective_alpha = pbr["alpha"]
    if pbr["transmission"] > 0:
        # Principled BSDF Transmission (Blender 3.x uses "Transmission Weight", 4.x uses "Transmission")
        try:
            bsdf.inputs["Transmission Weight"].default_value = pbr["transmission"]
        except KeyError:
            try:
                bsdf.inputs["Transmission"].default_value = pbr["transmission"]
            except KeyError:
                pass
        mat.blend_method = "BLEND"
        bsdf.inputs["Alpha"].default_value = effective_alpha
    elif effective_alpha < 0.99:
        bsdf.inputs["Alpha"].default_value = effective_alpha
        mat.blend_method = "BLEND"

    links.new(bsdf.outputs["BSDF"], out.inputs["Surface"])
    mat.use_backface_culling = False
    return mat


def make_material_seg(name: str, rgb: tuple) -> bpy.types.Material:
    mat = bpy.data.materials.new(name)
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    lnks = mat.node_tree.links
    nodes.clear()
    out = nodes.new("ShaderNodeOutputMaterial")
    diff = nodes.new("ShaderNodeBsdfDiffuse")
    emit = nodes.new("ShaderNodeEmission")
    mix = nodes.new("ShaderNodeMixShader")
    diff.inputs["Color"].default_value = (*rgb, 1.0)
    emit.inputs["Color"].default_value = (*rgb, 1.0)
    emit.inputs["Strength"].default_value = 0.4
    mix.inputs["Fac"].default_value = 0.35
    lnks.new(diff.outputs["BSDF"], mix.inputs[1])
    lnks.new(emit.outputs["Emission"], mix.inputs[2])
    lnks.new(mix.outputs["Shader"], out.inputs["Surface"])
    return mat


def make_material_collision(name: str, rgb: tuple) -> bpy.types.Material:
    """Viewer-style semi-transparent collision material."""
    mat = bpy.data.materials.new(name)
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    lnks = mat.node_tree.links
    nodes.clear()
    out = nodes.new("ShaderNodeOutputMaterial")
    emit = nodes.new("ShaderNodeEmission")
    transparent = nodes.new("ShaderNodeBsdfTransparent")
    mix = nodes.new("ShaderNodeMixShader")
    emit.inputs["Color"].default_value = (*rgb, 1.0)
    emit.inputs["Strength"].default_value = 1.0
    mix.inputs["Fac"].default_value = 0.88
    lnks.new(transparent.outputs["BSDF"], mix.inputs[1])
    lnks.new(emit.outputs["Emission"], mix.inputs[2])
    lnks.new(mix.outputs["Shader"], out.inputs["Surface"])
    mat.blend_method = "BLEND"
    mat.use_backface_culling = False
    return mat


def make_material_collision_edge(name: str) -> bpy.types.Material:
    mat = bpy.data.materials.new(name)
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    lnks = mat.node_tree.links
    nodes.clear()
    out = nodes.new("ShaderNodeOutputMaterial")
    emit = nodes.new("ShaderNodeEmission")
    emit.inputs["Color"].default_value = (0.10, 0.10, 0.11, 1.0)
    emit.inputs["Strength"].default_value = 0.8
    lnks.new(emit.outputs["Emission"], out.inputs["Surface"])
    mat.blend_method = "OPAQUE"
    return mat


def make_material_emission(name: str, rgb: tuple, strength: float = 1.0, alpha: float = 1.0) -> bpy.types.Material:
    mat = bpy.data.materials.new(name)
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    lnks = mat.node_tree.links
    nodes.clear()
    out = nodes.new("ShaderNodeOutputMaterial")
    emit = nodes.new("ShaderNodeEmission")
    emit.inputs["Color"].default_value = (*rgb, alpha)
    emit.inputs["Strength"].default_value = strength
    if alpha < 1.0:
        transparent = nodes.new("ShaderNodeBsdfTransparent")
        mix = nodes.new("ShaderNodeMixShader")
        mix.inputs["Fac"].default_value = alpha
        lnks.new(transparent.outputs["BSDF"], mix.inputs[1])
        lnks.new(emit.outputs["Emission"], mix.inputs[2])
        lnks.new(mix.outputs["Shader"], out.inputs["Surface"])
        mat.blend_method = "BLEND"
    else:
        lnks.new(emit.outputs["Emission"], out.inputs["Surface"])
    mat.use_backface_culling = False
    return mat


def make_material_overlay_solid(name: str, rgb: tuple, roughness: float = 0.45,
                                metallic: float = 0.0) -> bpy.types.Material:
    """Solid Principled-BSDF material for joint overlay icons that should look
    like real painted objects under HDRI lighting (no ghost / no emission)."""
    mat = bpy.data.materials.new(name)
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    lnks = mat.node_tree.links
    nodes.clear()
    out = nodes.new("ShaderNodeOutputMaterial")
    bsdf = nodes.new("ShaderNodeBsdfPrincipled")
    bsdf.inputs["Base Color"].default_value = (*rgb, 1.0)
    bsdf.inputs["Roughness"].default_value = roughness
    bsdf.inputs["Metallic"].default_value = metallic
    lnks.new(bsdf.outputs["BSDF"], out.inputs["Surface"])
    mat.use_backface_culling = False
    return mat


def make_material_ghost_visual(name: str) -> bpy.types.Material:
    mat = bpy.data.materials.new(name)
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    lnks = mat.node_tree.links
    nodes.clear()
    out = nodes.new("ShaderNodeOutputMaterial")
    bsdf = nodes.new("ShaderNodeBsdfPrincipled")
    bsdf.inputs["Base Color"].default_value = (0.76, 0.82, 0.88, 1.0)
    bsdf.inputs["Roughness"].default_value = 0.72
    bsdf.inputs["Metallic"].default_value = 0.0
    bsdf.inputs["Alpha"].default_value = 0.30
    mat.blend_method = "BLEND"
    mat.use_backface_culling = False
    lnks.new(bsdf.outputs["BSDF"], out.inputs["Surface"])
    return mat



# ── URDF parsing ──────────────────────────────────────────────────────────────

@dataclass
class GeomElement:
    origin_xyz: list
    origin_rpy: list
    geom_type: str
    geom_params: dict
    material_name: str | None
    name: str


@dataclass
class Link:
    name: str
    visuals: list[GeomElement] = field(default_factory=list)
    collisions: list[GeomElement] = field(default_factory=list)


@dataclass
class Joint:
    name: str
    jtype: str
    parent: str
    child: str
    origin_xyz: list
    origin_rpy: list
    axis: list
    mimic_joint: str | None = None
    mimic_multiplier: float = 1.0
    limit_lower: float | None = None
    limit_upper: float | None = None


def _parse_xyz(s, default=(0.0, 0.0, 0.0)) -> list:
    return [float(x) for x in s.split()] if s else list(default)


def parse_urdf(path: Path):
    root = ET.parse(path).getroot()

    materials: dict[str, list] = {}
    for m in root.findall("material"):
        c = m.find("color")
        if c is not None:
            materials[m.get("name", "")] = [float(x) for x in c.get("rgba", "0.8 0.8 0.8 1").split()]

    def parse_geom_elements(parent_el, tag) -> list[GeomElement]:
        elems = []
        for i, el in enumerate(parent_el.findall(tag)):
            ename = el.get("name") or f"{tag[0]}{i}"
            orig = el.find("origin")
            xyz = _parse_xyz(orig.get("xyz") if orig is not None else None)
            rpy = _parse_xyz(orig.get("rpy") if orig is not None else None)
            geom = el.find("geometry")
            gtype, gparams = "unknown", {}
            if geom is not None:
                box = geom.find("box")
                cyl = geom.find("cylinder")
                sph = geom.find("sphere")
                msh = geom.find("mesh")
                if box is not None:
                    gtype = "box"
                    gparams = {"size": [float(x) for x in box.get("size", "1 1 1").split()]}
                elif cyl is not None:
                    gtype = "cylinder"
                    gparams = {"radius": float(cyl.get("radius", 0.1)), "length": float(cyl.get("length", 1.0))}
                elif sph is not None:
                    gtype = "sphere"
                    gparams = {"radius": float(sph.get("radius", 0.1))}
                elif msh is not None:
                    gtype = "mesh"
                    gparams = {
                        "filename": msh.get("filename", ""),
                        "scale": _parse_xyz(msh.get("scale"), default=(1.0, 1.0, 1.0)),
                    }
            mat_el = el.find("material")
            mat_name = mat_el.get("name") if mat_el is not None else None
            if mat_el is not None:
                c = mat_el.find("color")
                if c is not None and mat_name:
                    materials[mat_name] = [float(x) for x in c.get("rgba", "0.8 0.8 0.8 1").split()]
            elems.append(GeomElement(xyz, rpy, gtype, gparams, mat_name, ename))
        return elems

    links: dict[str, Link] = {}
    for el in root.findall("link"):
        name = el.get("name", "")
        links[name] = Link(
            name=name,
            visuals=parse_geom_elements(el, "visual"),
            collisions=parse_geom_elements(el, "collision"),
        )

    joints: dict[str, Joint] = {}
    for el in root.findall("joint"):
        name = el.get("name", "")
        jtype = el.get("type", "fixed")
        parent = el.find("parent").get("link", "")
        child = el.find("child").get("link", "")
        orig = el.find("origin")
        xyz = _parse_xyz(orig.get("xyz") if orig is not None else None)
        rpy = _parse_xyz(orig.get("rpy") if orig is not None else None)
        ax_el = el.find("axis")
        axis = _parse_xyz(ax_el.get("xyz") if ax_el is not None else None, default=(0, 0, 1))
        mimic_joint = None
        mimic_mult = 1.0
        mimic_el = el.find("mimic")
        if mimic_el is not None:
            mimic_joint = mimic_el.get("joint")
            mimic_mult = float(mimic_el.get("multiplier", 1.0))
        limit_el = el.find("limit")
        lower_v = float(limit_el.get("lower")) if limit_el is not None and limit_el.get("lower") is not None else None
        upper_v = float(limit_el.get("upper")) if limit_el is not None and limit_el.get("upper") is not None else None
        joints[name] = Joint(name, jtype, parent, child, xyz, rpy, axis, mimic_joint, mimic_mult, lower_v, upper_v)

    return links, joints, materials


# ── Blender scene builder ─────────────────────────────────────────────────────

def clear_scene():
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete()
    for block in list(bpy.data.meshes):
        bpy.data.meshes.remove(block)
    for block in list(bpy.data.materials):
        bpy.data.materials.remove(block)
    for block in list(bpy.data.cameras):
        bpy.data.cameras.remove(block)
    for block in list(bpy.data.lights):
        bpy.data.lights.remove(block)


def _resolve_mesh_path(urdf_dir: Path, filename: str) -> Path:
    if filename.startswith("file://"):
        filename = filename[7:]
    if filename.startswith("package://"):
        filename = filename[len("package://"):]
    path = Path(filename)
    if path.is_absolute():
        return path
    return urdf_dir / path


def _load_obj_mesh(name: str, path: Path) -> bpy.types.Object | None:
    """Load a simple OBJ as Blender mesh data without axis conversion."""
    verts: list[tuple[float, float, float]] = []
    faces: list[list[int]] = []
    try:
        for line in path.read_text().splitlines():
            parts = line.strip().split()
            if not parts or parts[0].startswith("#"):
                continue
            if parts[0] == "v" and len(parts) >= 4:
                verts.append((float(parts[1]), float(parts[2]), float(parts[3])))
            elif parts[0] == "f" and len(parts) >= 4:
                idxs = []
                for token in parts[1:]:
                    raw = token.split("/")[0]
                    if not raw:
                        continue
                    idx = int(raw)
                    idxs.append(idx - 1 if idx > 0 else len(verts) + idx)
                if len(idxs) >= 3:
                    faces.append(idxs)
    except Exception as exc:
        print(f"[render_urdf_viz] Failed to load mesh {path}: {exc}")
        return None

    if not verts or not faces:
        print(f"[render_urdf_viz] Empty/unsupported OBJ mesh: {path}")
        return None

    me = bpy.data.meshes.new(name)
    me.from_pydata(verts, [], faces)
    me.update()
    for poly in me.polygons:
        poly.use_smooth = True
    obj = bpy.data.objects.new(name, me)
    bpy.context.collection.objects.link(obj)
    return obj


def _create_mesh_obj(name: str, geom: GeomElement, urdf_dir: Path) -> bpy.types.Object | None:
    bm = bmesh.new()
    gtype = geom.geom_type
    gp = geom.geom_params

    if gtype == "box":
        sx, sy, sz = gp["size"]
        bmesh.ops.create_cube(bm, size=1.0)
        bmesh.ops.scale(bm, vec=(sx, sy, sz), verts=bm.verts)
    elif gtype == "cylinder":
        r = gp["radius"]
        h = gp["length"]
        bmesh.ops.create_cone(bm, cap_ends=True, cap_tris=False,
                              segments=32, radius1=r, radius2=r, depth=h)
    elif gtype == "sphere":
        r = gp["radius"]
        bmesh.ops.create_uvsphere(bm, u_segments=32, v_segments=16, radius=r)
    elif gtype == "mesh":
        bm.free()
        filename = gp.get("filename", "")
        if not filename:
            return None
        mesh_path = _resolve_mesh_path(urdf_dir, filename)
        if not mesh_path.exists():
            print(f"[render_urdf_viz] Missing mesh file: {mesh_path}")
            return None
        if mesh_path.suffix.lower() != ".obj":
            print(f"[render_urdf_viz] Unsupported mesh type: {mesh_path}")
            return None
        obj = _load_obj_mesh(name, mesh_path)
        if obj is not None:
            obj.scale = Vector(gp.get("scale", [1.0, 1.0, 1.0]))
        return obj
    else:
        bm.free()
        return None

    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    bm.free()
    obj = bpy.data.objects.new(name, me)
    bpy.context.collection.objects.link(obj)
    if gtype in {"cylinder", "sphere"}:
        for poly in obj.data.polygons:
            poly.use_smooth = True
    return obj


def _darken(rgb: tuple[float, float, float], amount: float = 0.32) -> tuple[float, float, float]:
    return tuple(max(0.0, c * (1.0 - amount)) for c in rgb)


def _add_edge_curve_child(obj: bpy.types.Object, mat: bpy.types.Material, name: str) -> bpy.types.Object | None:
    mesh = obj.data
    if mesh is None or not getattr(mesh, "edges", None):
        return None

    curve = bpy.data.curves.new(name, "CURVE")
    curve.dimensions = "3D"
    curve.resolution_u = 1
    curve.bevel_depth = 0.0025
    curve.bevel_resolution = 0

    verts = mesh.vertices
    for edge in mesh.edges:
        a, b = edge.vertices
        pa = verts[a].co
        pb = verts[b].co
        spline = curve.splines.new("POLY")
        spline.points.add(1)
        spline.points[0].co = (pa.x, pa.y, pa.z, 1.0)
        spline.points[1].co = (pb.x, pb.y, pb.z, 1.0)

    edge_obj = bpy.data.objects.new(name, curve)
    edge_obj.data.materials.append(mat)
    bpy.context.collection.objects.link(edge_obj)
    edge_obj.parent = obj
    edge_obj.hide_render = True
    edge_obj.hide_viewport = True
    return edge_obj


def _axis_basis(axis: Vector) -> tuple[Vector, Vector]:
    helper = Vector((1, 0, 0))
    if abs(axis.normalized().dot(helper)) > 0.9:
        helper = Vector((0, 1, 0))
    u = helper.cross(axis).normalized()
    v = axis.cross(u).normalized()
    return u, v


def _orient_z_to_vector(obj: bpy.types.Object, direction: Vector) -> None:
    if direction.length < 1e-9:
        direction = Vector((0, 0, 1))
    obj.rotation_mode = "QUATERNION"
    obj.rotation_quaternion = direction.normalized().to_track_quat("Z", "Y")


def _create_cylinder_between(name: str, start: Vector, end: Vector, radius: float, mat: bpy.types.Material) -> bpy.types.Object:
    direction = end - start
    length = max(direction.length, 1e-6)
    bm = bmesh.new()
    bmesh.ops.create_cone(bm, cap_ends=True, cap_tris=False, segments=16,
                          radius1=radius, radius2=radius, depth=length)
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    bm.free()
    obj = bpy.data.objects.new(name, me)
    bpy.context.collection.objects.link(obj)
    obj.location = (start + end) * 0.5
    _orient_z_to_vector(obj, direction)
    obj.data.materials.append(mat)
    return obj


def _create_cone_head(name: str, tip: Vector, direction: Vector, height: float, radius: float,
                      mat: bpy.types.Material) -> bpy.types.Object:
    bm = bmesh.new()
    bmesh.ops.create_cone(bm, cap_ends=True, cap_tris=False, segments=24,
                          radius1=radius, radius2=0.0, depth=height)
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    bm.free()
    obj = bpy.data.objects.new(name, me)
    bpy.context.collection.objects.link(obj)
    direction = direction.normalized() if direction.length > 1e-9 else Vector((0, 0, 1))
    obj.location = tip - direction * (height * 0.5)
    _orient_z_to_vector(obj, direction)
    obj.data.materials.append(mat)
    return obj


def _create_sphere(name: str, radius: float, mat: bpy.types.Material) -> bpy.types.Object:
    bm = bmesh.new()
    bmesh.ops.create_uvsphere(bm, u_segments=24, v_segments=12, radius=radius)
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    bm.free()
    obj = bpy.data.objects.new(name, me)
    bpy.context.collection.objects.link(obj)
    obj.data.materials.append(mat)
    return obj


def _create_arc_arrow(name: str, axis: Vector, axis_offset: float, arc_radius: float, tube_radius: float,
                      head_len: float, head_width: float, mat: bpy.types.Material,
                      start_angle: float | None = None, sweep: float | None = None,
                      double_head: bool = False) -> list[bpy.types.Object]:
    axis = axis.normalized() if axis.length > 1e-9 else Vector((0, 0, 1))
    u, v = _axis_basis(axis)
    if start_angle is None:
        start_angle = math.pi * 0.20
    if sweep is None:
        sweep = math.pi * 1.35  # 243° — enough to read as rotation, not a full ring
    segments = 48
    points: list[Vector] = []
    for i in range(segments + 1):
        a = start_angle + (i / segments) * sweep
        points.append(axis * axis_offset + u * (math.cos(a) * arc_radius) + v * (math.sin(a) * arc_radius))

    curve = bpy.data.curves.new(name, "CURVE")
    curve.dimensions = "3D"
    curve.resolution_u = 2
    curve.bevel_depth = tube_radius
    curve.bevel_resolution = 3
    spline = curve.splines.new("POLY")
    spline.points.add(len(points) - 1)
    for point, co in zip(spline.points, points):
        point.co = (co.x, co.y, co.z, 1.0)

    arc_obj = bpy.data.objects.new(name, curve)
    arc_obj.data.materials.append(mat)
    bpy.context.collection.objects.link(arc_obj)

    objs: list[bpy.types.Object] = [arc_obj]
    end_angle = start_angle + sweep
    end_pos = points[-1]
    tangent_end = (-math.sin(end_angle) * u + math.cos(end_angle) * v).normalized()
    objs.append(_create_cone_head(f"{name}_head", end_pos, tangent_end, head_len, head_width, mat))
    if double_head:
        start_pos = points[0]
        # Tangent at start points "forward" along the arc; flip so the arrowhead
        # opens outward (matching the +sweep arrowhead at the other end).
        tangent_start = -(-math.sin(start_angle) * u + math.cos(start_angle) * v).normalized()
        objs.append(_create_cone_head(f"{name}_head_start", start_pos, tangent_start,
                                      head_len, head_width, mat))
    return objs


def _create_disc_cap(name: str, center: Vector, axis: Vector, radius: float, height: float,
                     mat: bpy.types.Material) -> bpy.types.Object:
    """Short flat cylinder centered at `center`, oriented along `axis`."""
    direction = axis.normalized() if axis.length > 1e-9 else Vector((0, 0, 1))
    half = direction * (height / 2)
    return _create_cylinder_between(name, center - half, center + half, radius, mat)


def _create_torus_ring(name: str, center: Vector, axis: Vector, ring_radius: float,
                       tube_radius: float, mat: bpy.types.Material) -> bpy.types.Object:
    """Closed-loop torus around `axis` at `center`, built from a beveled curve."""
    axis = axis.normalized() if axis.length > 1e-9 else Vector((0, 0, 1))
    u, v = _axis_basis(axis)
    segments = 48
    points: list[Vector] = []
    for i in range(segments):
        a = (i / segments) * 2.0 * math.pi
        points.append(center + u * (math.cos(a) * ring_radius) + v * (math.sin(a) * ring_radius))

    curve = bpy.data.curves.new(name, "CURVE")
    curve.dimensions = "3D"
    curve.resolution_u = 2
    curve.bevel_depth = tube_radius
    curve.bevel_resolution = 4
    spline = curve.splines.new("POLY")
    spline.use_cyclic_u = True
    spline.points.add(len(points) - 1)
    for point, co in zip(spline.points, points):
        point.co = (co.x, co.y, co.z, 1.0)

    obj = bpy.data.objects.new(name, curve)
    obj.data.materials.append(mat)
    bpy.context.collection.objects.link(obj)
    return obj


def _compact_axis(axis_values: list[float]) -> str:
    x, y, z = axis_values or [0, 0, 1]
    e = 0.01
    if abs(x - 1) < e and abs(y) < e and abs(z) < e:
        return "X"
    if abs(x + 1) < e and abs(y) < e and abs(z) < e:
        return "-X"
    if abs(x) < e and abs(y - 1) < e and abs(z) < e:
        return "Y"
    if abs(x) < e and abs(y + 1) < e and abs(z) < e:
        return "-Y"
    if abs(x) < e and abs(y) < e and abs(z - 1) < e:
        return "Z"
    if abs(x) < e and abs(y) < e and abs(z + 1) < e:
        return "-Z"
    return " ".join(f"{v:.1f}" for v in (x, y, z))


def _create_joint_label(name: str, text: str, color_mat: bpy.types.Material,
                        text_mat: bpy.types.Material) -> bpy.types.Object:
    root = bpy.data.objects.new(name, None)
    root.empty_display_type = "PLAIN_AXES"
    root.empty_display_size = 0.01
    bpy.context.collection.objects.link(root)

    dot = _create_sphere(f"{name}_dot", 0.018, color_mat)
    dot.parent = root
    dot.location = Vector((-0.09, 0, 0))

    curve = bpy.data.curves.new(f"{name}_text_curve", "FONT")
    curve.body = text
    curve.align_x = "LEFT"
    curve.align_y = "CENTER"
    curve.size = 0.07
    curve.extrude = 0.002
    obj = bpy.data.objects.new(f"{name}_text", curve)
    obj.data.materials.append(text_mat)
    bpy.context.collection.objects.link(obj)
    obj.parent = root
    obj.location = Vector((-0.055, 0, 0))
    return root


# ── joint overlay styles ──────────────────────────────────────────────────────
# Selected at render time via the JOINT_OVERLAY_STYLE env var. Each preset
# describes the shapes, colors, and proportions of the joint indicators so we
# can iterate on figure aesthetics without re-touching geometry code.
_AMBER = (1.0, 0.52, 0.07)
_REVOLUTE_RED = (0.91, 0.23, 0.23)
_PRISMATIC_BLUE = (0.17, 0.49, 0.89)
_CHARCOAL = (0.10, 0.10, 0.12)

# Color palette for the arc-on-real-model variants.
_ARC_AMBER   = (1.00, 0.52, 0.07)   # warm orange (current)
_ARC_MAGENTA = (1.00, 0.20, 0.55)   # hot pink / magenta
_ARC_CYAN    = (0.10, 0.78, 0.95)   # bright cyan
_ARC_RED     = (0.95, 0.15, 0.20)   # crimson red
_ARC_LIME    = (0.62, 0.92, 0.22)   # lime green

# Bigger, more readable proportions for the arc-on-real-model variants.
_ARC_REAL_BASE = {
    "color_mode": "single",
    "emission_strength": 1.7,
    "alpha": 0.95,
    "skip_ghost": True,
    "pose_degrees": 30,
    "axis": True,
    "axis_len_factor": 0.55,
    "axis_thick_factor": 0.013,
    "dot_factor": 0.090,
    "arc": True,
    "arc_radius_factor": 0.40,
    "arc_tube_factor": 0.024,
    "arc_head_len_ratio": 0.62,
    "arc_head_w_ratio": 0.42,
    "arc_sweep_deg": 243,
    "arc_start_deg": 36,
}

def _arc_real(color: tuple, always_in_front: bool = False) -> dict:
    cfg = dict(_ARC_REAL_BASE)
    cfg["single_color"] = color
    if always_in_front:
        cfg["always_in_front"] = True
    return cfg


def _arc_real_double(color: tuple, always_in_front: bool = False) -> dict:
    """Bidirectional rotation indicator: arrowheads on both ends of the arc.

    Why: from rear-facing camera angles a single-headed arc reads as if rotation
    flips direction. A symmetric double-headed arc represents the rotation axis
    without implying handedness, so it stays legible from any view.
    """
    cfg = _arc_real(color, always_in_front)
    cfg["arc_double_head"] = True
    # Tighter sweep so the two heads sit at clearly distinct positions instead
    # of nearly meeting around the back side.
    cfg["arc_sweep_deg"] = 200
    cfg["arc_start_deg"] = 80
    return cfg


JOINT_OVERLAY_STYLES: dict[str, dict] = {
    # ── Arc style on top of the textured model, scaled for readability ─────
    "arc_amber":   _arc_real(_ARC_AMBER),
    "arc_magenta": _arc_real(_ARC_MAGENTA),
    "arc_cyan":    _arc_real(_ARC_CYAN),
    "arc_red":     _arc_real(_ARC_RED),
    "arc_lime":    _arc_real(_ARC_LIME),

    # ── Same arc style but rendered always-in-front of the model ──────────
    # Two-pass render: model alone, then overlay alone on transparent film,
    # alpha-composited so the indicators are never occluded by geometry.
    "front_arc_amber":   _arc_real(_ARC_AMBER,   always_in_front=True),
    "front_arc_magenta": _arc_real(_ARC_MAGENTA, always_in_front=True),
    "front_arc_cyan":    _arc_real(_ARC_CYAN,    always_in_front=True),
    "front_arc_red":     _arc_real(_ARC_RED,     always_in_front=True),
    "front_arc_lime":    _arc_real(_ARC_LIME,    always_in_front=True),

    # ── Double-headed (bidirectional) arc — reads consistently from any angle ─
    "front_arc_double_amber":   _arc_real_double(_ARC_AMBER,   always_in_front=True),
    "front_arc_double_magenta": _arc_real_double(_ARC_MAGENTA, always_in_front=True),
    "front_arc_double_cyan":    _arc_real_double(_ARC_CYAN,    always_in_front=True),
    "front_arc_double_red":     _arc_real_double(_ARC_RED,     always_in_front=True),
    "front_arc_double_lime":    _arc_real_double(_ARC_LIME,    always_in_front=True),

    # ── Earlier exploratory styles (kept for reference) ────────────────────
    # A — current baseline: warm amber arc + axis nub + origin dot
    "A_amber_arc": {
        "color_mode": "single",
        "single_color": _AMBER,
        "emission_strength": 1.5,
        "alpha": 0.92,
        "axis": True,
        "axis_len_factor": 0.42,
        "axis_thick_factor": 0.008,
        "dot_factor": 0.060,
        "arc": True,
        "arc_radius_factor": 0.25,
        "arc_tube_factor": 0.014,
        "arc_head_len_ratio": 0.65,
        "arc_head_w_ratio": 0.42,
        "arc_sweep_deg": 243,
        "arc_start_deg": 36,
    },
    # B — joint-type colored: revolute = red, prismatic = blue
    "B_type_color": {
        "color_mode": "type",
        "type_colors": {
            "revolute": _REVOLUTE_RED,
            "continuous": _REVOLUTE_RED,
            "prismatic": _PRISMATIC_BLUE,
        },
        "emission_strength": 1.6,
        "alpha": 0.95,
        "axis": True,
        "axis_len_factor": 0.42,
        "axis_thick_factor": 0.009,
        "dot_factor": 0.066,
        "arc": True,
        "arc_radius_factor": 0.25,
        "arc_tube_factor": 0.015,
        "arc_head_len_ratio": 0.65,
        "arc_head_w_ratio": 0.42,
        "arc_sweep_deg": 243,
        "arc_start_deg": 36,
    },
    # C — minimal: just origin dot + thin axis line, no arc
    "C_minimal": {
        "color_mode": "single",
        "single_color": _AMBER,
        "emission_strength": 1.4,
        "alpha": 0.95,
        "axis": True,
        "axis_len_factor": 0.62,
        "axis_thick_factor": 0.012,
        "dot_factor": 0.090,
        "arc": False,
        "arc_radius_factor": 0.0,
        "arc_tube_factor": 0.0,
        "arc_head_len_ratio": 0.0,
        "arc_head_w_ratio": 0.0,
        "arc_sweep_deg": 0,
        "arc_start_deg": 0,
    },
    # D — bold paper: charcoal high-contrast for white-paper figures
    "D_bold_paper": {
        "color_mode": "single",
        "single_color": _CHARCOAL,
        "emission_strength": 0.6,
        "alpha": 1.0,
        "axis": True,
        "axis_len_factor": 0.50,
        "axis_thick_factor": 0.014,
        "dot_factor": 0.085,
        "arc": True,
        "arc_radius_factor": 0.28,
        "arc_tube_factor": 0.022,
        "arc_head_len_ratio": 0.62,
        "arc_head_w_ratio": 0.50,
        "arc_sweep_deg": 230,
        "arc_start_deg": 40,
    },
    # E — long thin axis with small tight arc (engineering CAD feel)
    "E_long_axis": {
        "color_mode": "single",
        "single_color": _AMBER,
        "emission_strength": 1.5,
        "alpha": 0.95,
        "axis": True,
        "axis_len_factor": 1.20,
        "axis_thick_factor": 0.006,
        "dot_factor": 0.055,
        "arc": True,
        "arc_radius_factor": 0.18,
        "arc_tube_factor": 0.012,
        "arc_head_len_ratio": 0.55,
        "arc_head_w_ratio": 0.40,
        "arc_sweep_deg": 200,
        "arc_start_deg": 50,
    },
    # G — chair-style 3D icons: real Principled BSDF objects (T-handle for
    # revolute, arrow for prismatic) rendered on top of the normal textured
    # model — no ghost pass, just the icons sitting in the scene.
    "G_chair_3d": {
        "icon_style": "chair",
        "skip_ghost": True,
        "pose_degrees": 0,
        "color_mode": "type",
        "type_colors": {
            "revolute":   (0.79, 0.13, 0.13),  # bright red
            "continuous": (0.79, 0.13, 0.13),
            "prismatic":  (0.66, 0.65, 0.20),  # olive / yellow-green
        },
        "shader": "solid",          # solid Principled BSDF (real lighting)
        "roughness": 0.45,
        "metallic": 0.05,
        # All sizes are factors of `unit` (≈ 12% of model diagonal). These
        # match the bold, readable scale in the chair reference image.
        "icon_length_factor":      2.0,    # shaft length along axis
        "icon_shaft_radius_factor": 0.080,
        # Revolute extras (T-handle: disc on +side, ring on -side)
        "icon_disc_radius_factor": 0.34,
        "icon_disc_height_factor": 0.040,
        "icon_ring_radius_factor": 0.20,
        "icon_ring_tube_factor":   0.022,
        # Prismatic extras (single arrow head)
        "icon_head_radius_factor": 0.20,
        "icon_head_height_factor": 0.36,
    },
    # H — single arrow along axis (reference legend match):
    #   prismatic = yellow arrow, revolute = red arrow + ring at base.
    "H_axis_arrows": {
        "icon_style": "axis_arrow",
        "skip_ghost": True,
        "pose_degrees": 0,
        "color_mode": "type",
        "type_colors": {
            "revolute":   (0.91, 0.18, 0.18),  # red
            "continuous": (0.91, 0.18, 0.18),
            "prismatic":  (0.98, 0.83, 0.10),  # yellow
        },
        "shader": "solid",
        "roughness": 0.45,
        "metallic": 0.05,
        # All factors relative to `unit` (~12% of model diagonal).
        "icon_length_factor":       1.6,
        "icon_shaft_radius_factor": 0.045,
        "icon_head_radius_factor":  0.16,
        "icon_head_height_factor":  0.32,
        # Base ring (only used for revolute joints)
        "icon_ring_radius_factor":  0.18,
        "icon_ring_tube_factor":    0.022,
    },
    # I — same legend as H, but the shaft is symmetric about the joint origin
    # (object sits in the middle of the marker) and the indicator renders
    # always-in-front so geometry never occludes it.
    "I_axis_through": {
        "icon_style": "axis_arrow",
        "skip_ghost": True,
        "always_in_front": True,
        "pose_degrees": 0,
        "color_mode": "type",
        "type_colors": {
            "revolute":   (0.91, 0.18, 0.18),
            "continuous": (0.91, 0.18, 0.18),
            "prismatic":  (0.98, 0.83, 0.10),
        },
        "shader": "solid",
        "roughness": 0.45,
        "metallic": 0.05,
        "icon_length_factor":       1.6,
        "icon_shaft_radius_factor": 0.040,
        "icon_head_radius_factor":  0.14,
        "icon_head_height_factor":  0.28,
        "icon_ring_radius_factor":  0.16,
        "icon_ring_tube_factor":    0.020,
        "symmetric": True,
    },
    # F — arrow only: origin dot + curved arrow, no axis nub
    "F_arrow_only": {
        "color_mode": "single",
        "single_color": _AMBER,
        "emission_strength": 1.5,
        "alpha": 0.95,
        "axis": False,
        "axis_len_factor": 0.0,
        "axis_thick_factor": 0.0,
        "dot_factor": 0.075,
        "arc": True,
        "arc_radius_factor": 0.30,
        "arc_tube_factor": 0.018,
        "arc_head_len_ratio": 0.62,
        "arc_head_w_ratio": 0.46,
        "arc_sweep_deg": 270,
        "arc_start_deg": 30,
    },
}


def _resolve_overlay_style() -> tuple[str, dict]:
    name = os.environ.get("JOINT_OVERLAY_STYLE", "A_amber_arc")
    if name not in JOINT_OVERLAY_STYLES:
        print(f"[joint_overlay] Unknown style '{name}', falling back to A_amber_arc")
        name = "A_amber_arc"
    return name, JOINT_OVERLAY_STYLES[name]


def _build_chair_style_overlay(joints, joint_empties, unit: float, style_name: str,
                               style: dict) -> tuple[list[bpy.types.Object], list[bpy.types.Object]]:
    """3D-object joint icons (red T-handle for revolute, olive arrow for
    prismatic) built as real Principled-BSDF meshes that render on top of the
    untouched textured model. Matches the reference legend / chair example."""
    type_colors = style["type_colors"]
    rough = style.get("roughness", 0.45)
    metal = style.get("metallic", 0.0)
    mats = {
        jtype: make_material_overlay_solid(f"joint_{style_name}_{jtype}",
                                           type_colors.get(jtype, _AMBER), rough, metal)
        for jtype in ("revolute", "continuous", "prismatic")
    }

    length      = unit * style["icon_length_factor"]
    shaft_r     = unit * style["icon_shaft_radius_factor"]
    disc_r      = unit * style["icon_disc_radius_factor"]
    disc_h      = unit * style["icon_disc_height_factor"]
    ring_r      = unit * style["icon_ring_radius_factor"]
    ring_tube   = unit * style["icon_ring_tube_factor"]
    head_r      = unit * style["icon_head_radius_factor"]
    head_h      = unit * style["icon_head_height_factor"]

    roots: list[bpy.types.Object] = []
    for joint in joints.values():
        if joint.jtype not in JOINT_COLORS:
            continue
        joint_empty = joint_empties.get(joint.name)
        if joint_empty is None:
            continue

        axis = Vector(joint.axis)
        if axis.length < 1e-9:
            axis = Vector((0, 0, 1))
        axis.normalize()
        mat = mats[joint.jtype]

        root = bpy.data.objects.new(f"joint_overlay_{joint.name}", None)
        root.empty_display_type = "PLAIN_AXES"
        root.empty_display_size = 0.02
        bpy.context.collection.objects.link(root)
        root.parent = joint_empty.parent
        root.location = Vector(joint.origin_xyz)
        root.rotation_mode = "XYZ"
        root.rotation_euler = Euler(joint.origin_rpy, "XYZ")
        root.hide_render = True
        root.hide_viewport = True
        roots.append(root)

        half = length / 2.0
        plus = axis * half
        minus = axis * -half
        zero = Vector((0.0, 0.0, 0.0))

        if joint.jtype in ("revolute", "continuous"):
            # T-handle: shaft along axis + disc cap on +side + thin ring on -side
            shaft = _create_cylinder_between(f"joint_shaft_{joint.name}", minus, plus, shaft_r, mat)
            disc = _create_disc_cap(f"joint_disc_{joint.name}", plus, axis, disc_r, disc_h, mat)
            ring = _create_torus_ring(f"joint_ring_{joint.name}", minus, axis, ring_r, ring_tube, mat)
            for obj in (shaft, disc, ring):
                obj.parent = root
        elif joint.jtype == "prismatic":
            # Single arrow: shaft up to (+half - head_h), then cone head at +half
            shaft_top = axis * (half - head_h * 0.5)
            shaft = _create_cylinder_between(f"joint_shaft_{joint.name}", minus, shaft_top, shaft_r, mat)
            head = _create_cone_head(f"joint_head_{joint.name}", plus, axis, head_h, head_r, mat)
            for obj in (shaft, head):
                obj.parent = root
            _ = zero  # keep symmetry with revolute branch

    return roots, []


def _build_axis_arrow_overlay(joints, joint_empties, unit: float, style_name: str,
                              style: dict, link_diag: dict | None = None,
                              link_center_world: dict | None = None) -> tuple[list[bpy.types.Object], list[bpy.types.Object]]:
    """Single arrow along the joint axis: shaft + cone head. Revolute joints
    additionally get a small torus ring at the base to flag rotation. Matches
    the user's legend reference (yellow prismatic / red revolute)."""
    type_colors = style["type_colors"]
    rough = style.get("roughness", 0.45)
    metal = style.get("metallic", 0.05)
    mats = {
        jtype: make_material_overlay_solid(f"joint_{style_name}_{jtype}",
                                           type_colors.get(jtype, _AMBER), rough, metal)
        for jtype in ("revolute", "continuous", "prismatic")
    }

    roots: list[bpy.types.Object] = []
    # Per-joint scaling: arrows fit the *child link* they articulate, not the
    # whole-model diagonal. Floor at 25% of the global unit so the sizing
    # never collapses to invisible on tiny links (e.g. windmill blade hubs).
    floor_unit = unit * 0.25
    for joint in joints.values():
        if joint.jtype not in JOINT_COLORS:
            continue
        joint_empty = joint_empties.get(joint.name)
        if joint_empty is None:
            continue

        if link_diag and joint.child in link_diag:
            j_unit = max(link_diag[joint.child] * 0.6, floor_unit)
            j_unit = min(j_unit, unit)
        else:
            j_unit = unit

        length    = j_unit * style["icon_length_factor"]
        shaft_r   = j_unit * style["icon_shaft_radius_factor"]
        head_r    = j_unit * style["icon_head_radius_factor"]
        head_h    = j_unit * style["icon_head_height_factor"]
        ring_r    = j_unit * style.get("icon_ring_radius_factor", 0.18)
        ring_tube = j_unit * style.get("icon_ring_tube_factor", 0.022)

        axis = Vector(joint.axis)
        if axis.length < 1e-9:
            axis = Vector((0, 0, 1))
        axis.normalize()
        mat = mats[joint.jtype]

        root = bpy.data.objects.new(f"joint_overlay_{joint.name}", None)
        root.empty_display_type = "PLAIN_AXES"
        root.empty_display_size = 0.02
        bpy.context.collection.objects.link(root)
        root.parent = joint_empty.parent
        # Revolute / continuous anchor at the joint origin so the ring sits on
        # the rotation center. Prismatic anchors at the child link's bbox center
        # so the arrow lines up with the visible moving part (e.g. keycap),
        # since URDF prismatic origins typically sit on the parent mounting face.
        if (joint.jtype == "prismatic" and link_center_world
                and joint.child in link_center_world and root.parent is not None):
            parent_inv = root.parent.matrix_world.inverted()
            root.location = parent_inv @ link_center_world[joint.child]
        else:
            root.location = Vector(joint.origin_xyz)
        root.rotation_mode = "XYZ"
        root.rotation_euler = Euler(joint.origin_rpy, "XYZ")
        root.hide_render = True
        root.hide_viewport = True
        roots.append(root)

        if style.get("symmetric"):
            shaft_base = axis * (-length / 2.0)
            head_tip = axis * (length / 2.0)
        else:
            shaft_base = Vector((0.0, 0.0, 0.0))
            head_tip = axis * length
        shaft_top = head_tip - axis * head_h
        shaft = _create_cylinder_between(f"joint_shaft_{joint.name}", shaft_base, shaft_top, shaft_r, mat)
        head = _create_cone_head(f"joint_head_{joint.name}", head_tip, axis, head_h, head_r, mat)
        shaft.parent = root
        head.parent = root

        if joint.jtype in ("revolute", "continuous"):
            ring = _create_torus_ring(f"joint_ring_{joint.name}", shaft_base, axis, ring_r, ring_tube, mat)
            ring.parent = root

    return roots, []


def build_joint_overlay(joints, joint_empties, half_extents, vis_objects=None) -> tuple[list[bpy.types.Object], list[bpy.types.Object]]:
    style_name, style = _resolve_overlay_style()
    print(f"[joint_overlay] style: {style_name}")

    model_extent = max((half_extents * 2).length, 1.0)
    # Scale freely with model size so per-joint sizing isn't capped on big
    # objects (e.g. a 10 m windmill needs ~1 m markers, not 0.5 m).
    unit = max(0.10, model_extent * 0.12)

    # Per-link bbox diagonal + world center; used by per-joint sizing/anchoring
    # so a tiny child link (e.g. one keyboard cap) gets a tiny indicator
    # centered on the visible keycap, not on the case-side joint origin.
    link_diag: dict[str, float] = {}
    link_center_world: dict[str, Vector] = {}
    if vis_objects:
        bpy.context.view_layer.update()
        per_link: dict[str, list[Vector]] = {}
        for lname, obj, _ in vis_objects:
            pts = per_link.setdefault(lname, [])
            for corner in obj.bound_box:
                pts.append(obj.matrix_world @ Vector(corner))
        for lname, pts in per_link.items():
            mn = Vector((min(p.x for p in pts), min(p.y for p in pts), min(p.z for p in pts)))
            mx = Vector((max(p.x for p in pts), max(p.y for p in pts), max(p.z for p in pts)))
            link_diag[lname] = max((mx - mn).length, 1e-4)
            link_center_world[lname] = (mn + mx) / 2.0

    if style.get("icon_style") == "chair":
        return _build_chair_style_overlay(joints, joint_empties, unit, style_name, style)
    if style.get("icon_style") == "axis_arrow":
        return _build_axis_arrow_overlay(joints, joint_empties, unit, style_name, style,
                                         link_diag, link_center_world)

    line_half  = unit * style["axis_len_factor"]
    line_thick = unit * style["axis_thick_factor"]
    dot_r      = unit * style["dot_factor"]
    arc_r      = unit * style["arc_radius_factor"]
    arc_tube   = unit * style["arc_tube_factor"]
    arc_head_len = arc_r * style["arc_head_len_ratio"]
    arc_head_w   = arc_r * style["arc_head_w_ratio"]
    arc_sweep_rad = math.radians(style["arc_sweep_deg"])
    arc_start_rad = math.radians(style["arc_start_deg"])
    cone_h = max(unit * 0.16, line_half * 0.4) if style["axis"] else unit * 0.16
    cone_w = max(unit * 0.068, line_thick * 6)
    em_strength = style["emission_strength"]
    em_alpha = style["alpha"]

    if style["color_mode"] == "type":
        type_colors = style["type_colors"]
        mats = {
            jtype: make_material_emission(f"joint_{style_name}_{jtype}",
                                          type_colors.get(jtype, _AMBER),
                                          em_strength, em_alpha)
            for jtype in ("revolute", "continuous", "prismatic")
        }
    else:
        single = style["single_color"]
        shared = make_material_emission(f"joint_{style_name}_single", single, em_strength, em_alpha)
        mats = {jtype: shared for jtype in ("revolute", "continuous", "prismatic")}

    roots: list[bpy.types.Object] = []
    labels: list[bpy.types.Object] = []
    for joint in joints.values():
        if joint.jtype not in JOINT_COLORS:
            continue
        joint_empty = joint_empties.get(joint.name)
        if joint_empty is None:
            continue

        axis = Vector(joint.axis)
        if axis.length < 1e-9:
            axis = Vector((0, 0, 1))
        axis.normalize()
        mat = mats[joint.jtype]

        root = bpy.data.objects.new(f"joint_overlay_{joint.name}", None)
        root.empty_display_type = "PLAIN_AXES"
        root.empty_display_size = 0.02
        bpy.context.collection.objects.link(root)
        root.parent = joint_empty.parent
        root.location = Vector(joint.origin_xyz)
        root.rotation_mode = "XYZ"
        root.rotation_euler = Euler(joint.origin_rpy, "XYZ")
        root.hide_render = True
        root.hide_viewport = True
        roots.append(root)

        if style["axis"] and line_half > 1e-6:
            axis_obj = _create_cylinder_between(f"joint_axis_{joint.name}",
                                                axis * -line_half, axis * line_half, line_thick, mat)
            axis_obj.parent = root

        origin_dot = _create_sphere(f"joint_origin_{joint.name}", dot_r, mat)
        origin_dot.parent = root

        if style["arc"] and joint.jtype in ("revolute", "continuous") and arc_r > 1e-6:
            for obj in _create_arc_arrow(f"joint_arc_{joint.name}",
                                         axis, 0.0, arc_r, arc_tube,
                                         arc_head_len, arc_head_w, mat,
                                         start_angle=arc_start_rad,
                                         sweep=arc_sweep_rad,
                                         double_head=bool(style.get("arc_double_head"))):
                obj.parent = root
        elif joint.jtype == "prismatic" and style["axis"] and line_half > 1e-6:
            pos = axis * line_half
            neg = axis * -line_half
            for obj in (
                _create_cone_head(f"joint_prismatic_{joint.name}_pos", pos, axis, cone_h, cone_w, mat),
                _create_cone_head(f"joint_prismatic_{joint.name}_neg", neg, -axis, cone_h, cone_w, mat),
            ):
                obj.parent = root

    return roots, labels


def _set_overlay_visibility(joint_overlay_roots: list[bpy.types.Object], visible: bool) -> None:
    for root in joint_overlay_roots:
        root.hide_render = not visible
        root.hide_viewport = not visible
        for child in root.children_recursive:
            child.hide_render = not visible
            child.hide_viewport = not visible


def orient_joint_labels_to_camera(joint_overlay_labels: list[bpy.types.Object], camera: bpy.types.Object) -> None:
    for label in joint_overlay_labels:
        world_pos = label.matrix_world.translation
        direction = camera.location - world_pos
        if direction.length < 1e-9:
            continue
        label.rotation_mode = "QUATERNION"
        label.rotation_quaternion = direction.to_track_quat("Z", "Y")


def build_scene(links, joints, materials, urdf_dir: Path):
    child_joints: dict[str, list[Joint]] = {n: [] for n in links}
    for j in joints.values():
        child_joints.setdefault(j.parent, []).append(j)

    all_children = {j.child for j in joints.values()}
    root_links = [n for n in links if n not in all_children]
    root_name = root_links[0] if root_links else next(iter(links))

    # per-link visual Blender materials (keyed by mat_name)
    link_vis_mats: dict[str, dict[str, bpy.types.Material]] = {}
    for lname, link in links.items():
        mats = {}
        for vis in link.visuals:
            mname = vis.material_name or "_default"
            if mname not in mats:
                rgba = materials.get(mname, [0.75, 0.75, 0.75, 1.0])
                mats[mname] = make_material_visual(f"vis_{lname}_{mname}", rgba, mname)
        if "_default" not in mats:
            mats["_default"] = make_material_visual(f"vis_{lname}__default",
                                                    [0.75, 0.75, 0.75, 1.0], "")
        link_vis_mats[lname] = mats

    # segmentation colors per link; collision colors follow the viewer debug palette per collision primitive
    link_names_ordered = list(links.keys())
    seg_colors: dict[str, tuple] = {}
    for i, lname in enumerate(link_names_ordered):
        seg_colors[lname] = SEG_PALETTE[i % len(SEG_PALETTE)]

    seg_mats: dict[str, bpy.types.Material] = {}
    for lname in link_names_ordered:
        seg_mats[lname] = make_material_seg(f"seg_{lname}", seg_colors[lname])
    collision_mats = [
        make_material_collision(f"col_elem_{i}", rgb)
        for i, rgb in enumerate(COL_PALETTE)
    ]
    collision_edge_mats = [
        make_material_collision_edge(f"col_edge_{i}")
        for i, _rgb in enumerate(COL_PALETTE)
    ]

    joint_empties: dict[str, bpy.types.Object] = {}
    link_empties: dict[str, bpy.types.Object] = {}
    vis_objects: list[tuple[str, bpy.types.Object, str]] = []  # (link_name, obj, mat_name)
    col_objects: list[tuple[bpy.types.Object, bpy.types.Object | None, int]] = []

    def build_link(link_name: str, parent_obj=None):
        link = links[link_name]

        le = bpy.data.objects.new(f"link_{link_name}", None)
        le.empty_display_type = "PLAIN_AXES"
        le.empty_display_size = 0.05
        bpy.context.collection.objects.link(le)
        if parent_obj:
            le.parent = parent_obj
        link_empties[link_name] = le

        vmats = link_vis_mats[link_name]
        for vis in link.visuals:
            obj = _create_mesh_obj(f"vis_{link_name}_{vis.name}", vis, urdf_dir)
            if obj is None:
                continue
            obj.parent = le
            obj.location = Vector(vis.origin_xyz)
            obj.rotation_mode = "XYZ"
            obj.rotation_euler = Euler(vis.origin_rpy, "XYZ")
            mname = vis.material_name or "_default"
            mat = vmats.get(mname) or vmats["_default"]
            if obj.data.materials:
                obj.data.materials[0] = mat
            else:
                obj.data.materials.append(mat)
            vis_objects.append((link_name, obj, mname))

        for coll in link.collisions:
            obj = _create_mesh_obj(f"col_{link_name}_{coll.name}", coll, urdf_dir)
            if obj is None:
                continue
            obj.parent = le
            obj.location = Vector(coll.origin_xyz)
            obj.rotation_mode = "XYZ"
            obj.rotation_euler = Euler(coll.origin_rpy, "XYZ")
            # material assigned later in set_mode_collision using per-element index
            obj.hide_render = True
            obj.hide_viewport = True
            col_idx = len(col_objects)
            mat_idx = col_idx % len(collision_mats)
            edge_obj = _add_edge_curve_child(obj, collision_edge_mats[mat_idx], f"edge_{link_name}_{coll.name}")
            col_objects.append((obj, edge_obj, mat_idx))

        for jnt in child_joints.get(link_name, []):
            je = bpy.data.objects.new(f"joint_{jnt.name}", None)
            je.empty_display_type = "ARROWS"
            je.empty_display_size = 0.05
            bpy.context.collection.objects.link(je)
            je.parent = le
            je.location = Vector(jnt.origin_xyz)
            je.rotation_mode = "XYZ"
            je.rotation_euler = Euler(jnt.origin_rpy, "XYZ")
            joint_empties[jnt.name] = je
            build_link(jnt.child, parent_obj=je)

    build_link(root_name)

    return (
        link_empties,
        joint_empties,
        vis_objects,
        col_objects,
        seg_mats,
        collision_mats,
        seg_colors,
        link_vis_mats,
    )


def set_joint_pose(joint_empties, joints, degrees: float):
    """Set a static articulation pose.

    `degrees` is treated as a target articulation magnitude. Each joint is then
    clamped/scaled to its own URDF limits so e.g. piano keys with a 4.5 mm
    prismatic travel don't fly past their stops at motion_30.
    """
    bpy.context.scene.frame_set(0)
    alpha = max(0.0, min(abs(degrees) / 30.0, 1.0))
    sign = 1.0 if degrees >= 0 else -1.0
    base_values: dict[str, float] = {}

    for jname, j in joints.items():
        if j.jtype in ("continuous", "revolute"):
            requested = math.radians(degrees)
            if j.jtype == "revolute" and j.limit_lower is not None and j.limit_upper is not None:
                base_values[jname] = max(j.limit_lower, min(requested, j.limit_upper))
            else:
                base_values[jname] = requested
        elif j.jtype == "prismatic":
            if j.limit_lower is not None and j.limit_upper is not None:
                lo, hi = j.limit_lower, j.limit_upper
                target = hi if sign >= 0 else lo
                base_values[jname] = lo + alpha * (target - lo) if sign >= 0 else hi + alpha * (target - hi)
            else:
                base_values[jname] = 0.1 * alpha * sign
        else:
            base_values[jname] = 0.0

    for jname, j in joints.items():
        je = joint_empties.get(jname)
        if je is None:
            continue
        if j.jtype not in ("continuous", "revolute", "prismatic"):
            continue

        value = base_values.get(j.mimic_joint, base_values[jname]) * j.mimic_multiplier
        axis = Vector(j.axis)
        if axis.length < 1e-9:
            axis = Vector((0, 0, 1))
        axis.normalize()

        if j.jtype in ("continuous", "revolute"):
            je.rotation_mode = "QUATERNION"
            rpy_q = Euler(j.origin_rpy, "XYZ").to_quaternion()
            je.rotation_quaternion = rpy_q @ Quaternion(axis, value)
            je.location = Vector(j.origin_xyz)
        else:
            je.rotation_mode = "XYZ"
            je.rotation_euler = Euler(j.origin_rpy, "XYZ")
            je.location = Vector(j.origin_xyz) + axis * value

    bpy.context.view_layer.update()


def get_scene_bounds(vis_objects):
    """World-space robust AABB of visual objects. Returns (center, half_extents)."""
    points: list[Vector] = []
    bpy.context.view_layer.update()
    for _, obj, _ in vis_objects:
        for corner in obj.bound_box:
            points.append(obj.matrix_world @ Vector(corner))
    if not points:
        return Vector((0, 0, 2)), Vector((3, 1, 3))

    def bounds_from(values: list[Vector]) -> tuple[Vector, Vector]:
        min_co = Vector((min(p.x for p in values), min(p.y for p in values), min(p.z for p in values)))
        max_co = Vector((max(p.x for p in values), max(p.y for p in values), max(p.z for p in values)))
        return min_co, max_co

    def percentile(vals: list[float], q: float) -> float:
        ordered = sorted(vals)
        if len(ordered) == 1:
            return ordered[0]
        pos = (len(ordered) - 1) * q
        lo = math.floor(pos)
        hi = math.ceil(pos)
        if lo == hi:
            return ordered[lo]
        return ordered[lo] * (hi - pos) + ordered[hi] * (pos - lo)

    full_min, full_max = bounds_from(points)
    if len(points) >= 80:
        min_co = Vector((
            percentile([p.x for p in points], 0.02),
            percentile([p.y for p in points], 0.02),
            percentile([p.z for p in points], 0.02),
        ))
        max_co = Vector((
            percentile([p.x for p in points], 0.98),
            percentile([p.y for p in points], 0.98),
            percentile([p.z for p in points], 0.98),
        ))
        full_diag = (full_max - full_min).length
        robust_diag = max((max_co - min_co).length, 1e-6)
        if full_diag / robust_diag > 2.5:
            print("[render_urdf_viz] Robust camera bounds trimmed extreme visual outliers "
                  f"(full/robust diagonal ratio {full_diag / robust_diag:.1f}).")
    else:
        min_co, max_co = full_min, full_max

    center = (min_co + max_co) / 2
    half = (max_co - min_co) / 2
    # Tiny floor only for degenerate axes (planar parts) — must not bloat small objects.
    floor = 1e-3
    half.x = max(half.x, floor)
    half.y = max(half.y, floor)
    half.z = max(half.z, floor)
    return center, half


try:
    _CAM_ELEVATION_DEG = float(os.environ.get("CAM_ELEVATION_DEG", "20"))
except ValueError:
    _CAM_ELEVATION_DEG = 20  # elevation used for all cameras


def _fit_camera_distance(half_extents, res_x: int, res_y: int, azimuth_deg: float) -> float:
    """Camera distance so the full bounding box fits in frame, accounting for camera elevation."""
    sensor_w = 36.0  # mm (Blender default)
    sensor_h = sensor_w * res_y / res_x
    half_fov_h = math.atan(sensor_w / 2 / _LENS_MM)
    half_fov_v = math.atan(sensor_h / 2 / _LENS_MM)
    try:
        MARGIN = float(os.environ.get("CAM_MARGIN", "1.08"))
    except ValueError:
        MARGIN = 1.08
    hx, hy, hz = half_extents.x, half_extents.y, half_extents.z

    az = math.radians(azimuth_deg)
    el = math.radians(_CAM_ELEVATION_DEG)
    right = Vector((math.cos(az), -math.sin(az), 0.0))
    up = Vector((
        -math.sin(el) * math.sin(az),
        -math.sin(el) * math.cos(az),
        math.cos(el),
    ))
    apparent_h = hx * abs(right.x) + hy * abs(right.y) + hz * abs(right.z)
    apparent_v = hx * abs(up.x) + hy * abs(up.y) + hz * abs(up.z)

    dist_h = apparent_h / math.tan(half_fov_h) * MARGIN
    dist_v = apparent_v / math.tan(half_fov_v) * MARGIN
    return max(dist_h, dist_v)


def setup_cameras(center, half_extents, res_x: int = 1920, res_y: int = 1080):
    cameras = []
    try:
        n_angles = max(1, int(os.environ.get("CAM_NUM_ANGLES", "4")))
    except ValueError:
        n_angles = 4
    if n_angles == 4:
        angles = [
            (0, _CAM_ELEVATION_DEG, "front"),
            (90, _CAM_ELEVATION_DEG, "right"),
            (180, _CAM_ELEVATION_DEG, "back"),
            (270, _CAM_ELEVATION_DEG, "left"),
        ]
    else:
        angles = [
            (i * 360.0 / n_angles, _CAM_ELEVATION_DEG, f"az{i * 360.0 / n_angles:.0f}")
            for i in range(n_angles)
        ]
    print(f"[render_urdf_viz] Camera bbox half-extents: "
          f"x={half_extents.x:.2f} y={half_extents.y:.2f} z={half_extents.z:.2f}")

    for i, (az, el, _label) in enumerate(angles):
        dist = _fit_camera_distance(half_extents, res_x, res_y, az)
        print(f"[render_urdf_viz] Camera {i} {_label}: distance {dist:.2f}m")
        az_r = math.radians(az)
        el_r = math.radians(el)
        x = center.x + dist * math.cos(el_r) * math.sin(az_r)
        y = center.y + dist * math.cos(el_r) * math.cos(az_r)
        z = center.z + dist * math.sin(el_r)

        cam_data = bpy.data.cameras.new(f"cam_{i}")
        cam_data.lens = _LENS_MM
        cam_data.clip_start = 0.01
        cam_data.clip_end = dist * 10
        cam_obj = bpy.data.objects.new(f"camera_{i}", cam_data)
        bpy.context.collection.objects.link(cam_obj)
        cam_obj.location = Vector((x, y, z))

        direction = center - cam_obj.location
        rot_quat = direction.to_track_quat("-Z", "Y")
        cam_obj.rotation_euler = rot_quat.to_euler()

        cameras.append(cam_obj)

    return cameras


def setup_lighting(hdri_path: str | None = None):
    world = bpy.context.scene.world
    if world is None:
        world = bpy.data.worlds.new("World")
        bpy.context.scene.world = world
    world.use_nodes = True
    nt = world.node_tree
    nt.nodes.clear()

    out   = nt.nodes.new("ShaderNodeOutputWorld")
    bg    = nt.nodes.new("ShaderNodeBackground")
    nt.links.new(bg.outputs["Background"], out.inputs["Surface"])

    if hdri_path and Path(hdri_path).exists():
        env     = nt.nodes.new("ShaderNodeTexEnvironment")
        mapping = nt.nodes.new("ShaderNodeMapping")
        texco   = nt.nodes.new("ShaderNodeTexCoord")

        env.image = bpy.data.images.load(hdri_path, check_existing=True)
        env.image.colorspace_settings.name = "Linear Rec.709"

        nt.links.new(texco.outputs["Generated"], mapping.inputs["Vector"])
        nt.links.new(mapping.outputs["Vector"],  env.inputs["Vector"])
        nt.links.new(env.outputs["Color"],        bg.inputs["Color"])

        # Rotate HDRI so the brightest light comes from the camera side →
        # shadow fans dramatically behind the object, visible to viewer.
        # Z = azimuth spin, X = slight tilt for a 3/4 sun angle.
        mapping.inputs["Rotation"].default_value = (
            math.radians(15),   # tilt sun slightly downward for long shadow
            0.0,
            math.radians(165),  # ~half-turn: sun from front-left of camera
        )
        bg.inputs["Strength"].default_value = 1.6
        print(f"[render_urdf_viz] HDRI loaded: {hdri_path}")
    else:
        # fallback: plain sky if no HDRI
        bg.inputs["Color"].default_value = (0.52, 0.58, 0.68, 1.0)
        bg.inputs["Strength"].default_value = 0.5
        if hdri_path:
            print(f"[render_urdf_viz] HDRI not found ({hdri_path}), using fallback")



def configure_render(samples: int, res_x: int, res_y: int):
    scene = bpy.context.scene
    scene.render.engine = "CYCLES"
    scene.cycles.samples = samples
    scene.cycles.use_denoising = True
    scene.render.resolution_x = res_x
    scene.render.resolution_y = res_y
    # transparent background — PNG alpha channel
    scene.render.film_transparent = True
    scene.render.image_settings.file_format = "PNG"
    scene.render.image_settings.color_mode = "RGBA"
    scene.render.image_settings.color_depth = "8"

    # prefer GPU (Metal on Apple Silicon / CUDA on NVIDIA)
    try:
        cycles_prefs = bpy.context.preferences.addons["cycles"].preferences
        for device_type in ("METAL", "CUDA", "OPTIX", "HIP"):
            try:
                cycles_prefs.compute_device_type = device_type
                cycles_prefs.get_devices()
                for d in cycles_prefs.devices:
                    d.use = True
                scene.cycles.device = "GPU"
                print(f"[render_urdf_viz] GPU render device: {device_type}")
                break
            except Exception:
                continue
    except Exception:
        pass


# ── render mode switches ──────────────────────────────────────────────────────

def _show_vis(vis_objects, col_objects):
    for _, obj, _ in vis_objects:
        obj.hide_render = False
        obj.hide_viewport = False
    for obj, edge_obj, _ in col_objects:
        obj.hide_render = True
        obj.hide_viewport = True
        if edge_obj is not None:
            edge_obj.hide_render = True
            edge_obj.hide_viewport = True


def _restore_vis_mats(vis_objects, link_vis_mats):
    for lname, obj, mname in vis_objects:
        vmats = link_vis_mats[lname]
        mat = vmats.get(mname) or vmats.get("_default")
        if mat and obj.data.materials:
            obj.data.materials[0] = mat


def set_mode_motion_degrees(degrees: float, joint_empties, joints, vis_objects, col_objects, link_vis_mats):
    _show_vis(vis_objects, col_objects)
    _restore_vis_mats(vis_objects, link_vis_mats)
    set_joint_pose(joint_empties, joints, degrees)


def set_joint_pose_halfway(joint_empties, joints):
    """Set each articulated joint to a fraction of its URDF range (default 0.7).

    Override the fraction with the MOTION_HALF_FRAC env var (e.g. 0.5 for true
    midpoint, 0.9 for almost full). Continuous joints have no limits so they
    get a fixed quarter turn (π/2).
    """
    try:
        frac = float(os.environ.get("MOTION_HALF_FRAC", "0.7"))
    except ValueError:
        frac = 0.7
    bpy.context.scene.frame_set(0)
    for jname, j in joints.items():
        je = joint_empties.get(jname)
        if je is None or j.jtype not in ("revolute", "continuous", "prismatic"):
            continue
        axis = Vector(j.axis)
        if axis.length < 1e-9:
            axis = Vector((0, 0, 1))
        axis.normalize()
        rpy_q = Euler(j.origin_rpy, "XYZ").to_quaternion()

        if j.jtype == "revolute":
            if j.limit_lower is not None and j.limit_upper is not None:
                value = j.limit_lower + frac * (j.limit_upper - j.limit_lower)
            else:
                value = 0.0
            je.rotation_mode = "QUATERNION"
            je.rotation_quaternion = rpy_q @ Quaternion(axis, value)
            je.location = Vector(j.origin_xyz)
        elif j.jtype == "continuous":
            je.rotation_mode = "QUATERNION"
            je.rotation_quaternion = rpy_q @ Quaternion(axis, math.pi / 2)
            je.location = Vector(j.origin_xyz)
        else:  # prismatic
            if j.limit_lower is not None and j.limit_upper is not None:
                travel = j.limit_lower + frac * (j.limit_upper - j.limit_lower)
            else:
                travel = 0.05
            je.rotation_mode = "XYZ"
            je.rotation_euler = Euler(j.origin_rpy, "XYZ")
            je.location = Vector(j.origin_xyz) + axis * travel
    bpy.context.view_layer.update()


def set_mode_motion_halfway(joint_empties, joints, vis_objects, col_objects, link_vis_mats):
    _show_vis(vis_objects, col_objects)
    _restore_vis_mats(vis_objects, link_vis_mats)
    set_joint_pose_halfway(joint_empties, joints)


def compute_random_joint_values(joints):
    """One sample per articulated joint for motion_random.

    Spatial checkerboard: bin each joint into rows by its Y origin and assign
    a column index inside each row by X origin. Joints with even (row + col)
    parity are fully pressed; the rest stay at rest. Result is ~50% pressed,
    evenly distributed across the layout. RANDOM_FLIP_PROB (default 0.0)
    randomly flips individual joints for variation; RANDOM_SEED (default 0)
    selects which parity is pressed.
    """
    import random as _r
    from collections import defaultdict
    try:
        seed = int(os.environ.get("RANDOM_SEED", "0"))
    except ValueError:
        seed = 0
    try:
        flip_prob = float(os.environ.get("RANDOM_FLIP_PROB", "0.15"))
    except ValueError:
        flip_prob = 0.15
    rng = _r.Random(seed)

    try:
        depth_mult = float(os.environ.get("RANDOM_PRESS_DEPTH", "2.0"))
    except ValueError:
        depth_mult = 2.0
    artic = [(n, j) for n, j in joints.items()
             if j.jtype in ("revolute", "continuous", "prismatic")]
    pressed: set[str] = set()
    if artic:
        ys = [j.origin_xyz[1] for _, j in artic]
        y_min, y_max = min(ys), max(ys)
        y_range = max(y_max - y_min, 1e-9)
        # ~6 rows for a keyboard; degenerate shapes collapse to one row.
        row_step = y_range / 6
        rows: dict[int, list] = defaultdict(list)
        for n, j in artic:
            r = int(round((j.origin_xyz[1] - y_min) / row_step)) if row_step > 1e-9 else 0
            rows[r].append((n, j))
        for r in rows:
            rows[r].sort(key=lambda kj: kj[1].origin_xyz[0])
        parity_target = rng.randint(0, 1)
        for r, items in rows.items():
            for c, (n, _) in enumerate(items):
                if (r + c) % 2 == parity_target:
                    pressed.add(n)
        for n, _ in artic:
            if flip_prob > 0.0 and rng.random() < flip_prob:
                if n in pressed:
                    pressed.discard(n)
                else:
                    pressed.add(n)

    values: dict[str, float] = {}
    for jname, j in joints.items():
        if j.jtype not in ("revolute", "continuous", "prismatic"):
            continue
        f = depth_mult if jname in pressed else 0.0
        if j.jtype == "revolute":
            if j.limit_lower is not None and j.limit_upper is not None:
                values[jname] = j.limit_lower + f * (j.limit_upper - j.limit_lower)
            else:
                values[jname] = 0.0
        elif j.jtype == "continuous":
            values[jname] = f * (math.pi / 2)
        else:  # prismatic
            if j.limit_lower is not None and j.limit_upper is not None:
                values[jname] = j.limit_lower + f * (j.limit_upper - j.limit_lower)
            else:
                values[jname] = 0.05 * f
    return values


def set_joint_pose_random(joint_empties, joints, values):
    bpy.context.scene.frame_set(0)
    for jname, j in joints.items():
        je = joint_empties.get(jname)
        if je is None or j.jtype not in ("revolute", "continuous", "prismatic"):
            continue
        axis = Vector(j.axis)
        if axis.length < 1e-9:
            axis = Vector((0, 0, 1))
        axis.normalize()
        rpy_q = Euler(j.origin_rpy, "XYZ").to_quaternion()
        src = values.get(j.mimic_joint, values.get(jname, 0.0))
        value = src * j.mimic_multiplier

        if j.jtype in ("revolute", "continuous"):
            je.rotation_mode = "QUATERNION"
            je.rotation_quaternion = rpy_q @ Quaternion(axis, value)
            je.location = Vector(j.origin_xyz)
        else:
            je.rotation_mode = "XYZ"
            je.rotation_euler = Euler(j.origin_rpy, "XYZ")
            je.location = Vector(j.origin_xyz) + axis * value
    bpy.context.view_layer.update()


def set_mode_motion_random(joint_empties, joints, vis_objects, col_objects, link_vis_mats, values):
    _show_vis(vis_objects, col_objects)
    _restore_vis_mats(vis_objects, link_vis_mats)
    set_joint_pose_random(joint_empties, joints, values)


def set_mode_segmentation(vis_objects, col_objects, seg_mats):
    _show_vis(vis_objects, col_objects)
    for lname, obj, _ in vis_objects:
        mat = seg_mats[lname]
        if obj.data.materials:
            obj.data.materials[0] = mat
        else:
            obj.data.materials.append(mat)
    bpy.context.scene.frame_set(0)


def set_mode_collision(vis_objects, col_objects, collision_mats):
    for _, obj, _ in vis_objects:
        obj.hide_render = True
        obj.hide_viewport = True
    for obj, edge_obj, mat_idx in col_objects:
        obj.hide_render = False
        obj.hide_viewport = False
        mat = collision_mats[mat_idx]
        if not obj.data.materials:
            obj.data.materials.append(mat)
        else:
            obj.data.materials[0] = mat
        if edge_obj is not None:
            edge_obj.hide_render = False
            edge_obj.hide_viewport = False
    bpy.context.scene.frame_set(0)


def set_mode_joint_overlay(joint_empties, joints, vis_objects, col_objects, ghost_mat,
                           joint_overlay_roots, link_vis_mats):
    _show_vis(vis_objects, col_objects)
    _, style = _resolve_overlay_style()
    skip_ghost = bool(style.get("skip_ghost"))
    pose_deg = style.get("pose_degrees", 30)

    if skip_ghost:
        # Keep the real per-link visual materials so the overlay icons sit on
        # top of the normally-rendered textured model (chair-reference style).
        _restore_vis_mats(vis_objects, link_vis_mats)
    else:
        for _, obj, _ in vis_objects:
            if obj.data.materials:
                obj.data.materials[0] = ghost_mat
            else:
                obj.data.materials.append(ghost_mat)
    set_joint_pose(joint_empties, joints, pose_deg)
    _set_overlay_visibility(joint_overlay_roots, True)


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    argv = sys.argv
    if "--" in argv:
        argv = argv[argv.index("--") + 1:]
    else:
        argv = []

    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", required=True, type=Path)
    ap.add_argument("--output", required=True, type=Path)
    ap.add_argument("--samples", type=int, default=128)
    ap.add_argument("--resolution", default="1920x1080")
    ap.add_argument("--hdri", default=None, help="Path to .exr/.hdr environment map")
    ap.add_argument("--modes", default=None,
                    help="Comma-separated subset of modes to render, e.g. motion_1,motion_30")
    ap.add_argument("--no-joint-overlay", action="store_true",
                    help="Skip the amber joint overlay on motion renders")
    args = ap.parse_args(argv)

    res_x, res_y = (int(x) for x in args.resolution.lower().split("x"))
    out_dir = args.output
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"[render_urdf_viz] URDF: {args.urdf}")
    print(f"[render_urdf_viz] Output: {out_dir}")

    links, joints, materials = parse_urdf(args.urdf)
    print(f"[render_urdf_viz] Links: {list(links.keys())}")
    print(f"[render_urdf_viz] Materials: {list(materials.keys())}")

    clear_scene()
    configure_render(args.samples, res_x, res_y)

    (
        link_empties,
        joint_empties,
        vis_objects,
        col_objects,
        seg_mats,
        collision_mats,
        seg_colors,
        link_vis_mats,
    ) = build_scene(links, joints, materials, args.urdf.parent)

    # Compute bounds across all articulation poses so motion_30 / motion_half never clip.
    all_points: list[Vector] = []
    for deg in (0, 1, 30):
        set_joint_pose(joint_empties, joints, deg)
        bpy.context.view_layer.update()
        for _, obj, _ in vis_objects:
            for corner in obj.bound_box:
                all_points.append(obj.matrix_world @ Vector(corner))
    set_joint_pose_halfway(joint_empties, joints)
    for _, obj, _ in vis_objects:
        for corner in obj.bound_box:
            all_points.append(obj.matrix_world @ Vector(corner))
    random_joint_values = compute_random_joint_values(joints)
    # Only expand the camera frame for the random-press pose when the user is
    # actually rendering motion_random — otherwise the frame zooms out to fit
    # an unseen pose (visible as a "shift" in joint_overlay renders).
    will_render_random = args.modes is None or "motion_random" in [m.strip() for m in args.modes.split(",")]
    if will_render_random:
        set_joint_pose_random(joint_empties, joints, random_joint_values)
        bpy.context.view_layer.update()
        for _, obj, _ in vis_objects:
            for corner in obj.bound_box:
                all_points.append(obj.matrix_world @ Vector(corner))
    set_joint_pose(joint_empties, joints, 0)
    bpy.context.view_layer.update()

    if all_points:
        min_co = Vector((min(p.x for p in all_points), min(p.y for p in all_points), min(p.z for p in all_points)))
        max_co = Vector((max(p.x for p in all_points), max(p.y for p in all_points), max(p.z for p in all_points)))
        center = (min_co + max_co) / 2
        half_extents = (max_co - min_co) / 2
        # Tiny floor only for degenerate axes — must not bloat small objects.
        floor = 1e-3
        half_extents.x = max(half_extents.x, floor)
        half_extents.y = max(half_extents.y, floor)
        half_extents.z = max(half_extents.z, floor)
    else:
        center, half_extents = get_scene_bounds(vis_objects)

    print(f"[render_urdf_viz] Bounds (all poses): center={tuple(round(v,2) for v in center)}, "
          f"extents={tuple(round(v,2) for v in half_extents*2)}")

    ghost_mat = make_material_ghost_visual("joint_overlay_ghost_visual")
    joint_overlay_roots, joint_overlay_labels = build_joint_overlay(joints, joint_empties, half_extents, vis_objects)

    cameras = setup_cameras(center, half_extents, res_x, res_y)
    setup_lighting(args.hdri)

    # Optional angle filter via env (e.g. JOINT_OVERLAY_ANGLES=0 or 0,2)
    # Preserves original camera indices so output dirs stay angle_<original_idx>.
    indexed_cameras = list(enumerate(cameras))
    angles_env = os.environ.get("JOINT_OVERLAY_ANGLES", "").strip()
    if angles_env:
        try:
            keep = {int(x) for x in angles_env.split(",") if x.strip()}
            indexed_cameras = [(i, c) for i, c in indexed_cameras if i in keep]
            print(f"[render_urdf_viz] JOINT_OVERLAY_ANGLES filter: keeping {sorted(keep)} → {len(indexed_cameras)} cameras")
        except ValueError:
            print(f"[render_urdf_viz] Bad JOINT_OVERLAY_ANGLES='{angles_env}', ignoring")

    scene = bpy.context.scene

    all_render_modes = [
        ("motion_0",         lambda: set_mode_motion_degrees(0,  joint_empties, joints, vis_objects, col_objects, link_vis_mats)),
        ("motion_1",         lambda: set_mode_motion_degrees(1,  joint_empties, joints, vis_objects, col_objects, link_vis_mats)),
        ("motion_15",        lambda: set_mode_motion_degrees(15, joint_empties, joints, vis_objects, col_objects, link_vis_mats)),
        ("motion_30",        lambda: set_mode_motion_degrees(30, joint_empties, joints, vis_objects, col_objects, link_vis_mats)),
        ("motion_half",      lambda: set_mode_motion_halfway(joint_empties, joints, vis_objects, col_objects, link_vis_mats)),
        ("motion_random",    lambda: set_mode_motion_random(joint_empties, joints, vis_objects, col_objects, link_vis_mats, random_joint_values)),
        ("part_segmentation",lambda: (set_joint_pose(joint_empties, joints, 0), set_mode_segmentation(vis_objects, col_objects, seg_mats))),
        ("collision",        lambda: (set_joint_pose(joint_empties, joints, 0), set_mode_collision(vis_objects, col_objects, collision_mats))),
        ("joint_overlay",    lambda: set_mode_joint_overlay(joint_empties, joints, vis_objects, col_objects,
                                                            ghost_mat, joint_overlay_roots, link_vis_mats)),
    ]
    if args.modes:
        requested = [m.strip() for m in args.modes.split(",")]
        render_modes = [(n, fn) for n, fn in all_render_modes if n in requested]
    else:
        render_modes = all_render_modes

    total = len(indexed_cameras) * len(render_modes)
    done = 0

    # Motion renders that always carry the joint overlay baked in
    _OVERLAY_ON_MOTION = set() if args.no_joint_overlay else {"motion_1", "motion_30", "motion_half", "motion_random"}

    for cam_idx, cam in indexed_cameras:
        scene.camera = cam
        angle_dir = out_dir / f"angle_{cam_idx}"
        angle_dir.mkdir(exist_ok=True)

        for mode_name, setup_fn in render_modes:
            _set_overlay_visibility(joint_overlay_roots, False)
            setup_fn()
            overlay_on = mode_name in _OVERLAY_ON_MOTION or mode_name == "joint_overlay"
            if overlay_on:
                _set_overlay_visibility(joint_overlay_roots, True)
                orient_joint_labels_to_camera(joint_overlay_labels, cam)
            out_path = angle_dir / f"{mode_name}.png"

            always_front = (
                overlay_on
                and bool(_resolve_overlay_style()[1].get("always_in_front"))
            )
            if always_front:
                # Two-pass render so the overlay is never occluded:
                #   bg = model only (overlay hidden)
                #   fg = overlay only (model hidden, transparent film)
                # PIL composite happens in visualize.py since Blender's
                # bundled Python doesn't ship Pillow.
                bg_path = angle_dir / f"{mode_name}__bg.png"
                fg_path = angle_dir / f"{mode_name}__fg.png"

                _set_overlay_visibility(joint_overlay_roots, False)
                scene.render.filepath = str(bg_path)
                print(f"[render_urdf_viz] [{done+1}/{total}] cam {cam_idx} / {mode_name} (bg)")
                bpy.ops.render.render(write_still=True)

                vis_hidden_state = []
                col_hidden_state = []
                for _, obj, _ in vis_objects:
                    vis_hidden_state.append(obj.hide_render)
                    obj.hide_render = True
                for obj, edge_obj, _ in col_objects:
                    col_hidden_state.append((obj.hide_render,
                                             None if edge_obj is None else edge_obj.hide_render))
                    obj.hide_render = True
                    if edge_obj is not None:
                        edge_obj.hide_render = True
                _set_overlay_visibility(joint_overlay_roots, True)
                scene.render.filepath = str(fg_path)
                print(f"[render_urdf_viz] [{done+1}/{total}] cam {cam_idx} / {mode_name} (fg)")
                bpy.ops.render.render(write_still=True)

                for (state,), (_, obj, _) in zip(((s,) for s in vis_hidden_state), vis_objects):
                    obj.hide_render = state
                for (vis_state, edge_state), (obj, edge_obj, _) in zip(col_hidden_state, col_objects):
                    obj.hide_render = vis_state
                    if edge_obj is not None and edge_state is not None:
                        edge_obj.hide_render = edge_state
            else:
                scene.render.filepath = str(out_path)
                print(f"[render_urdf_viz] [{done+1}/{total}] cam {cam_idx} / {mode_name}")
                bpy.ops.render.render(write_still=True)
            done += 1

        # restore for next camera
        set_mode_motion_degrees(0, joint_empties, joints, vis_objects, col_objects, link_vis_mats)

    legend = {
        lname: "#{:02x}{:02x}{:02x}".format(
            int(rgb[0] * 255), int(rgb[1] * 255), int(rgb[2] * 255))
        for lname, rgb in seg_colors.items()
    }
    (out_dir / "legend.json").write_text(json.dumps(legend, indent=2))
    print(f"[render_urdf_viz] Done. {total} images → {out_dir}")


if __name__ == "__main__":
    main()
