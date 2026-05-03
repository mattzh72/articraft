"""URDF -> Blender (single .blend or grid). Self-contained version.

Drop this file plus ``render_urdf_viz.py`` into any folder and the converter
turns one or many URDFs into an animated ``.blend``: links + joints become a
parented Empty hierarchy, every articulated DoF is keyframed across the
timeline, materials are Principled BSDF (PBR roughness/metallic inferred from
the material's name), and real-world scale (URDF meters → Blender meters) is
preserved.

Modes
-----
1) one record into its own .blend::

    blender -b -P teaser/urdf_to_blender.py -- <record_id_or_urdf_path> [out.blend]

2) many records into ONE .blend, grid-arranged at real-world scale::

    blender -b -P teaser/urdf_to_blender.py -- --grid <out.blend> --from <list>
    blender -b -P teaser/urdf_to_blender.py -- --grid <out.blend> <id1> <id2> ...

Animation: every joint sweeps ``0° -> +25° -> 0° -> -25° -> 0°``
simultaneously across the timeline.  ``rv.set_joint_pose`` clamps to per-joint
URDF limits, follows ``<mimic>`` chains, and maps prismatic magnitude to a
fraction of the joint's travel — so a 4 mm prismatic key moves 4 mm and a 2π
continuous joint rotates fully.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path
from typing import Optional

import bpy
from mathutils import Euler, Matrix, Quaternion, Vector

# Pull in the URDF builder. Prefer a local copy next to this script (so the
# folder is portable); fall back to the project's plots/ copy if not present.
HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent
for _cand in (HERE, REPO_ROOT / "plots"):
    if (_cand / "render_urdf_viz.py").exists():
        sys.path.insert(0, str(_cand))
        break
import render_urdf_viz as rv  # noqa: E402


# ---------------------------------------------------------------------------
# Defaults
# ---------------------------------------------------------------------------

RECORDS_DIR = REPO_ROOT / "data" / "records"
MATERIALIZATION_DIR = REPO_ROOT / "data" / "cache" / "record_materialization"
# Default output: a sibling "blender_out" folder next to this script, so
# running the teaser/ copy doesn't write into plots/blender/.
DEFAULT_OUTPUT_DIR = HERE / "blender_out"

FPS = 30
ANIM_FRAMES = 240        # 8 s at 30 fps
# Sentinel magnitude: rv.set_joint_pose clamps revolute to its URDF [lower, upper]
# and saturates prismatic alpha = |deg|/30 at 1.0 (full declared travel) for any
# value >= 30, so passing 1000 means "go to whatever the URDF says is the limit"
# without per-joint tuning. Continuous joints are overridden by spin_continuous
# below, so this value doesn't affect them.
ART_AMP_DEG = 1000.0
N_OSC_CYCLES = 2          # back-and-forth cycles for revolute/prismatic per timeline
SPIN_REVS_PER_TIMELINE = 4  # full revolutions for continuous joints per timeline
GRID_PADDING_M = 0.4

# Environment HDRI used for world lighting (override on the CLI with --hdri).
# Lives next to this script so the folder is portable.
DEFAULT_HDRI = HERE / "symmetrical_garden_02_4k.exr"
FLOOR_MARGIN_M = 2.0     # extra meters of floor beyond the bbox span on each side


# ---------------------------------------------------------------------------
# Scene setup
# ---------------------------------------------------------------------------


def reset_scene() -> None:
    bpy.ops.wm.read_factory_settings(use_empty=True)
    scene = bpy.context.scene
    scene.unit_settings.system = "METRIC"
    scene.unit_settings.scale_length = 1.0
    scene.render.fps = FPS
    scene.frame_start = 1
    scene.frame_end = ANIM_FRAMES
    # Eevee-friendly viewport shading.
    scene.render.engine = "BLENDER_EEVEE_NEXT" if "BLENDER_EEVEE_NEXT" in (
        item.identifier for item in scene.render.bl_rna.properties["engine"].enum_items
    ) else scene.render.engine


def resolve_urdf(arg: str) -> tuple[str, Path]:
    """Accept a record id, an absolute path line from example.txt, or a URDF path."""
    p = Path(arg)
    if p.suffix == ".urdf" and p.exists():
        return p.parent.name, p
    record_id = p.name if p.is_absolute() else arg
    urdf = MATERIALIZATION_DIR / record_id / "model.urdf"
    if not urdf.exists():
        raise FileNotFoundError(
            f"no URDF for {record_id!r}. expected {urdf}. "
            f"run `just compile data/records/{record_id}` first."
        )
    return record_id, urdf


# ---------------------------------------------------------------------------
# Animation (mirrors build_animation.py / viewer behavior)
# ---------------------------------------------------------------------------


def keyframe_joint_pose(joint_empties: dict, joints: dict, frame: int) -> None:
    """Insert location + rotation keyframes on every joint empty at ``frame``."""
    for jname, je in joint_empties.items():
        j = joints.get(jname)
        if j is None or j.jtype not in ("revolute", "continuous", "prismatic"):
            continue
        je.keyframe_insert(data_path="location", frame=frame)
        if je.rotation_mode == "QUATERNION":
            je.keyframe_insert(data_path="rotation_quaternion", frame=frame)
        else:
            je.keyframe_insert(data_path="rotation_euler", frame=frame)


def animate_articulation(
    joint_empties: dict,
    joints: dict,
    start: int,
    end: int,
    amp_deg: float = ART_AMP_DEG,
    n_cycles: int = N_OSC_CYCLES,
    spin_revs: float = SPIN_REVS_PER_TIMELINE,
) -> None:
    """Animate all joints across [start, end].

    Revolute / prismatic joints oscillate ``0 -> +amp -> 0 -> -amp -> 0`` for
    ``n_cycles`` cycles, passing through ``set_joint_pose`` so each joint stays
    inside its URDF limits.  Continuous joints are then overridden with a
    monotonic spin of ``spin_revs`` full revolutions about their URDF axis, so
    they look like real spinning things rather than back-and-forth wiggles.
    """
    duration = max(1, end - start)
    segments = max(1, n_cycles) * 4
    quarter_to_deg = (0.0, +amp_deg, 0.0, -amp_deg)
    for seg in range(segments + 1):
        t = seg / segments
        deg = quarter_to_deg[seg % 4]
        rv.set_joint_pose(joint_empties, joints, deg)
        keyframe_joint_pose(joint_empties, joints, start + round(t * duration))
    bpy.context.view_layer.update()

    spin_continuous(joint_empties, joints, start, end, revs=spin_revs)


def spin_continuous(
    joint_empties: dict,
    joints: dict,
    start: int,
    end: int,
    revs: float,
) -> None:
    """Replace continuous-joint rotation keyframes with a linear spin.

    URDF ``continuous`` joints have no <limit>, so oscillating them through a
    fixed angle looks broken — a wind turbine should rotate, not wiggle.  We
    discard whatever ``rotation_quaternion`` keyframes ``animate_articulation``
    just laid down on each continuous joint's empty and replace them with an
    evenly-spaced ramp from 0 to ``revs * 2π`` about the joint's URDF axis.
    Four sub-keys per revolution keep slerp from short-cutting the rotation.

    Mimic chains whose parent is continuous will desync — they were keyframed
    against the parent's pre-spin pose during the oscillation pass.
    """
    duration = max(1, end - start)
    total_rad = float(revs) * 2.0 * math.pi
    n_steps = max(8, int(math.ceil(abs(revs) * 4)))

    for jname, je in joint_empties.items():
        j = joints.get(jname)
        if j is None or j.jtype != "continuous":
            continue

        axis = Vector(j.axis)
        if axis.length < 1e-9:
            axis = Vector((0.0, 0.0, 1.0))
        axis.normalize()

        # Wipe the rotation_quaternion fcurves the oscillation pass left on
        # this empty so the spin doesn't fight them.
        action = je.animation_data.action if je.animation_data else None
        if action is not None:
            for fc in list(action.fcurves):
                if fc.data_path == "rotation_quaternion":
                    action.fcurves.remove(fc)

        je.rotation_mode = "QUATERNION"
        je.location = Vector(j.origin_xyz)
        rpy_q = Euler(j.origin_rpy, "XYZ").to_quaternion()
        for i in range(n_steps + 1):
            t = i / n_steps
            je.rotation_quaternion = rpy_q @ Quaternion(axis, total_rad * t)
            je.keyframe_insert(
                data_path="rotation_quaternion",
                frame=start + round(t * duration),
            )

        # Constant angular velocity: linear interpolation between keys.
        action = je.animation_data.action if je.animation_data else None
        if action is not None:
            for fc in action.fcurves:
                if fc.data_path == "rotation_quaternion":
                    for kp in fc.keyframe_points:
                        kp.interpolation = "LINEAR"


# ---------------------------------------------------------------------------
# Per-record build
# ---------------------------------------------------------------------------


def _root_link_name(links: dict, joints: dict) -> str:
    children = {j.child for j in joints.values()}
    roots = [n for n in links if n not in children]
    return roots[0] if roots else next(iter(links))


def _move_to_collection(objs, collection: bpy.types.Collection) -> None:
    for o in objs:
        for c in list(o.users_collection):
            c.objects.unlink(o)
        if o.name not in collection.objects:
            collection.objects.link(o)


def build_record(arg: str, collection: Optional[bpy.types.Collection] = None):
    """Parse + build one record into the current scene.

    Returns ``(record_id, links, joints, link_empties, joint_empties, vis_objects, root_empty)``.
    If ``collection`` is given, all objects produced by the build are moved into
    it (so each record sits in its own outliner Collection).
    """
    rid, urdf = resolve_urdf(arg)
    links, joints, materials = rv.parse_urdf(urdf)
    before = set(bpy.data.objects)
    link_empties, joint_empties, vis_objects, *_rest = rv.build_scene(
        links, joints, materials, urdf.parent
    )
    new_objs = [o for o in bpy.data.objects if o not in before]

    # Drop collision meshes + their edge-curve children. They're hidden in
    # render/viewport (so Blender's depsgraph never refreshes their cached
    # world transform), which leaves them anchored to whatever world position
    # they had at build time even after we move the parent holder. We don't
    # need them for animation playback either.
    visible_new: list = []
    for o in new_objs:
        if o.name.startswith("col_") or o.name.startswith("edge_"):
            bpy.data.objects.remove(o, do_unlink=True)
        else:
            visible_new.append(o)

    # Force matrix_parent_inverse to identity so the matrix_basis (location +
    # rotation) that render_urdf_viz set explicitly is the only contribution to
    # the local transform.
    for o in visible_new:
        if o.parent is not None:
            o.matrix_parent_inverse = Matrix.Identity(4)
    bpy.context.view_layer.update()

    if collection is not None:
        _move_to_collection(visible_new, collection)
    root_empty = link_empties[_root_link_name(links, joints)]
    return rid, links, joints, link_empties, joint_empties, vis_objects, root_empty


def measure_world_bbox(vis_objects) -> tuple[Vector, Vector]:
    mn = Vector((math.inf,) * 3)
    mx = Vector((-math.inf,) * 3)
    found = False
    for _, obj, _ in vis_objects:
        if obj.type != "MESH":
            continue
        found = True
        for c in obj.bound_box:
            w = obj.matrix_world @ Vector(c)
            for i in range(3):
                if w[i] < mn[i]:
                    mn[i] = w[i]
                if w[i] > mx[i]:
                    mx[i] = w[i]
    if not found:
        return Vector((0, 0, 0)), Vector((0, 0, 0))
    return mn, mx


def envelope_bbox(vis_objects, joint_empties, joints, amp_deg: float) -> tuple[Vector, Vector]:
    """Bbox sampled across the full articulation range so animation never crops."""
    samples = (-amp_deg, -amp_deg / 2, 0.0, amp_deg / 2, amp_deg)
    mn = Vector((math.inf,) * 3)
    mx = Vector((-math.inf,) * 3)
    for s in samples:
        rv.set_joint_pose(joint_empties, joints, s)
        bpy.context.view_layer.update()
        smn, smx = measure_world_bbox(vis_objects)
        for i in range(3):
            if smn[i] < mn[i]:
                mn[i] = smn[i]
            if smx[i] > mx[i]:
                mx[i] = smx[i]
    rv.set_joint_pose(joint_empties, joints, 0)
    bpy.context.view_layer.update()
    return mn, mx


# ---------------------------------------------------------------------------
# Camera / lighting
# ---------------------------------------------------------------------------


def add_camera_and_lights(center: Vector, span: float) -> None:
    span = max(span, 1.0)
    cam_data = bpy.data.cameras.new("camera")
    cam = bpy.data.objects.new("camera", cam_data)
    bpy.context.collection.objects.link(cam)
    cam.location = (center.x + span * 1.6, center.y - span * 1.8, center.z + span * 1.0)
    direction = center - Vector(cam.location)
    cam.rotation_euler = direction.to_track_quat("-Z", "Y").to_euler()
    bpy.context.scene.camera = cam

    sun_data = bpy.data.lights.new("sun", type="SUN")
    sun_data.energy = 3.0
    sun = bpy.data.objects.new("sun", sun_data)
    sun.location = (center.x, center.y, center.z + span * 1.2)
    sun.rotation_euler = (math.radians(45), math.radians(30), 0)
    bpy.context.collection.objects.link(sun)


def setup_world(hdri_path: Optional[Path]) -> None:
    """Wire an EXR/HDR environment map into the world shader.

    Falls back to a plain sky tint when the file is missing — same behavior as
    ``render_urdf_viz.setup_lighting``.
    """
    rv.setup_lighting(str(hdri_path) if hdri_path else None)


def add_floor(center: Vector, span: float, z: float = 0.0) -> bpy.types.Object:
    """Drop a large neutral plane under the scene at ``z``.

    Sized to comfortably overshoot the framed span so it never crops in render.
    """
    size = max(span * 6.0, 4.0) + FLOOR_MARGIN_M * 2.0
    bpy.ops.mesh.primitive_plane_add(size=size, location=(center.x, center.y, z))
    floor = bpy.context.active_object
    floor.name = "floor"

    mat = bpy.data.materials.new("floor")
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes.get("Principled BSDF")
    if bsdf is not None:
        bsdf.inputs["Base Color"].default_value = (0.55, 0.55, 0.55, 1.0)
        if "Roughness" in bsdf.inputs:
            bsdf.inputs["Roughness"].default_value = 0.85
        if "Metallic" in bsdf.inputs:
            bsdf.inputs["Metallic"].default_value = 0.0
    floor.data.materials.append(mat)
    return floor


# ---------------------------------------------------------------------------
# Single-record mode
# ---------------------------------------------------------------------------


def convert_one(
    arg: str,
    output_path: Optional[Path] = None,
    hdri_path: Optional[Path] = DEFAULT_HDRI,
    add_ground: bool = True,
) -> Path:
    rid, urdf = resolve_urdf(arg)
    print(f"-> {rid}\n   urdf: {urdf}")
    reset_scene()
    rid, links, joints, le, je, vo, root_empty = build_record(arg)
    print(f"   parsed: {len(links)} links, {len(joints)} joints")

    # Drop the assembly to z = 0 so it sits on the floor like in grid mode.
    rv.set_joint_pose(je, joints, 0)
    bpy.context.view_layer.update()
    rest_mn, _ = measure_world_bbox(vo)
    if math.isfinite(rest_mn.z):
        root_empty.location = root_empty.location + Vector((0.0, 0.0, -rest_mn.z))
        bpy.context.view_layer.update()

    # Lock the rest pose at frame 1, then run the viewer's articulation sweep.
    rv.set_joint_pose(je, joints, 0)
    keyframe_joint_pose(je, joints, 1)
    animate_articulation(je, joints, 1, ANIM_FRAMES, amp_deg=ART_AMP_DEG)

    # Reset to rest, measure, frame the camera.
    rv.set_joint_pose(je, joints, 0)
    bpy.context.view_layer.update()
    mn, mx = envelope_bbox(vo, je, joints, ART_AMP_DEG)
    center = (mn + mx) * 0.5
    span = max((mx - mn).x, (mx - mn).y, (mx - mn).z, 0.5)
    add_camera_and_lights(center, span)
    setup_world(hdri_path)
    if add_ground:
        add_floor(Vector((center.x, center.y, 0.0)), span)

    out = output_path or (DEFAULT_OUTPUT_DIR / f"{rid}.blend")
    out.parent.mkdir(parents=True, exist_ok=True)
    bpy.ops.wm.save_as_mainfile(filepath=str(out))
    print(f"   saved: {out}")
    return out


# ---------------------------------------------------------------------------
# Grid mode
# ---------------------------------------------------------------------------


def convert_grid(
    args: list[str],
    output_path: Path,
    padding: float = GRID_PADDING_M,
    hdri_path: Optional[Path] = DEFAULT_HDRI,
    add_ground: bool = True,
) -> Path:
    """Build many records into one .blend, real-world scale, animated like the viewer."""
    reset_scene()
    scene = bpy.context.scene

    # 1. Resolve everything up-front so we can fail loudly on missing URDFs.
    resolved: list[tuple[str, Path]] = []
    for a in args:
        try:
            rid, urdf = resolve_urdf(a)
        except FileNotFoundError as e:
            print(f"  ! skip {a}: {e}")
            continue
        resolved.append((rid, urdf))
    if not resolved:
        raise RuntimeError("no records resolved")

    n = len(resolved)
    cols = max(1, math.ceil(math.sqrt(n)))

    # 2. Build each record at world origin, animate, then measure its envelope.
    placements: list[tuple[str, bpy.types.Object, Vector, Vector]] = []
    for i, (rid, urdf) in enumerate(resolved):
        print(f"[{i+1}/{n}] {rid}")
        coll = bpy.data.collections.new(rid)
        scene.collection.children.link(coll)

        rid2, links, joints, le, je, vo, root_empty = build_record(rid, collection=coll)
        print(f"     parsed: {len(links)} links, {len(joints)} joints")

        # Wrap the root in a holder so we can translate the whole assembly later.
        holder_name = f"obj_{rid}"[:60]
        holder = bpy.data.objects.new(holder_name, None)
        coll.objects.link(holder)
        holder.empty_display_type = "PLAIN_AXES"
        holder.empty_display_size = 0.2
        root_empty.parent = holder

        # Lock rest at frame 1 then sweep all joints across the timeline.
        rv.set_joint_pose(je, joints, 0)
        keyframe_joint_pose(je, joints, 1)
        animate_articulation(je, joints, 1, ANIM_FRAMES, amp_deg=ART_AMP_DEG)

        # Measure the full articulation envelope (all poses, not just rest).
        mn, mx = envelope_bbox(vo, je, joints, ART_AMP_DEG)
        placements.append((rid, holder, mn, mx))

    # 3. Cell size = largest object's xy footprint (over the entire sweep) + padding.
    sizes_xy = [(mx.x - mn.x, mx.y - mn.y) for (_, _, mn, mx) in placements]
    cell_x = max((s[0] for s in sizes_xy), default=1.0) + padding
    cell_y = max((s[1] for s in sizes_xy), default=1.0) + padding

    # 4. Drop each holder into its grid cell, sitting on z = 0.
    for idx, (rid, holder, mn, mx) in enumerate(placements):
        row, col = divmod(idx, cols)
        cx = col * cell_x
        cy = -row * cell_y
        center_x = (mn.x + mx.x) * 0.5
        center_y = (mn.y + mx.y) * 0.5
        holder.location = (cx - center_x, cy - center_y, -mn.z)
    bpy.context.view_layer.update()

    # 5. Camera framing the whole grid.
    rows = math.ceil(n / cols)
    grid_center = Vector((
        (cols - 1) * cell_x * 0.5,
        -(rows - 1) * cell_y * 0.5,
        0.0,
    ))
    grid_span = max(cols * cell_x, rows * cell_y)
    add_camera_and_lights(grid_center, grid_span)
    setup_world(hdri_path)
    if add_ground:
        add_floor(grid_center, grid_span)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    bpy.ops.wm.save_as_mainfile(filepath=str(output_path))
    print(f"saved grid: {output_path}  ({n} objects, {cols}x{rows}, cell={cell_x:.2f}x{cell_y:.2f}m)")
    return output_path


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def parse_argv() -> list[str]:
    if "--" in sys.argv:
        return sys.argv[sys.argv.index("--") + 1:]
    return sys.argv[1:]


def expand_list_file(path: Path) -> list[str]:
    items: list[str] = []
    for raw in path.read_text().splitlines():
        s = raw.strip()
        if not s or s.endswith(":") or s.startswith("#"):
            continue
        items.append(s)
    return items


def _extract_env_flags(
    args: list[str],
) -> tuple[list[str], Optional[Path], bool, float]:
    """Strip ``--hdri PATH``, ``--no-floor``, and ``--padding M`` from anywhere in ``args``.

    Returns ``(remaining_args, hdri_path, add_ground, padding_m)``.
    """
    out: list[str] = []
    hdri: Optional[Path] = DEFAULT_HDRI
    add_ground = True
    padding = GRID_PADDING_M
    i = 0
    while i < len(args):
        a = args[i]
        if a == "--hdri":
            if i + 1 >= len(args):
                raise SystemExit("--hdri requires a path")
            hdri = Path(args[i + 1])
            i += 2
            continue
        if a == "--no-hdri":
            hdri = None
            i += 1
            continue
        if a == "--no-floor":
            add_ground = False
            i += 1
            continue
        if a == "--padding":
            if i + 1 >= len(args):
                raise SystemExit("--padding requires a value in meters")
            try:
                padding = float(args[i + 1])
            except ValueError:
                raise SystemExit(f"--padding: not a number: {args[i + 1]!r}")
            i += 2
            continue
        out.append(a)
        i += 1
    return out, hdri, add_ground, padding


def main() -> None:
    args = parse_argv()
    args, hdri, add_ground, padding = _extract_env_flags(args)
    if not args:
        print("usage: blender -b -P urdf_to_blender.py -- <record_id|urdf_path> [out.blend]")
        print("       blender -b -P urdf_to_blender.py -- --list teaser/example.txt")
        print("       blender -b -P urdf_to_blender.py -- --grid out.blend --from list.txt")
        print("       blender -b -P urdf_to_blender.py -- --grid out.blend <id1> <id2> ...")
        print("env flags (any mode): --hdri <path.exr> | --no-hdri | --no-floor | --padding <meters>")
        return

    if args[0] == "--list":
        if len(args) < 2:
            raise SystemExit("--list requires a path")
        list_path = Path(args[1])
        ids = expand_list_file(list_path)
        for i, rid in enumerate(ids, 1):
            print(f"\n[{i}/{len(ids)}] {rid}")
            try:
                convert_one(rid, hdri_path=hdri, add_ground=add_ground)
            except Exception as e:
                print(f"   FAILED: {e}")
        return

    if args[0] == "--grid":
        if len(args) < 3:
            raise SystemExit("usage: --grid <output.blend> [--from <list-file> | <id> ...]")
        out = Path(args[1])
        rest = args[2:]
        if rest[0] == "--from":
            if len(rest) < 2:
                raise SystemExit("--from requires a list file")
            ids = expand_list_file(Path(rest[1]))
        else:
            ids = rest
        convert_grid(ids, out, padding=padding, hdri_path=hdri, add_ground=add_ground)
        return

    arg = args[0]
    out = Path(args[1]) if len(args) >= 2 else None
    convert_one(arg, out, hdri_path=hdri, add_ground=add_ground)


if __name__ == "__main__":
    main()
