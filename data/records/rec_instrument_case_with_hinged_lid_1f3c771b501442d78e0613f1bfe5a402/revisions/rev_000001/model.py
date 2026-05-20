from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    sample_catmull_rom_spline_2d,
    tube_from_spline_points,
)


SEAM_Z = 0.080
REAR_Y = 0.190
OPEN_ANGLE = math.radians(95.0)


def _case_outline() -> list[tuple[float, float]]:
    """Violin-case planform: narrow scroll end, pinched waist, broad lower bout."""
    control = [
        (-0.445, 0.000),
        (-0.420, 0.078),
        (-0.315, 0.116),
        (-0.105, 0.128),
        (0.045, 0.105),
        (0.190, 0.166),
        (0.345, 0.182),
        (0.445, 0.128),
        (0.462, 0.000),
        (0.445, -0.128),
        (0.345, -0.182),
        (0.190, -0.166),
        (0.045, -0.105),
        (-0.105, -0.128),
        (-0.315, -0.116),
        (-0.420, -0.078),
    ]
    pts = sample_catmull_rom_spline_2d(
        control,
        samples_per_segment=5,
        closed=True,
        alpha=0.55,
    )
    if pts and abs(pts[0][0] - pts[-1][0]) < 1e-9 and abs(pts[0][1] - pts[-1][1]) < 1e-9:
        pts = pts[:-1]
    return pts


def _scaled_loop(
    outline: list[tuple[float, float]],
    *,
    sx: float,
    sy: float,
    z: float,
    y_offset: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [(x * sx, y * sy + y_offset, z) for x, y in outline]


def _add_loop(geom: MeshGeometry, points: list[tuple[float, float, float]]) -> list[int]:
    return [geom.add_vertex(x, y, z) for x, y, z in points]


def _connect_loops(geom: MeshGeometry, a: list[int], b: list[int], *, flip: bool = False) -> None:
    n = len(a)
    for i in range(n):
        j = (i + 1) % n
        if flip:
            geom.add_face(a[i], b[j], b[i])
            geom.add_face(a[i], a[j], b[j])
        else:
            geom.add_face(a[i], b[i], b[j])
            geom.add_face(a[i], b[j], a[j])


def _cap_loop(geom: MeshGeometry, loop: list[int], center: tuple[float, float, float], *, flip: bool = False) -> None:
    c = geom.add_vertex(*center)
    n = len(loop)
    for i in range(n):
        j = (i + 1) % n
        if flip:
            geom.add_face(c, loop[j], loop[i])
        else:
            geom.add_face(c, loop[i], loop[j])


def _lower_shell_geometry() -> MeshGeometry:
    outline = _case_outline()
    geom = MeshGeometry()

    outer_top = _add_loop(geom, _scaled_loop(outline, sx=1.00, sy=1.00, z=SEAM_Z))
    outer_bottom = _add_loop(geom, _scaled_loop(outline, sx=0.94, sy=0.84, z=0.006))
    inner_top = _add_loop(geom, _scaled_loop(outline, sx=0.90, sy=0.76, z=SEAM_Z - 0.002))
    inner_floor = _add_loop(geom, _scaled_loop(outline, sx=0.72, sy=0.48, z=0.024))

    _connect_loops(geom, outer_bottom, outer_top)
    _connect_loops(geom, outer_top, inner_top)
    _connect_loops(geom, inner_top, inner_floor, flip=True)
    _connect_loops(geom, inner_floor, outer_bottom)
    _cap_loop(geom, inner_floor, (0.0, 0.0, 0.024), flip=True)
    return geom


def _upper_shell_geometry() -> MeshGeometry:
    outline = _case_outline()
    geom = MeshGeometry()
    y_offset = -REAR_Y

    lip = _add_loop(geom, _scaled_loop(outline, sx=1.00, sy=1.00, z=0.000, y_offset=y_offset))
    shoulder = _add_loop(geom, _scaled_loop(outline, sx=0.96, sy=0.86, z=0.036, y_offset=y_offset))
    crown = _add_loop(geom, _scaled_loop(outline, sx=0.79, sy=0.52, z=0.076, y_offset=y_offset))
    inner_lip = _add_loop(geom, _scaled_loop(outline, sx=0.90, sy=0.76, z=0.006, y_offset=y_offset))
    inner_roof = _add_loop(geom, _scaled_loop(outline, sx=0.70, sy=0.43, z=0.048, y_offset=y_offset))

    _connect_loops(geom, lip, shoulder)
    _connect_loops(geom, shoulder, crown)
    _cap_loop(geom, crown, (0.0, y_offset, 0.081))
    _connect_loops(geom, lip, inner_lip, flip=True)
    _connect_loops(geom, inner_lip, inner_roof, flip=True)
    _cap_loop(geom, inner_roof, (0.0, y_offset, 0.048), flip=True)
    return geom


def _instrument_cavity_geometry() -> MeshGeometry:
    """A single plush insert shaped like the violin and its neck channel."""
    control = [
        (-0.390, 0.000),
        (-0.360, 0.034),
        (-0.205, 0.042),
        (-0.115, 0.054),
        (-0.055, 0.090),
        (0.055, 0.105),
        (0.130, 0.070),
        (0.235, 0.135),
        (0.365, 0.112),
        (0.405, 0.000),
        (0.365, -0.112),
        (0.235, -0.135),
        (0.130, -0.070),
        (0.055, -0.105),
        (-0.055, -0.090),
        (-0.115, -0.054),
        (-0.205, -0.042),
        (-0.360, -0.034),
    ]
    profile = sample_catmull_rom_spline_2d(control, samples_per_segment=4, closed=True, alpha=0.55)
    if profile and abs(profile[0][0] - profile[-1][0]) < 1e-9 and abs(profile[0][1] - profile[-1][1]) < 1e-9:
        profile = profile[:-1]
    return ExtrudeGeometry.from_z0(profile, 0.007, cap=True, closed=True)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hard_shell_violin_case")
    model.material("black_hardshell", color=(0.015, 0.018, 0.023, 1.0))
    model.material("burgundy_velvet", color=(0.33, 0.025, 0.070, 1.0))
    model.material("brushed_metal", color=(0.72, 0.70, 0.66, 1.0))
    model.material("rubber_grip", color=(0.025, 0.025, 0.025, 1.0))

    lower = model.part("lower_shell")
    lower.visual(
        mesh_from_geometry(_lower_shell_geometry(), "lower_shell_body"),
        material="black_hardshell",
        name="lower_shell_body",
    )
    lower.visual(
        mesh_from_geometry(_instrument_cavity_geometry(), "velvet_cavity"),
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material="burgundy_velvet",
        name="velvet_cavity",
    )
    lower.visual(
        Box((0.118, 0.036, 0.014)),
        origin=Origin(xyz=(-0.160, 0.0, 0.038)),
        material="burgundy_velvet",
        name="neck_support",
    )
    lower.visual(
        Box((0.128, 0.046, 0.014)),
        origin=Origin(xyz=(0.205, 0.0, 0.038)),
        material="burgundy_velvet",
        name="body_support",
    )
    lower.visual(
        Box((0.740, 0.018, 0.006)),
        origin=Origin(xyz=(0.0, REAR_Y - 0.006, SEAM_Z - 0.003)),
        material="brushed_metal",
        name="hinge_leaf",
    )
    lower.visual(
        Cylinder(radius=0.007, length=0.740),
        origin=Origin(xyz=(0.0, REAR_Y + 0.004, SEAM_Z + 0.003), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="brushed_metal",
        name="hinge_bar",
    )

    upper = model.part("upper_shell")
    upper.visual(
        mesh_from_geometry(_upper_shell_geometry(), "upper_shell_body"),
        material="black_hardshell",
        name="upper_shell_body",
    )
    upper.visual(
        Box((0.740, 0.040, 0.005)),
        origin=Origin(xyz=(0.0, -0.012, 0.020)),
        material="brushed_metal",
        name="hinge_leaf",
    )
    front_y = -REAR_Y - 0.128
    upper.visual(
        Box((0.046, 0.018, 0.024)),
        origin=Origin(xyz=(-0.095, front_y, 0.030)),
        material="brushed_metal",
        name="handle_mount_0",
    )
    upper.visual(
        Box((0.046, 0.018, 0.024)),
        origin=Origin(xyz=(0.095, front_y, 0.030)),
        material="brushed_metal",
        name="handle_mount_1",
    )
    handle = tube_from_spline_points(
        [
            (-0.095, front_y - 0.006, 0.030),
            (-0.070, front_y - 0.052, 0.028),
            (0.000, front_y - 0.074, 0.028),
            (0.070, front_y - 0.052, 0.028),
            (0.095, front_y - 0.006, 0.030),
        ],
        radius=0.009,
        samples_per_segment=12,
        radial_segments=18,
        cap_ends=True,
    )
    upper.visual(
        mesh_from_geometry(handle, "handle_grip"),
        material="rubber_grip",
        name="handle_grip",
    )

    model.articulation(
        "rear_hinge",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=upper,
        origin=Origin(xyz=(0.0, REAR_Y, SEAM_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=OPEN_ANGLE),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower = object_model.get_part("lower_shell")
    upper = object_model.get_part("upper_shell")
    hinge = object_model.get_articulation("rear_hinge")

    ctx.check(
        "lid opens about 95 degrees",
        abs(hinge.motion_limits.upper - OPEN_ANGLE) < math.radians(0.5),
        details=f"upper={hinge.motion_limits.upper}",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            upper,
            lower,
            axis="z",
            positive_elem="upper_shell_body",
            negative_elem="lower_shell_body",
            max_gap=0.002,
            max_penetration=0.001,
            name="closed clamshell rims meet at the seam",
        )
        ctx.expect_overlap(
            upper,
            lower,
            axes="xy",
            elem_a="upper_shell_body",
            elem_b="lower_shell_body",
            min_overlap=0.25,
            name="upper and lower shells share the violin-case footprint",
        )
        rest_aabb = ctx.part_element_world_aabb(upper, elem="handle_grip")

    ctx.expect_within(
        lower,
        lower,
        axes="xy",
        inner_elem="velvet_cavity",
        outer_elem="lower_shell_body",
        margin=0.002,
        name="violin-shaped plush cavity sits inside the lower shell",
    )

    with ctx.pose({hinge: OPEN_ANGLE}):
        open_aabb = ctx.part_element_world_aabb(upper, elem="handle_grip")
        ctx.expect_gap(
            upper,
            lower,
            axis="z",
            positive_elem="handle_grip",
            negative_elem="lower_shell_body",
            min_gap=0.20,
            name="opened lid lifts the front handle clear of the lower shell",
        )

    if rest_aabb is not None and open_aabb is not None:
        rest_center_z = (rest_aabb[0][2] + rest_aabb[1][2]) * 0.5
        open_center_z = (open_aabb[0][2] + open_aabb[1][2]) * 0.5
        ctx.check(
            "hinge motion raises the front edge",
            open_center_z > rest_center_z + 0.25,
            details=f"rest_z={rest_center_z:.3f}, open_z={open_center_z:.3f}",
        )

    return ctx.report()


object_model = build_object_model()
