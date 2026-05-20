from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    LoftGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _circle_points(radius: float, z: float, *, count: int = 18, phase: float = 0.0):
    return [
        (
            radius * math.cos(phase + (2.0 * math.pi * index / count)),
            radius * math.sin(phase + (2.0 * math.pi * index / count)),
            z,
        )
        for index in range(count)
    ]


def _aabb_center(aabb):
    if aabb is None:
        return None
    return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="offshore_lighthouse")

    tower_white = model.material("tower_white", rgba=(0.90, 0.91, 0.88, 1.0))
    weathered_concrete = model.material("weathered_concrete", rgba=(0.60, 0.61, 0.59, 1.0))
    lantern_frame = model.material("lantern_frame", rgba=(0.16, 0.20, 0.18, 1.0))
    lantern_glass = model.material("lantern_glass", rgba=(0.73, 0.86, 0.92, 0.34))
    roof_red = model.material("roof_red", rgba=(0.57, 0.14, 0.12, 1.0))
    rail_white = model.material("rail_white", rgba=(0.93, 0.94, 0.92, 1.0))
    beacon_gray = model.material("beacon_gray", rgba=(0.46, 0.49, 0.52, 1.0))
    beacon_dark = model.material("beacon_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    beacon_lens = model.material("beacon_lens", rgba=(0.85, 0.92, 0.96, 0.40))
    hatch_red = model.material("hatch_red", rgba=(0.63, 0.16, 0.14, 1.0))

    gallery_upper_rail = _save_mesh(
        "gallery_upper_rail",
        tube_from_spline_points(
            _circle_points(1.56, 11.18, count=22, phase=math.pi / 22.0),
            radius=0.018,
            samples_per_segment=8,
            radial_segments=16,
            closed_spline=True,
            cap_ends=False,
        ),
    )
    gallery_lower_rail = _save_mesh(
        "gallery_lower_rail",
        tube_from_spline_points(
            _circle_points(1.52, 10.93, count=22, phase=math.pi / 22.0),
            radius=0.014,
            samples_per_segment=8,
            radial_segments=16,
            closed_spline=True,
            cap_ends=False,
        ),
    )

    tower_shell = _save_mesh(
        "tower_shell",
        LatheGeometry.from_shell_profiles(
            [
                (2.30, 1.00),
                (2.16, 1.45),
                (1.96, 2.60),
                (1.70, 5.20),
                (1.45, 8.20),
                (1.28, 10.20),
                (1.24, 10.55),
            ],
            [
                (1.76, 1.05),
                (1.64, 1.46),
                (1.48, 2.62),
                (1.25, 5.20),
                (1.05, 8.18),
                (0.92, 10.18),
                (0.88, 10.52),
            ],
            segments=88,
            start_cap="flat",
            end_cap="flat",
        ),
    )

    roof_mesh = _save_mesh(
        "lantern_roof",
        LoftGeometry(
            [
                [
                    (-0.86, -0.86, 0.00),
                    (0.86, -0.86, 0.00),
                    (0.86, 0.86, 0.00),
                    (-0.86, 0.86, 0.00),
                ],
                [
                    (-0.58, -0.58, 0.18),
                    (0.58, -0.58, 0.18),
                    (0.58, 0.58, 0.18),
                    (-0.58, 0.58, 0.18),
                ],
                [
                    (-0.12, -0.12, 0.48),
                    (0.12, -0.12, 0.48),
                    (0.12, 0.12, 0.48),
                    (-0.12, 0.12, 0.48),
                ],
            ],
            cap=True,
            closed=True,
        ),
    )

    structure = model.part("lighthouse_structure")
    structure.visual(
        Cylinder(radius=2.65, length=0.90),
        origin=Origin(xyz=(0.0, 0.0, 0.45)),
        material=weathered_concrete,
        name="offshore_caisson",
    )
    structure.visual(
        Cylinder(radius=2.32, length=0.78),
        origin=Origin(xyz=(0.0, 0.0, 1.08)),
        material=weathered_concrete,
        name="splash_course",
    )
    structure.visual(
        tower_shell,
        material=tower_white,
        name="tower_shell",
    )
    structure.visual(
        Cylinder(radius=1.30, length=0.46),
        origin=Origin(xyz=(0.0, 0.0, 10.32)),
        material=tower_white,
        name="tower_head",
    )
    structure.visual(
        Cylinder(radius=1.42, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 10.48)),
        material=weathered_concrete,
        name="gallery_support_ring",
    )
    structure.visual(
        Cylinder(radius=1.74, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 10.61)),
        material=weathered_concrete,
        name="gallery_deck",
    )
    structure.visual(
        Cylinder(radius=0.92, length=0.30),
        origin=Origin(xyz=(0.0, 0.0, 10.77)),
        material=lantern_frame,
        name="lantern_base_drum",
    )
    structure.visual(
        Box((1.42, 1.42, 0.09)),
        origin=Origin(xyz=(0.0, 0.0, 10.87)),
        material=lantern_frame,
        name="lantern_sill",
    )
    structure.visual(
        gallery_upper_rail,
        material=rail_white,
        name="gallery_upper_rail",
    )
    structure.visual(
        gallery_lower_rail,
        material=rail_white,
        name="gallery_lower_rail",
    )

    for index in range(12):
        angle = 2.0 * math.pi * index / 12.0
        structure.visual(
            Cylinder(radius=0.020, length=0.52),
            origin=Origin(
                xyz=(1.54 * math.cos(angle), 1.54 * math.sin(angle), 10.95)
            ),
            material=rail_white,
            name=f"gallery_stanchion_{index:02d}",
        )

    for sx in (-0.62, 0.62):
        for sy in (-0.62, 0.62):
            structure.visual(
                Box((0.06, 0.06, 1.02)),
                origin=Origin(xyz=(sx, sy, 11.39)),
                material=lantern_frame,
                name=f"corner_post_{'p' if sx > 0 else 'n'}x_{'p' if sy > 0 else 'n'}y",
            )

    structure.visual(
        Box((1.18, 0.025, 0.96)),
        origin=Origin(xyz=(0.0, 0.62, 11.39)),
        material=lantern_glass,
        name="front_panel",
    )
    structure.visual(
        Box((0.025, 1.18, 0.96)),
        origin=Origin(xyz=(0.62, 0.0, 11.39)),
        material=lantern_glass,
        name="right_panel",
    )
    structure.visual(
        Box((0.025, 1.18, 0.96)),
        origin=Origin(xyz=(-0.62, 0.0, 11.39)),
        material=lantern_glass,
        name="left_panel",
    )
    structure.visual(
        Box((0.34, 0.025, 0.96)),
        origin=Origin(xyz=(-0.42, -0.62, 11.39)),
        material=lantern_glass,
        name="rear_left_panel",
    )
    structure.visual(
        Box((0.34, 0.025, 0.96)),
        origin=Origin(xyz=(0.42, -0.62, 11.39)),
        material=lantern_glass,
        name="rear_right_panel",
    )
    structure.visual(
        Box((0.045, 0.055, 0.97)),
        origin=Origin(xyz=(-0.175, -0.62, 11.397)),
        material=lantern_frame,
        name="hatch_jamb_left",
    )
    structure.visual(
        Box((0.045, 0.055, 0.97)),
        origin=Origin(xyz=(0.175, -0.62, 11.397)),
        material=lantern_frame,
        name="hatch_jamb_right",
    )
    structure.visual(
        Box((0.395, 0.055, 0.066)),
        origin=Origin(xyz=(0.0, -0.62, 10.913)),
        material=lantern_frame,
        name="hatch_threshold",
    )
    structure.visual(
        Box((0.395, 0.055, 0.072)),
        origin=Origin(xyz=(0.0, -0.62, 11.884)),
        material=lantern_frame,
        name="hatch_lintel",
    )
    structure.visual(
        Box((1.42, 1.42, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 11.92)),
        material=lantern_frame,
        name="lantern_head_ring",
    )
    structure.visual(
        roof_mesh,
        origin=Origin(xyz=(0.0, 0.0, 11.96)),
        material=roof_red,
        name="roof_shell",
    )
    structure.visual(
        Cylinder(radius=0.11, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 12.35)),
        material=lantern_frame,
        name="roof_vent",
    )
    structure.visual(
        Cylinder(radius=0.08, length=0.30),
        origin=Origin(xyz=(0.0, 0.0, 12.59)),
        material=lantern_frame,
        name="beacon_pedestal",
    )
    structure.inertial = Inertial.from_geometry(
        Box((5.30, 5.30, 12.80)),
        mass=18000.0,
        origin=Origin(xyz=(0.0, 0.0, 6.40)),
    )

    beacon = model.part("beacon_head")
    beacon.visual(
        Cylinder(radius=0.16, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=beacon_dark,
        name="turntable_disc",
    )
    beacon.visual(
        Box((0.24, 0.22, 0.10)),
        origin=Origin(xyz=(0.05, 0.0, 0.070)),
        material=beacon_gray,
        name="beacon_saddle",
    )
    beacon.visual(
        Cylinder(radius=0.11, length=0.30),
        origin=Origin(xyz=(0.16, 0.0, 0.19), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=beacon_gray,
        name="beacon_barrel",
    )
    beacon.visual(
        Cylinder(radius=0.126, length=0.05),
        origin=Origin(xyz=(0.31, 0.0, 0.19), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=beacon_dark,
        name="beacon_hood",
    )
    beacon.visual(
        Cylinder(radius=0.094, length=0.018),
        origin=Origin(xyz=(0.34, 0.0, 0.19), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=beacon_lens,
        name="beacon_lens",
    )
    beacon.visual(
        Box((0.12, 0.24, 0.16)),
        origin=Origin(xyz=(-0.06, 0.0, 0.17)),
        material=beacon_dark,
        name="rear_service_box",
    )
    beacon.visual(
        Box((0.18, 0.18, 0.05)),
        origin=Origin(xyz=(0.08, 0.0, 0.31)),
        material=beacon_gray,
        name="beacon_top_cover",
    )
    beacon.inertial = Inertial.from_geometry(
        Box((0.52, 0.26, 0.38)),
        mass=120.0,
        origin=Origin(xyz=(0.09, 0.0, 0.19)),
    )

    hatch = model.part("access_hatch")
    hatch.visual(
        Box((0.30, 0.028, 0.80)),
        origin=Origin(xyz=(0.15, 0.0, 0.0)),
        material=hatch_red,
        name="hatch_panel",
    )
    hatch.visual(
        Box((0.22, 0.012, 0.40)),
        origin=Origin(xyz=(0.15, 0.017, 0.0)),
        material=lantern_frame,
        name="hatch_stiffener",
    )
    hatch.visual(
        Box((0.036, 0.012, 0.74)),
        origin=Origin(xyz=(-0.018, 0.008, 0.0)),
        material=beacon_dark,
        name="hinge_leaf",
    )
    for index, z_pos in enumerate((-0.25, 0.0, 0.25)):
        hatch.visual(
            Cylinder(radius=0.012, length=0.05),
            origin=Origin(xyz=(0.0, -0.010, z_pos)),
            material=beacon_dark,
            name=f"hinge_barrel_{index}",
        )
    hatch.visual(
        Cylinder(radius=0.006, length=0.07),
        origin=Origin(xyz=(0.24, -0.032, 0.01), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=beacon_dark,
        name="hatch_handle",
    )
    hatch.inertial = Inertial.from_geometry(
        Box((0.31, 0.08, 0.82)),
        mass=12.0,
        origin=Origin(xyz=(0.155, -0.006, 0.0)),
    )

    model.articulation(
        "beacon_spin",
        ArticulationType.CONTINUOUS,
        parent=structure,
        child=beacon,
        origin=Origin(xyz=(0.0, 0.0, 12.74)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=4.0),
    )
    model.articulation(
        "hatch_hinge",
        ArticulationType.REVOLUTE,
        parent=structure,
        child=hatch,
        origin=Origin(xyz=(-0.15, -0.6615, 11.39)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.4,
            lower=0.0,
            upper=1.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    structure = object_model.get_part("lighthouse_structure")
    beacon = object_model.get_part("beacon_head")
    hatch = object_model.get_part("access_hatch")
    beacon_spin = object_model.get_articulation("beacon_spin")
    hatch_hinge = object_model.get_articulation("hatch_hinge")

    ctx.check(
        "primary parts are present",
        structure is not None and beacon is not None and hatch is not None,
        details="Expected structure, beacon head, and access hatch parts.",
    )
    ctx.check(
        "beacon articulation is continuous vertical rotation",
        beacon_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(value, 6) for value in beacon_spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={beacon_spin.articulation_type}, axis={beacon_spin.axis}",
    )
    ctx.check(
        "hatch hinge is side-mounted vertical hinge",
        hatch_hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(value, 6) for value in hatch_hinge.axis) == (0.0, 0.0, -1.0)
        and hatch_hinge.motion_limits is not None
        and hatch_hinge.motion_limits.upper is not None
        and hatch_hinge.motion_limits.upper > 1.4,
        details=f"type={hatch_hinge.articulation_type}, axis={hatch_hinge.axis}, limits={hatch_hinge.motion_limits}",
    )

    with ctx.pose({beacon_spin: 0.0}):
        beacon_origin = ctx.part_world_position(beacon)
        barrel_closed = _aabb_center(ctx.part_element_world_aabb(beacon, elem="beacon_barrel"))
    with ctx.pose({beacon_spin: math.pi / 2.0}):
        barrel_quarter = _aabb_center(ctx.part_element_world_aabb(beacon, elem="beacon_barrel"))
    ctx.check(
        "beacon head actually slews around the pedestal axis",
        beacon_origin is not None
        and barrel_closed is not None
        and barrel_quarter is not None
        and barrel_closed[0] > beacon_origin[0] + 0.10
        and abs(barrel_closed[1] - beacon_origin[1]) < 0.04
        and barrel_quarter[1] > beacon_origin[1] + 0.10
        and abs(barrel_quarter[0] - beacon_origin[0]) < 0.05,
        details=f"origin={beacon_origin}, q0={barrel_closed}, q90={barrel_quarter}",
    )

    with ctx.pose({hatch_hinge: 0.0}):
        hatch_closed = _aabb_center(ctx.part_element_world_aabb(hatch, elem="hatch_panel"))
    with ctx.pose({hatch_hinge: 1.10}):
        hatch_open = _aabb_center(ctx.part_element_world_aabb(hatch, elem="hatch_panel"))
    ctx.check(
        "hatch opens outward from the lantern wall",
        hatch_closed is not None
        and hatch_open is not None
        and hatch_open[1] < hatch_closed[1] - 0.10
        and hatch_open[0] < hatch_closed[0] - 0.06,
        details=f"closed={hatch_closed}, open={hatch_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
