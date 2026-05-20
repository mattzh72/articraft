from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)

BASE_LENGTH = 0.84
BASE_RAIL_WIDTH = 0.06
BASE_RAIL_Y = 0.085
BASE_TOP_Z = 0.04
HOUSING_MOUNT_Z = 0.052

STATION_X = (-0.24, 0.0, 0.24)
HOUSING_LENGTH = 0.11
HOUSING_WIDTH = 0.18
HOUSING_BORE_Z = 0.13
HOUSING_TOP_Z = 0.152
HOUSING_BORE_RADIUS = 0.0215

SHAFT_AXIS_Z = HOUSING_MOUNT_Z + HOUSING_BORE_Z
JOURNAL_RADIUS = 0.019
SEAL_COVER_THICKNESS = 0.012
TOP_COVER_THICKNESS = 0.010


def _cylinder_x(radius: float, length: float, start_x: float) -> cq.Workplane:
    return cq.Workplane("YZ").circle(radius).extrude(length).translate((start_x, 0.0, 0.0))


def _box_bottom(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    sx, sy, sz = size
    cx, cy, cz = center
    return (
        cq.Workplane("XY")
        .box(sx, sy, sz, centered=(True, True, False))
        .translate((cx, cy, cz))
    )


def _make_base_frame() -> cq.Workplane:
    frame = (
        _box_bottom((BASE_LENGTH, BASE_RAIL_WIDTH, BASE_TOP_Z), (0.0, -BASE_RAIL_Y, 0.0))
        .union(_box_bottom((BASE_LENGTH, BASE_RAIL_WIDTH, BASE_TOP_Z), (0.0, BASE_RAIL_Y, 0.0)))
        .union(_box_bottom((0.12, 0.23, 0.028), (-0.31, 0.0, 0.0)))
        .union(_box_bottom((0.14, 0.24, 0.028), (0.0, 0.0, 0.0)))
        .union(_box_bottom((0.12, 0.23, 0.028), (0.31, 0.0, 0.0)))
    )
    for x in STATION_X:
        frame = frame.union(_box_bottom((0.16, 0.22, 0.008), (x, 0.0, BASE_TOP_Z)))
    for x in (-0.35, 0.35):
        frame = frame.union(_box_bottom((0.07, 0.10, 0.012), (x, -BASE_RAIL_Y, BASE_TOP_Z)))
        frame = frame.union(_box_bottom((0.07, 0.10, 0.012), (x, BASE_RAIL_Y, BASE_TOP_Z)))
    return frame


def _make_station_housing(*, seal_side: int = 0) -> cq.Workplane:
    body = _box_bottom((HOUSING_LENGTH, HOUSING_WIDTH, 0.020), (0.0, 0.0, 0.0))
    body = body.union(_box_bottom((0.102, 0.136, 0.065), (0.0, 0.0, 0.020)))
    body = body.union(_box_bottom((0.080, 0.092, 0.067), (0.0, 0.0, 0.085)))
    body = body.cut(_box_bottom((0.064, 0.034, 0.060), (0.0, 0.0, 0.026)))
    body = body.cut(_cylinder_x(HOUSING_BORE_RADIUS, HOUSING_LENGTH + 0.020, -0.065).translate((0.0, 0.0, HOUSING_BORE_Z)))
    for y in (-0.062, 0.062):
        body = body.cut(
            _box_bottom((0.030, 0.024, 0.040), (-0.030, y, 0.020))
        )
        body = body.cut(
            _box_bottom((0.030, 0.024, 0.040), (0.030, y, 0.020))
        )
    return body


def _make_top_cover() -> cq.Workplane:
    cover = _box_bottom((0.078, 0.058, TOP_COVER_THICKNESS), (0.0, 0.0, 0.0))
    cover = cover.union(_box_bottom((0.034, 0.032, 0.010), (0.0, 0.0, TOP_COVER_THICKNESS)))
    cover = cover.union(_box_bottom((0.016, 0.016, 0.014), (0.0, 0.0, TOP_COVER_THICKNESS + 0.010)))
    for x in (-0.026, 0.026):
        for y in (-0.018, 0.018):
            cover = cover.union(
                cq.Workplane("XY")
                .circle(0.006)
                .extrude(0.008)
                .translate((x, y, TOP_COVER_THICKNESS))
            )
    return cover


def _make_seal_cover() -> cq.Workplane:
    cover = _cylinder_x(0.050, SEAL_COVER_THICKNESS, 0.0).translate((0.0, 0.0, 0.0))
    cover = cover.cut(_cylinder_x(0.024, SEAL_COVER_THICKNESS + 0.002, -0.001))
    cover = cover.union(_cylinder_x(0.036, 0.010, SEAL_COVER_THICKNESS))
    for angle in (0.0, 0.5 * math.pi, math.pi, 1.5 * math.pi):
        y = 0.036 * math.cos(angle)
        z = 0.036 * math.sin(angle)
        cover = cover.union(
            _cylinder_x(0.0055, 0.008, SEAL_COVER_THICKNESS - 0.002).translate((0.0, y, z))
        )
    return cover


def _make_torque_arm() -> cq.Workplane:
    profile = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.040, 0.000),
                (-0.010, 0.000),
                (0.012, 0.018),
                (0.050, 0.095),
                (0.050, 0.128),
                (0.018, 0.128),
                (-0.015, 0.060),
                (-0.040, 0.018),
            ]
        )
        .close()
        .extrude(0.018)
        .translate((0.0, -0.009, 0.0))
    )
    profile = profile.union(_box_bottom((0.050, 0.018, 0.012), (-0.020, 0.0, 0.0)))
    profile = profile.union(_box_bottom((0.028, 0.018, 0.040), (0.046, 0.0, 0.086)))
    return profile


def _make_shaft_flange(start_x: float, thickness: float, radius: float, bolt_circle: float) -> cq.Workplane:
    flange = _cylinder_x(radius, thickness, start_x)
    for angle in [i * math.tau / 6.0 for i in range(6)]:
        y = bolt_circle * math.cos(angle)
        z = bolt_circle * math.sin(angle)
        flange = flange.cut(_cylinder_x(0.0045, thickness + 0.002, start_x - 0.001).translate((0.0, y, z)))
    return flange


def _make_shaft_segments() -> dict[str, cq.Workplane]:
    return {
        "left_flange": _make_shaft_flange(-0.386, 0.020, 0.045, 0.030),
        "left_collar": _cylinder_x(0.031, 0.024, -0.366),
        "left_neck": _cylinder_x(0.024, 0.047, -0.342),
        "left_journal": _cylinder_x(JOURNAL_RADIUS, 0.110, -0.295),
        "left_inner_shoulder": _cylinder_x(0.028, 0.020, -0.185),
        "left_tube": _cylinder_x(0.023, 0.110, -0.165),
        "center_journal": _cylinder_x(JOURNAL_RADIUS, 0.110, -0.055),
        "right_tube": _cylinder_x(0.023, 0.110, 0.055),
        "right_inner_shoulder": _cylinder_x(0.028, 0.020, 0.165),
        "right_journal": _cylinder_x(JOURNAL_RADIUS, 0.110, 0.185),
        "right_neck": _cylinder_x(0.024, 0.047, 0.295),
        "right_collar": _cylinder_x(0.031, 0.024, 0.342),
        "right_flange": _make_shaft_flange(0.366, 0.020, 0.040, 0.026).union(_cylinder_x(0.026, 0.016, 0.386)),
    }


def _fixed_joint(
    model: ArticulatedObject,
    name: str,
    parent,
    child,
    xyz: tuple[float, float, float],
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    model.articulation(
        name,
        ArticulationType.FIXED,
        parent=parent,
        child=child,
        origin=Origin(xyz=xyz, rpy=rpy),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rotary_shaft_study", assets=ASSETS)

    steel = model.material("steel", rgba=(0.56, 0.58, 0.61, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.32, 0.34, 0.37, 1.0))
    machine_paint = model.material("machine_paint", rgba=(0.18, 0.26, 0.30, 1.0))
    cover_paint = model.material("cover_paint", rgba=(0.36, 0.28, 0.18, 1.0))
    bracket_paint = model.material("bracket_paint", rgba=(0.34, 0.34, 0.30, 1.0))

    base = model.part("base_frame")
    base.visual(
        mesh_from_cadquery(_make_base_frame(), "base_frame.obj", assets=ASSETS),
        material=machine_paint,
        name="frame_shell",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, 0.24, BASE_TOP_Z + 0.02)),
        mass=26.0,
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
    )

    left_housing = model.part("left_housing")
    left_housing.visual(
        mesh_from_cadquery(_make_station_housing(seal_side=-1), "left_housing.obj", assets=ASSETS),
        material=dark_steel,
        name="housing_shell",
    )
    left_housing.inertial = Inertial.from_geometry(
        Box((HOUSING_LENGTH + SEAL_COVER_THICKNESS, HOUSING_WIDTH, HOUSING_TOP_Z)),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.0, HOUSING_TOP_Z / 2.0)),
    )

    center_housing = model.part("center_housing")
    center_housing.visual(
        mesh_from_cadquery(_make_station_housing(seal_side=0), "center_housing.obj", assets=ASSETS),
        material=dark_steel,
        name="housing_shell",
    )
    center_housing.inertial = Inertial.from_geometry(
        Box((HOUSING_LENGTH, HOUSING_WIDTH, HOUSING_TOP_Z)),
        mass=5.8,
        origin=Origin(xyz=(0.0, 0.0, HOUSING_TOP_Z / 2.0)),
    )

    right_housing = model.part("right_housing")
    right_housing.visual(
        mesh_from_cadquery(_make_station_housing(seal_side=1), "right_housing.obj", assets=ASSETS),
        material=dark_steel,
        name="housing_shell",
    )
    right_housing.inertial = Inertial.from_geometry(
        Box((HOUSING_LENGTH + SEAL_COVER_THICKNESS, HOUSING_WIDTH, HOUSING_TOP_Z)),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.0, HOUSING_TOP_Z / 2.0)),
    )

    left_cover = model.part("left_access_cover")
    left_cover.visual(
        mesh_from_cadquery(_make_top_cover(), "left_access_cover.obj", assets=ASSETS),
        material=cover_paint,
        name="cover_shell",
    )
    left_cover.inertial = Inertial.from_geometry(
        Box((0.078, 0.058, 0.032)),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
    )

    center_cover = model.part("center_access_cover")
    center_cover.visual(
        mesh_from_cadquery(_make_top_cover(), "center_access_cover.obj", assets=ASSETS),
        material=cover_paint,
        name="cover_shell",
    )
    center_cover.inertial = Inertial.from_geometry(
        Box((0.078, 0.058, 0.032)),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
    )

    right_cover = model.part("right_access_cover")
    right_cover.visual(
        mesh_from_cadquery(_make_top_cover(), "right_access_cover.obj", assets=ASSETS),
        material=cover_paint,
        name="cover_shell",
    )
    right_cover.inertial = Inertial.from_geometry(
        Box((0.078, 0.058, 0.032)),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
    )

    left_seal_cover = model.part("left_seal_cover")
    left_seal_cover.visual(
        mesh_from_cadquery(_make_seal_cover(), "left_seal_cover.obj", assets=ASSETS),
        material=steel,
        name="seal_shell",
    )
    left_seal_cover.inertial = Inertial.from_geometry(
        Box((0.022, 0.10, 0.10)),
        mass=0.6,
        origin=Origin(xyz=(0.011, 0.0, 0.0)),
    )

    right_seal_cover = model.part("right_seal_cover")
    right_seal_cover.visual(
        mesh_from_cadquery(_make_seal_cover(), "right_seal_cover.obj", assets=ASSETS),
        material=steel,
        name="seal_shell",
    )
    right_seal_cover.inertial = Inertial.from_geometry(
        Box((0.022, 0.10, 0.10)),
        mass=0.6,
        origin=Origin(xyz=(0.011, 0.0, 0.0)),
    )

    left_torque_arm = model.part("left_torque_arm")
    left_torque_arm.visual(
        mesh_from_cadquery(_make_torque_arm(), "left_torque_arm.obj", assets=ASSETS),
        material=bracket_paint,
        name="arm_shell",
    )
    left_torque_arm.inertial = Inertial.from_geometry(
        Box((0.10, 0.018, 0.13)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
    )

    right_torque_arm = model.part("right_torque_arm")
    right_torque_arm.visual(
        mesh_from_cadquery(_make_torque_arm(), "right_torque_arm.obj", assets=ASSETS),
        material=bracket_paint,
        name="arm_shell",
    )
    right_torque_arm.inertial = Inertial.from_geometry(
        Box((0.10, 0.018, 0.13)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
    )

    shaft = model.part("shaft")
    for visual_name, shape in _make_shaft_segments().items():
        shaft.visual(
            mesh_from_cadquery(shape, f"{visual_name}.obj", assets=ASSETS),
            material=steel,
            name=visual_name,
        )
    shaft.inertial = Inertial.from_geometry(
        Box((0.42, 0.09, 0.09)),
        mass=12.5,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    _fixed_joint(model, "base_to_left_housing", base, left_housing, (STATION_X[0], 0.0, HOUSING_MOUNT_Z))
    _fixed_joint(model, "base_to_center_housing", base, center_housing, (STATION_X[1], 0.0, HOUSING_MOUNT_Z))
    _fixed_joint(model, "base_to_right_housing", base, right_housing, (STATION_X[2], 0.0, HOUSING_MOUNT_Z))

    _fixed_joint(model, "left_housing_to_cover", left_housing, left_cover, (0.0, 0.0, HOUSING_TOP_Z))
    _fixed_joint(model, "center_housing_to_cover", center_housing, center_cover, (0.0, 0.0, HOUSING_TOP_Z))
    _fixed_joint(model, "right_housing_to_cover", right_housing, right_cover, (0.0, 0.0, HOUSING_TOP_Z))

    _fixed_joint(
        model,
        "left_housing_to_seal_cover",
        left_housing,
        left_seal_cover,
        (-HOUSING_LENGTH / 2.0, 0.0, HOUSING_BORE_Z),
        rpy=(0.0, math.pi, 0.0),
    )
    _fixed_joint(
        model,
        "right_housing_to_seal_cover",
        right_housing,
        right_seal_cover,
        (HOUSING_LENGTH / 2.0, 0.0, HOUSING_BORE_Z),
    )

    _fixed_joint(model, "left_housing_to_torque_arm", left_housing, left_torque_arm, (0.0, -0.099, 0.0))
    _fixed_joint(
        model,
        "right_housing_to_torque_arm",
        right_housing,
        right_torque_arm,
        (0.0, 0.099, 0.0),
        rpy=(0.0, 0.0, math.pi),
    )

    model.articulation(
        "base_to_shaft",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=shaft,
        origin=Origin(xyz=(0.0, 0.0, SHAFT_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=12.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base_frame")
    left_housing = object_model.get_part("left_housing")
    center_housing = object_model.get_part("center_housing")
    right_housing = object_model.get_part("right_housing")
    left_cover = object_model.get_part("left_access_cover")
    center_cover = object_model.get_part("center_access_cover")
    right_cover = object_model.get_part("right_access_cover")
    left_seal_cover = object_model.get_part("left_seal_cover")
    right_seal_cover = object_model.get_part("right_seal_cover")
    left_torque_arm = object_model.get_part("left_torque_arm")
    right_torque_arm = object_model.get_part("right_torque_arm")
    shaft = object_model.get_part("shaft")
    shaft_spin = object_model.get_articulation("base_to_shaft")

    left_journal = shaft.get_visual("left_journal")
    center_journal = shaft.get_visual("center_journal")
    right_journal = shaft.get_visual("right_journal")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "shaft_joint_is_continuous",
        shaft_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint type was {shaft_spin.articulation_type}",
    )
    ctx.check(
        "shaft_joint_axis_is_x",
        tuple(round(v, 3) for v in shaft_spin.axis) == (1.0, 0.0, 0.0),
        details=f"joint axis was {shaft_spin.axis}",
    )

    for housing, name in (
        (left_housing, "left"),
        (center_housing, "center"),
        (right_housing, "right"),
    ):
        ctx.expect_gap(housing, base, axis="z", max_gap=0.001, max_penetration=0.0, name=f"{name}_housing_seats_on_base")
        ctx.expect_overlap(housing, base, axes="xy", min_overlap=0.10, name=f"{name}_housing_footprint_on_base")

    for cover, housing, name in (
        (left_cover, left_housing, "left"),
        (center_cover, center_housing, "center"),
        (right_cover, right_housing, "right"),
    ):
        ctx.expect_gap(cover, housing, axis="z", max_gap=0.001, max_penetration=0.0, name=f"{name}_cover_seats_on_housing")
        ctx.expect_overlap(cover, housing, axes="xy", min_overlap=0.05, name=f"{name}_cover_overlaps_housing_top")

    ctx.expect_gap(left_housing, left_seal_cover, axis="x", max_gap=0.001, max_penetration=0.0, name="left_seal_cover_mount")
    ctx.expect_gap(right_seal_cover, right_housing, axis="x", max_gap=0.001, max_penetration=0.0, name="right_seal_cover_mount")
    ctx.expect_overlap(left_seal_cover, left_housing, axes="yz", min_overlap=0.07, name="left_seal_cover_register")
    ctx.expect_overlap(right_seal_cover, right_housing, axes="yz", min_overlap=0.07, name="right_seal_cover_register")

    ctx.expect_gap(left_housing, left_torque_arm, axis="y", max_gap=0.001, max_penetration=0.0, name="left_torque_arm_to_housing")
    ctx.expect_gap(right_torque_arm, right_housing, axis="y", max_gap=0.001, max_penetration=0.0, name="right_torque_arm_to_housing")
    ctx.expect_gap(left_torque_arm, base, axis="z", max_gap=0.001, max_penetration=0.0, name="left_torque_arm_to_base")
    ctx.expect_gap(right_torque_arm, base, axis="z", max_gap=0.001, max_penetration=0.0, name="right_torque_arm_to_base")
    ctx.expect_overlap(left_torque_arm, base, axes="xy", min_overlap=0.01, name="left_torque_arm_footprint")
    ctx.expect_overlap(right_torque_arm, base, axes="xy", min_overlap=0.01, name="right_torque_arm_footprint")

    ctx.expect_gap(shaft, base, axis="z", min_gap=0.09, max_gap=0.14, positive_elem=center_journal, name="shaft_clears_base_frame")

    for angle in (0.0, 1.3, 2.6):
        with ctx.pose({shaft_spin: angle}):
            ctx.expect_within(
                shaft,
                left_housing,
                axes="yz",
                margin=0.0,
                inner_elem=left_journal,
                name=f"left_journal_supported_pose_{angle:.1f}",
            )
            ctx.expect_within(
                shaft,
                center_housing,
                axes="yz",
                margin=0.0,
                inner_elem=center_journal,
                name=f"center_journal_supported_pose_{angle:.1f}",
            )
            ctx.expect_within(
                shaft,
                right_housing,
                axes="yz",
                margin=0.0,
                inner_elem=right_journal,
                name=f"right_journal_supported_pose_{angle:.1f}",
            )
            ctx.expect_overlap(
                shaft,
                left_housing,
                axes="x",
                min_overlap=0.05,
                elem_a=left_journal,
                name=f"left_journal_station_overlap_pose_{angle:.1f}",
            )
            ctx.expect_overlap(
                shaft,
                center_housing,
                axes="x",
                min_overlap=0.05,
                elem_a=center_journal,
                name=f"center_journal_station_overlap_pose_{angle:.1f}",
            )
            ctx.expect_overlap(
                shaft,
                right_housing,
                axes="x",
                min_overlap=0.05,
                elem_a=right_journal,
                name=f"right_journal_station_overlap_pose_{angle:.1f}",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
