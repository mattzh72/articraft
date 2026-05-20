from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LENGTH = 0.34
BASE_WIDTH = 0.28
BASE_THICKNESS = 0.02
PEDESTAL_X = -0.055
PEDESTAL_LENGTH = 0.12
PEDESTAL_WIDTH = 0.09
PEDESTAL_HEIGHT = 0.34
SHOULDER_AXIS_Z = 0.455

SHOULDER_LINK_LENGTH = 0.36
FORELINK_LENGTH = 0.34


def _y_cylinder(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .circle(radius)
        .extrude(length / 2.0, both=True)
        .translate(center)
    )


def _box(
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    fillet_axis: str | None = None,
    fillet_radius: float = 0.0,
) -> cq.Workplane:
    solid = cq.Workplane("XY").box(*size)
    if fillet_axis is not None and fillet_radius > 0.0:
        solid = solid.edges(f"|{fillet_axis.upper()}").fillet(fillet_radius)
    return solid.translate(center)


def _base_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS)
        .edges("|Z")
        .fillet(0.028)
        .translate((0.0, 0.0, BASE_THICKNESS / 2.0))
    )
    for x_pos in (-0.115, 0.115):
        for y_pos in (-0.085, 0.085):
            plate = plate.cut(
                cq.Workplane("XY")
                .circle(0.0105)
                .extrude(BASE_THICKNESS + 0.004)
                .translate((x_pos, y_pos, -0.002))
            )

    pedestal = _box(
        (PEDESTAL_LENGTH, PEDESTAL_WIDTH, PEDESTAL_HEIGHT),
        (PEDESTAL_X, 0.0, BASE_THICKNESS + PEDESTAL_HEIGHT / 2.0),
        fillet_axis="Z",
        fillet_radius=0.010,
    )
    shoulder_head = _box(
        (0.105, 0.105, 0.058),
        (-0.062, 0.0, 0.398),
        fillet_axis="Y",
        fillet_radius=0.008,
    )
    front_gusset = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.104, 0.228),
                (-0.016, 0.390),
                (-0.016, 0.428),
                (-0.104, 0.312),
            ]
        )
        .close()
        .extrude(PEDESTAL_WIDTH / 2.0, both=True)
    )

    ear_gap = 0.046
    ear_thickness = 0.014
    ear_y = ear_gap / 2.0 + ear_thickness / 2.0
    ear_height = 0.072
    ear_length = 0.060
    ear_cap_radius = 0.024

    ear_left = _box(
        (ear_length, ear_thickness, ear_height),
        (0.0, ear_y, SHOULDER_AXIS_Z - 0.010),
    ).union(_y_cylinder(ear_cap_radius, ear_thickness, (0.0, ear_y, SHOULDER_AXIS_Z)))
    ear_right = _box(
        (ear_length, ear_thickness, ear_height),
        (0.0, -ear_y, SHOULDER_AXIS_Z - 0.010),
    ).union(_y_cylinder(ear_cap_radius, ear_thickness, (0.0, -ear_y, SHOULDER_AXIS_Z)))

    return (
        plate.union(pedestal)
        .union(shoulder_head)
        .union(front_gusset)
        .union(ear_left)
        .union(ear_right)
    )


def _shoulder_link_shape() -> cq.Workplane:
    barrel = _y_cylinder(0.019, 0.046, (0.0, 0.0, 0.0))
    neck = _box((0.082, 0.038, 0.110), (0.041, 0.0, -0.004), fillet_axis="X", fillet_radius=0.008)
    main_body = _box((0.252, 0.070, 0.155), (0.186, 0.0, -0.004), fillet_axis="X", fillet_radius=0.012)
    distal_root = _box((0.050, 0.050, 0.100), (0.315, 0.0, 0.0), fillet_axis="X", fillet_radius=0.006)

    ear_gap = 0.038
    ear_thickness = 0.012
    ear_y = ear_gap / 2.0 + ear_thickness / 2.0
    ear_block = (0.040, ear_thickness, 0.072)
    elbow_x = SHOULDER_LINK_LENGTH

    ear_left = _box(ear_block, (0.340, ear_y, 0.0)).union(
        _y_cylinder(0.020, ear_thickness, (elbow_x, ear_y, 0.0))
    )
    ear_right = _box(ear_block, (0.340, -ear_y, 0.0)).union(
        _y_cylinder(0.020, ear_thickness, (elbow_x, -ear_y, 0.0))
    )

    return barrel.union(neck).union(main_body).union(distal_root).union(ear_left).union(ear_right)


def _forelink_shape() -> cq.Workplane:
    barrel = _y_cylinder(0.016, 0.038, (0.0, 0.0, 0.0))
    neck = _box((0.066, 0.030, 0.085), (0.033, 0.0, 0.0), fillet_axis="X", fillet_radius=0.006)
    main_body = _box((0.232, 0.052, 0.100), (0.176, 0.0, 0.0), fillet_axis="X", fillet_radius=0.010)
    distal_root = _box((0.045, 0.040, 0.076), (0.300, 0.0, 0.0), fillet_axis="X", fillet_radius=0.005)

    ear_gap = 0.034
    ear_thickness = 0.010
    ear_y = ear_gap / 2.0 + ear_thickness / 2.0
    wrist_x = FORELINK_LENGTH

    ear_left = _box((0.035, ear_thickness, 0.058), (0.3225, ear_y, 0.0)).union(
        _y_cylinder(0.016, ear_thickness, (wrist_x, ear_y, 0.0))
    )
    ear_right = _box((0.035, ear_thickness, 0.058), (0.3225, -ear_y, 0.0)).union(
        _y_cylinder(0.016, ear_thickness, (wrist_x, -ear_y, 0.0))
    )

    return barrel.union(neck).union(main_body).union(distal_root).union(ear_left).union(ear_right)


def _wrist_block_shape() -> cq.Workplane:
    barrel = _y_cylinder(0.013, 0.034, (0.0, 0.0, 0.0))
    neck = _box((0.040, 0.026, 0.060), (0.020, 0.0, 0.0), fillet_axis="X", fillet_radius=0.004)
    block = _box((0.092, 0.080, 0.078), (0.074, 0.0, 0.0), fillet_axis="X", fillet_radius=0.010)
    face_plate = _box((0.018, 0.092, 0.086), (0.129, 0.0, 0.0), fillet_axis="Y", fillet_radius=0.005)
    top_cap = _box((0.050, 0.060, 0.016), (0.090, 0.0, 0.039))
    return barrel.union(neck).union(block).union(face_plate).union(top_cap)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_inspection_arm")

    model.material("powder_charcoal", rgba=(0.18, 0.20, 0.22, 1.0))
    model.material("cast_aluminum", rgba=(0.67, 0.70, 0.73, 1.0))
    model.material("graphite", rgba=(0.30, 0.32, 0.35, 1.0))

    base = model.part("pedestal_base")
    base.visual(
        mesh_from_cadquery(_base_shape(), "pedestal_base"),
        material="powder_charcoal",
        name="base_shell",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.20, 0.16, 0.46)),
        mass=28.0,
        origin=Origin(xyz=(-0.030, 0.0, 0.23)),
    )

    shoulder_link = model.part("shoulder_link")
    shoulder_link.visual(
        mesh_from_cadquery(_shoulder_link_shape(), "shoulder_link"),
        material="cast_aluminum",
        name="shoulder_shell",
    )
    shoulder_link.inertial = Inertial.from_geometry(
        Box((0.38, 0.07, 0.17)),
        mass=8.5,
        origin=Origin(xyz=(0.19, 0.0, 0.0)),
    )

    forelink = model.part("forelink")
    forelink.visual(
        mesh_from_cadquery(_forelink_shape(), "forelink"),
        material="cast_aluminum",
        name="forelink_shell",
    )
    forelink.inertial = Inertial.from_geometry(
        Box((0.35, 0.052, 0.10)),
        mass=4.6,
        origin=Origin(xyz=(0.175, 0.0, 0.0)),
    )

    wrist_block = model.part("wrist_block")
    wrist_block.visual(
        mesh_from_cadquery(_wrist_block_shape(), "wrist_block"),
        material="graphite",
        name="wrist_shell",
    )
    wrist_block.inertial = Inertial.from_geometry(
        Box((0.15, 0.092, 0.086)),
        mass=1.8,
        origin=Origin(xyz=(0.075, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_shoulder",
        ArticulationType.REVOLUTE,
        parent=base,
        child=shoulder_link,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.55, upper=1.20, effort=80.0, velocity=1.0),
    )
    model.articulation(
        "shoulder_to_forelink",
        ArticulationType.REVOLUTE,
        parent=shoulder_link,
        child=forelink,
        origin=Origin(xyz=(SHOULDER_LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.30, upper=1.35, effort=55.0, velocity=1.3),
    )
    model.articulation(
        "forelink_to_wrist",
        ArticulationType.REVOLUTE,
        parent=forelink,
        child=wrist_block,
        origin=Origin(xyz=(FORELINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.10, upper=1.15, effort=22.0, velocity=1.8),
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
    base = object_model.get_part("pedestal_base")
    shoulder_link = object_model.get_part("shoulder_link")
    forelink = object_model.get_part("forelink")
    wrist_block = object_model.get_part("wrist_block")

    shoulder_joint = object_model.get_articulation("base_to_shoulder")
    elbow_joint = object_model.get_articulation("shoulder_to_forelink")
    wrist_joint = object_model.get_articulation("forelink_to_wrist")

    ctx.check(
        "all primary parts are present",
        all(part is not None for part in (base, shoulder_link, forelink, wrist_block)),
    )
    ctx.check(
        "all revolute joints are present",
        all(joint is not None for joint in (shoulder_joint, elbow_joint, wrist_joint)),
    )

    ctx.allow_overlap(
        base,
        shoulder_link,
        elem_a="base_shell",
        elem_b="shoulder_shell",
        reason="The shoulder knuckle is modeled as a zero-clearance supported pin stack in one mesh-backed hinge interface.",
    )
    ctx.allow_overlap(
        shoulder_link,
        forelink,
        elem_a="shoulder_shell",
        elem_b="forelink_shell",
        reason="The elbow hinge uses a simplified zero-clearance barrel-and-clevis fit to represent the supported pin axis.",
    )
    ctx.allow_overlap(
        forelink,
        wrist_block,
        elem_a="forelink_shell",
        elem_b="wrist_shell",
        reason="The wrist hinge is represented as a compact zero-clearance supported pin fit in the mesh-backed tip assembly.",
    )

    ctx.expect_origin_gap(
        shoulder_link,
        base,
        axis="z",
        min_gap=0.43,
        max_gap=0.48,
        name="shoulder pivot sits above the pedestal",
    )
    ctx.expect_overlap(
        base,
        shoulder_link,
        axes="yz",
        min_overlap=0.03,
        name="base and shoulder share a supported shoulder hinge envelope",
    )
    ctx.expect_overlap(
        shoulder_link,
        forelink,
        axes="yz",
        min_overlap=0.02,
        name="shoulder link and forelink align at the elbow hinge",
    )
    ctx.expect_overlap(
        forelink,
        wrist_block,
        axes="yz",
        min_overlap=0.016,
        name="forelink and wrist block align at the wrist hinge",
    )
    ctx.expect_contact(
        base,
        shoulder_link,
        elem_a="base_shell",
        elem_b="shoulder_shell",
        name="shoulder hinge remains mounted on the pedestal yoke",
    )
    ctx.expect_contact(
        shoulder_link,
        forelink,
        elem_a="shoulder_shell",
        elem_b="forelink_shell",
        name="elbow hinge remains mounted in the shoulder clevis",
    )
    ctx.expect_contact(
        forelink,
        wrist_block,
        elem_a="forelink_shell",
        elem_b="wrist_shell",
        name="wrist hinge remains mounted in the forelink clevis",
    )

    elbow_rest = ctx.part_world_position(forelink)
    with ctx.pose({shoulder_joint: 0.75}):
        elbow_raised = ctx.part_world_position(forelink)
    ctx.check(
        "positive shoulder rotation raises the elbow",
        elbow_rest is not None
        and elbow_raised is not None
        and elbow_raised[2] > elbow_rest[2] + 0.10,
        details=f"rest={elbow_rest}, raised={elbow_raised}",
    )

    with ctx.pose({shoulder_joint: 0.65, elbow_joint: 0.0}):
        wrist_origin_rest = ctx.part_world_position(wrist_block)
    with ctx.pose({shoulder_joint: 0.65, elbow_joint: 0.90}):
        wrist_origin_raised = ctx.part_world_position(wrist_block)
    ctx.check(
        "positive elbow rotation raises the wrist joint",
        wrist_origin_rest is not None
        and wrist_origin_raised is not None
        and wrist_origin_raised[2] > wrist_origin_rest[2] + 0.08,
        details=f"rest={wrist_origin_rest}, raised={wrist_origin_raised}",
    )

    def _center_z(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> float | None:
        if aabb is None:
            return None
        return 0.5 * (aabb[0][2] + aabb[1][2])

    with ctx.pose({shoulder_joint: 0.25, elbow_joint: 0.20, wrist_joint: 0.0}):
        wrist_neutral = ctx.part_element_world_aabb(wrist_block, elem="wrist_shell")
    with ctx.pose({shoulder_joint: 0.25, elbow_joint: 0.20, wrist_joint: 0.80}):
        wrist_pitched = ctx.part_element_world_aabb(wrist_block, elem="wrist_shell")
    wrist_neutral_z = _center_z(wrist_neutral)
    wrist_pitched_z = _center_z(wrist_pitched)
    ctx.check(
        "positive wrist rotation lifts the tip block",
        wrist_neutral_z is not None
        and wrist_pitched_z is not None
        and wrist_pitched_z > wrist_neutral_z + 0.015,
        details=f"neutral_z={wrist_neutral_z}, pitched_z={wrist_pitched_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
