from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi, sqrt

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


PART_WIDTH = 0.014
LUG_WIDTH = 0.006
TAB_WIDTH = 0.024

BASE_BACK_LEN = 0.034
BASE_TAB_HEIGHT = 0.008
BASE_KNUCKLE_HEIGHT = 0.021

SIDE_PLATE_WIDTH = 0.004
SIDE_PLATE_CENTER_Y = 0.0053
JOINT_BACK = 0.0065
JOINT_FRONT = 0.0035
ROOT_NECK_LEN = 0.010

PROXIMAL_SPAN = 0.052
MIDDLE_SPAN = 0.036
DISTAL_LENGTH = 0.030


def _rounded_beam(length: float, width: float, height: float, fillet: float) -> cq.Workplane:
    fillet = min(fillet, width * 0.49, height * 0.49)
    beam = cq.Workplane("YZ").rect(width, height).extrude(length)
    if fillet > 0.0:
        beam = beam.edges("|X").fillet(fillet)
    return beam


def _y_cylinder(
    radius: float,
    length: float,
    *,
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .center(x, z)
        .circle(radius)
        .extrude(length)
        .translate((0.0, y - length / 2.0, 0.0))
    )


def _root_lug(root_radius: float, lug_height: float) -> cq.Workplane:
    lug = _y_cylinder(root_radius, LUG_WIDTH)
    neck = _rounded_beam(ROOT_NECK_LEN, LUG_WIDTH, lug_height, fillet=0.0006)
    return lug.union(neck)


def _beam_between(
    x0: float,
    x1: float,
    *,
    width: float,
    height: float,
    y: float = 0.0,
    z: float = 0.0,
    fillet: float = 0.0,
) -> cq.Workplane:
    return _rounded_beam(x1 - x0, width, height, fillet=fillet).translate((x0, y, z))


def _side_plate_pair(
    x0: float,
    x1: float,
    *,
    width: float,
    height: float,
    y_center: float = SIDE_PLATE_CENTER_Y,
    fillet: float = 0.0,
) -> cq.Workplane:
    pair = None
    for sign in (-1.0, 1.0):
        plate = _beam_between(
            x0,
            x1,
            width=width,
            height=height,
            y=sign * y_center,
            fillet=fillet,
        )
        pair = plate if pair is None else pair.union(plate)
    return pair


def _fork_end(joint_x: float, *, barrel_radius: float, arm_height: float) -> cq.Workplane:
    fork = _side_plate_pair(
        joint_x - JOINT_BACK,
        joint_x,
        width=SIDE_PLATE_WIDTH,
        height=arm_height,
        fillet=0.0004,
    )
    for sign in (-1.0, 1.0):
        barrel = _y_cylinder(
            barrel_radius,
            SIDE_PLATE_WIDTH,
            x=joint_x,
            y=sign * SIDE_PLATE_CENTER_Y,
        )
        fork = fork.union(barrel)
    return fork


def _make_base_shape() -> cq.Workplane:
    tab = _beam_between(-BASE_BACK_LEN, -0.014, width=TAB_WIDTH, height=BASE_TAB_HEIGHT, fillet=0.0012)
    pedestal = _beam_between(-0.022, -0.014, width=0.016, height=0.012, fillet=0.0008)
    side_straps = _side_plate_pair(-0.014, 0.0, width=SIDE_PLATE_WIDTH, height=0.014, fillet=0.0004)
    knuckle = _fork_end(0.0, barrel_radius=0.0055, arm_height=0.014)
    return tab.union(pedestal).union(side_straps).union(knuckle)


def _make_segment_with_fork(
    *,
    span: float,
    root_radius: float,
    root_height: float,
    body_width: float,
    body_height: float,
    distal_height: float,
) -> cq.Workplane:
    root = _root_lug(root_radius, root_height)
    split_start = span - (JOINT_BACK + JOINT_FRONT)
    shaft = _beam_between(
        ROOT_NECK_LEN,
        split_start,
        width=body_width,
        height=body_height,
        fillet=0.0010,
    )
    dorsal_pad = _rounded_beam(
        max((split_start - ROOT_NECK_LEN) * 0.36, 0.010),
        max(body_width - 0.001, LUG_WIDTH),
        body_height + 0.0015,
        fillet=0.0008,
    ).translate((ROOT_NECK_LEN + 0.008, 0.0, 0.001))
    side_straps = _side_plate_pair(
        split_start,
        span,
        width=SIDE_PLATE_WIDTH,
        height=body_height,
        fillet=0.0004,
    )
    fork = _fork_end(span, barrel_radius=distal_height * 0.30, arm_height=distal_height)
    return root.union(shaft).union(dorsal_pad).union(side_straps).union(fork)


def _make_distal_shape() -> cq.Workplane:
    root = _root_lug(0.0038, 0.012)
    shaft_start = ROOT_NECK_LEN
    tip_radius = 0.0052
    shaft_len = DISTAL_LENGTH - shaft_start - tip_radius
    shaft = _beam_between(
        shaft_start,
        shaft_start + shaft_len,
        width=0.011,
        height=0.0115,
        fillet=0.0009,
    )
    fingertip = _y_cylinder(tip_radius, 0.011, x=DISTAL_LENGTH - tip_radius)
    return root.union(shaft).union(fingertip)


def _axis_parallel(a: tuple[float, float, float], b: tuple[float, float, float]) -> bool:
    dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
    mag_a = sqrt(a[0] ** 2 + a[1] ** 2 + a[2] ** 2)
    mag_b = sqrt(b[0] ** 2 + b[1] ** 2 + b[2] ** 2)
    if mag_a == 0.0 or mag_b == 0.0:
        return False
    return abs(abs(dot) - mag_a * mag_b) < 1e-9


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="broad_knuckle_finger_linkage")

    base_finish = model.material("base_finish", rgba=(0.22, 0.24, 0.27, 1.0))
    proximal_finish = model.material("proximal_finish", rgba=(0.68, 0.70, 0.72, 1.0))
    middle_finish = model.material("middle_finish", rgba=(0.63, 0.66, 0.69, 1.0))
    distal_finish = model.material("distal_finish", rgba=(0.58, 0.61, 0.64, 1.0))

    base_tab = model.part("base_tab")
    base_tab.visual(
        Box((0.022, TAB_WIDTH, 0.008)),
        origin=Origin(xyz=(-0.023, 0.0, 0.0)),
        material=base_finish,
        name="mount_tab",
    )
    base_tab.visual(
        Box((0.011, 0.010, 0.010)),
        origin=Origin(xyz=(-0.0060, 0.0, 0.0)),
        material=base_finish,
        name="center_bridge",
    )
    for sign, suffix in ((-1.0, "inner"), (1.0, "outer")):
        base_tab.visual(
            Box((0.014, 0.003, 0.014)),
            origin=Origin(xyz=(-0.0070, sign * 0.0060, 0.0)),
            material=base_finish,
            name=f"base_cheek_{suffix}",
        )
        base_tab.visual(
            Cylinder(radius=0.0060, length=0.003),
            origin=Origin(xyz=(0.0, sign * 0.0060, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=base_finish,
            name=f"base_barrel_{suffix}",
        )
    base_tab.inertial = Inertial.from_geometry(
        Box((0.044, TAB_WIDTH, BASE_KNUCKLE_HEIGHT)),
        mass=0.08,
        origin=Origin(xyz=(-0.020, 0.0, 0.0)),
    )

    proximal = model.part("proximal_phalanx")
    proximal.visual(
        Cylinder(radius=0.0046, length=0.0040),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=proximal_finish,
        name="root_lug",
    )
    proximal.visual(
        Box((0.010, 0.0040, 0.013)),
        origin=Origin(xyz=(0.005, 0.0, 0.0)),
        material=proximal_finish,
        name="root_neck",
    )
    proximal.visual(
        Box((0.030, 0.013, 0.015)),
        origin=Origin(xyz=(0.025, 0.0, 0.0005)),
        material=proximal_finish,
        name="main_beam",
    )
    proximal.visual(
        Box((0.016, 0.011, 0.016)),
        origin=Origin(xyz=(0.030, 0.0, 0.0020)),
        material=proximal_finish,
        name="dorsal_pad",
    )
    for sign, suffix in ((-1.0, "inner"), (1.0, "outer")):
        proximal.visual(
            Box((0.012, 0.003, 0.015)),
            origin=Origin(xyz=(0.046, sign * 0.0060, 0.0)),
            material=proximal_finish,
            name=f"distal_cheek_{suffix}",
        )
        proximal.visual(
            Cylinder(radius=0.0058, length=0.003),
            origin=Origin(xyz=(PROXIMAL_SPAN, sign * 0.0060, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=proximal_finish,
            name=f"distal_barrel_{suffix}",
        )
    proximal.inertial = Inertial.from_geometry(
        Box((0.060, PART_WIDTH, 0.018)),
        mass=0.06,
        origin=Origin(xyz=(0.026, 0.0, 0.0)),
    )

    middle = model.part("middle_phalanx")
    middle.visual(
        Cylinder(radius=0.0044, length=0.0090),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=middle_finish,
        name="root_lug",
    )
    middle.visual(
        Box((0.010, 0.0060, 0.012)),
        origin=Origin(xyz=(0.005, 0.0, 0.0)),
        material=middle_finish,
        name="root_neck",
    )
    middle.visual(
        Box((0.020, 0.012, 0.013)),
        origin=Origin(xyz=(0.020, 0.0, 0.0005)),
        material=middle_finish,
        name="main_beam",
    )
    middle.visual(
        Box((0.012, 0.010, 0.014)),
        origin=Origin(xyz=(0.022, 0.0, 0.0015)),
        material=middle_finish,
        name="dorsal_pad",
    )
    for sign, suffix in ((-1.0, "inner"), (1.0, "outer")):
        middle.visual(
            Box((0.010, 0.003, 0.0135)),
            origin=Origin(xyz=(0.031, sign * 0.0060, 0.0)),
            material=middle_finish,
            name=f"distal_cheek_{suffix}",
        )
        middle.visual(
            Cylinder(radius=0.0055, length=0.003),
            origin=Origin(xyz=(MIDDLE_SPAN, sign * 0.0060, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=middle_finish,
            name=f"distal_barrel_{suffix}",
        )
    middle.inertial = Inertial.from_geometry(
        Box((0.043, PART_WIDTH, 0.016)),
        mass=0.04,
        origin=Origin(xyz=(0.018, 0.0, 0.0)),
    )

    distal = model.part("distal_phalanx")
    distal.visual(
        Cylinder(radius=0.0041, length=0.0090),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=distal_finish,
        name="root_lug",
    )
    distal.visual(
        Box((0.010, 0.0060, 0.011)),
        origin=Origin(xyz=(0.005, 0.0, 0.0)),
        material=distal_finish,
        name="root_neck",
    )
    distal.visual(
        Box((0.014, 0.011, 0.011)),
        origin=Origin(xyz=(0.017, 0.0, -0.0003)),
        material=distal_finish,
        name="main_beam",
    )
    distal.visual(
        Cylinder(radius=0.0052, length=0.011),
        origin=Origin(xyz=(0.0248, 0.0, -0.0004), rpy=(pi / 2.0, 0.0, 0.0)),
        material=distal_finish,
        name="fingertip",
    )
    distal.inertial = Inertial.from_geometry(
        Box((0.035, 0.014, 0.013)),
        mass=0.025,
        origin=Origin(xyz=(0.014, 0.0, -0.0005)),
    )

    model.articulation(
        "base_to_proximal",
        ArticulationType.REVOLUTE,
        parent=base_tab,
        child=proximal,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=3.0, lower=-0.25, upper=1.20),
    )
    model.articulation(
        "proximal_to_middle",
        ArticulationType.REVOLUTE,
        parent=proximal,
        child=middle,
        origin=Origin(xyz=(PROXIMAL_SPAN, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.5, velocity=3.5, lower=0.0, upper=1.45),
    )
    model.articulation(
        "middle_to_distal",
        ArticulationType.REVOLUTE,
        parent=middle,
        child=distal,
        origin=Origin(xyz=(MIDDLE_SPAN, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=0.0, upper=1.30),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_tab = object_model.get_part("base_tab")
    proximal = object_model.get_part("proximal_phalanx")
    middle = object_model.get_part("middle_phalanx")
    distal = object_model.get_part("distal_phalanx")

    base_joint = object_model.get_articulation("base_to_proximal")
    middle_joint = object_model.get_articulation("proximal_to_middle")
    distal_joint = object_model.get_articulation("middle_to_distal")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
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
        "all four linkage parts exist",
        len(object_model.parts) == 4,
        details=f"parts={[part.name for part in object_model.parts]}",
    )
    ctx.check(
        "three parallel knuckle axes",
        _axis_parallel(base_joint.axis, middle_joint.axis)
        and _axis_parallel(base_joint.axis, distal_joint.axis),
        details=(
            f"base={base_joint.axis}, middle={middle_joint.axis}, distal={distal_joint.axis}"
        ),
    )

    ctx.expect_contact(base_tab, proximal, contact_tol=0.0008, name="base knuckle supports proximal")
    ctx.expect_contact(proximal, middle, contact_tol=0.0008, name="proximal knuckle supports middle")
    ctx.expect_contact(middle, distal, contact_tol=0.0008, name="middle knuckle supports distal")

    ctx.expect_overlap(
        base_tab,
        proximal,
        axes="xz",
        min_overlap=0.010,
        name="base and proximal visibly overlap at hinge",
    )
    ctx.expect_overlap(
        proximal,
        middle,
        axes="xz",
        min_overlap=0.009,
        name="proximal and middle visibly overlap at hinge",
    )
    ctx.expect_overlap(
        middle,
        distal,
        axes="xz",
        min_overlap=0.008,
        name="middle and distal visibly overlap at hinge",
    )

    with ctx.pose(
        {
            base_joint: 0.85,
            middle_joint: 1.00,
            distal_joint: 0.80,
        }
    ):
        ctx.expect_origin_gap(
            base_tab,
            distal,
            axis="z",
            min_gap=0.020,
            name="positive curl drops fingertip below base axis",
        )
        ctx.expect_origin_gap(
            distal,
            base_tab,
            axis="x",
            max_gap=0.075,
            name="curl retracts fingertip toward palm side",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
