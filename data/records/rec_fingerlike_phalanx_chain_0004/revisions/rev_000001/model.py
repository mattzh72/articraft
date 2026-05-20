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

INNER_GAP = 0.016
PLATE_THICKNESS = 0.003
TOTAL_WIDTH = INNER_GAP + 2.0 * PLATE_THICKNESS
PIN_RADIUS = 0.0028
PIN_HEAD_RADIUS = 0.0045
PIN_HEAD_THICKNESS = 0.0014
PIN_CLEARANCE = 0.0004
BARREL_RADIUS = 0.0058
FORK_BOSS_RADIUS = 0.0066
BEAM_WIDTH = 0.0064

PROXIMAL_LENGTH = 0.066
MIDDLE_LENGTH = 0.044
DISTAL_LENGTH = 0.033
BASE_REAR_LENGTH = 0.030


def _make_cylinder_xz(center_x: float, radius: float, length_y: float) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .center(center_x, 0.0)
        .circle(radius)
        .extrude(length_y / 2.0, both=True)
    )


def _make_beam(
    start_x: float,
    end_x: float,
    start_height: float,
    end_height: float,
    width_y: float,
) -> cq.Workplane:
    profile = [
        (start_x, -start_height / 2.0),
        (end_x, -end_height / 2.0),
        (end_x, end_height / 2.0),
        (start_x, start_height / 2.0),
    ]
    return (
        cq.Workplane("XZ")
        .polyline(profile)
        .close()
        .extrude(width_y / 2.0, both=True)
    )


def _make_fork_plate(
    start_x: float,
    end_x: float,
    start_height: float,
    end_height: float,
    y_center: float,
    hole_radius: float | None = None,
) -> cq.Workplane:
    plate = _make_beam(start_x, end_x, start_height, end_height, PLATE_THICKNESS).translate(
        (0.0, y_center, 0.0)
    )
    start_pad = _make_cylinder_xz(start_x, start_height * 0.32, PLATE_THICKNESS).translate(
        (0.0, y_center, 0.0)
    )
    end_boss = _make_cylinder_xz(end_x, FORK_BOSS_RADIUS, PLATE_THICKNESS).translate(
        (0.0, y_center, 0.0)
    )
    plate = plate.union(start_pad).union(end_boss)
    if hole_radius is not None:
        plate = plate.cut(
            _make_cylinder_xz(end_x, hole_radius, PLATE_THICKNESS + 0.002).translate(
                (0.0, y_center, 0.0)
            )
        )
    return plate


def _make_barrel() -> cq.Workplane:
    outer = _make_cylinder_xz(0.0, BARREL_RADIUS, INNER_GAP)
    bore = _make_cylinder_xz(0.0, PIN_RADIUS + PIN_CLEARANCE, TOTAL_WIDTH + 0.004)
    return outer.cut(bore)


def _make_pin(x_pos: float) -> cq.Workplane:
    shaft = _make_cylinder_xz(x_pos, PIN_RADIUS, TOTAL_WIDTH)
    left_head = (
        cq.Workplane("XZ")
        .workplane(offset=-(TOTAL_WIDTH / 2.0 + PIN_HEAD_THICKNESS / 2.0))
        .center(x_pos, 0.0)
        .circle(PIN_HEAD_RADIUS)
        .extrude(PIN_HEAD_THICKNESS / 2.0, both=True)
    )
    right_head = (
        cq.Workplane("XZ")
        .workplane(offset=(TOTAL_WIDTH / 2.0 + PIN_HEAD_THICKNESS / 2.0))
        .center(x_pos, 0.0)
        .circle(PIN_HEAD_RADIUS)
        .extrude(PIN_HEAD_THICKNESS / 2.0, both=True)
    )
    return shaft.union(left_head).union(right_head)


def _make_tip_pad(x_center: float, length: float, width: float, height: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(length, width, height)
        .translate((x_center, 0.0, 0.0))
    )


def _make_link_shell(
    length: float,
    base_height: float,
    tip_height: float,
    *,
    has_distal_pin: bool,
) -> cq.Workplane:
    y_offset = INNER_GAP / 2.0 + PLATE_THICKNESS / 2.0
    fork_start = length - 0.0105
    beam = _make_beam(
        BARREL_RADIUS * 0.58,
        fork_start + 0.0015,
        base_height,
        max(tip_height, base_height * 0.68),
        BEAM_WIDTH,
    )
    left_plate = _make_fork_plate(
        fork_start,
        length,
        max(tip_height, base_height * 0.70),
        tip_height,
        -y_offset,
        hole_radius=PIN_RADIUS + PIN_CLEARANCE,
    )
    right_plate = _make_fork_plate(
        fork_start,
        length,
        max(tip_height, base_height * 0.70),
        tip_height,
        y_offset,
        hole_radius=PIN_RADIUS + PIN_CLEARANCE,
    )
    shell = _make_barrel().union(beam).union(left_plate).union(right_plate)
    if not has_distal_pin:
        shell = shell.union(
            _make_tip_pad(
                length - 0.0045,
                0.010,
                BEAM_WIDTH + 0.0018,
                max(0.0052, tip_height * 0.74),
            )
        )
    return shell


def _make_base_shell() -> cq.Workplane:
    y_offset = INNER_GAP / 2.0 + PLATE_THICKNESS / 2.0
    left_ear = _make_fork_plate(
        -BASE_REAR_LENGTH * 0.60,
        0.0,
        0.024,
        0.020,
        -y_offset,
        hole_radius=PIN_RADIUS + PIN_CLEARANCE,
    )
    right_ear = _make_fork_plate(
        -BASE_REAR_LENGTH * 0.60,
        0.0,
        0.024,
        0.020,
        y_offset,
        hole_radius=PIN_RADIUS + PIN_CLEARANCE,
    )
    rear_beam = _make_beam(
        -BASE_REAR_LENGTH * 1.02,
        -0.008,
        0.018,
        0.016,
        BEAM_WIDTH + 0.003,
    )
    rear_pad = (
        cq.Workplane("XY")
        .box(BASE_REAR_LENGTH * 0.22, BEAM_WIDTH + 0.006, 0.014)
        .translate((-BASE_REAR_LENGTH * 0.98, 0.0, 0.0))
    )
    return left_ear.union(right_ear).union(rear_beam).union(rear_pad)


def _part_box_inertial(
    size_x: float,
    size_y: float,
    size_z: float,
    center_x: float,
    mass: float,
) -> Inertial:
    geom = Box((size_x, size_y, size_z))
    return Inertial.from_geometry(
        geom,
        mass=mass,
        origin=Origin(xyz=(center_x, 0.0, 0.0)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="finger_phalanx_chain", assets=ASSETS)

    steel = model.material("steel", rgba=(0.58, 0.60, 0.64, 1.0))
    pin_steel = model.material("pin_steel", rgba=(0.74, 0.76, 0.80, 1.0))
    polymer = model.material("polymer", rgba=(0.18, 0.20, 0.23, 1.0))

    base = model.part("base_knuckle")
    base.visual(
        mesh_from_cadquery(_make_base_shell(), "base_knuckle_shell.obj", assets=ASSETS),
        material=steel,
        name="shell",
    )
    base.visual(
        mesh_from_cadquery(_make_pin(0.0), "base_knuckle_pin.obj", assets=ASSETS),
        material=pin_steel,
        name="pin",
    )
    base.inertial = _part_box_inertial(
        BASE_REAR_LENGTH * 1.15,
        TOTAL_WIDTH + 2.0 * PIN_HEAD_THICKNESS,
        0.024,
        -BASE_REAR_LENGTH * 0.62,
        mass=0.22,
    )

    proximal = model.part("proximal_phalanx")
    proximal.visual(
        mesh_from_cadquery(
            _make_link_shell(PROXIMAL_LENGTH, 0.019, 0.014, has_distal_pin=True),
            "proximal_shell.obj",
            assets=ASSETS,
        ),
        material=polymer,
        name="shell",
    )
    proximal.visual(
        mesh_from_cadquery(_make_barrel(), "proximal_barrel.obj", assets=ASSETS),
        material=steel,
        name="barrel",
    )
    proximal.inertial = _part_box_inertial(
        PROXIMAL_LENGTH + BARREL_RADIUS,
        TOTAL_WIDTH + 2.0 * PIN_HEAD_THICKNESS,
        0.019,
        PROXIMAL_LENGTH * 0.48,
        mass=0.13,
    )

    middle = model.part("middle_phalanx")
    middle.visual(
        mesh_from_cadquery(
            _make_link_shell(MIDDLE_LENGTH, 0.015, 0.011, has_distal_pin=True),
            "middle_shell.obj",
            assets=ASSETS,
        ),
        material=polymer,
        name="shell",
    )
    middle.visual(
        mesh_from_cadquery(_make_barrel(), "middle_barrel.obj", assets=ASSETS),
        material=steel,
        name="barrel",
    )
    middle.inertial = _part_box_inertial(
        MIDDLE_LENGTH + BARREL_RADIUS,
        TOTAL_WIDTH + 2.0 * PIN_HEAD_THICKNESS,
        0.015,
        MIDDLE_LENGTH * 0.48,
        mass=0.09,
    )

    distal = model.part("distal_phalanx")
    distal.visual(
        mesh_from_cadquery(
            _make_link_shell(DISTAL_LENGTH, 0.0125, 0.0085, has_distal_pin=False),
            "distal_shell.obj",
            assets=ASSETS,
        ),
        material=polymer,
        name="shell",
    )
    distal.visual(
        mesh_from_cadquery(_make_barrel(), "distal_barrel.obj", assets=ASSETS),
        material=steel,
        name="barrel",
    )
    distal.inertial = _part_box_inertial(
        DISTAL_LENGTH + BARREL_RADIUS,
        TOTAL_WIDTH,
        0.013,
        DISTAL_LENGTH * 0.46,
        mass=0.06,
    )

    model.articulation(
        "base_to_proximal",
        ArticulationType.REVOLUTE,
        parent=base,
        child=proximal,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.6,
            lower=0.0,
            upper=math.pi / 2.0,
        ),
    )
    model.articulation(
        "proximal_to_middle",
        ArticulationType.REVOLUTE,
        parent=proximal,
        child=middle,
        origin=Origin(xyz=(PROXIMAL_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=2.4,
            lower=0.0,
            upper=math.radians(75.0),
        ),
    )
    model.articulation(
        "middle_to_distal",
        ArticulationType.REVOLUTE,
        parent=middle,
        child=distal,
        origin=Origin(xyz=(MIDDLE_LENGTH, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.5,
            velocity=2.2,
            lower=0.0,
            upper=math.radians(75.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base_knuckle")
    proximal = object_model.get_part("proximal_phalanx")
    middle = object_model.get_part("middle_phalanx")
    distal = object_model.get_part("distal_phalanx")

    base_to_proximal = object_model.get_articulation("base_to_proximal")
    proximal_to_middle = object_model.get_articulation("proximal_to_middle")
    middle_to_distal = object_model.get_articulation("middle_to_distal")

    base_shell = base.get_visual("shell")
    proximal_shell = proximal.get_visual("shell")
    proximal_barrel = proximal.get_visual("barrel")
    middle_shell = middle.get_visual("shell")
    middle_barrel = middle.get_visual("barrel")
    distal_shell = distal.get_visual("shell")
    distal_barrel = distal.get_visual("barrel")

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
    ctx.allow_overlap(
        base,
        proximal,
        elem_a=base_shell,
        elem_b=proximal_shell,
        reason="Base clevis and proximal hinge barrel are modeled as a tightly nested seated hinge capture.",
    )
    ctx.allow_overlap(
        proximal,
        middle,
        elem_a=proximal_shell,
        elem_b=middle_shell,
        reason="Proximal clevis and middle hinge barrel are modeled as a tightly nested seated hinge capture.",
    )
    ctx.allow_overlap(
        middle,
        distal,
        elem_a=middle_shell,
        elem_b=distal_shell,
        reason="Middle clevis and distal hinge barrel are modeled as a tightly nested seated hinge capture.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        proximal,
        base,
        elem_a=proximal_barrel,
        elem_b=base_shell,
        name="proximal_barrel_seated_in_base_knuckle",
    )
    ctx.expect_contact(
        middle,
        proximal,
        elem_a=middle_barrel,
        elem_b=proximal_shell,
        name="middle_barrel_seated_in_proximal_link",
    )
    ctx.expect_contact(
        distal,
        middle,
        elem_a=distal_barrel,
        elem_b=middle_shell,
        name="distal_barrel_seated_in_middle_link",
    )

    ctx.expect_overlap(
        proximal,
        base,
        axes="yz",
        min_overlap=0.010,
        elem_a=proximal_barrel,
        elem_b=base_shell,
        name="base_joint_has_visible_side_plate_capture",
    )
    ctx.expect_overlap(
        middle,
        proximal,
        axes="yz",
        min_overlap=0.009,
        elem_a=middle_barrel,
        elem_b=proximal_shell,
        name="middle_joint_has_visible_side_plate_capture",
    )
    ctx.expect_overlap(
        distal,
        middle,
        axes="yz",
        min_overlap=0.008,
        elem_a=distal_barrel,
        elem_b=middle_shell,
        name="distal_joint_has_visible_side_plate_capture",
    )

    def _center_from_aabb(aabb: tuple[tuple[float, float, float], tuple[float, float, float]]) -> tuple[float, float, float]:
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) / 2.0 for i in range(3))

    base_limits = base_to_proximal.motion_limits
    middle_limits = proximal_to_middle.motion_limits
    distal_limits = middle_to_distal.motion_limits
    ctx.check(
        "base_joint_limits_and_axis",
        base_to_proximal.axis == (0.0, 1.0, 0.0)
        and math.isclose(base_limits.lower, 0.0, abs_tol=1e-6)
        and math.isclose(base_limits.upper, math.pi / 2.0, abs_tol=1e-6),
        details="Base joint should bend about +Y from 0 to 90 degrees.",
    )
    ctx.check(
        "distal_joint_limits_and_axis",
        proximal_to_middle.axis == (0.0, 1.0, 0.0)
        and middle_to_distal.axis == (0.0, 1.0, 0.0)
        and math.isclose(middle_limits.lower, 0.0, abs_tol=1e-6)
        and math.isclose(middle_limits.upper, math.radians(75.0), abs_tol=1e-6)
        and math.isclose(distal_limits.lower, 0.0, abs_tol=1e-6)
        and math.isclose(distal_limits.upper, math.radians(75.0), abs_tol=1e-6),
        details="Middle and distal joints should bend about +Y from 0 to 75 degrees.",
    )

    rest_middle_pos = ctx.part_world_position(middle)
    rest_distal_shell = ctx.part_element_world_aabb(distal, elem=distal_shell)
    with ctx.pose(
        {
            base_to_proximal: math.pi / 2.0,
            proximal_to_middle: math.radians(50.0),
            middle_to_distal: math.radians(50.0),
        }
    ):
        flex_middle_pos = ctx.part_world_position(middle)
        flex_distal_shell = ctx.part_element_world_aabb(distal, elem=distal_shell)

    middle_ok = (
        rest_middle_pos is not None
        and flex_middle_pos is not None
        and abs(flex_middle_pos[0]) < 0.010
        and flex_middle_pos[2] < rest_middle_pos[2] - 0.040
    )
    ctx.check(
        "base_joint_rotates_chain_downward_in_one_plane",
        middle_ok,
        details="The middle joint origin should swing from +X toward -Z when the base joint flexes.",
    )

    distal_ok = False
    if rest_distal_shell is not None and flex_distal_shell is not None:
        rest_center = _center_from_aabb(rest_distal_shell)
        flex_center = _center_from_aabb(flex_distal_shell)
        distal_ok = flex_center[2] < rest_center[2] - 0.050 and abs(flex_center[1]) < 0.004
    ctx.check(
        "distal_tip_curls_without_leaving_bending_plane",
        distal_ok,
        details="The distal phalanx tip should move downward strongly while staying centered in Y.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
