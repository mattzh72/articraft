from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


HOUSING_SIZE = 0.082
HOUSING_HEIGHT = 0.020
BEZEL_HEIGHT = 0.0026
GIMBAL_Z = 0.011

LOWER_CAVITY_SIZE = 0.052
LOWER_CAVITY_BOTTOM = 0.003
LOWER_CAVITY_HEIGHT = 0.014

UPPER_RELIEF_X = 0.044
UPPER_RELIEF_Y = 0.026
UPPER_RELIEF_BOTTOM = 0.013
UPPER_RELIEF_HEIGHT = HOUSING_HEIGHT - UPPER_RELIEF_BOTTOM

OUTER_PIN_RADIUS = 0.0025
OUTER_PIN_HALF_LENGTH = 0.005
OUTER_PIN_OFFSET = 0.024

INNER_PIN_RADIUS = 0.0020
OUTER_FORK_ARM_CENTER_X = 0.015
INNER_PIN_HALF_LENGTH = 0.004
INNER_PIN_CENTER_X = OUTER_FORK_ARM_CENTER_X
OUTER_LIMIT = 0.20
INNER_LIMIT = 0.20


def _translated_box(size: tuple[float, float, float], xyz: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size, centered=(True, True, True)).translate(xyz)


def _translated_cylinder_y(radius: float, half_length: float, xyz: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XZ").circle(radius).extrude(half_length, both=True).translate(xyz)


def _translated_cylinder_x(radius: float, half_length: float, xyz: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("YZ").circle(radius).extrude(half_length, both=True).translate(xyz)


def _octagon_points(radius: float) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(math.radians(22.5 + 45.0 * i)),
            radius * math.sin(math.radians(22.5 + 45.0 * i)),
        )
        for i in range(8)
    ]


def _make_housing_shell() -> cq.Workplane:
    left_rail = cq.Workplane("XY").box(0.010, 0.060, 0.014, centered=(True, True, False)).translate((-0.030, 0.0, 0.0))
    right_rail = cq.Workplane("XY").box(0.010, 0.060, 0.014, centered=(True, True, False)).translate((0.030, 0.0, 0.0))
    front_tower = cq.Workplane("XY").box(0.020, 0.010, 0.018, centered=(True, True, False)).translate((0.0, OUTER_PIN_OFFSET, 0.0))
    back_tower = cq.Workplane("XY").box(0.020, 0.010, 0.018, centered=(True, True, False)).translate((0.0, -OUTER_PIN_OFFSET, 0.0))
    top_collar = (
        cq.Workplane("XY")
        .box(0.060, 0.060, 0.004, centered=(True, True, False))
        .cut(cq.Workplane("XY").box(0.042, 0.034, 0.006, centered=(True, True, False)).translate((0.0, 0.0, -0.001)))
        .translate((0.0, 0.0, 0.012))
    )
    brace_fl = _translated_box((0.010, 0.010, 0.008), (-0.022, 0.022, 0.006))
    brace_fr = _translated_box((0.010, 0.010, 0.008), (0.022, 0.022, 0.006))
    brace_bl = _translated_box((0.010, 0.010, 0.008), (-0.022, -0.022, 0.006))
    brace_br = _translated_box((0.010, 0.010, 0.008), (0.022, -0.022, 0.006))

    shell = (
        left_rail.union(right_rail)
        .union(front_tower)
        .union(back_tower)
        .union(top_collar)
        .union(brace_fl)
        .union(brace_fr)
        .union(brace_bl)
        .union(brace_br)
    )

    front_bore = _translated_cylinder_y(OUTER_PIN_RADIUS, 0.0052, (0.0, OUTER_PIN_OFFSET, GIMBAL_Z))
    back_bore = _translated_cylinder_y(OUTER_PIN_RADIUS, 0.0052, (0.0, -OUTER_PIN_OFFSET, GIMBAL_Z))
    return shell.cut(front_bore).cut(back_bore)


def _make_bezel_trim() -> cq.Workplane:
    outer = cq.Workplane("XY").polyline(_octagon_points(0.0235)).close().extrude(BEZEL_HEIGHT)
    inner = (
        cq.Workplane("XY")
        .polyline(_octagon_points(0.0162))
        .close()
        .extrude(BEZEL_HEIGHT + 0.001)
        .translate((0.0, 0.0, -0.0005))
    )
    bezel = outer.cut(inner).translate((0.0, 0.0, 0.016))

    north = cq.Workplane("XY").box(0.012, 0.0035, 0.0016, centered=(True, True, False)).translate((0.0, 0.0235, 0.016))
    south = cq.Workplane("XY").box(0.012, 0.0035, 0.0016, centered=(True, True, False)).translate((0.0, -0.0235, 0.016))
    east = cq.Workplane("XY").box(0.0035, 0.012, 0.0016, centered=(True, True, False)).translate((0.0235, 0.0, 0.016))
    west = cq.Workplane("XY").box(0.0035, 0.012, 0.0016, centered=(True, True, False)).translate((-0.0235, 0.0, 0.016))
    return bezel.union(north).union(south).union(east).union(west)


def _make_outer_fork() -> cq.Workplane:
    bridge = _translated_box((0.022, 0.006, 0.003), (0.0, 0.0, -0.0105))
    left_arm = _translated_box((0.004, 0.0045, 0.015), (-OUTER_FORK_ARM_CENTER_X, 0.0, -0.0035))
    right_arm = _translated_box((0.004, 0.0045, 0.015), (OUTER_FORK_ARM_CENTER_X, 0.0, -0.0035))
    left_gusset = _translated_box((0.010, 0.003, 0.004), (-0.008, 0.0, -0.0075))
    right_gusset = _translated_box((0.010, 0.003, 0.004), (0.008, 0.0, -0.0075))
    front_spindle = _translated_cylinder_y(OUTER_PIN_RADIUS, 0.010, (0.0, OUTER_PIN_OFFSET, 0.0))
    back_spindle = _translated_cylinder_y(OUTER_PIN_RADIUS, 0.010, (0.0, -OUTER_PIN_OFFSET, 0.0))
    front_neck = _translated_box((0.0045, 0.010, 0.0045), (0.0, 0.0135, -0.0015))
    back_neck = _translated_box((0.0045, 0.010, 0.0045), (0.0, -0.0135, -0.0015))

    fork = (
        bridge.union(left_arm)
        .union(right_arm)
        .union(left_gusset)
        .union(right_gusset)
        .union(front_neck)
        .union(back_neck)
        .union(front_spindle)
        .union(back_spindle)
    )

    arm_bore_left = _translated_cylinder_x(INNER_PIN_RADIUS + 0.0002, 0.0034, (-OUTER_FORK_ARM_CENTER_X, 0.0, 0.0))
    arm_bore_right = _translated_cylinder_x(INNER_PIN_RADIUS + 0.0002, 0.0034, (OUTER_FORK_ARM_CENTER_X, 0.0, 0.0))
    return fork.cut(arm_bore_left).cut(arm_bore_right)


def _make_inner_gimbal() -> cq.Workplane:
    ring = cq.Workplane("YZ").circle(0.0068).extrude(0.0020, both=True)
    ring = ring.cut(cq.Workplane("YZ").circle(0.0050).extrude(0.0030, both=True))

    trunnion_bar = _translated_box((0.024, 0.0024, 0.0024), (0.0, 0.0, 0.0))
    hub = cq.Workplane("XY").circle(0.0046).extrude(0.0032).translate((0.0, 0.0, -0.0010))
    stem = cq.Workplane("XY").circle(0.0037).extrude(0.0120).translate((0.0, 0.0, 0.0016))
    shoulder = cq.Workplane("XY").circle(0.0050).extrude(0.0018).translate((0.0, 0.0, 0.0130))
    left_pin = _translated_cylinder_x(INNER_PIN_RADIUS, INNER_PIN_HALF_LENGTH, (-INNER_PIN_CENTER_X, 0.0, 0.0))
    right_pin = _translated_cylinder_x(INNER_PIN_RADIUS, INNER_PIN_HALF_LENGTH, (INNER_PIN_CENTER_X, 0.0, 0.0))

    return ring.union(trunnion_bar).union(hub).union(stem).union(shoulder).union(left_pin).union(right_pin)


def _make_thumb_cap() -> cq.Workplane:
    skirt = cq.Workplane("XY").circle(0.0085).extrude(0.0030)
    crown = cq.Workplane("XY").circle(0.0070).extrude(0.0022).translate((0.0, 0.0, 0.0030))
    cap = skirt.union(crown)
    return cap.translate((0.0, 0.0, 0.0102))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="guarded_thumb_stick")

    housing_polymer = model.material("housing_polymer", rgba=(0.16, 0.17, 0.19, 1.0))
    bezel_trim = model.material("bezel_trim", rgba=(0.08, 0.08, 0.09, 1.0))
    fork_metal = model.material("fork_metal", rgba=(0.42, 0.44, 0.47, 1.0))
    gimbal_metal = model.material("gimbal_metal", rgba=(0.55, 0.57, 0.60, 1.0))
    cap_rubber = model.material("cap_rubber", rgba=(0.10, 0.10, 0.11, 1.0))

    housing = model.part("housing")
    housing.visual(Box((0.010, 0.060, 0.014)), origin=Origin(xyz=(-0.032, 0.0, 0.007)), material=housing_polymer, name="left_rail")
    housing.visual(Box((0.010, 0.060, 0.014)), origin=Origin(xyz=(0.032, 0.0, 0.007)), material=housing_polymer, name="right_rail")
    housing.visual(Box((0.014, 0.005, 0.018)), origin=Origin(xyz=(0.0, 0.0265, 0.009)), material=housing_polymer, name="front_tower")
    housing.visual(Box((0.014, 0.005, 0.018)), origin=Origin(xyz=(0.0, -0.0265, 0.009)), material=housing_polymer, name="back_tower")
    housing.visual(Box((0.010, 0.036, 0.004)), origin=Origin(xyz=(-0.025, 0.0, 0.016)), material=housing_polymer, name="left_collar")
    housing.visual(Box((0.010, 0.036, 0.004)), origin=Origin(xyz=(0.025, 0.0, 0.016)), material=housing_polymer, name="right_collar")
    housing.visual(Box((0.030, 0.010, 0.004)), origin=Origin(xyz=(0.0, 0.025, 0.016)), material=housing_polymer, name="front_collar")
    housing.visual(Box((0.030, 0.010, 0.004)), origin=Origin(xyz=(0.0, -0.025, 0.016)), material=housing_polymer, name="back_collar")
    housing.visual(Box((0.010, 0.010, 0.008)), origin=Origin(xyz=(-0.024, 0.024, 0.012)), material=housing_polymer, name="brace_fl")
    housing.visual(Box((0.010, 0.010, 0.008)), origin=Origin(xyz=(0.024, 0.024, 0.012)), material=housing_polymer, name="brace_fr")
    housing.visual(Box((0.010, 0.010, 0.008)), origin=Origin(xyz=(-0.024, -0.024, 0.012)), material=housing_polymer, name="brace_bl")
    housing.visual(Box((0.010, 0.010, 0.008)), origin=Origin(xyz=(0.024, -0.024, 0.012)), material=housing_polymer, name="brace_br")
    housing.visual(
        Box((0.006, 0.006, 0.004)),
        origin=Origin(xyz=(0.0, 0.0265, GIMBAL_Z)),
        material=housing_polymer,
        name="front_bearing_pad",
    )
    housing.visual(
        Box((0.006, 0.006, 0.004)),
        origin=Origin(xyz=(0.0, -0.0265, GIMBAL_Z)),
        material=housing_polymer,
        name="back_bearing_pad",
    )
    housing.visual(
        mesh_from_cadquery(_make_bezel_trim(), "bezel_trim"),
        material=bezel_trim,
        name="bezel_trim",
    )

    outer_fork = model.part("outer_fork")
    outer_fork.visual(Box((0.022, 0.006, 0.003)), origin=Origin(xyz=(0.0, 0.0, -0.0090)), material=fork_metal, name="bridge")
    outer_fork.visual(Box((0.004, 0.0045, 0.014)), origin=Origin(xyz=(-0.016, 0.0, -0.0015)), material=fork_metal, name="left_arm")
    outer_fork.visual(Box((0.004, 0.0045, 0.014)), origin=Origin(xyz=(0.016, 0.0, -0.0015)), material=fork_metal, name="right_arm")
    outer_fork.visual(Box((0.014, 0.0035, 0.004)), origin=Origin(xyz=(-0.011, 0.0, -0.0065)), material=fork_metal, name="left_gusset")
    outer_fork.visual(Box((0.014, 0.0035, 0.004)), origin=Origin(xyz=(0.011, 0.0, -0.0065)), material=fork_metal, name="right_gusset")
    outer_fork.visual(Box((0.0045, 0.017, 0.0045)), origin=Origin(xyz=(0.0, 0.0115, -0.0010)), material=fork_metal, name="front_neck")
    outer_fork.visual(Box((0.0045, 0.017, 0.0045)), origin=Origin(xyz=(0.0, -0.0115, -0.0010)), material=fork_metal, name="back_neck")
    outer_fork.visual(Box((0.0040, 0.010, 0.009)), origin=Origin(xyz=(0.0, 0.0075, -0.0045)), material=fork_metal, name="front_strut")
    outer_fork.visual(Box((0.0040, 0.010, 0.009)), origin=Origin(xyz=(0.0, -0.0075, -0.0045)), material=fork_metal, name="back_strut")
    outer_fork.visual(
        Cylinder(radius=OUTER_PIN_RADIUS, length=0.005),
        origin=Origin(xyz=(0.0, 0.0215, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=fork_metal,
        name="front_pin",
    )
    outer_fork.visual(
        Cylinder(radius=OUTER_PIN_RADIUS, length=0.005),
        origin=Origin(xyz=(0.0, -0.0215, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=fork_metal,
        name="back_pin",
    )

    inner_ring = model.part("inner_ring")
    inner_ring.visual(
        Cylinder(radius=INNER_PIN_RADIUS, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=gimbal_metal,
        name="trunnion_pin",
    )
    inner_ring.visual(Box((0.0022, 0.0100, 0.0022)), origin=Origin(xyz=(0.0, 0.0, 0.0055)), material=gimbal_metal, name="ring_top")
    inner_ring.visual(Box((0.0022, 0.0100, 0.0022)), origin=Origin(xyz=(0.0, 0.0, -0.0055)), material=gimbal_metal, name="ring_bottom")
    inner_ring.visual(Box((0.0022, 0.0022, 0.0100)), origin=Origin(xyz=(0.0, 0.0055, 0.0)), material=gimbal_metal, name="ring_right")
    inner_ring.visual(Box((0.0022, 0.0022, 0.0100)), origin=Origin(xyz=(0.0, -0.0055, 0.0)), material=gimbal_metal, name="ring_left")
    inner_ring.visual(Cylinder(radius=0.0046, length=0.0032), origin=Origin(xyz=(0.0, 0.0, -0.0010)), material=gimbal_metal, name="hub")
    inner_ring.visual(Cylinder(radius=0.0037, length=0.0100), origin=Origin(xyz=(0.0, 0.0, 0.0056)), material=gimbal_metal, name="stem")
    inner_ring.visual(Cylinder(radius=0.0050, length=0.0014), origin=Origin(xyz=(0.0, 0.0, 0.0113)), material=gimbal_metal, name="shoulder")
    inner_ring.visual(
        Box((0.001, 0.001, 0.001)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=gimbal_metal,
        name="inner_gimbal",
    )
    inner_ring.visual(
        Cylinder(radius=0.0075, length=0.0030),
        origin=Origin(xyz=(0.0, 0.0, 0.0118)),
        material=cap_rubber,
        name="thumb_cap",
    )
    inner_ring.visual(
        Cylinder(radius=0.0062, length=0.0016),
        origin=Origin(xyz=(0.0, 0.0, 0.0138)),
        material=cap_rubber,
        name="thumb_cap_top",
    )

    model.articulation(
        "housing_to_outer_fork",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=outer_fork,
        origin=Origin(xyz=(0.0, 0.0, GIMBAL_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=3.0,
            lower=-OUTER_LIMIT,
            upper=OUTER_LIMIT,
        ),
    )
    model.articulation(
        "outer_fork_to_inner_ring",
        ArticulationType.REVOLUTE,
        parent=outer_fork,
        child=inner_ring,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=3.0,
            lower=-INNER_LIMIT,
            upper=INNER_LIMIT,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    outer_fork = object_model.get_part("outer_fork")
    inner_ring = object_model.get_part("inner_ring")
    outer_joint = object_model.get_articulation("housing_to_outer_fork")
    inner_joint = object_model.get_articulation("outer_fork_to_inner_ring")
    bezel = housing.get_visual("bezel_trim")
    thumb_cap = inner_ring.get_visual("thumb_cap")

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
        "outer_fork_axis_matches_front_back_supports",
        tuple(outer_joint.axis) == (0.0, 1.0, 0.0),
        f"expected outer axis (0, 1, 0), got {outer_joint.axis}",
    )
    ctx.check(
        "inner_ring_axis_is_orthogonal_horizontal",
        tuple(inner_joint.axis) == (1.0, 0.0, 0.0),
        f"expected inner axis (1, 0, 0), got {inner_joint.axis}",
    )

    ctx.expect_contact(outer_fork, housing, name="outer_fork_is_borne_by_housing_pivots")
    ctx.expect_contact(inner_ring, outer_fork, name="inner_ring_is_borne_by_fork_journals")
    ctx.expect_gap(
        inner_ring,
        housing,
        axis="z",
        positive_elem=thumb_cap,
        negative_elem=bezel,
        min_gap=0.001,
        max_gap=0.004,
        name="thumb_cap_sits_just_above_bezel",
    )
    ctx.expect_overlap(
        inner_ring,
        housing,
        axes="xy",
        elem_a=thumb_cap,
        elem_b=bezel,
        min_overlap=0.014,
        name="thumb_cap_remains_centered_over_bezel",
    )

    with ctx.pose({outer_joint: OUTER_LIMIT, inner_joint: -INNER_LIMIT}):
        ctx.expect_gap(
            inner_ring,
            housing,
            axis="z",
            positive_elem=thumb_cap,
            negative_elem=bezel,
            min_gap=0.0001,
            name="thumb_cap_clears_bezel_at_compound_limit",
        )

    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
