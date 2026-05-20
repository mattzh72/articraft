from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _box_mesh(
    size: tuple[float, float, float], center: tuple[float, float, float]
) -> BoxGeometry:
    return BoxGeometry(size).translate(*center)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _build_guide_mesh(*, open_inward: bool) -> object:
    width = 0.024
    depth = 0.028
    height = 0.094

    outer_plate_x = -width * 0.5 + 0.002 if open_inward else width * 0.5 - 0.002
    lip_center_x = -0.003 if open_inward else 0.003

    geom = _box_mesh((0.004, depth, height), (outer_plate_x, 0.0, height * 0.5))
    geom.merge(
        _box_mesh((0.018, 0.004, height), (lip_center_x, depth * 0.5 - 0.002, height * 0.5))
    )
    geom.merge(
        _box_mesh((0.018, 0.004, height), (lip_center_x, -depth * 0.5 + 0.002, height * 0.5))
    )
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="office_guillotine_cutter")

    bed_grey = model.material("bed_grey", rgba=(0.70, 0.72, 0.75, 1.0))
    fence_dark = model.material("fence_dark", rgba=(0.16, 0.18, 0.20, 1.0))
    painted_red = model.material("painted_red", rgba=(0.70, 0.12, 0.12, 1.0))
    steel = model.material("steel", rgba=(0.76, 0.78, 0.80, 1.0))
    black = model.material("black", rgba=(0.12, 0.12, 0.13, 1.0))

    bed_width = 0.46
    bed_depth = 0.32
    bed_thickness = 0.012
    bed_top = bed_thickness

    blade_pivot = (0.192, 0.126, 0.068)
    clamp_origin = (-0.105, 0.050, 0.120)
    guide_spacing = 0.150
    guide_left_x = clamp_origin[0] - guide_spacing * 0.5
    guide_right_x = clamp_origin[0] + guide_spacing * 0.5

    base = model.part("base")
    base.visual(
        Box((bed_width, bed_depth, bed_thickness)),
        origin=Origin(xyz=(0.0, 0.0, bed_thickness * 0.5)),
        material=bed_grey,
        name="bed",
    )
    base.visual(
        Box((0.014, 0.248, 0.022)),
        origin=Origin(xyz=(-0.223, 0.0, bed_top + 0.011)),
        material=fence_dark,
        name="side_fence",
    )
    base.visual(
        Box((0.110, 0.014, 0.022)),
        origin=Origin(xyz=(-0.175, 0.153, bed_top + 0.011)),
        material=fence_dark,
        name="rear_stop",
    )
    base.visual(
        Box((0.014, 0.016, 0.064)),
        origin=Origin(xyz=(0.218, 0.151, bed_top + 0.032)),
        material=fence_dark,
        name="pivot_rear_bracket",
    )
    base.visual(
        Box((0.044, 0.022, 0.028)),
        origin=Origin(xyz=(0.196, 0.114, bed_top + 0.014)),
        material=fence_dark,
        name="pivot_lower_gusset",
    )
    base.visual(
        Cylinder(radius=0.022, length=0.012),
        origin=Origin(xyz=(0.198, 0.126, 0.068), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=steel,
        name="pivot_cheek",
    )
    base.visual(
        Box((0.016, 0.010, 0.020)),
        origin=Origin(xyz=(0.208, 0.146, 0.060)),
        material=steel,
        name="pivot_cheek_mount",
    )

    left_guide_mesh = mesh_from_geometry(_build_guide_mesh(open_inward=True), "left_guide")
    right_guide_mesh = mesh_from_geometry(_build_guide_mesh(open_inward=False), "right_guide")
    base.visual(
        left_guide_mesh,
        origin=Origin(xyz=(guide_left_x, clamp_origin[1], bed_top)),
        material=steel,
        name="left_guide",
    )
    base.visual(
        right_guide_mesh,
        origin=Origin(xyz=(guide_right_x, clamp_origin[1], bed_top)),
        material=steel,
        name="right_guide",
    )
    base.visual(
        Box((0.182, 0.007, 0.012)),
        origin=Origin(xyz=(clamp_origin[0], clamp_origin[1] + 0.0105, 0.126)),
        material=steel,
        name="bridge_front",
    )
    base.visual(
        Box((0.182, 0.007, 0.012)),
        origin=Origin(xyz=(clamp_origin[0], clamp_origin[1] - 0.0105, 0.126)),
        material=steel,
        name="bridge_rear",
    )
    base.visual(
        Box((0.006, 0.007, 0.108)),
        origin=Origin(xyz=(guide_left_x - 0.011, clamp_origin[1] + 0.0105, 0.066)),
        material=steel,
        name="front_left_bridge_post",
    )
    base.visual(
        Box((0.006, 0.007, 0.108)),
        origin=Origin(xyz=(guide_right_x + 0.011, clamp_origin[1] + 0.0105, 0.066)),
        material=steel,
        name="front_right_bridge_post",
    )
    base.visual(
        Box((0.006, 0.007, 0.108)),
        origin=Origin(xyz=(guide_left_x - 0.011, clamp_origin[1] - 0.0105, 0.066)),
        material=steel,
        name="rear_left_bridge_post",
    )
    base.visual(
        Box((0.006, 0.007, 0.108)),
        origin=Origin(xyz=(guide_right_x + 0.011, clamp_origin[1] - 0.0105, 0.066)),
        material=steel,
        name="rear_right_bridge_post",
    )

    blade_arm = model.part("blade_arm")
    arm_yaw = math.radians(-145.0)
    blade_arm.visual(
        Cylinder(radius=0.022, length=0.010),
        origin=Origin(xyz=(-0.005, 0.0, 0.0), rpy=(0.0, math.pi * 0.5, 0.0)),
        material=steel,
        name="hinge_hub",
    )
    arm_dir_x = math.cos(arm_yaw)
    arm_dir_y = math.sin(arm_yaw)
    edge_offset_x = -math.sin(arm_yaw) * 0.012
    edge_offset_y = math.cos(arm_yaw) * 0.012
    connector_a = (-0.012, 0.0, -0.018)
    connector_b = (arm_dir_x * 0.118, arm_dir_y * 0.118, -0.022)
    blade_arm.visual(
        Cylinder(radius=0.006, length=_distance(connector_a, connector_b)),
        origin=Origin(
            xyz=_midpoint(connector_a, connector_b),
            rpy=_rpy_for_cylinder(connector_a, connector_b),
        ),
        material=painted_red,
        name="arm_connector",
    )
    blade_arm.visual(
        Box((0.274, 0.036, 0.024)),
        origin=Origin(
            xyz=(arm_dir_x * 0.255, arm_dir_y * 0.255, -0.030),
            rpy=(0.0, 0.0, arm_yaw),
        ),
        material=painted_red,
        name="arm_body",
    )
    blade_arm.visual(
        Box((0.300, 0.012, 0.012)),
        origin=Origin(
            xyz=(arm_dir_x * 0.240 + edge_offset_x, arm_dir_y * 0.240 + edge_offset_y, -0.042),
            rpy=(0.0, 0.0, arm_yaw),
        ),
        material=steel,
        name="blade_strip",
    )
    blade_arm.visual(
        Box((0.112, 0.046, 0.018)),
        origin=Origin(
            xyz=(arm_dir_x * 0.332, arm_dir_y * 0.332, -0.022),
            rpy=(0.0, 0.0, arm_yaw),
        ),
        material=black,
        name="handle_grip",
    )

    hold_clamp = model.part("hold_clamp")
    hold_clamp.visual(
        Box((0.142, 0.007, 0.012)),
        origin=Origin(xyz=(0.0, 0.0105, -0.006)),
        material=steel,
        name="top_front",
    )
    hold_clamp.visual(
        Box((0.142, 0.007, 0.012)),
        origin=Origin(xyz=(0.0, -0.0105, -0.006)),
        material=steel,
        name="top_rear",
    )
    hold_clamp.visual(
        Box((0.026, 0.018, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, -0.007)),
        material=steel,
        name="nut_block",
    )
    hold_clamp.visual(
        Box((0.010, 0.016, 0.068)),
        origin=Origin(xyz=(-0.075, 0.0, -0.046)),
        material=steel,
        name="left_runner",
    )
    hold_clamp.visual(
        Box((0.010, 0.016, 0.068)),
        origin=Origin(xyz=(0.075, 0.0, -0.046)),
        material=steel,
        name="right_runner",
    )
    hold_clamp.visual(
        Box((0.130, 0.016, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, -0.076)),
        material=steel,
        name="pressure_bar",
    )
    hold_clamp.visual(
        Box((0.010, 0.016, 0.016)),
        origin=Origin(xyz=(-0.0675, 0.0, -0.076)),
        material=steel,
        name="left_jaw_link",
    )
    hold_clamp.visual(
        Box((0.010, 0.016, 0.016)),
        origin=Origin(xyz=(0.0675, 0.0, -0.076)),
        material=steel,
        name="right_jaw_link",
    )

    hand_knob = model.part("hand_knob")
    hand_knob.visual(
        Cylinder(radius=0.014, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=black,
        name="knob_collar",
    )
    hand_knob.visual(
        Cylinder(radius=0.019, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=black,
        name="knob_body",
    )
    hand_knob.visual(
        Box((0.032, 0.010, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=black,
        name="knob_wing_x",
    )
    hand_knob.visual(
        Box((0.010, 0.032, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=black,
        name="knob_wing_y",
    )
    hand_knob.visual(
        Cylinder(radius=0.004, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.000)),
        material=steel,
        name="screw_shaft",
    )

    model.articulation(
        "blade_arm_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=blade_arm,
        origin=Origin(xyz=blade_pivot),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )
    model.articulation(
        "clamp_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=hold_clamp,
        origin=Origin(xyz=clamp_origin),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.06,
            lower=-0.024,
            upper=0.0,
        ),
    )
    model.articulation(
        "knob_spin",
        ArticulationType.CONTINUOUS,
        parent=hold_clamp,
        child=hand_knob,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    blade_arm = object_model.get_part("blade_arm")
    hold_clamp = object_model.get_part("hold_clamp")
    hand_knob = object_model.get_part("hand_knob")

    blade_arm_hinge = object_model.get_articulation("blade_arm_hinge")
    clamp_slide = object_model.get_articulation("clamp_slide")
    knob_spin = object_model.get_articulation("knob_spin")

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
    ctx.allow_overlap(
        hand_knob,
        hold_clamp,
        elem_a="screw_shaft",
        elem_b="nut_block",
        reason="The hand knob's threaded screw shaft intentionally passes through the clamp's captive nut block.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "all_expected_parts_present",
        len(object_model.parts) == 4,
        f"Expected 4 parts, found {len(object_model.parts)}.",
    )
    ctx.check(
        "articulation_axes_match_mechanisms",
        tuple(blade_arm_hinge.axis) == (-1.0, 0.0, 0.0)
        and tuple(clamp_slide.axis) == (0.0, 0.0, 1.0)
        and tuple(knob_spin.axis) == (0.0, 0.0, 1.0),
        (
            f"blade_arm_hinge.axis={blade_arm_hinge.axis}, "
            f"clamp_slide.axis={clamp_slide.axis}, knob_spin.axis={knob_spin.axis}"
        ),
    )

    ctx.expect_contact(blade_arm, base, elem_a="hinge_hub", elem_b="pivot_cheek")
    ctx.expect_contact(hold_clamp, base, elem_a="top_front", elem_b="bridge_front")
    ctx.expect_contact(hold_clamp, base, elem_a="top_rear", elem_b="bridge_rear")
    ctx.expect_contact(hand_knob, hold_clamp, elem_a="screw_shaft", elem_b="nut_block")
    ctx.expect_overlap(blade_arm, base, axes="xy", elem_a="blade_strip", elem_b="bed", min_overlap=0.18)

    with ctx.pose({blade_arm_hinge: math.radians(78.0)}):
        ctx.expect_gap(
            blade_arm,
            base,
            axis="z",
            positive_elem="blade_strip",
            negative_elem="bed",
            min_gap=0.095,
            name="blade_arm_opens_clear_of_bed",
        )

    with ctx.pose({clamp_slide: -0.024}):
        ctx.expect_gap(
            hold_clamp,
            base,
            axis="z",
            positive_elem="pressure_bar",
            negative_elem="bed",
            min_gap=-0.0005,
            max_gap=0.0025,
            name="clamp_bar_reaches_paper_stack_plane",
        )
        ctx.expect_within(
            hold_clamp,
            base,
            axes="xy",
            inner_elem="left_runner",
            outer_elem="left_guide",
            name="left_runner_stays_captured",
        )
        ctx.expect_within(
            hold_clamp,
            base,
            axes="xy",
            inner_elem="right_runner",
            outer_elem="right_guide",
            name="right_runner_stays_captured",
        )
        ctx.expect_contact(
            hand_knob,
            hold_clamp,
            elem_a="screw_shaft",
            elem_b="nut_block",
            name="knob_remains_seated_while_clamp_moves",
        )

    with ctx.pose({knob_spin: 1.2}):
        ctx.expect_contact(
            hand_knob,
            hold_clamp,
            elem_a="screw_shaft",
            elem_b="nut_block",
            name="knob_rotates_without_detaching",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
