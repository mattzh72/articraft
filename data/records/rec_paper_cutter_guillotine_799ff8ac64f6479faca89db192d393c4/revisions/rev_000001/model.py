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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _midpoint(a, b):
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a, b):
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(a, b):
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="photo_trimming_guillotine")

    charcoal = model.material("charcoal", rgba=(0.18, 0.19, 0.20, 1.0))
    graphite = model.material("graphite", rgba=(0.28, 0.30, 0.32, 1.0))
    aluminum = model.material("aluminum", rgba=(0.79, 0.81, 0.83, 1.0))
    steel = model.material("steel", rgba=(0.62, 0.64, 0.67, 1.0))
    cutting_mat = model.material("cutting_mat", rgba=(0.58, 0.66, 0.70, 1.0))
    handle_red = model.material("handle_red", rgba=(0.72, 0.14, 0.12, 1.0))
    stop_orange = model.material("stop_orange", rgba=(0.91, 0.47, 0.12, 1.0))
    rubber = model.material("rubber", rgba=(0.10, 0.10, 0.11, 1.0))

    bed_width = 0.34
    bed_depth = 0.24
    bed_thickness = 0.012
    arm_angle = math.radians(-33.0)
    arm_dir = (math.cos(arm_angle), math.sin(arm_angle), 0.0)
    pivot_axis = (arm_dir[1], -arm_dir[0], 0.0)

    base = model.part("base")
    base.visual(
        Box((bed_width, bed_depth, bed_thickness)),
        origin=Origin(xyz=(0.0, 0.0, bed_thickness * 0.5)),
        material=charcoal,
        name="bed_plate",
    )
    base.visual(
        Box((0.302, 0.200, 0.001)),
        origin=Origin(xyz=(-0.006, -0.006, bed_thickness + 0.0005)),
        material=cutting_mat,
        name="bed_surface",
    )
    base.visual(
        Box((0.146, 0.012, 0.016)),
        origin=Origin(xyz=(0.073, 0.114, 0.020)),
        material=graphite,
        name="rear_fence",
    )
    base.visual(
        Box((0.012, 0.012, 0.020)),
        origin=Origin(xyz=(0.164, 0.082, 0.022)),
        material=graphite,
        name="rail_support_rear",
    )
    base.visual(
        Box((0.012, 0.012, 0.020)),
        origin=Origin(xyz=(0.164, -0.082, 0.022)),
        material=graphite,
        name="rail_support_front",
    )
    base.visual(
        Box((0.316, 0.008, 0.003)),
        origin=Origin(xyz=(-0.004, -0.003, bed_thickness + 0.0015), rpy=(0.0, 0.0, arm_angle)),
        material=steel,
        name="cut_strip",
    )
    base.visual(
        Box((0.046, 0.040, 0.006)),
        origin=Origin(xyz=(-0.145, 0.100, bed_thickness + 0.003)),
        material=graphite,
        name="pivot_plinth",
    )
    for x_sign, y_sign in ((-1.0, -1.0), (-1.0, 1.0), (1.0, -1.0), (1.0, 1.0)):
        base.visual(
            Cylinder(radius=0.008, length=0.004),
            origin=Origin(
                xyz=(0.140 * x_sign, 0.095 * y_sign, 0.002),
            ),
            material=rubber,
        )
    base.inertial = Inertial.from_geometry(
        Box((bed_width, bed_depth, 0.035)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
    )

    guide_rail = model.part("guide_rail")
    guide_rail.visual(
        Box((0.012, 0.176, 0.012)),
        material=aluminum,
        name="rail_body",
    )
    guide_rail.visual(
        Box((0.012, 0.010, 0.016)),
        origin=Origin(xyz=(0.0, 0.083, 0.002)),
        material=steel,
        name="rear_rail_cap",
    )
    guide_rail.visual(
        Box((0.012, 0.010, 0.016)),
        origin=Origin(xyz=(0.0, -0.083, 0.002)),
        material=steel,
        name="front_rail_cap",
    )
    guide_rail.inertial = Inertial.from_geometry(
        Box((0.012, 0.176, 0.016)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
    )

    paper_stop = model.part("paper_stop")
    paper_stop.visual(
        Box((0.030, 0.024, 0.005)),
        origin=Origin(xyz=(0.0, 0.0, 0.0085)),
        material=stop_orange,
        name="stop_top",
    )
    paper_stop.visual(
        Box((0.030, 0.024, 0.005)),
        origin=Origin(xyz=(0.0, 0.0, -0.0085)),
        material=stop_orange,
        name="stop_bottom",
    )
    paper_stop.visual(
        Box((0.005, 0.024, 0.022)),
        origin=Origin(xyz=(-0.0085, 0.0, 0.0)),
        material=stop_orange,
        name="stop_left_wall",
    )
    paper_stop.visual(
        Box((0.005, 0.024, 0.022)),
        origin=Origin(xyz=(0.0085, 0.0, 0.0)),
        material=stop_orange,
        name="stop_right_wall",
    )
    paper_stop.visual(
        Box((0.036, 0.004, 0.042)),
        origin=Origin(xyz=(-0.029, 0.0, 0.010)),
        material=stop_orange,
        name="stop_fence",
    )
    paper_stop.visual(
        Cylinder(radius=0.005, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=rubber,
        name="stop_knob",
    )
    paper_stop.visual(
        Cylinder(radius=0.0028, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=steel,
        name="stop_knob_stem",
    )
    paper_stop.inertial = Inertial.from_geometry(
        Box((0.047, 0.024, 0.042)),
        mass=0.12,
        origin=Origin(xyz=(-0.012, 0.0, 0.010)),
    )

    blade_arm = model.part("blade_arm")
    shoe_center = (0.022 * arm_dir[0], 0.022 * arm_dir[1], 0.004)
    blade_arm.visual(
        Box((0.050, 0.034, 0.008)),
        origin=Origin(xyz=shoe_center, rpy=(0.0, 0.0, arm_angle)),
        material=graphite,
        name="hinge_shoe",
    )
    blade_arm.visual(
        Box((0.315, 0.024, 0.008)),
        origin=Origin(
            xyz=(0.160 * arm_dir[0], 0.160 * arm_dir[1], 0.0055),
            rpy=(0.0, 0.0, arm_angle),
        ),
        material=graphite,
        name="arm_beam",
    )
    blade_arm.visual(
        Box((0.268, 0.004, 0.002)),
        origin=Origin(
            xyz=(0.160 * arm_dir[0], 0.160 * arm_dir[1], 0.001),
            rpy=(0.0, 0.0, arm_angle),
        ),
        material=steel,
        name="blade_insert",
    )
    grip_a = (0.232 * arm_dir[0], 0.232 * arm_dir[1], 0.014)
    grip_b = (0.304 * arm_dir[0], 0.304 * arm_dir[1], 0.014)
    blade_arm.visual(
        Cylinder(radius=0.0105, length=_distance(grip_a, grip_b)),
        origin=Origin(xyz=_midpoint(grip_a, grip_b), rpy=_rpy_for_cylinder(grip_a, grip_b)),
        material=handle_red,
        name="handle_grip",
    )
    blade_arm.inertial = Inertial.from_geometry(
        Box((0.320, 0.050, 0.030)),
        mass=0.55,
        origin=Origin(xyz=(0.160 * arm_dir[0], 0.160 * arm_dir[1], 0.010)),
    )

    model.articulation(
        "base_to_guide_rail",
        ArticulationType.FIXED,
        parent=base,
        child=guide_rail,
        origin=Origin(xyz=(0.176, -0.002, 0.038)),
    )
    model.articulation(
        "guide_rail_to_paper_stop",
        ArticulationType.PRISMATIC,
        parent=guide_rail,
        child=paper_stop,
        origin=Origin(xyz=(0.0, 0.058, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=0.12, lower=0.0, upper=0.116),
    )
    model.articulation(
        "base_to_blade_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=blade_arm,
        origin=Origin(xyz=(-0.145, 0.100, 0.018)),
        axis=pivot_axis,
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.6,
            lower=0.0,
            upper=math.radians(72.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    base = object_model.get_part("base")
    guide_rail = object_model.get_part("guide_rail")
    paper_stop = object_model.get_part("paper_stop")
    blade_arm = object_model.get_part("blade_arm")

    blade_pivot = object_model.get_articulation("base_to_blade_arm")
    stop_slide = object_model.get_articulation("guide_rail_to_paper_stop")

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "blade pivot axis is planar",
        abs(blade_pivot.axis[2]) < 1e-9 and math.hypot(blade_pivot.axis[0], blade_pivot.axis[1]) > 0.9,
        details=f"unexpected blade pivot axis {blade_pivot.axis}",
    )
    ctx.check(
        "paper stop slides along side rail",
        abs(stop_slide.axis[0]) < 1e-9
        and abs(stop_slide.axis[2]) < 1e-9
        and stop_slide.axis[1] < -0.9,
        details=f"unexpected stop slide axis {stop_slide.axis}",
    )

    ctx.expect_contact(guide_rail, base, elem_a="rear_rail_cap", elem_b="rail_support_rear")
    ctx.expect_contact(guide_rail, base, elem_a="front_rail_cap", elem_b="rail_support_front")
    ctx.expect_contact(blade_arm, base, elem_a="hinge_shoe", elem_b="pivot_plinth")
    ctx.expect_contact(paper_stop, guide_rail)
    ctx.expect_within(guide_rail, paper_stop, axes="xz", margin=0.0)

    with ctx.pose({blade_pivot: 0.0}):
        ctx.expect_overlap(blade_arm, base, axes="xy", elem_a="arm_beam", elem_b="bed_plate", min_overlap=0.08)

    beam_closed = ctx.part_element_world_aabb(blade_arm, elem="arm_beam")
    assert beam_closed is not None
    with ctx.pose({blade_pivot: math.radians(62.0)}):
        beam_open = ctx.part_element_world_aabb(blade_arm, elem="arm_beam")
        assert beam_open is not None
        assert beam_open[1][2] > beam_closed[1][2] + 0.20
        ctx.fail_if_parts_overlap_in_current_pose(name="open blade pose has no overlaps")
        ctx.expect_gap(
            blade_arm,
            base,
            axis="z",
            min_gap=0.008,
            positive_elem="arm_beam",
            negative_elem="bed_plate",
        )
        ctx.expect_contact(blade_arm, base, elem_a="hinge_shoe", elem_b="pivot_plinth")

    stop_rest = ctx.part_world_position(paper_stop)
    assert stop_rest is not None
    with ctx.pose({stop_slide: 0.136}):
        stop_far = ctx.part_world_position(paper_stop)
        assert stop_far is not None
        assert stop_far[1] < stop_rest[1] - 0.12
        assert abs(stop_far[0] - stop_rest[0]) < 1e-6
        assert abs(stop_far[2] - stop_rest[2]) < 1e-6
        ctx.expect_contact(paper_stop, guide_rail)
        ctx.expect_within(guide_rail, paper_stop, axes="xz", margin=0.0)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
