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
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_mount_fold_flat_display_easel")

    charcoal = model.material("charcoal", rgba=(0.19, 0.20, 0.21, 1.0))
    graphite = model.material("graphite", rgba=(0.30, 0.31, 0.33, 1.0))
    black_oxide = model.material("black_oxide", rgba=(0.12, 0.12, 0.13, 1.0))
    rubber = model.material("rubber", rgba=(0.09, 0.09, 0.10, 1.0))

    plate_w = 0.26
    plate_h = 0.36
    plate_t = 0.018
    arm_w = 0.24
    arm_h = 0.28
    arm_back_t = 0.012
    hinge_x = 0.028
    hinge_z = plate_h / 2.0 - 0.004
    brace_hinge_x = 0.022
    brace_hinge_z = -0.090

    wall_plate = model.part("wall_plate")
    wall_plate.visual(
        Box((plate_t, plate_w, plate_h)),
        origin=Origin(xyz=(plate_t / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="wall_board",
    )
    wall_plate.visual(
        Box((0.024, 0.238, 0.024)),
        origin=Origin(xyz=(0.012, 0.0, hinge_z - 0.008)),
        material=charcoal,
        name="top_leaf",
    )
    wall_plate.visual(
        Box((0.014, 0.110, 0.028)),
        origin=Origin(xyz=(0.007, 0.0, brace_hinge_z - 0.002)),
        material=charcoal,
        name="brace_hinge_block",
    )
    wall_plate.visual(
        Box((0.020, 0.180, 0.030)),
        origin=Origin(xyz=(0.010, 0.0, -plate_h / 2.0 + 0.025)),
        material=graphite,
        name="bottom_stiffener",
    )
    for index, z_pos in enumerate((0.120, 0.0, -0.120)):
        wall_plate.visual(
            Cylinder(radius=0.0055, length=0.010),
            origin=Origin(
                xyz=(plate_t * 0.56, 0.0, z_pos),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=black_oxide,
            name=f"mount_screw_{index}",
        )

    hinge_radius = 0.007
    plate_knuckles = (
        ("plate_hinge_knuckle_left", -0.088, 0.056),
        ("plate_hinge_knuckle_center", 0.0, 0.028),
        ("plate_hinge_knuckle_right", 0.088, 0.056),
    )
    for name, y_pos, length in plate_knuckles:
        wall_plate.visual(
            Cylinder(radius=hinge_radius, length=length),
            origin=Origin(
                xyz=(hinge_x, y_pos, hinge_z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=black_oxide,
            name=name,
        )

    brace_knuckle_radius = 0.006
    wall_plate.visual(
        Cylinder(radius=brace_knuckle_radius, length=0.028),
        origin=Origin(
            xyz=(brace_hinge_x, 0.0, brace_hinge_z),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=black_oxide,
        name="plate_brace_knuckle",
    )
    wall_plate.inertial = Inertial.from_geometry(
        Box((0.05, plate_w, plate_h)),
        mass=1.8,
        origin=Origin(xyz=(0.025, 0.0, 0.0)),
    )

    panel_arm = model.part("panel_arm")
    panel_arm.visual(
        Box((0.018, 0.216, 0.022)),
        origin=Origin(xyz=(0.017, 0.0, -0.011)),
        material=graphite,
        name="top_rail",
    )
    panel_arm.visual(
        Box((0.012, 0.040, 0.018)),
        origin=Origin(xyz=(0.013, -0.037, -0.009)),
        material=graphite,
        name="left_hinge_strap",
    )
    panel_arm.visual(
        Box((0.012, 0.040, 0.018)),
        origin=Origin(xyz=(0.013, 0.037, -0.009)),
        material=graphite,
        name="right_hinge_strap",
    )
    panel_arm.visual(
        Box((arm_back_t, 0.232, 0.250)),
        origin=Origin(xyz=(0.012, 0.0, -0.147)),
        material=graphite,
        name="panel_back",
    )
    panel_arm.visual(
        Box((0.030, 0.010, 0.250)),
        origin=Origin(xyz=(0.021, -0.111, -0.147)),
        material=charcoal,
        name="left_side_rail",
    )
    panel_arm.visual(
        Box((0.030, 0.010, 0.250)),
        origin=Origin(xyz=(0.021, 0.111, -0.147)),
        material=charcoal,
        name="right_side_rail",
    )
    panel_arm.visual(
        Box((0.036, 0.220, 0.016)),
        origin=Origin(xyz=(0.024, 0.0, -arm_h + 0.008)),
        material=charcoal,
        name="bottom_lip",
    )
    panel_arm.visual(
        Box((0.006, 0.210, 0.024)),
        origin=Origin(xyz=(0.038, 0.0, -arm_h + 0.012)),
        material=charcoal,
        name="retaining_fence",
    )
    panel_arm.visual(
        Box((0.020, 0.080, 0.016)),
        origin=Origin(xyz=(0.016, 0.0, -0.148)),
        material=charcoal,
        name="brace_stop",
    )
    panel_arm.visual(
        Box((0.018, 0.050, 0.180)),
        origin=Origin(xyz=(0.016, 0.0, -0.142)),
        material=graphite,
        name="center_rib",
    )

    panel_knuckles = (
        ("panel_hinge_knuckle_left", -0.037, 0.038),
        ("panel_hinge_knuckle_right", 0.037, 0.038),
    )
    for name, y_pos, length in panel_knuckles:
        panel_arm.visual(
            Cylinder(radius=hinge_radius, length=length),
            origin=Origin(
                xyz=(0.0, y_pos, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=black_oxide,
            name=name,
        )
    panel_arm.inertial = Inertial.from_geometry(
        Box((0.06, arm_w, arm_h)),
        mass=1.1,
        origin=Origin(xyz=(0.030, 0.0, -arm_h / 2.0)),
    )

    leg_brace = model.part("leg_brace")
    leg_brace.visual(
        Box((0.008, 0.042, 0.012)),
        origin=Origin(xyz=(0.005, 0.0, 0.006)),
        material=black_oxide,
        name="brace_yoke",
    )
    leg_brace.visual(
        Box((0.006, 0.024, 0.212)),
        origin=Origin(xyz=(0.003, 0.0, 0.110)),
        material=black_oxide,
        name="brace_bar",
    )
    leg_brace.visual(
        Box((0.010, 0.038, 0.022)),
        origin=Origin(xyz=(0.005, 0.0, 0.219)),
        material=black_oxide,
        name="tip_carrier",
    )
    leg_brace.visual(
        Sphere(radius=0.006),
        origin=Origin(xyz=(0.010, 0.0, 0.224)),
        material=rubber,
        name="tip_roller",
    )
    leg_brace.visual(
        Cylinder(radius=brace_knuckle_radius, length=0.028),
        origin=Origin(
            xyz=(0.0, -0.024, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=black_oxide,
        name="brace_knuckle_left",
    )
    leg_brace.visual(
        Cylinder(radius=brace_knuckle_radius, length=0.028),
        origin=Origin(
            xyz=(0.0, 0.024, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=black_oxide,
        name="brace_knuckle_right",
    )
    leg_brace.inertial = Inertial.from_geometry(
        Box((0.03, 0.05, 0.24)),
        mass=0.35,
        origin=Origin(xyz=(0.015, 0.0, 0.120)),
    )

    model.articulation(
        "wall_plate_to_panel_arm",
        ArticulationType.REVOLUTE,
        parent=wall_plate,
        child=panel_arm,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.8,
            lower=0.0,
            upper=1.15,
        ),
    )
    model.articulation(
        "wall_plate_to_leg_brace",
        ArticulationType.REVOLUTE,
        parent=wall_plate,
        child=leg_brace,
        origin=Origin(xyz=(brace_hinge_x, 0.0, brace_hinge_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.8,
            lower=0.0,
            upper=1.20,
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

    wall_plate = object_model.get_part("wall_plate")
    panel_arm = object_model.get_part("panel_arm")
    leg_brace = object_model.get_part("leg_brace")
    panel_hinge = object_model.get_articulation("wall_plate_to_panel_arm")
    brace_hinge = object_model.get_articulation("wall_plate_to_leg_brace")

    display_panel_q = 0.75
    display_brace_q = 0.53

    ctx.check(
        "panel hinge axis opens outward from the wall",
        tuple(panel_hinge.axis) == (0.0, -1.0, 0.0),
        details=f"axis={panel_hinge.axis}",
    )
    ctx.check(
        "brace hinge axis kicks the brace outward",
        tuple(brace_hinge.axis) == (0.0, 1.0, 0.0),
        details=f"axis={brace_hinge.axis}",
    )

    ctx.expect_gap(
        panel_arm,
        wall_plate,
        axis="x",
        positive_elem="panel_back",
        negative_elem="wall_board",
        min_gap=0.010,
        max_gap=0.018,
        name="folded panel sits just proud of the wall plate",
    )
    ctx.expect_overlap(
        panel_arm,
        wall_plate,
        axes="yz",
        elem_a="panel_back",
        elem_b="wall_board",
        min_overlap=0.220,
        name="folded panel covers the wall plate footprint",
    )
    ctx.expect_gap(
        leg_brace,
        wall_plate,
        axis="x",
        positive_elem="brace_bar",
        negative_elem="wall_board",
        min_gap=0.002,
        max_gap=0.012,
        name="folded brace nests in front of the wall plate",
    )

    closed_panel_aabb = ctx.part_world_aabb(panel_arm)
    closed_brace_aabb = ctx.part_world_aabb(leg_brace)
    open_panel_aabb = None
    open_brace_aabb = None

    with ctx.pose({panel_hinge: display_panel_q, brace_hinge: display_brace_q}):
        ctx.expect_contact(
            leg_brace,
            panel_arm,
            elem_a="tip_roller",
            elem_b="brace_stop",
            contact_tol=0.004,
            name="brace roller props the display arm at its open angle",
        )
        ctx.expect_overlap(
            leg_brace,
            panel_arm,
            axes="y",
            elem_a="tip_roller",
            elem_b="brace_stop",
            min_overlap=0.012,
            name="brace roller stays centered under the brace stop",
        )
        ctx.expect_gap(
            panel_arm,
            wall_plate,
            axis="x",
            positive_elem="retaining_fence",
            negative_elem="wall_board",
            min_gap=0.140,
            name="open display arm projects well away from the wall",
        )
        open_panel_aabb = ctx.part_world_aabb(panel_arm)
        open_brace_aabb = ctx.part_world_aabb(leg_brace)

    ctx.check(
        "panel arm swings outward from folded to display pose",
        closed_panel_aabb is not None
        and open_panel_aabb is not None
        and open_panel_aabb[1][0] > closed_panel_aabb[1][0] + 0.110,
        details=f"closed={closed_panel_aabb}, open={open_panel_aabb}",
    )
    ctx.check(
        "leg brace swings outward beneath the arm",
        closed_brace_aabb is not None
        and open_brace_aabb is not None
        and open_brace_aabb[1][0] > closed_brace_aabb[1][0] + 0.080,
        details=f"closed={closed_brace_aabb}, open={open_brace_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
