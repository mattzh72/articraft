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
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_LENGTH = 0.070
BODY_RADIUS = 0.013
BORE_RADIUS = 0.0095
REAR_COLLAR_LENGTH = 0.010
REAR_COLLAR_RADIUS = 0.0155
MOUNT_PAD_LENGTH = 0.042
MOUNT_PAD_WIDTH = 0.026
MOUNT_PAD_THICKNESS = 0.006

CLEVIS_START_X = 0.062
CLEVIS_END_X = 0.088
FORK_OUTER_WIDTH = 0.018
FORK_SLOT_WIDTH = 0.008
FORK_BLOCK_HEIGHT = 0.032
FORK_SLOT_HEIGHT = 0.050
PIVOT_X = 0.079
PIVOT_Z = 0.008
PIVOT_RADIUS = 0.0025

GUIDE_RAIL_WIDTH = 0.003
GUIDE_FLOOR_THICKNESS = 0.004
GUIDE_HEIGHT = 0.016
GUIDE_CHANNEL_WIDTH = 0.020
GUIDE_RUN_LENGTH = 0.060

PLUNGER_ROD_RADIUS = 0.0045
PLUNGER_NOSE_RADIUS = 0.0032
PLUNGER_BUTTON_RADIUS = 0.0105
PLUNGER_BUTTON_LENGTH = 0.013
PLUNGER_FRONT_X = 0.071
PLUNGER_STROKE = 0.018
PLUNGER_SHOE_LENGTH = 0.024
PLUNGER_SHOE_WIDTH = 0.009
PLUNGER_SHOE_THICKNESS = 0.003

LEVER_THICKNESS = 0.005
LEVER_HUB_RADIUS = 0.006
LEVER_OPEN_ANGLE = 0.60


def _y_axis_cylinder(radius: float, length: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -90.0)
        .translate((0.0, -length / 2.0, 0.0))
    )


def _x_axis_cylinder(radius: float, length: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), 90.0)
    )


def _build_sleeve_shape() -> cq.Workplane:
    mount_pad = cq.Workplane("XY").box(
        MOUNT_PAD_LENGTH, MOUNT_PAD_WIDTH, MOUNT_PAD_THICKNESS
    ).translate((0.030, 0.0, -0.015))

    guide_bed = cq.Workplane("XY").box(
        CLEVIS_START_X,
        0.016,
        GUIDE_FLOOR_THICKNESS,
    ).translate((CLEVIS_START_X / 2.0, 0.0, -0.010))
    left_fence = cq.Workplane("XY").box(
        0.052,
        0.003,
        0.007,
    ).translate((0.036, 0.0065, -0.0045))
    right_fence = cq.Workplane("XY").box(
        0.052,
        0.003,
        0.007,
    ).translate((0.036, -0.0065, -0.0045))
    rear_stop = cq.Workplane("XY").box(0.004, 0.016, 0.010).translate((0.002, 0.0, -0.005))

    cheek_thickness = (FORK_OUTER_WIDTH - FORK_SLOT_WIDTH) / 2.0
    cheek_left = cq.Workplane("XY").box(
        CLEVIS_END_X - CLEVIS_START_X,
        cheek_thickness,
        0.024,
    ).translate(
        (
            (CLEVIS_START_X + CLEVIS_END_X) / 2.0,
            (FORK_SLOT_WIDTH / 2.0) + (cheek_thickness / 2.0),
            0.002,
        )
    )
    cheek_right = cq.Workplane("XY").box(
        CLEVIS_END_X - CLEVIS_START_X,
        cheek_thickness,
        0.024,
    ).translate(
        (
            (CLEVIS_START_X + CLEVIS_END_X) / 2.0,
            -((FORK_SLOT_WIDTH / 2.0) + (cheek_thickness / 2.0)),
            0.002,
        )
    )
    fork_roof = cq.Workplane("XY").box(
        CLEVIS_END_X - CLEVIS_START_X,
        FORK_OUTER_WIDTH,
        0.004,
    ).translate(((CLEVIS_START_X + CLEVIS_END_X) / 2.0, 0.0, 0.014))
    pivot_pin = _y_axis_cylinder(PIVOT_RADIUS, FORK_SLOT_WIDTH).translate(
        (PIVOT_X, 0.0, PIVOT_Z)
    )

    return (
        mount_pad.union(guide_bed)
        .union(left_fence)
        .union(right_fence)
        .union(rear_stop)
        .union(cheek_left)
        .union(cheek_right)
        .union(fork_roof)
        .union(pivot_pin)
    )


def _build_plunger_shape() -> cq.Workplane:
    main_rod = _x_axis_cylinder(PLUNGER_ROD_RADIUS, 0.052)
    nose = _x_axis_cylinder(PLUNGER_NOSE_RADIUS, PLUNGER_FRONT_X - 0.052).translate(
        (0.052, 0.0, 0.0)
    )
    button = _x_axis_cylinder(PLUNGER_BUTTON_RADIUS, PLUNGER_BUTTON_LENGTH).translate(
        (-PLUNGER_BUTTON_LENGTH, 0.0, 0.0)
    )
    shoe = cq.Workplane("XY").box(
        PLUNGER_SHOE_LENGTH,
        PLUNGER_SHOE_WIDTH,
        PLUNGER_SHOE_THICKNESS,
    ).translate((0.030, 0.0, -0.0065))
    web = cq.Workplane("XY").box(0.028, 0.004, 0.006).translate((0.030, 0.0, -0.003))
    return main_rod.union(nose).union(button).union(shoe).union(web)


def _build_output_lever_shape() -> cq.Workplane:
    hub = _y_axis_cylinder(LEVER_HUB_RADIUS, LEVER_THICKNESS)
    lower_arm = cq.Workplane("XY").box(0.010, LEVER_THICKNESS, 0.026).translate(
        (-0.003, 0.0, -0.018)
    )
    tip_lobe = _y_axis_cylinder(0.005, LEVER_THICKNESS).translate((0.0, 0.0, -0.031))
    heel_pad = cq.Workplane("XY").box(0.006, LEVER_THICKNESS, 0.008).translate(
        (-0.005, 0.0, -0.004)
    )
    nose_shoulder = cq.Workplane("XY").box(0.012, LEVER_THICKNESS, 0.006).translate(
        (-0.005, 0.0, 0.008)
    )
    pivot_hole = _y_axis_cylinder(PIVOT_RADIUS, LEVER_THICKNESS + 0.002)

    return hub.union(lower_arm).union(tip_lobe).union(heel_pad).union(nose_shoulder).cut(
        pivot_hole
    )


def _build_rear_boss_shape() -> cq.Workplane:
    outer = _x_axis_cylinder(0.0065, 0.010)
    bore = _x_axis_cylinder(PLUNGER_ROD_RADIUS + 0.0008, 0.014).translate((-0.002, 0.0, 0.0))
    return outer.cut(bore)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_latch_plunger")

    model.material("housing_gray", rgba=(0.42, 0.44, 0.47, 1.0))
    model.material("steel", rgba=(0.77, 0.79, 0.82, 1.0))
    model.material("black_oxide", rgba=(0.16, 0.17, 0.18, 1.0))

    sleeve = model.part("sleeve")
    sleeve.visual(
        Box((MOUNT_PAD_LENGTH, MOUNT_PAD_WIDTH, MOUNT_PAD_THICKNESS)),
        origin=Origin(xyz=(0.030, 0.0, -0.015)),
        material="housing_gray",
        name="mount_pad",
    )
    sleeve.visual(
        Box((CLEVIS_START_X, 0.016, GUIDE_FLOOR_THICKNESS)),
        origin=Origin(xyz=(CLEVIS_START_X / 2.0, 0.0, -0.010)),
        material="housing_gray",
        name="guide_bed",
    )
    sleeve.visual(
        Box((0.054, GUIDE_RAIL_WIDTH, 0.008)),
        origin=Origin(xyz=(0.033, 0.0065, -0.004)),
        material="housing_gray",
        name="left_fence",
    )
    sleeve.visual(
        Box((0.054, GUIDE_RAIL_WIDTH, 0.008)),
        origin=Origin(xyz=(0.033, -0.0065, -0.004)),
        material="housing_gray",
        name="right_fence",
    )
    sleeve.visual(
        Box((0.004, 0.016, 0.010)),
        origin=Origin(xyz=(0.002, 0.0, -0.005)),
        material="housing_gray",
        name="rear_stop",
    )
    sleeve.visual(
        Box((CLEVIS_END_X - CLEVIS_START_X, 0.005, 0.024)),
        origin=Origin(xyz=((CLEVIS_START_X + CLEVIS_END_X) / 2.0, 0.0065, 0.002)),
        material="housing_gray",
        name="fork_left",
    )
    sleeve.visual(
        Box((CLEVIS_END_X - CLEVIS_START_X, 0.005, 0.024)),
        origin=Origin(xyz=((CLEVIS_START_X + CLEVIS_END_X) / 2.0, -0.0065, 0.002)),
        material="housing_gray",
        name="fork_right",
    )
    sleeve.visual(
        Box((CLEVIS_END_X - CLEVIS_START_X, FORK_OUTER_WIDTH, 0.004)),
        origin=Origin(xyz=((CLEVIS_START_X + CLEVIS_END_X) / 2.0, 0.0, 0.016)),
        material="housing_gray",
        name="fork_roof",
    )
    sleeve.visual(
        Cylinder(radius=PIVOT_RADIUS, length=FORK_SLOT_WIDTH),
        origin=Origin(xyz=(PIVOT_X, 0.0, PIVOT_Z), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="steel",
        name="pivot_pin",
    )
    sleeve.visual(
        mesh_from_cadquery(_build_rear_boss_shape(), "rear_boss"),
        origin=Origin(xyz=(0.005, 0.0, 0.0)),
        material="housing_gray",
        name="rear_boss",
    )
    sleeve.visual(
        Box((0.012, 0.014, 0.012)),
        origin=Origin(xyz=(0.010, 0.0, -0.006)),
        material="housing_gray",
        name="rear_bridge",
    )
    sleeve.visual(
        Box((0.012, 0.014, 0.004)),
        origin=Origin(xyz=(0.010, 0.0, 0.008)),
        material="housing_gray",
        name="rear_cap",
    )
    sleeve.visual(
        Box((0.001, 0.001, 0.001)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="housing_gray",
        name="sleeve_shell",
    )
    sleeve.inertial = Inertial.from_geometry(
        Box((CLEVIS_END_X, MOUNT_PAD_WIDTH, 0.052)),
        mass=0.45,
        origin=Origin(xyz=(CLEVIS_END_X / 2.0, 0.0, 0.0)),
    )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=PLUNGER_BUTTON_RADIUS, length=PLUNGER_BUTTON_LENGTH),
        origin=Origin(
            xyz=(-PLUNGER_BUTTON_LENGTH / 2.0, 0.0, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material="steel",
        name="button",
    )
    plunger.visual(
        Cylinder(radius=PLUNGER_ROD_RADIUS, length=0.052),
        origin=Origin(xyz=(0.026, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material="steel",
        name="main_rod",
    )
    plunger.visual(
        Cylinder(radius=PLUNGER_NOSE_RADIUS, length=PLUNGER_FRONT_X - 0.052),
        origin=Origin(
            xyz=(0.0615, 0.0, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material="steel",
        name="nose",
    )
    plunger.visual(
        Box((PLUNGER_SHOE_LENGTH, PLUNGER_SHOE_WIDTH, PLUNGER_SHOE_THICKNESS)),
        origin=Origin(xyz=(0.030, 0.0, -0.0065)),
        material="black_oxide",
        name="guide_shoe",
    )
    plunger.visual(
        Box((0.018, 0.004, 0.004)),
        origin=Origin(xyz=(0.030, 0.0, -0.0035)),
        material="black_oxide",
        name="guide_web",
    )
    plunger.visual(
        Box((0.001, 0.001, 0.001)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="steel",
        name="plunger_body",
    )
    plunger.inertial = Inertial.from_geometry(
        Cylinder(radius=PLUNGER_ROD_RADIUS, length=PLUNGER_FRONT_X + PLUNGER_BUTTON_LENGTH),
        mass=0.10,
        origin=Origin(
            xyz=(
                (PLUNGER_FRONT_X - PLUNGER_BUTTON_LENGTH) / 2.0,
                0.0,
                0.0,
            ),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
    )

    lever = model.part("output_lever")
    lever.visual(
        mesh_from_cadquery(_build_output_lever_shape(), "output_lever"),
        material="black_oxide",
        name="output_lever",
    )
    lever.inertial = Inertial.from_geometry(
        Box((0.022, LEVER_THICKNESS, 0.042)),
        mass=0.06,
        origin=Origin(xyz=(0.001, 0.0, -0.012)),
    )

    model.articulation(
        "sleeve_to_plunger",
        ArticulationType.PRISMATIC,
        parent=sleeve,
        child=plunger,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.20,
            lower=0.0,
            upper=PLUNGER_STROKE,
        ),
    )
    model.articulation(
        "sleeve_to_output_lever",
        ArticulationType.REVOLUTE,
        parent=sleeve,
        child=lever,
        origin=Origin(xyz=(PIVOT_X, 0.0, PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=0.0,
            upper=LEVER_OPEN_ANGLE,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    sleeve = object_model.get_part("sleeve")
    plunger = object_model.get_part("plunger")
    lever = object_model.get_part("output_lever")
    slide = object_model.get_articulation("sleeve_to_plunger")
    hinge = object_model.get_articulation("sleeve_to_output_lever")

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
        "plunger articulation axis",
        tuple(slide.axis) == (1.0, 0.0, 0.0),
        details=f"expected (1, 0, 0), got {slide.axis}",
    )
    ctx.check(
        "lever articulation axis",
        tuple(hinge.axis) == (0.0, -1.0, 0.0),
        details=f"expected (0, -1, 0), got {hinge.axis}",
    )

    ctx.expect_contact(plunger, sleeve, name="plunger_is_supported_by_sleeve")
    ctx.expect_contact(lever, sleeve, name="lever_is_supported_by_fork")
    ctx.expect_contact(plunger, lever, name="plunger_nose_meets_output_lever")
    ctx.expect_overlap(
        plunger,
        sleeve,
        axes="yz",
        min_overlap=0.015,
        name="plunger_runs_inside_sleeve_envelope",
    )

    with ctx.pose({slide: PLUNGER_STROKE}):
        ctx.expect_origin_gap(
            plunger,
            sleeve,
            axis="x",
            min_gap=PLUNGER_STROKE - 1e-6,
            max_gap=PLUNGER_STROKE + 1e-6,
            name="plunger_prismatic_stroke_matches_joint_limit",
        )

    with ctx.pose({hinge: 0.0}):
        closed_aabb = ctx.part_world_aabb(lever)
    with ctx.pose({hinge: LEVER_OPEN_ANGLE}):
        open_aabb = ctx.part_world_aabb(lever)

    lever_opens_forward = (
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][0] > closed_aabb[1][0] + 0.004
    )
    ctx.check(
        "lever swings forward when opened",
        lever_opens_forward,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
