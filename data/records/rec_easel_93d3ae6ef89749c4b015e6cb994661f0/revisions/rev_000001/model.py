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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="outdoor_poster_easel")

    aluminum = model.material("aluminum", rgba=(0.78, 0.80, 0.82, 1.0))
    anodized = model.material("anodized", rgba=(0.58, 0.60, 0.63, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.16, 0.17, 0.18, 1.0))
    rubber = model.material("rubber", rgba=(0.13, 0.13, 0.13, 1.0))

    frame_width = 0.58
    leg_x = frame_width / 2.0
    leg_width = 0.026
    leg_depth = 0.020
    front_leg_height = 1.42
    tray_slot_center_z = 0.42
    clamp_axis_z = 1.438
    rear_rest_angle = math.radians(19.0)

    front_frame = model.part("front_frame")
    front_frame.visual(
        Box((leg_width, leg_depth, front_leg_height)),
        origin=Origin(xyz=(-leg_x, 0.0, front_leg_height / 2.0)),
        material=aluminum,
        name="left_leg",
    )
    front_frame.visual(
        Box((leg_width, leg_depth, front_leg_height)),
        origin=Origin(xyz=(leg_x, 0.0, front_leg_height / 2.0)),
        material=aluminum,
        name="right_leg",
    )
    front_frame.visual(
        Box((frame_width + leg_width, 0.024, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 1.4025)),
        material=aluminum,
        name="top_bridge",
    )
    front_frame.visual(
        Box((0.18, 0.028, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 1.4375)),
        material=anodized,
        name="crown_block",
    )
    front_frame.visual(
        Box((0.59, 0.014, 0.020)),
        origin=Origin(xyz=(0.0, 0.007, 1.428)),
        material=anodized,
        name="front_hinge_rail",
    )
    front_frame.visual(
        Box((0.010, 0.0016, 0.48)),
        origin=Origin(xyz=(-leg_x, 0.0092, tray_slot_center_z)),
        material=dark_trim,
        name="left_slot_strip",
    )
    front_frame.visual(
        Box((0.010, 0.0016, 0.48)),
        origin=Origin(xyz=(leg_x, 0.0092, tray_slot_center_z)),
        material=dark_trim,
        name="right_slot_strip",
    )
    front_frame.visual(
        Box((0.034, 0.024, 0.010)),
        origin=Origin(xyz=(-leg_x, 0.0, 0.005)),
        material=rubber,
        name="left_foot_cap",
    )
    front_frame.visual(
        Box((0.034, 0.024, 0.010)),
        origin=Origin(xyz=(leg_x, 0.0, 0.005)),
        material=rubber,
        name="right_foot_cap",
    )
    front_frame.inertial = Inertial.from_geometry(
        Box((frame_width + leg_width, 0.030, 1.455)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.0, 0.7275)),
    )

    tray = model.part("tray")
    tray.visual(
        Box((0.046, 0.008, 0.22)),
        origin=Origin(xyz=(-leg_x, 0.004, 0.0)),
        material=anodized,
        name="left_mount_plate",
    )
    tray.visual(
        Box((0.046, 0.008, 0.22)),
        origin=Origin(xyz=(leg_x, 0.004, 0.0)),
        material=anodized,
        name="right_mount_plate",
    )
    tray.visual(
        Box((0.534, 0.048, 0.008)),
        origin=Origin(xyz=(0.0, 0.028, -0.094)),
        material=aluminum,
        name="tray_shelf",
    )
    tray.visual(
        Box((0.500, 0.008, 0.024)),
        origin=Origin(xyz=(0.0, 0.010, -0.078)),
        material=anodized,
        name="rear_lip",
    )
    tray.visual(
        Box((0.534, 0.006, 0.028)),
        origin=Origin(xyz=(0.0, 0.049, -0.076)),
        material=aluminum,
        name="front_lip",
    )
    tray.visual(
        Cylinder(radius=0.013, length=0.016),
        origin=Origin(xyz=(-leg_x, 0.016, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="left_knob",
    )
    tray.visual(
        Cylinder(radius=0.013, length=0.016),
        origin=Origin(xyz=(leg_x, 0.016, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="right_knob",
    )
    tray.inertial = Inertial.from_geometry(
        Box((0.62, 0.07, 0.24)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.028, -0.05)),
    )

    clamp_bar = model.part("clamp_bar")
    clamp_bar.visual(
        Box((0.026, 0.012, 0.020)),
        origin=Origin(xyz=(-0.282, 0.006, -0.010)),
        material=anodized,
        name="left_ear",
    )
    clamp_bar.visual(
        Box((0.026, 0.012, 0.020)),
        origin=Origin(xyz=(0.282, 0.006, -0.010)),
        material=anodized,
        name="right_ear",
    )
    clamp_bar.visual(
        Box((0.560, 0.018, 0.024)),
        origin=Origin(xyz=(0.0, 0.014, -0.018)),
        material=aluminum,
        name="bar_body",
    )
    clamp_bar.visual(
        Box((0.540, 0.006, 0.008)),
        origin=Origin(xyz=(0.0, 0.004, -0.030)),
        material=rubber,
        name="jaw_pad",
    )
    clamp_bar.inertial = Inertial.from_geometry(
        Box((0.58, 0.03, 0.04)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.014, -0.018)),
    )

    rear_leg = model.part("rear_leg")
    rear_leg.visual(
        Box((0.070, 0.014, 0.022)),
        origin=Origin(xyz=(0.0, -0.007, -0.011)),
        material=anodized,
        name="hinge_shoe",
    )
    rear_leg.visual(
        Box((0.024, 0.018, 1.52)),
        origin=Origin(xyz=(0.0, 0.0, -0.771)),
        material=aluminum,
        name="brace_tube",
    )
    rear_leg.visual(
        Box((0.080, 0.040, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, -1.526)),
        material=rubber,
        name="rear_foot",
    )
    rear_leg.inertial = Inertial.from_geometry(
        Box((0.08, 0.05, 1.54)),
        mass=1.15,
        origin=Origin(xyz=(0.0, 0.0, -0.77)),
    )

    model.articulation(
        "frame_to_tray",
        ArticulationType.PRISMATIC,
        parent=front_frame,
        child=tray,
        origin=Origin(xyz=(0.0, leg_depth / 2.0, tray_slot_center_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=0.10,
            lower=-0.16,
            upper=0.22,
        ),
    )
    model.articulation(
        "frame_to_clamp_bar",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=clamp_bar,
        origin=Origin(xyz=(0.0, 0.014, clamp_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=1.2,
            lower=-0.10,
            upper=1.30,
        ),
    )
    model.articulation(
        "frame_to_rear_leg",
        ArticulationType.REVOLUTE,
        parent=front_frame,
        child=rear_leg,
        origin=Origin(
            xyz=(0.0, -0.014, clamp_axis_z),
            rpy=(-rear_rest_angle, 0.0, 0.0),
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=0.8,
            lower=-0.45,
            upper=0.16,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    front_frame = object_model.get_part("front_frame")
    tray = object_model.get_part("tray")
    clamp_bar = object_model.get_part("clamp_bar")
    rear_leg = object_model.get_part("rear_leg")

    tray_slide = object_model.get_articulation("frame_to_tray")
    clamp_hinge = object_model.get_articulation("frame_to_clamp_bar")
    rear_hinge = object_model.get_articulation("frame_to_rear_leg")

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

    ctx.check("front_frame_present", front_frame is not None)
    ctx.check("tray_present", tray is not None)
    ctx.check("clamp_bar_present", clamp_bar is not None)
    ctx.check("rear_leg_present", rear_leg is not None)

    ctx.check(
        "tray_prismatic_axis_is_vertical",
        tuple(tray_slide.axis) == (0.0, 0.0, 1.0),
        details=f"axis={tray_slide.axis}",
    )
    ctx.check(
        "clamp_bar_hinge_axis_is_lateral",
        tuple(clamp_hinge.axis) == (1.0, 0.0, 0.0),
        details=f"axis={clamp_hinge.axis}",
    )
    ctx.check(
        "rear_leg_hinge_axis_is_lateral",
        tuple(rear_hinge.axis) == (1.0, 0.0, 0.0),
        details=f"axis={rear_hinge.axis}",
    )

    ctx.expect_contact(tray, front_frame, elem_a="left_mount_plate")
    ctx.expect_contact(clamp_bar, front_frame, elem_a="left_ear")
    ctx.expect_contact(rear_leg, front_frame, elem_a="hinge_shoe")

    ctx.expect_gap(
        tray,
        front_frame,
        axis="y",
        positive_elem="tray_shelf",
        negative_elem="left_leg",
        min_gap=0.003,
        max_gap=0.010,
        name="tray_projects_forward_of_front_legs",
    )
    ctx.expect_within(
        tray,
        front_frame,
        axes="x",
        inner_elem="tray_shelf",
        margin=0.04,
        name="tray_stays_between_front_legs",
    )
    ctx.expect_gap(
        clamp_bar,
        tray,
        axis="z",
        positive_elem="jaw_pad",
        negative_elem="tray_shelf",
        min_gap=0.95,
        name="clamp_bar_is_well_above_tray",
    )
    ctx.expect_within(
        rear_leg,
        front_frame,
        axes="x",
        margin=0.05,
        name="rear_leg_centered_under_crown",
    )

    tray_rest = ctx.part_world_position(tray)
    assert tray_rest is not None
    with ctx.pose({tray_slide: 0.18}):
        tray_high = ctx.part_world_position(tray)
        assert tray_high is not None
        ctx.check(
            "tray_moves_up_the_slots",
            abs(tray_high[0] - tray_rest[0]) < 1e-6
            and abs(tray_high[1] - tray_rest[1]) < 1e-6
            and tray_high[2] > tray_rest[2] + 0.16,
            details=f"rest={tray_rest}, high={tray_high}",
        )

    clamp_rest = ctx.part_element_world_aabb(clamp_bar, elem="jaw_pad")
    assert clamp_rest is not None
    with ctx.pose({clamp_hinge: 1.10}):
        clamp_open = ctx.part_element_world_aabb(clamp_bar, elem="jaw_pad")
        assert clamp_open is not None
        ctx.check(
            "clamp_bar_swings_up_to_open",
            clamp_open[0][1] > clamp_rest[0][1] + 0.02
            and clamp_open[1][2] > clamp_rest[1][2] + 0.015,
            details=f"rest={clamp_rest}, open={clamp_open}",
        )

    rear_rest = ctx.part_element_world_aabb(rear_leg, elem="rear_foot")
    assert rear_rest is not None
    with ctx.pose({rear_hinge: -0.35}):
        rear_open = ctx.part_element_world_aabb(rear_leg, elem="rear_foot")
        assert rear_open is not None
        ctx.check(
            "rear_leg_kicks_back_from_the_crown",
            rear_open[0][1] < rear_rest[0][1] - 0.16,
            details=f"rest={rear_rest}, open={rear_open}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
