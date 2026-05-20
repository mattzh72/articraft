from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vertical_slide_wrist", assets=ASSETS)

    steel_dark = model.material("steel_dark", rgba=(0.32, 0.35, 0.38, 1.0))
    steel_mid = model.material("steel_mid", rgba=(0.56, 0.59, 0.62, 1.0))
    aluminum = model.material("aluminum", rgba=(0.78, 0.80, 0.82, 1.0))
    joint_black = model.material("joint_black", rgba=(0.15, 0.15, 0.16, 1.0))

    column = model.part("column")
    column.visual(
        Box((0.12, 0.14, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        material=steel_dark,
        name="base_plate",
    )
    column.visual(
        Box((0.04, 0.032, 0.32)),
        origin=Origin(xyz=(0.0, -0.010, 0.18)),
        material=steel_mid,
        name="mast_spine",
    )
    column.visual(
        Box((0.014, 0.022, 0.27)),
        origin=Origin(xyz=(-0.024, 0.017, 0.165)),
        material=aluminum,
        name="left_rail",
    )
    column.visual(
        Box((0.014, 0.022, 0.27)),
        origin=Origin(xyz=(0.024, 0.017, 0.165)),
        material=aluminum,
        name="right_rail",
    )
    column.inertial = Inertial.from_geometry(
        Box((0.12, 0.14, 0.32)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.020, 0.032, 0.070)),
        origin=Origin(xyz=(-0.024, 0.044, 0.0)),
        material=aluminum,
        name="left_shoe",
    )
    carriage.visual(
        Box((0.020, 0.032, 0.070)),
        origin=Origin(xyz=(0.024, 0.044, 0.0)),
        material=aluminum,
        name="right_shoe",
    )
    carriage.visual(
        Box((0.084, 0.016, 0.050)),
        origin=Origin(xyz=(0.0, 0.062, 0.0)),
        material=steel_mid,
        name="front_bridge",
    )
    carriage.visual(
        Box((0.060, 0.024, 0.020)),
        origin=Origin(xyz=(0.0, 0.078, 0.0)),
        material=steel_mid,
        name="clevis_block",
    )
    carriage.visual(
        Box((0.008, 0.030, 0.054)),
        origin=Origin(xyz=(-0.028, 0.103, 0.0)),
        material=steel_mid,
        name="left_ear",
    )
    carriage.visual(
        Box((0.008, 0.030, 0.054)),
        origin=Origin(xyz=(0.028, 0.103, 0.0)),
        material=steel_mid,
        name="right_ear",
    )
    carriage.visual(
        Cylinder(radius=0.006, length=0.048),
        origin=Origin(xyz=(0.0, 0.106, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=joint_black,
        name="wrist_pin",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.084, 0.090, 0.070)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.060, 0.0)),
    )

    wrist = model.part("wrist")
    wrist.visual(
        Cylinder(radius=0.009, length=0.036),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=joint_black,
        name="hub",
    )
    wrist.visual(
        Box((0.024, 0.022, 0.020)),
        origin=Origin(xyz=(0.0, 0.020, 0.0)),
        material=steel_mid,
        name="neck",
    )
    wrist.visual(
        Box((0.044, 0.012, 0.032)),
        origin=Origin(xyz=(0.0, 0.032, 0.0)),
        material=aluminum,
        name="wrist_plate",
    )
    wrist.inertial = Inertial.from_geometry(
        Box((0.044, 0.044, 0.032)),
        mass=0.7,
        origin=Origin(xyz=(0.0, 0.022, 0.0)),
    )

    model.articulation(
        "column_to_carriage",
        ArticulationType.PRISMATIC,
        parent=column,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1200.0,
            velocity=0.35,
            lower=0.0,
            upper=0.18,
        ),
    )
    model.articulation(
        "carriage_to_wrist",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=wrist,
        origin=Origin(xyz=(0.0, 0.106, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=2.5,
            lower=-math.pi / 2.0,
            upper=math.pi / 2.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    column = object_model.get_part("column")
    carriage = object_model.get_part("carriage")
    wrist = object_model.get_part("wrist")
    slide = object_model.get_articulation("column_to_carriage")
    wrist_hinge = object_model.get_articulation("carriage_to_wrist")

    mast_spine = column.get_visual("mast_spine")
    left_rail = column.get_visual("left_rail")
    right_rail = column.get_visual("right_rail")
    left_shoe = carriage.get_visual("left_shoe")
    right_shoe = carriage.get_visual("right_shoe")
    front_bridge = carriage.get_visual("front_bridge")
    wrist_pin = carriage.get_visual("wrist_pin")
    hub = wrist.get_visual("hub")
    wrist_plate = wrist.get_visual("wrist_plate")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.allow_overlap(
        carriage,
        wrist,
        elem_a=wrist_pin,
        elem_b=hub,
        reason="The wrist hub intentionally rotates around the carriage-supported pin.",
    )
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_gap(
        carriage,
        column,
        axis="y",
        positive_elem=left_shoe,
        negative_elem=left_rail,
        max_gap=0.001,
        max_penetration=1e-5,
        name="left_carriage_shoe_seats_on_left_rail",
    )
    ctx.expect_gap(
        carriage,
        column,
        axis="y",
        positive_elem=right_shoe,
        negative_elem=right_rail,
        max_gap=0.001,
        max_penetration=1e-5,
        name="right_carriage_shoe_seats_on_right_rail",
    )
    ctx.expect_overlap(
        carriage,
        column,
        axes="xz",
        elem_a=left_shoe,
        elem_b=left_rail,
        min_overlap=0.0135,
        name="left_shoe_overlaps_left_rail_guide_footprint",
    )
    ctx.expect_overlap(
        carriage,
        column,
        axes="xz",
        elem_a=right_shoe,
        elem_b=right_rail,
        min_overlap=0.0135,
        name="right_shoe_overlaps_right_rail_guide_footprint",
    )
    ctx.expect_gap(
        wrist,
        carriage,
        axis="y",
        positive_elem=wrist_plate,
        negative_elem=front_bridge,
        min_gap=0.02,
        name="wrist_plate_sits_forward_of_carriage_bridge",
    )

    with ctx.pose({slide: 0.18}):
        ctx.expect_gap(
            carriage,
            column,
            axis="y",
            positive_elem=left_shoe,
            negative_elem=left_rail,
            max_gap=0.001,
            max_penetration=1e-5,
            name="left_shoe_stays_seated_at_full_lift",
        )
        ctx.expect_gap(
            carriage,
            column,
            axis="y",
            positive_elem=right_shoe,
            negative_elem=right_rail,
            max_gap=0.001,
            max_penetration=1e-5,
            name="right_shoe_stays_seated_at_full_lift",
        )
        low_pos = None
        high_pos = ctx.part_world_position(carriage)

    with ctx.pose({slide: 0.0}):
        low_pos = ctx.part_world_position(carriage)

    if low_pos is not None and high_pos is not None:
        ctx.check(
            "carriage_travel_is_180_mm",
            abs((high_pos[2] - low_pos[2]) - 0.18) <= 1e-6,
            details=f"expected 0.18 m travel, got {high_pos[2] - low_pos[2]:.6f} m",
        )
    else:
        ctx.fail("carriage_travel_is_180_mm", "could not measure carriage world positions")

    ctx.check(
        "slide_axis_is_vertical",
        tuple(slide.axis) == (0.0, 0.0, 1.0),
        details=f"expected slide axis (0, 0, 1), got {slide.axis}",
    )
    ctx.check(
        "wrist_axis_is_horizontal_x",
        tuple(wrist_hinge.axis) == (1.0, 0.0, 0.0),
        details=f"expected wrist axis (1, 0, 0), got {wrist_hinge.axis}",
    )

    with ctx.pose({wrist_hinge: math.pi / 2.0}):
        ctx.expect_gap(
            wrist,
            carriage,
            axis="y",
            positive_elem=wrist_plate,
            negative_elem=front_bridge,
            min_gap=0.0,
            name="wrist_plate_clears_bridge_at_up_pitch",
        )
    with ctx.pose({wrist_hinge: -math.pi / 2.0}):
        ctx.expect_gap(
            wrist,
            carriage,
            axis="y",
            positive_elem=wrist_plate,
            negative_elem=front_bridge,
            min_gap=0.0,
            name="wrist_plate_clears_bridge_at_down_pitch",
        )
    ctx.expect_contact(
        column,
        column,
        elem_a=mast_spine,
        elem_b=left_rail,
        name="column_geometry_is_connected",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
