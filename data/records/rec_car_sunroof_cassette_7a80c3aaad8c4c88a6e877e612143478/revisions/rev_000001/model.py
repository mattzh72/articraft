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


def _add_corner_carriage(
    model: ArticulatedObject,
    *,
    parent: str,
    carriage_name: str,
    carriage_joint_name: str,
    roller_name: str,
    roller_joint_name: str,
    x: float,
    y: float,
    carriage_material,
    steel,
    roller_rubber,
) -> None:
    carriage = model.part(carriage_name)
    carriage.visual(
        Box((0.034, 0.050, 0.010)),
        origin=Origin(xyz=(0.000, 0.000, -0.010)),
        material=carriage_material,
        name="mount_plate",
    )
    carriage.visual(
        Box((0.010, 0.006, 0.018)),
        origin=Origin(xyz=(0.000, -0.013, -0.019)),
        material=steel,
        name="left_cheek",
    )
    carriage.visual(
        Box((0.010, 0.006, 0.018)),
        origin=Origin(xyz=(0.000, 0.013, -0.019)),
        material=steel,
        name="right_cheek",
    )
    carriage.visual(
        Cylinder(radius=0.003, length=0.010),
        origin=Origin(xyz=(0.000, 0.000, -0.021), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="axle",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((0.034, 0.050, 0.032)),
        mass=0.08,
        origin=Origin(xyz=(0.000, 0.000, -0.016)),
    )
    model.articulation(
        carriage_joint_name,
        ArticulationType.FIXED,
        parent=parent,
        child=carriage,
        origin=Origin(xyz=(x, y, 0.0)),
    )

    roller = model.part(roller_name)
    roller.visual(
        Cylinder(radius=0.009, length=0.014),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=roller_rubber,
        name="roller_tire",
    )
    roller.visual(
        Cylinder(radius=0.0055, length=0.016),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="roller_hub",
    )
    roller.inertial = Inertial.from_geometry(Box((0.016, 0.020, 0.020)), mass=0.03)
    model.articulation(
        roller_joint_name,
        ArticulationType.CONTINUOUS,
        parent=carriage,
        child=roller,
        origin=Origin(xyz=(0.000, 0.000, -0.021)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=30.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tilt_slide_sunroof_cassette")

    aluminum = model.material("aluminum", rgba=(0.78, 0.80, 0.83, 1.0))
    dark_aluminum = model.material("dark_aluminum", rgba=(0.58, 0.61, 0.65, 1.0))
    black_seal = model.material("black_seal", rgba=(0.08, 0.08, 0.09, 1.0))
    tinted_glass = model.material("tinted_glass", rgba=(0.30, 0.40, 0.48, 0.42))
    steel = model.material("steel", rgba=(0.68, 0.70, 0.73, 1.0))
    roller_rubber = model.material("roller_rubber", rgba=(0.06, 0.06, 0.07, 1.0))

    frame = model.part("cassette_frame")
    frame.visual(
        Box((0.060, 1.100, 0.050)),
        origin=Origin(xyz=(-0.430, 0.000, 0.025)),
        material=dark_aluminum,
        name="left_gutter",
    )
    frame.visual(
        Box((0.060, 1.100, 0.050)),
        origin=Origin(xyz=(0.430, 0.000, 0.025)),
        material=dark_aluminum,
        name="right_gutter",
    )
    frame.visual(
        Box((0.800, 0.100, 0.068)),
        origin=Origin(xyz=(0.000, -0.520, 0.034)),
        material=dark_aluminum,
        name="front_crossmember",
    )
    frame.visual(
        Box((0.800, 0.110, 0.050)),
        origin=Origin(xyz=(0.000, 0.495, 0.025)),
        material=dark_aluminum,
        name="rear_crossmember",
    )
    frame.visual(
        Box((0.800, 0.220, 0.018)),
        origin=Origin(xyz=(0.000, -0.385, 0.009)),
        material=aluminum,
        name="front_drain_tray",
    )
    frame.visual(
        Box((0.800, 0.220, 0.018)),
        origin=Origin(xyz=(0.000, 0.385, 0.009)),
        material=aluminum,
        name="rear_storage_tray",
    )
    frame.visual(
        Box((0.024, 0.900, 0.010)),
        origin=Origin(xyz=(-0.360, 0.000, 0.023)),
        material=aluminum,
        name="left_guide_rail",
    )
    frame.visual(
        Box((0.024, 0.900, 0.010)),
        origin=Origin(xyz=(0.360, 0.000, 0.023)),
        material=aluminum,
        name="right_guide_rail",
    )
    frame.visual(
        Box((0.020, 0.620, 0.040)),
        origin=Origin(xyz=(-0.390, -0.180, 0.048)),
        material=aluminum,
        name="left_lip_riser",
    )
    frame.visual(
        Box((0.020, 0.620, 0.040)),
        origin=Origin(xyz=(0.390, -0.180, 0.048)),
        material=aluminum,
        name="right_lip_riser",
    )
    frame.visual(
        Box((0.040, 0.620, 0.006)),
        origin=Origin(xyz=(-0.380, -0.180, 0.071)),
        material=aluminum,
        name="left_side_lip",
    )
    frame.visual(
        Box((0.040, 0.620, 0.006)),
        origin=Origin(xyz=(0.380, -0.180, 0.071)),
        material=aluminum,
        name="right_side_lip",
    )
    frame.visual(
        Box((0.760, 0.014, 0.006)),
        origin=Origin(xyz=(0.000, -0.495, 0.071)),
        material=aluminum,
        name="front_top_lip",
    )
    frame.inertial = Inertial.from_geometry(
        Box((0.920, 1.100, 0.080)),
        mass=9.5,
        origin=Origin(xyz=(0.000, 0.000, 0.040)),
    )

    slide_stage = model.part("slide_stage")
    slide_stage.visual(
        Cylinder(radius=0.009, length=0.620),
        origin=Origin(xyz=(0.000, 0.030, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="hinge_pin",
    )
    slide_stage.visual(
        Box((0.120, 0.060, 0.020)),
        origin=Origin(xyz=(0.000, 0.000, -0.010)),
        material=steel,
        name="center_mount",
    )
    slide_stage.visual(
        Box((0.630, 0.028, 0.010)),
        origin=Origin(xyz=(0.000, -0.420, -0.020)),
        material=dark_aluminum,
        name="front_spreader",
    )
    slide_stage.visual(
        Box((0.018, 0.360, 0.010)),
        origin=Origin(xyz=(-0.315, -0.240, -0.020)),
        material=dark_aluminum,
        name="left_runner",
    )
    slide_stage.visual(
        Box((0.018, 0.360, 0.010)),
        origin=Origin(xyz=(0.315, -0.240, -0.020)),
        material=dark_aluminum,
        name="right_runner",
    )
    slide_stage.visual(
        Box((0.630, 0.028, 0.010)),
        origin=Origin(xyz=(0.000, -0.060, -0.020)),
        material=dark_aluminum,
        name="rear_spreader",
    )
    slide_stage.visual(
        Box((0.040, 0.120, 0.010)),
        origin=Origin(xyz=(0.000, -0.090, -0.020)),
        material=dark_aluminum,
        name="center_spine",
    )
    slide_stage.inertial = Inertial.from_geometry(
        Box((0.720, 0.460, 0.030)),
        mass=1.5,
        origin=Origin(xyz=(0.000, -0.200, -0.015)),
    )
    model.articulation(
        "frame_to_slide_stage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=slide_stage,
        origin=Origin(xyz=(0.000, 0.000, 0.058)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.45, lower=0.0, upper=0.320),
    )

    glass_panel = model.part("glass_panel")
    glass_panel.visual(
        Cylinder(radius=0.011, length=0.600),
        origin=Origin(xyz=(0.000, 0.030, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="hinge_knuckle",
    )
    glass_panel.visual(
        Box((0.720, 0.450, 0.016)),
        origin=Origin(xyz=(0.000, -0.220, 0.008)),
        material=black_seal,
        name="panel_frame",
    )
    glass_panel.visual(
        Box((0.640, 0.370, 0.005)),
        origin=Origin(xyz=(0.000, -0.220, 0.0105)),
        material=tinted_glass,
        name="glass_lite",
    )
    glass_panel.visual(
        Box((0.720, 0.030, 0.001)),
        origin=Origin(xyz=(0.000, -0.495, 0.0155)),
        material=black_seal,
        name="front_top_edge",
    )
    glass_panel.visual(
        Box((0.720, 0.050, 0.003)),
        origin=Origin(xyz=(0.000, -0.470, 0.014)),
        material=black_seal,
        name="front_edge_web",
    )
    glass_panel.visual(
        Box((0.180, 0.020, 0.006)),
        origin=Origin(xyz=(0.000, -0.220, 0.003)),
        material=dark_aluminum,
        name="underside_stiffener",
    )
    glass_panel.inertial = Inertial.from_geometry(
        Box((0.720, 0.450, 0.028)),
        mass=6.8,
        origin=Origin(xyz=(0.000, -0.220, 0.014)),
    )
    model.articulation(
        "slide_stage_to_glass_panel",
        ArticulationType.REVOLUTE,
        parent=slide_stage,
        child=glass_panel,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=-0.350, upper=0.0),
    )

    _add_corner_carriage(
        model,
        parent="slide_stage",
        carriage_name="front_left_carriage",
        carriage_joint_name="slide_stage_to_front_left_carriage",
        roller_name="front_left_roller",
        roller_joint_name="front_left_carriage_to_front_left_roller",
        x=-0.372,
        y=-0.420,
        carriage_material=dark_aluminum,
        steel=steel,
        roller_rubber=roller_rubber,
    )
    _add_corner_carriage(
        model,
        parent="slide_stage",
        carriage_name="front_right_carriage",
        carriage_joint_name="slide_stage_to_front_right_carriage",
        roller_name="front_right_roller",
        roller_joint_name="front_right_carriage_to_front_right_roller",
        x=0.372,
        y=-0.420,
        carriage_material=dark_aluminum,
        steel=steel,
        roller_rubber=roller_rubber,
    )
    _add_corner_carriage(
        model,
        parent="slide_stage",
        carriage_name="rear_left_carriage",
        carriage_joint_name="slide_stage_to_rear_left_carriage",
        roller_name="rear_left_roller",
        roller_joint_name="rear_left_carriage_to_rear_left_roller",
        x=-0.372,
        y=-0.060,
        carriage_material=dark_aluminum,
        steel=steel,
        roller_rubber=roller_rubber,
    )
    _add_corner_carriage(
        model,
        parent="slide_stage",
        carriage_name="rear_right_carriage",
        carriage_joint_name="slide_stage_to_rear_right_carriage",
        roller_name="rear_right_roller",
        roller_joint_name="rear_right_carriage_to_rear_right_roller",
        x=0.372,
        y=-0.060,
        carriage_material=dark_aluminum,
        steel=steel,
        roller_rubber=roller_rubber,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("cassette_frame")
    slide_stage = object_model.get_part("slide_stage")
    glass_panel = object_model.get_part("glass_panel")
    front_left_carriage = object_model.get_part("front_left_carriage")
    front_right_carriage = object_model.get_part("front_right_carriage")
    rear_left_carriage = object_model.get_part("rear_left_carriage")
    rear_right_carriage = object_model.get_part("rear_right_carriage")
    front_left_roller = object_model.get_part("front_left_roller")
    front_right_roller = object_model.get_part("front_right_roller")
    rear_left_roller = object_model.get_part("rear_left_roller")
    rear_right_roller = object_model.get_part("rear_right_roller")

    slide_joint = object_model.get_articulation("frame_to_slide_stage")
    tilt_joint = object_model.get_articulation("slide_stage_to_glass_panel")
    roller_joints = [
        object_model.get_articulation("front_left_carriage_to_front_left_roller"),
        object_model.get_articulation("front_right_carriage_to_front_right_roller"),
        object_model.get_articulation("rear_left_carriage_to_rear_left_roller"),
        object_model.get_articulation("rear_right_carriage_to_rear_right_roller"),
    ]

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()

    ctx.allow_overlap(
        slide_stage,
        glass_panel,
        reason="Simplified rear hinge bracket and knuckle are modeled as a compact captured assembly.",
    )
    ctx.allow_overlap(
        front_left_carriage,
        front_left_roller,
        reason="Simplified front-left roller and carriage share a compact bearing envelope.",
    )
    ctx.allow_overlap(
        front_right_carriage,
        front_right_roller,
        reason="Simplified front-right roller and carriage share a compact bearing envelope.",
    )
    ctx.allow_overlap(
        rear_left_carriage,
        rear_left_roller,
        reason="Simplified rear-left roller and carriage share a compact bearing envelope.",
    )
    ctx.allow_overlap(
        rear_right_carriage,
        rear_right_roller,
        reason="Simplified rear-right roller and carriage share a compact bearing envelope.",
    )
    ctx.allow_overlap(
        frame,
        front_left_carriage,
        reason="Front-left shoe carriage is represented as a nested guide shoe inside the left cassette channel.",
    )
    ctx.allow_overlap(
        frame,
        front_right_carriage,
        reason="Front-right shoe carriage is represented as a nested guide shoe inside the right cassette channel.",
    )
    ctx.allow_overlap(
        frame,
        rear_left_carriage,
        reason="Rear-left shoe carriage is represented as a nested guide shoe inside the left cassette channel.",
    )
    ctx.allow_overlap(
        frame,
        rear_right_carriage,
        reason="Rear-right shoe carriage is represented as a nested guide shoe inside the right cassette channel.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    for part_name in (
        "cassette_frame",
        "slide_stage",
        "glass_panel",
        "front_left_carriage",
        "front_right_carriage",
        "rear_left_carriage",
        "rear_right_carriage",
        "front_left_roller",
        "front_right_roller",
        "rear_left_roller",
        "rear_right_roller",
    ):
        ctx.check(f"{part_name}_present", object_model.get_part(part_name) is not None)

    ctx.check(
        "slide_joint_axis_is_rearward",
        tuple(slide_joint.axis) == (0.0, 1.0, 0.0),
        details=f"Expected slide axis (0, 1, 0), got {slide_joint.axis!r}",
    )
    ctx.check(
        "tilt_joint_axis_is_transverse",
        tuple(tilt_joint.axis) == (1.0, 0.0, 0.0),
        details=f"Expected tilt axis (1, 0, 0), got {tilt_joint.axis!r}",
    )
    for index, roller_joint in enumerate(roller_joints):
        ctx.check(
            f"roller_joint_{index}_axis_is_transverse",
            tuple(roller_joint.axis) == (1.0, 0.0, 0.0),
            details=f"Expected roller axis (1, 0, 0), got {roller_joint.axis!r}",
        )

    ctx.expect_contact(glass_panel, slide_stage, elem_a="hinge_knuckle", elem_b="hinge_pin")
    ctx.expect_contact(front_left_roller, frame, elem_b="left_guide_rail")
    ctx.expect_contact(front_right_roller, frame, elem_b="right_guide_rail")
    ctx.expect_contact(rear_left_roller, frame, elem_b="left_guide_rail")
    ctx.expect_contact(rear_right_roller, frame, elem_b="right_guide_rail")
    ctx.expect_contact(glass_panel, frame, elem_a="front_top_edge", elem_b="front_top_lip")

    ctx.expect_overlap(glass_panel, frame, axes="x", min_overlap=0.70)
    ctx.expect_overlap(glass_panel, frame, axes="y", min_overlap=0.50)
    ctx.expect_within(glass_panel, frame, axes="x", margin=0.03)
    ctx.expect_overlap(front_left_roller, frame, axes="xy", min_overlap=0.008, elem_b="left_guide_rail")
    ctx.expect_overlap(front_right_roller, frame, axes="xy", min_overlap=0.008, elem_b="right_guide_rail")
    ctx.expect_overlap(rear_left_roller, frame, axes="xy", min_overlap=0.008, elem_b="left_guide_rail")
    ctx.expect_overlap(rear_right_roller, frame, axes="xy", min_overlap=0.008, elem_b="right_guide_rail")
    ctx.expect_gap(
        glass_panel,
        frame,
        axis="z",
        positive_elem="front_top_edge",
        negative_elem="front_top_lip",
        max_gap=0.001,
        max_penetration=0.001,
    )

    panel_rest = ctx.part_world_position(glass_panel)
    front_rest = ctx.part_element_world_aabb(glass_panel, elem="front_top_edge")
    hinge_rest = ctx.part_element_world_aabb(glass_panel, elem="hinge_knuckle")
    assert panel_rest is not None
    assert front_rest is not None
    assert hinge_rest is not None

    slide_limits = slide_joint.motion_limits
    assert slide_limits is not None
    assert slide_limits.lower is not None
    assert slide_limits.upper is not None
    with ctx.pose({slide_joint: slide_limits.lower}):
        ctx.fail_if_parts_overlap_in_current_pose(name="slide_joint_lower_no_overlap")
        ctx.fail_if_isolated_parts(name="slide_joint_lower_no_floating")
    with ctx.pose({slide_joint: slide_limits.upper}):
        panel_open = ctx.part_world_position(glass_panel)
        assert panel_open is not None
        ctx.check(
            "panel_slides_rearward",
            panel_open[1] > panel_rest[1] + 0.30,
            details=f"Expected rearward travel > 0.30 m, got {panel_open[1] - panel_rest[1]:.4f} m",
        )
        ctx.expect_contact(front_left_roller, frame, elem_b="left_guide_rail")
        ctx.expect_contact(front_right_roller, frame, elem_b="right_guide_rail")
        ctx.expect_contact(rear_left_roller, frame, elem_b="left_guide_rail")
        ctx.expect_contact(rear_right_roller, frame, elem_b="right_guide_rail")
        ctx.fail_if_parts_overlap_in_current_pose(name="slide_joint_upper_no_overlap")
        ctx.fail_if_isolated_parts(name="slide_joint_upper_no_floating")

    tilt_limits = tilt_joint.motion_limits
    assert tilt_limits is not None
    assert tilt_limits.lower is not None
    assert tilt_limits.upper is not None
    with ctx.pose({tilt_joint: tilt_limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="tilt_joint_upper_no_overlap")
        ctx.fail_if_isolated_parts(name="tilt_joint_upper_no_floating")
    with ctx.pose({tilt_joint: tilt_limits.lower}):
        front_tilt = ctx.part_element_world_aabb(glass_panel, elem="front_top_edge")
        hinge_tilt = ctx.part_element_world_aabb(glass_panel, elem="hinge_knuckle")
        assert front_tilt is not None
        assert hinge_tilt is not None
        ctx.expect_gap(
            glass_panel,
            frame,
            axis="z",
            positive_elem="front_top_edge",
            negative_elem="front_top_lip",
            min_gap=0.030,
        )
        ctx.check(
            "rear_hinge_stays_near_pivot_height",
            abs(hinge_tilt[0][2] - hinge_rest[0][2]) <= 0.020,
            details=f"Expected hinge to stay near pivot height, got {hinge_tilt[0][2] - hinge_rest[0][2]:.4f} m shift",
        )
        ctx.check(
            "front_edge_lifts_when_tilted",
            front_tilt[0][2] > front_rest[0][2] + 0.030,
            details=f"Expected front edge lift > 0.03 m, got {front_tilt[0][2] - front_rest[0][2]:.4f} m",
        )
        ctx.expect_contact(front_left_roller, frame, elem_b="left_guide_rail")
        ctx.expect_contact(front_right_roller, frame, elem_b="right_guide_rail")
        ctx.expect_contact(rear_left_roller, frame, elem_b="left_guide_rail")
        ctx.expect_contact(rear_right_roller, frame, elem_b="right_guide_rail")
        ctx.fail_if_parts_overlap_in_current_pose(name="tilt_joint_lower_no_overlap")
        ctx.fail_if_isolated_parts(name="tilt_joint_lower_no_floating")

    with ctx.pose({slide_joint: slide_limits.upper, tilt_joint: tilt_limits.lower}):
        ctx.expect_contact(front_left_roller, frame, elem_b="left_guide_rail")
        ctx.expect_contact(front_right_roller, frame, elem_b="right_guide_rail")
        ctx.expect_contact(rear_left_roller, frame, elem_b="left_guide_rail")
        ctx.expect_contact(rear_right_roller, frame, elem_b="right_guide_rail")
        ctx.expect_gap(
            glass_panel,
            frame,
            axis="z",
            positive_elem="front_top_edge",
            negative_elem="front_top_lip",
            min_gap=0.030,
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="combined_open_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="combined_open_pose_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
