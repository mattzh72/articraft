from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
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
    model = ArticulatedObject(name="sunroof_cassette", assets=ASSETS)

    cassette_black = model.material("cassette_black", rgba=(0.12, 0.13, 0.14, 1.0))
    track_steel = model.material("track_steel", rgba=(0.45, 0.47, 0.50, 1.0))
    seal_rubber = model.material("seal_rubber", rgba=(0.05, 0.05, 0.06, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.30, 0.44, 0.56, 0.44))
    glass_frit = model.material("glass_frit", rgba=(0.10, 0.10, 0.11, 0.90))

    frame_outer_width = 0.86
    frame_outer_length = 0.96
    slider_origin_y = -0.32
    panel_width = 0.60
    panel_length = 0.64

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((frame_outer_width, frame_outer_length, 0.060)),
        mass=9.0,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
    )
    frame.visual(
        Box((frame_outer_width, frame_outer_length, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=cassette_black,
        name="base_tray",
    )
    frame.visual(
        Box((0.034, 0.820, 0.028)),
        origin=Origin(xyz=(-0.413, 0.0, 0.026)),
        material=cassette_black,
        name="left_side_cassette",
    )
    frame.visual(
        Box((0.034, 0.820, 0.028)),
        origin=Origin(xyz=(0.413, 0.0, 0.026)),
        material=cassette_black,
        name="right_side_cassette",
    )
    frame.visual(
        Box((frame_outer_width, 0.110, 0.028)),
        origin=Origin(xyz=(0.0, -0.425, 0.026)),
        material=cassette_black,
        name="front_header",
    )
    frame.visual(
        Box((frame_outer_width, 0.110, 0.028)),
        origin=Origin(xyz=(0.0, 0.425, 0.026)),
        material=cassette_black,
        name="rear_header",
    )
    frame.visual(
        Box((0.088, 0.780, 0.006)),
        origin=Origin(xyz=(-0.354, 0.020, 0.015)),
        material=track_steel,
        name="left_track_floor",
    )
    frame.visual(
        Box((0.088, 0.780, 0.006)),
        origin=Origin(xyz=(0.354, 0.020, 0.015)),
        material=track_steel,
        name="right_track_floor",
    )
    frame.visual(
        Box((0.018, 0.460, 0.028)),
        origin=Origin(xyz=(-0.320, 0.160, 0.026)),
        material=track_steel,
        name="left_inner_guide",
    )
    frame.visual(
        Box((0.018, 0.460, 0.028)),
        origin=Origin(xyz=(0.320, 0.160, 0.026)),
        material=track_steel,
        name="right_inner_guide",
    )
    frame.visual(
        Box((0.026, 0.780, 0.032)),
        origin=Origin(xyz=(-0.401, 0.020, 0.028)),
        material=track_steel,
        name="left_outer_railwall",
    )
    frame.visual(
        Box((0.026, 0.780, 0.032)),
        origin=Origin(xyz=(0.401, 0.020, 0.028)),
        material=track_steel,
        name="right_outer_railwall",
    )
    frame.visual(
        Box((panel_width, 0.014, 0.004)),
        origin=Origin(xyz=(0.0, -0.327, 0.038)),
        material=seal_rubber,
        name="front_seal_strip",
    )
    frame.visual(
        Box((panel_width, 0.014, 0.004)),
        origin=Origin(xyz=(0.0, 0.327, 0.038)),
        material=seal_rubber,
        name="rear_seal_strip",
    )
    frame.visual(
        Box((0.014, panel_length, 0.004)),
        origin=Origin(xyz=(-0.307, 0.0, 0.038)),
        material=seal_rubber,
        name="left_seal_strip",
    )
    frame.visual(
        Box((0.014, panel_length, 0.004)),
        origin=Origin(xyz=(0.307, 0.0, 0.038)),
        material=seal_rubber,
        name="right_seal_strip",
    )

    carriage = model.part("carriage")
    carriage.inertial = Inertial.from_geometry(
        Box((0.720, 0.520, 0.050)),
        mass=1.7,
        origin=Origin(xyz=(0.0, 0.240, 0.025)),
    )
    carriage.visual(
        Box((0.042, 0.460, 0.007)),
        origin=Origin(xyz=(-0.3585, 0.180, 0.0035)),
        material=track_steel,
        name="left_runner",
    )
    carriage.visual(
        Box((0.042, 0.460, 0.007)),
        origin=Origin(xyz=(0.3585, 0.180, 0.0035)),
        material=track_steel,
        name="right_runner",
    )
    carriage.visual(
        Box((0.675, 0.040, 0.007)),
        origin=Origin(xyz=(0.0, 0.020, 0.0035)),
        material=track_steel,
        name="cross_tie",
    )
    carriage.visual(
        Box((0.024, 0.036, 0.015)),
        origin=Origin(xyz=(-0.220, 0.022, 0.0135)),
        material=track_steel,
        name="left_pivot_post",
    )
    carriage.visual(
        Box((0.024, 0.036, 0.015)),
        origin=Origin(xyz=(0.220, 0.022, 0.0135)),
        material=track_steel,
        name="right_pivot_post",
    )
    carriage.visual(
        Cylinder(radius=0.0025, length=0.054),
        origin=Origin(xyz=(-0.220, 0.004, 0.022), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=track_steel,
        name="left_hinge_pin",
    )
    carriage.visual(
        Cylinder(radius=0.0025, length=0.054),
        origin=Origin(xyz=(0.220, 0.004, 0.022), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=track_steel,
        name="right_hinge_pin",
    )

    glass = model.part("glass_panel")
    glass.inertial = Inertial.from_geometry(
        Box((panel_width, panel_length, 0.008)),
        mass=5.2,
        origin=Origin(xyz=(0.0, 0.320, 0.004)),
    )
    glass.visual(
        Box((panel_width, panel_length, 0.008)),
        origin=Origin(xyz=(0.0, 0.320, 0.004)),
        material=glass_tint,
        name="glass_plate",
    )
    glass.visual(
        Box((panel_width - 0.040, 0.030, 0.001)),
        origin=Origin(xyz=(0.0, 0.015, 0.0005)),
        material=glass_tint,
        name="front_edge_probe",
    )
    glass.visual(
        Box((panel_width - 0.040, 0.030, 0.001)),
        origin=Origin(xyz=(0.0, 0.625, 0.0005)),
        material=glass_tint,
        name="rear_edge_probe",
    )
    glass.visual(
        Box((panel_width, 0.050, 0.0015)),
        origin=Origin(xyz=(0.0, 0.025, 0.0085)),
        material=glass_frit,
        name="front_frit_band",
    )
    glass.visual(
        Box((panel_width, 0.050, 0.0015)),
        origin=Origin(xyz=(0.0, 0.615, 0.0085)),
        material=glass_frit,
        name="rear_frit_band",
    )
    glass.visual(
        Box((0.036, panel_length - 0.100, 0.0015)),
        origin=Origin(xyz=(-(panel_width * 0.5) + 0.018, 0.320, 0.0085)),
        material=glass_frit,
        name="left_frit_band",
    )
    glass.visual(
        Box((0.036, panel_length - 0.100, 0.0015)),
        origin=Origin(xyz=((panel_width * 0.5) - 0.018, 0.320, 0.0085)),
        material=glass_frit,
        name="right_frit_band",
    )
    glass.visual(
        Box((0.400, 0.024, 0.006)),
        origin=Origin(xyz=(0.0, 0.012, -0.002)),
        material=glass_frit,
        name="front_hinge_strip",
    )
    glass.visual(
        Cylinder(radius=0.0030, length=0.058),
        origin=Origin(xyz=(-0.220, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass_frit,
        name="left_hinge_barrel",
    )
    glass.visual(
        Cylinder(radius=0.0030, length=0.058),
        origin=Origin(xyz=(0.220, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass_frit,
        name="right_hinge_barrel",
    )

    model.articulation(
        "frame_to_carriage_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(0.0, slider_origin_y, 0.018)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.25, lower=0.0, upper=0.200),
    )
    model.articulation(
        "carriage_to_glass_vent",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=glass,
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=0.6, lower=0.0, upper=0.085),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)

    frame = object_model.get_part("frame")
    carriage = object_model.get_part("carriage")
    glass = object_model.get_part("glass_panel")
    slide = object_model.get_articulation("frame_to_carriage_slide")
    vent = object_model.get_articulation("carriage_to_glass_vent")

    left_track_floor = frame.get_visual("left_track_floor")
    right_track_floor = frame.get_visual("right_track_floor")
    left_inner_guide = frame.get_visual("left_inner_guide")
    right_inner_guide = frame.get_visual("right_inner_guide")
    left_outer_railwall = frame.get_visual("left_outer_railwall")
    right_outer_railwall = frame.get_visual("right_outer_railwall")
    front_seal = frame.get_visual("front_seal_strip")
    rear_seal = frame.get_visual("rear_seal_strip")
    left_seal = frame.get_visual("left_seal_strip")
    right_seal = frame.get_visual("right_seal_strip")

    left_runner = carriage.get_visual("left_runner")
    right_runner = carriage.get_visual("right_runner")
    left_hinge_pin = carriage.get_visual("left_hinge_pin")
    right_hinge_pin = carriage.get_visual("right_hinge_pin")

    glass_plate = glass.get_visual("glass_plate")
    front_edge_probe = glass.get_visual("front_edge_probe")
    rear_edge_probe = glass.get_visual("rear_edge_probe")
    front_hinge_strip = glass.get_visual("front_hinge_strip")
    left_hinge_barrel = glass.get_visual("left_hinge_barrel")
    right_hinge_barrel = glass.get_visual("right_hinge_barrel")

    slide_limits = slide.motion_limits
    vent_limits = vent.motion_limits

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.check(
        "slide articulation axis is longitudinal",
        tuple(slide.axis) == (0.0, 1.0, 0.0),
        details=f"expected (0, 1, 0), got {slide.axis}",
    )
    ctx.check(
        "vent articulation axis is transverse",
        tuple(vent.axis) == (1.0, 0.0, 0.0),
        details=f"expected (1, 0, 0), got {vent.axis}",
    )
    ctx.check(
        "slide travel is realistic",
        slide_limits is not None and slide_limits.upper is not None and 0.18 <= slide_limits.upper <= 0.22,
        details="sunroof panel should slide rearward by roughly 0.2 m",
    )
    ctx.check(
        "vent angle is realistic",
        vent_limits is not None and vent_limits.upper is not None and 0.06 <= vent_limits.upper <= 0.10,
        details="sunroof vent angle should be a shallow pop-up, not a full hatch opening",
    )

    ctx.allow_overlap(
        carriage,
        glass,
        elem_a=left_hinge_pin,
        elem_b=left_hinge_barrel,
        reason="The left hinge barrel is represented solid rather than bored through, so the pin occupies the sleeve volume.",
    )
    ctx.allow_overlap(
        carriage,
        glass,
        elem_a=right_hinge_pin,
        elem_b=right_hinge_barrel,
        reason="The right hinge barrel is represented solid rather than bored through, so the pin occupies the sleeve volume.",
    )
    ctx.allow_overlap(
        carriage,
        glass,
        elem_a=left_hinge_pin,
        elem_b=front_hinge_strip,
        reason="The front hinge reinforcement strip is modeled without the local pin pass-through cutout, so the left pin intentionally occupies that hidden bore region.",
    )
    ctx.allow_overlap(
        carriage,
        glass,
        elem_a=right_hinge_pin,
        elem_b=front_hinge_strip,
        reason="The front hinge reinforcement strip is modeled without the local pin pass-through cutout, so the right pin intentionally occupies that hidden bore region.",
    )

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_gap(
        carriage,
        frame,
        axis="x",
        positive_elem=left_runner,
        negative_elem=left_outer_railwall,
        min_gap=0.006,
        max_gap=0.014,
        name="left runner clears outer rail wall",
    )
    ctx.expect_gap(
        frame,
        carriage,
        axis="x",
        positive_elem=left_inner_guide,
        negative_elem=left_runner,
        min_gap=0.006,
        max_gap=0.014,
        name="left runner clears inner guide",
    )
    ctx.expect_gap(
        carriage,
        frame,
        axis="x",
        positive_elem=right_runner,
        negative_elem=right_inner_guide,
        min_gap=0.006,
        max_gap=0.014,
        name="right runner clears inner guide",
    )
    ctx.expect_gap(
        frame,
        carriage,
        axis="x",
        positive_elem=right_outer_railwall,
        negative_elem=right_runner,
        min_gap=0.006,
        max_gap=0.014,
        name="right runner clears outer rail wall",
    )

    with ctx.pose({slide: 0.0, vent: 0.0}):
        ctx.fail_if_isolated_parts(name="closed_pose_no_floating")
        ctx.fail_if_parts_overlap_in_current_pose(name="closed_pose_no_overlap")
        ctx.expect_within(carriage, frame, axes="xy", inner_elem=left_runner, outer_elem=left_track_floor)
        ctx.expect_within(carriage, frame, axes="xy", inner_elem=right_runner, outer_elem=right_track_floor)
        ctx.expect_gap(
            carriage,
            frame,
            axis="z",
            positive_elem=left_runner,
            negative_elem=left_track_floor,
            max_gap=0.001,
            max_penetration=0.0,
            name="left runner rides on track floor",
        )
        ctx.expect_gap(
            carriage,
            frame,
            axis="z",
            positive_elem=right_runner,
            negative_elem=right_track_floor,
            max_gap=0.001,
            max_penetration=0.0,
            name="right runner rides on track floor",
        )
        ctx.expect_within(glass, frame, axes="xy")
        ctx.expect_gap(
            glass,
            frame,
            axis="z",
            positive_elem=front_edge_probe,
            negative_elem=front_seal,
            max_gap=0.0015,
            max_penetration=1e-6,
            name="front edge sits on front seal",
        )
        ctx.expect_gap(
            glass,
            frame,
            axis="z",
            positive_elem=rear_edge_probe,
            negative_elem=rear_seal,
            max_gap=0.0015,
            max_penetration=1e-6,
            name="rear edge sits on rear seal",
        )
        ctx.expect_gap(
            glass,
            frame,
            axis="z",
            positive_elem=glass_plate,
            negative_elem=left_seal,
            max_gap=0.0015,
            max_penetration=1e-6,
            name="left edge sits on left seal",
        )
        ctx.expect_gap(
            glass,
            frame,
            axis="z",
            positive_elem=glass_plate,
            negative_elem=right_seal,
            max_gap=0.0015,
            max_penetration=1e-6,
            name="right edge sits on right seal",
        )
        ctx.expect_contact(
            carriage,
            glass,
            elem_a=left_hinge_pin,
            elem_b=left_hinge_barrel,
            name="left hinge pin stays captured",
        )
        ctx.expect_contact(
            carriage,
            glass,
            elem_a=right_hinge_pin,
            elem_b=right_hinge_barrel,
            name="right hinge pin stays captured",
        )

    with ctx.pose({slide: 0.0, vent: 0.085}):
        ctx.fail_if_isolated_parts(name="vent_pose_no_floating")
        ctx.fail_if_parts_overlap_in_current_pose(name="vent_pose_no_overlap")
        ctx.expect_contact(carriage, glass, elem_a=left_hinge_pin, elem_b=left_hinge_barrel)
        ctx.expect_contact(carriage, glass, elem_a=right_hinge_pin, elem_b=right_hinge_barrel)
        ctx.expect_gap(
            glass,
            frame,
            axis="z",
            positive_elem=front_edge_probe,
            negative_elem=front_seal,
            max_gap=0.002,
            max_penetration=0.001,
            name="front edge remains seated while vented",
        )
        ctx.expect_gap(
            glass,
            frame,
            axis="z",
            positive_elem=rear_edge_probe,
            negative_elem=rear_seal,
            min_gap=0.040,
            name="rear edge lifts in vent position",
        )

    with ctx.pose({slide: 0.200, vent: 0.0}):
        ctx.fail_if_isolated_parts(name="slide_pose_no_floating")
        ctx.fail_if_parts_overlap_in_current_pose(name="slide_pose_no_overlap")
        ctx.expect_within(carriage, frame, axes="xy", inner_elem=left_runner, outer_elem=left_track_floor)
        ctx.expect_within(carriage, frame, axes="xy", inner_elem=right_runner, outer_elem=right_track_floor)
        ctx.expect_contact(carriage, glass, elem_a=left_hinge_pin, elem_b=left_hinge_barrel)
        ctx.expect_contact(carriage, glass, elem_a=right_hinge_pin, elem_b=right_hinge_barrel)
        ctx.expect_gap(
            glass,
            frame,
            axis="y",
            positive_elem=front_edge_probe,
            negative_elem=front_seal,
            min_gap=0.180,
            name="panel front edge translates rearward out of the opening",
        )
        ctx.expect_gap(
            glass,
            frame,
            axis="y",
            positive_elem=rear_edge_probe,
            negative_elem=rear_seal,
            min_gap=0.150,
            name="panel rear edge projects behind the rear seal when slid open",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
