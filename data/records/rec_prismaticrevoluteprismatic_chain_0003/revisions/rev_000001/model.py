from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
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

BASE_LENGTH = 0.46
BASE_WIDTH = 0.19
BASE_THICK = 0.018
PRIMARY_PEDESTAL_HEIGHT = 0.018
PRIMARY_RAIL_HEIGHT = 0.010
PRIMARY_RAIL_WIDTH = 0.020
PRIMARY_RAIL_CENTER_Y = 0.055
PRIMARY_RAIL_LENGTH = 0.39
COVER_LEDGE_HEIGHT = 0.014

PRIMARY_TRAVEL = 0.18
SECONDARY_TRAVEL = 0.09

CARRIAGE_LENGTH = 0.110
CARRIAGE_WIDTH = 0.156
CARRIAGE_BODY_HEIGHT = 0.016
CARRIAGE_GUIDE_HEIGHT = 0.024
CARRIAGE_SIDE_PLATE_THICK = 0.012
ELBOW_PIVOT_X = 0.028
ELBOW_PIVOT_Z = 0.100

HUB_RADIUS = 0.022
HUB_WIDTH = 0.046

ARM_BEAM_LENGTH = 0.215
ARM_BEAM_WIDTH = 0.072
ARM_BEAM_HEIGHT = 0.050
SECONDARY_RAIL_CENTER_Y = 0.023
SECONDARY_RAIL_WIDTH = 0.012
SECONDARY_RAIL_HEIGHT = 0.010
SECONDARY_RAIL_LENGTH = 0.150
SECONDARY_RAIL_BOTTOM_Z = 0.035

BASE_RAIL_TOP_Z = BASE_THICK + PRIMARY_PEDESTAL_HEIGHT + PRIMARY_RAIL_HEIGHT


def _box(
    x: float,
    y: float,
    z: float,
    *,
    centered: tuple[bool, bool, bool] = (True, True, False),
    translate: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> cq.Workplane:
    return cq.Workplane("XY").box(x, y, z, centered=centered).translate(translate)


def _y_cylinder(radius: float, length: float, *, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XZ").circle(radius).extrude(length, both=True).translate(center)


def _mesh(shape: cq.Workplane, filename: str):
    return mesh_from_cadquery(shape, filename, assets=ASSETS)


def _build_base_geometry() -> dict[str, cq.Workplane]:
    frame_main = _box(BASE_LENGTH, BASE_WIDTH, BASE_THICK)
    frame_main = frame_main.union(
        _box(
            PRIMARY_RAIL_LENGTH,
            0.038,
            PRIMARY_PEDESTAL_HEIGHT,
            translate=(0.0, PRIMARY_RAIL_CENTER_Y, BASE_THICK),
        )
    )
    frame_main = frame_main.union(
        _box(
            PRIMARY_RAIL_LENGTH,
            0.038,
            PRIMARY_PEDESTAL_HEIGHT,
            translate=(0.0, -PRIMARY_RAIL_CENTER_Y, BASE_THICK),
        )
    )
    frame_main = frame_main.union(
        _box(
            0.024,
            BASE_WIDTH - 0.020,
            0.050,
            translate=(-BASE_LENGTH / 2.0 + 0.020, 0.0, BASE_THICK),
        )
    )
    frame_main = frame_main.union(
        _box(
            0.024,
            BASE_WIDTH - 0.020,
            0.050,
            translate=(BASE_LENGTH / 2.0 - 0.020, 0.0, BASE_THICK),
        )
    )
    frame_main = frame_main.union(
        _box(
            0.220,
            0.012,
            0.008,
            translate=(0.0, 0.030, BASE_THICK + COVER_LEDGE_HEIGHT),
        )
    )
    frame_main = frame_main.union(
        _box(
            0.220,
            0.012,
            0.008,
            translate=(0.0, -0.030, BASE_THICK + COVER_LEDGE_HEIGHT),
        )
    )
    frame_main = frame_main.union(
        _box(
            0.080,
            0.060,
            0.022,
            translate=(-BASE_LENGTH / 2.0 + 0.060, 0.0, BASE_THICK),
        )
    )
    frame_main = frame_main.union(
        _box(
            0.080,
            0.060,
            0.022,
            translate=(BASE_LENGTH / 2.0 - 0.060, 0.0, BASE_THICK),
        )
    )

    rail_left = _box(
        PRIMARY_RAIL_LENGTH,
        PRIMARY_RAIL_WIDTH,
        PRIMARY_RAIL_HEIGHT,
        translate=(0.0, PRIMARY_RAIL_CENTER_Y, BASE_THICK + PRIMARY_PEDESTAL_HEIGHT),
    )
    rail_right = _box(
        PRIMARY_RAIL_LENGTH,
        PRIMARY_RAIL_WIDTH,
        PRIMARY_RAIL_HEIGHT,
        translate=(0.0, -PRIMARY_RAIL_CENTER_Y, BASE_THICK + PRIMARY_PEDESTAL_HEIGHT),
    )
    bumper_rear = _box(
        0.012,
        0.050,
        0.016,
        translate=(-BASE_LENGTH / 2.0 + 0.046, 0.0, BASE_RAIL_TOP_Z),
    )
    bumper_front = _box(
        0.012,
        0.050,
        0.016,
        translate=(BASE_LENGTH / 2.0 - 0.046, 0.0, BASE_RAIL_TOP_Z),
    )
    return {
        "frame_main": frame_main,
        "rail_left": rail_left,
        "rail_right": rail_right,
        "bumper_front": bumper_front,
        "bumper_rear": bumper_rear,
    }


def _build_base_cover() -> cq.Workplane:
    top_plate = _box(0.180, 0.060, 0.004, translate=(0.0, 0.0, 0.007))
    left_flange = _box(0.180, 0.008, 0.007, translate=(0.0, 0.026, 0.0))
    right_flange = _box(0.180, 0.008, 0.007, translate=(0.0, -0.026, 0.0))
    return top_plate.union(left_flange).union(right_flange)


def _build_carriage_geometry() -> dict[str, cq.Workplane]:
    saddle = _box(CARRIAGE_LENGTH, CARRIAGE_WIDTH, CARRIAGE_BODY_HEIGHT)
    guide_left = _box(
        0.094,
        0.012,
        CARRIAGE_GUIDE_HEIGHT,
        translate=(0.0, 0.082, 0.0),
    )
    guide_right = _box(
        0.094,
        0.012,
        CARRIAGE_GUIDE_HEIGHT,
        translate=(0.0, -0.082, 0.0),
    )
    body = saddle.union(guide_left).union(guide_right)
    body = body.union(_box(0.050, 0.040, 0.040, translate=(0.016, 0.0, CARRIAGE_BODY_HEIGHT)))
    body = body.union(_box(0.022, 0.110, 0.018, translate=(-0.032, 0.0, CARRIAGE_BODY_HEIGHT)))
    body = body.union(_box(0.016, 0.040, 0.028, translate=(CARRIAGE_LENGTH / 2.0 - 0.014, 0.0, 0.0)))

    side_plate_left = _box(
        0.054,
        CARRIAGE_SIDE_PLATE_THICK,
        0.106,
        translate=(ELBOW_PIVOT_X, 0.032, CARRIAGE_BODY_HEIGHT),
    )
    side_plate_right = _box(
        0.054,
        CARRIAGE_SIDE_PLATE_THICK,
        0.106,
        translate=(ELBOW_PIVOT_X, -0.032, CARRIAGE_BODY_HEIGHT),
    )
    return {
        "carriage_body": body,
        "side_plate_left": side_plate_left,
        "side_plate_right": side_plate_right,
    }


def _build_arm_geometry() -> dict[str, cq.Workplane]:
    beam = _box(
        ARM_BEAM_LENGTH,
        ARM_BEAM_WIDTH,
        ARM_BEAM_HEIGHT,
        centered=(False, True, True),
        translate=(0.024 + ARM_BEAM_LENGTH / 2.0, 0.0, -ARM_BEAM_HEIGHT / 2.0),
    )
    root_block = _box(
        0.055,
        0.062,
        0.070,
        centered=(False, True, True),
        translate=(0.0 + 0.055 / 2.0, 0.0, -0.035),
    )
    rear_guard = _box(
        0.018,
        0.082,
        0.060,
        centered=(False, True, True),
        translate=(-0.010 + 0.018 / 2.0, 0.0, -0.030),
    )
    arm_body = beam.union(root_block).union(rear_guard)
    arm_body = arm_body.union(
        _box(
            0.020,
            0.068,
            0.020,
            centered=(False, True, True),
            translate=(0.236, 0.0, 0.000),
        )
    )

    hub = _y_cylinder(HUB_RADIUS, HUB_WIDTH, center=(0.0, 0.0, 0.0))
    hub = hub.union(_y_cylinder(HUB_RADIUS + 0.004, 0.004, center=(0.0, HUB_WIDTH / 2.0 - 0.002, 0.0)))
    hub = hub.union(_y_cylinder(HUB_RADIUS + 0.004, 0.004, center=(0.0, -HUB_WIDTH / 2.0 + 0.002, 0.0)))

    sec_rail_left = _box(
        SECONDARY_RAIL_LENGTH,
        SECONDARY_RAIL_WIDTH,
        SECONDARY_RAIL_HEIGHT,
        centered=(False, True, False),
        translate=(0.070, SECONDARY_RAIL_CENTER_Y, SECONDARY_RAIL_BOTTOM_Z),
    )
    sec_rail_right = _box(
        SECONDARY_RAIL_LENGTH,
        SECONDARY_RAIL_WIDTH,
        SECONDARY_RAIL_HEIGHT,
        centered=(False, True, False),
        translate=(0.070, -SECONDARY_RAIL_CENTER_Y, SECONDARY_RAIL_BOTTOM_Z),
    )
    return {
        "arm_body": arm_body,
        "hub": hub,
        "sec_rail_left": sec_rail_left,
        "sec_rail_right": sec_rail_right,
    }


def _build_secondary_slider() -> cq.Workplane:
    body = _box(
        0.118,
        0.056,
        0.014,
        centered=(False, True, False),
        translate=(-0.018, 0.0, 0.0),
    )
    guide_left = _box(
        0.082,
        0.010,
        0.020,
        centered=(False, True, False),
        translate=(-0.004, 0.036, 0.0),
    )
    guide_right = _box(
        0.082,
        0.010,
        0.020,
        centered=(False, True, False),
        translate=(-0.004, -0.036, 0.0),
    )
    front_plate = _box(
        0.040,
        0.072,
        0.024,
        centered=(False, True, False),
        translate=(0.092, 0.0, 0.0),
    )
    bumper = _box(
        0.016,
        0.044,
        0.010,
        centered=(False, True, False),
        translate=(-0.030, 0.0, 0.002),
    )
    return body.union(guide_left).union(guide_right).union(front_plate).union(bumper)


def _build_secondary_cover() -> cq.Workplane:
    top = _box(0.122, 0.070, 0.003, translate=(0.0, 0.0, 0.016))
    foot_left = _box(0.122, 0.010, 0.016, translate=(0.0, 0.030, 0.0))
    foot_right = _box(0.122, 0.010, 0.016, translate=(0.0, -0.030, 0.0))
    return top.union(foot_left).union(foot_right)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="prismatic_revolute_prismatic_study", assets=ASSETS)

    steel_dark = model.material("steel_dark", rgba=(0.28, 0.30, 0.33, 1.0))
    steel_mid = model.material("steel_mid", rgba=(0.52, 0.55, 0.58, 1.0))
    steel_light = model.material("steel_light", rgba=(0.74, 0.76, 0.79, 1.0))
    cover_gray = model.material("cover_gray", rgba=(0.64, 0.66, 0.69, 1.0))
    bumper_black = model.material("bumper_black", rgba=(0.10, 0.10, 0.11, 1.0))

    base_shapes = _build_base_geometry()
    carriage_shapes = _build_carriage_geometry()
    arm_shapes = _build_arm_geometry()

    base_frame = model.part("base_frame")
    base_frame.visual(_mesh(base_shapes["frame_main"], "base_frame_main.obj"), material=steel_dark, name="frame_main")
    base_frame.visual(_mesh(base_shapes["rail_left"], "base_rail_left.obj"), material=steel_light, name="rail_left")
    base_frame.visual(_mesh(base_shapes["rail_right"], "base_rail_right.obj"), material=steel_light, name="rail_right")
    base_frame.visual(_mesh(base_shapes["bumper_front"], "base_bumper_front.obj"), material=bumper_black, name="bumper_front")
    base_frame.visual(_mesh(base_shapes["bumper_rear"], "base_bumper_rear.obj"), material=bumper_black, name="bumper_rear")
    base_frame.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, 0.090)),
        mass=10.0,
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
    )

    base_cover = model.part("base_cover")
    base_cover.visual(_mesh(_build_base_cover(), "base_cover.obj"), material=cover_gray, name="cover_shell")
    base_cover.inertial = Inertial.from_geometry(
        Box((0.180, 0.060, 0.014)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
    )

    primary_carriage = model.part("primary_carriage")
    primary_carriage.visual(
        _mesh(carriage_shapes["carriage_body"], "primary_carriage_body.obj"),
        material=steel_mid,
        name="carriage_body",
    )
    primary_carriage.visual(
        _mesh(carriage_shapes["side_plate_left"], "primary_carriage_side_left.obj"),
        material=steel_light,
        name="side_plate_left",
    )
    primary_carriage.visual(
        _mesh(carriage_shapes["side_plate_right"], "primary_carriage_side_right.obj"),
        material=steel_light,
        name="side_plate_right",
    )
    primary_carriage.inertial = Inertial.from_geometry(
        Box((CARRIAGE_LENGTH, CARRIAGE_WIDTH, 0.130)),
        mass=4.0,
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
    )

    elbow_arm = model.part("elbow_arm")
    elbow_arm.visual(_mesh(arm_shapes["arm_body"], "elbow_arm_body.obj"), material=steel_dark, name="arm_body")
    elbow_arm.visual(_mesh(arm_shapes["hub"], "elbow_arm_hub.obj"), material=steel_light, name="hub")
    elbow_arm.visual(
        _mesh(arm_shapes["sec_rail_left"], "elbow_arm_sec_rail_left.obj"),
        material=steel_light,
        name="sec_rail_left",
    )
    elbow_arm.visual(
        _mesh(arm_shapes["sec_rail_right"], "elbow_arm_sec_rail_right.obj"),
        material=steel_light,
        name="sec_rail_right",
    )
    elbow_arm.inertial = Inertial.from_geometry(
        Box((0.280, 0.082, 0.080)),
        mass=5.2,
        origin=Origin(xyz=(0.130, 0.0, 0.0)),
    )

    secondary_slider = model.part("secondary_slider")
    secondary_slider.visual(_mesh(_build_secondary_slider(), "secondary_slider.obj"), material=steel_mid, name="slider_body")
    secondary_slider.inertial = Inertial.from_geometry(
        Box((0.158, 0.072, 0.024)),
        mass=2.3,
        origin=Origin(xyz=(0.050, 0.0, 0.012)),
    )

    secondary_cover = model.part("secondary_cover")
    secondary_cover.visual(_mesh(_build_secondary_cover(), "secondary_cover.obj"), material=cover_gray, name="cover_shell")
    secondary_cover.inertial = Inertial.from_geometry(
        Box((0.122, 0.070, 0.019)),
        mass=0.6,
        origin=Origin(xyz=(0.0, 0.0, 0.0095)),
    )

    model.articulation(
        "base_to_cover",
        ArticulationType.FIXED,
        parent=base_frame,
        child=base_cover,
        origin=Origin(xyz=(0.0, 0.0, BASE_THICK + COVER_LEDGE_HEIGHT + 0.008)),
    )
    model.articulation(
        "primary_slide",
        ArticulationType.PRISMATIC,
        parent=base_frame,
        child=primary_carriage,
        origin=Origin(xyz=(-0.120, 0.0, BASE_RAIL_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1200.0, velocity=0.40, lower=0.0, upper=PRIMARY_TRAVEL),
    )
    model.articulation(
        "carriage_to_arm",
        ArticulationType.REVOLUTE,
        parent=primary_carriage,
        child=elbow_arm,
        origin=Origin(xyz=(ELBOW_PIVOT_X, 0.0, ELBOW_PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=450.0, velocity=1.2, lower=-0.45, upper=1.15),
    )
    model.articulation(
        "arm_to_slider",
        ArticulationType.PRISMATIC,
        parent=elbow_arm,
        child=secondary_slider,
        origin=Origin(xyz=(0.075, 0.0, SECONDARY_RAIL_BOTTOM_Z + SECONDARY_RAIL_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=600.0, velocity=0.35, lower=0.0, upper=SECONDARY_TRAVEL),
    )
    model.articulation(
        "arm_to_secondary_cover",
        ArticulationType.FIXED,
        parent=elbow_arm,
        child=secondary_cover,
        origin=Origin(xyz=(0.138, 0.0, SECONDARY_RAIL_BOTTOM_Z + SECONDARY_RAIL_HEIGHT)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base_frame = object_model.get_part("base_frame")
    base_cover = object_model.get_part("base_cover")
    primary_carriage = object_model.get_part("primary_carriage")
    elbow_arm = object_model.get_part("elbow_arm")
    secondary_slider = object_model.get_part("secondary_slider")
    secondary_cover = object_model.get_part("secondary_cover")

    base_to_cover = object_model.get_articulation("base_to_cover")
    primary_slide = object_model.get_articulation("primary_slide")
    carriage_to_arm = object_model.get_articulation("carriage_to_arm")
    arm_to_slider = object_model.get_articulation("arm_to_slider")
    arm_to_secondary_cover = object_model.get_articulation("arm_to_secondary_cover")

    frame_main = base_frame.get_visual("frame_main")
    rail_left = base_frame.get_visual("rail_left")
    rail_right = base_frame.get_visual("rail_right")
    cover_shell = base_cover.get_visual("cover_shell")
    carriage_body = primary_carriage.get_visual("carriage_body")
    side_plate_left = primary_carriage.get_visual("side_plate_left")
    side_plate_right = primary_carriage.get_visual("side_plate_right")
    arm_body = elbow_arm.get_visual("arm_body")
    hub = elbow_arm.get_visual("hub")
    sec_rail_left = elbow_arm.get_visual("sec_rail_left")
    sec_rail_right = elbow_arm.get_visual("sec_rail_right")
    slider_body = secondary_slider.get_visual("slider_body")
    secondary_cover_shell = secondary_cover.get_visual("cover_shell")

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
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "part_and_joint_inventory",
        all(
            item is not None
            for item in (
                base_frame,
                base_cover,
                primary_carriage,
                elbow_arm,
                secondary_slider,
                secondary_cover,
                base_to_cover,
                primary_slide,
                carriage_to_arm,
                arm_to_slider,
                arm_to_secondary_cover,
            )
        ),
        "Expected all study parts and articulations to exist.",
    )

    ctx.expect_contact(base_cover, base_frame, elem_a=cover_shell, elem_b=frame_main, contact_tol=0.0005)
    ctx.expect_overlap(base_cover, base_frame, axes="xy", elem_a=cover_shell, elem_b=frame_main, min_overlap=0.045)

    ctx.expect_gap(
        primary_carriage,
        base_frame,
        axis="z",
        positive_elem=carriage_body,
        negative_elem=rail_left,
        max_gap=0.001,
        max_penetration=0.0,
        name="carriage_seated_on_left_rail",
    )
    ctx.expect_gap(
        primary_carriage,
        base_frame,
        axis="z",
        positive_elem=carriage_body,
        negative_elem=rail_right,
        max_gap=0.001,
        max_penetration=0.0,
        name="carriage_seated_on_right_rail",
    )
    ctx.expect_within(primary_carriage, base_frame, axes="y", margin=0.010)

    ctx.expect_gap(
        primary_carriage,
        elbow_arm,
        axis="y",
        positive_elem=side_plate_left,
        negative_elem=hub,
        min_gap=0.001,
        max_gap=0.006,
        name="left_side_plate_clears_hub",
    )
    ctx.expect_gap(
        elbow_arm,
        primary_carriage,
        axis="y",
        positive_elem=hub,
        negative_elem=side_plate_right,
        min_gap=0.001,
        max_gap=0.006,
        name="right_side_plate_clears_hub",
    )
    ctx.expect_overlap(primary_carriage, elbow_arm, axes="z", elem_a=side_plate_left, elem_b=hub, min_overlap=0.030)

    ctx.expect_gap(
        secondary_slider,
        elbow_arm,
        axis="z",
        positive_elem=slider_body,
        negative_elem=sec_rail_left,
        max_gap=0.001,
        max_penetration=0.0,
        name="slider_seated_on_left_secondary_rail",
    )
    ctx.expect_gap(
        secondary_slider,
        elbow_arm,
        axis="z",
        positive_elem=slider_body,
        negative_elem=sec_rail_right,
        max_gap=0.001,
        max_penetration=0.0,
        name="slider_seated_on_right_secondary_rail",
    )
    ctx.expect_contact(secondary_cover, elbow_arm, elem_a=secondary_cover_shell, elem_b=arm_body, contact_tol=0.0005)
    ctx.expect_gap(
        secondary_cover,
        secondary_slider,
        axis="z",
        positive_elem=secondary_cover_shell,
        negative_elem=slider_body,
        min_gap=0.0015,
        max_gap=0.010,
        name="secondary_cover_bridges_over_slider",
    )

    rest_carriage = ctx.part_world_position(primary_carriage)
    with ctx.pose({primary_slide: 0.140}):
        moved_carriage = ctx.part_world_position(primary_carriage)
        ctx.expect_gap(
            primary_carriage,
            base_frame,
            axis="z",
            positive_elem=carriage_body,
            negative_elem=rail_left,
            max_gap=0.001,
            max_penetration=0.0,
            name="carriage_keeps_left_rail_contact_when_extended",
        )
        ctx.expect_gap(
            primary_carriage,
            base_frame,
            axis="z",
            positive_elem=carriage_body,
            negative_elem=rail_right,
            max_gap=0.001,
            max_penetration=0.0,
            name="carriage_keeps_right_rail_contact_when_extended",
        )
    ctx.check(
        "primary_stage_translates_along_x",
        rest_carriage is not None
        and moved_carriage is not None
        and moved_carriage[0] > rest_carriage[0] + 0.10
        and abs(moved_carriage[1] - rest_carriage[1]) < 1e-6
        and abs(moved_carriage[2] - rest_carriage[2]) < 1e-6,
        f"rest={rest_carriage}, moved={moved_carriage}",
    )

    rest_arm_aabb = ctx.part_element_world_aabb(elbow_arm, elem=arm_body)
    with ctx.pose({carriage_to_arm: 0.85}):
        raised_arm_aabb = ctx.part_element_world_aabb(elbow_arm, elem=arm_body)
        ctx.expect_gap(
            elbow_arm,
            base_frame,
            axis="z",
            positive_elem=arm_body,
            negative_elem=frame_main,
            min_gap=0.020,
            name="arm_clears_base_when_raised",
        )
    ctx.check(
        "revolute_elbow_lifts_forearm",
        rest_arm_aabb is not None
        and raised_arm_aabb is not None
        and raised_arm_aabb[1][2] > rest_arm_aabb[1][2] + 0.070,
        f"rest_aabb={rest_arm_aabb}, raised_aabb={raised_arm_aabb}",
    )

    with ctx.pose({carriage_to_arm: 0.30}):
        rest_slider = ctx.part_world_position(secondary_slider)
    with ctx.pose({carriage_to_arm: 0.30, arm_to_slider: 0.070}):
        extended_slider = ctx.part_world_position(secondary_slider)
        ctx.expect_gap(
            secondary_cover,
            secondary_slider,
            axis="z",
            positive_elem=secondary_cover_shell,
            negative_elem=slider_body,
            min_gap=0.0015,
            max_gap=0.010,
            name="secondary_cover_clearance_when_slider_extended",
        )
    ctx.check(
        "secondary_stage_translates_along_local_beam_axis",
        rest_slider is not None
        and extended_slider is not None
        and extended_slider[0] > rest_slider[0] + 0.050,
        f"rest={rest_slider}, extended={extended_slider}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
