from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _rounded_deck_mesh():
    return (
        cq.Workplane("XY")
        .box(0.72, 0.155, 0.046)
        .edges("|Z")
        .fillet(0.030)
        .edges(">Z")
        .fillet(0.008)
        .edges("<Z")
        .fillet(0.004)
    )


def _head_tube_mesh():
    # Hollow bearing sleeve around the steering stem, authored open so the
    # yawing stem can pass through without colliding with a solid proxy.
    return cq.Workplane("XY").circle(0.040).circle(0.025).extrude(0.120)


def _wheel_meshes(prefix: str):
    rim = WheelGeometry(
        0.052,
        0.034,
        rim=WheelRim(
            inner_radius=0.034,
            flange_height=0.004,
            flange_thickness=0.0025,
            bead_seat_depth=0.002,
        ),
        hub=WheelHub(
            radius=0.020,
            width=0.026,
            cap_style="domed",
            bolt_pattern=BoltPattern(
                count=5,
                circle_diameter=0.026,
                hole_diameter=0.003,
            ),
        ),
        face=WheelFace(dish_depth=0.004, front_inset=0.002, rear_inset=0.002),
        spokes=WheelSpokes(style="split_y", count=5, thickness=0.0025, window_radius=0.007),
        bore=WheelBore(style="round", diameter=0.018),
    )
    tire = TireGeometry(
        0.075,
        0.038,
        inner_radius=0.052,
        carcass=TireCarcass(belt_width_ratio=0.72, sidewall_bulge=0.035),
        tread=TireTread(style="block", depth=0.0035, count=20, land_ratio=0.60),
        grooves=(TireGroove(center_offset=0.0, width=0.004, depth=0.002),),
        sidewall=TireSidewall(style="rounded", bulge=0.04),
        shoulder=TireShoulder(width=0.004, radius=0.0025),
    )
    return mesh_from_geometry(rim, f"{prefix}_rim"), mesh_from_geometry(tire, f"{prefix}_tire")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_travel_scooter")

    blue = model.material("satin_blue", rgba=(0.06, 0.18, 0.48, 1.0))
    grip_black = model.material("textured_black", rgba=(0.01, 0.011, 0.012, 1.0))
    rubber = model.material("soft_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    aluminum = model.material("brushed_aluminum", rgba=(0.72, 0.74, 0.72, 1.0))
    dark_metal = model.material("dark_anodized_metal", rgba=(0.12, 0.13, 0.14, 1.0))

    deck = model.part("deck")
    deck.visual(
        mesh_from_cadquery(_rounded_deck_mesh(), "rounded_deck"),
        origin=Origin(xyz=(0.035, 0.0, 0.126)),
        material=blue,
        name="deck_shell",
    )
    deck.visual(
        Box((0.54, 0.120, 0.006)),
        origin=Origin(xyz=(0.010, 0.0, 0.152)),
        material=grip_black,
        name="rubber_grip",
    )
    deck.visual(
        Box((0.070, 0.095, 0.026)),
        origin=Origin(xyz=(0.365, 0.0, 0.129)),
        material=blue,
        name="front_neck",
    )
    deck.visual(
        mesh_from_cadquery(_head_tube_mesh(), "head_tube"),
        origin=Origin(xyz=(0.435, 0.0, 0.130)),
        material=dark_metal,
        name="head_tube",
    )
    # Rear dropouts hold the rear axle just behind the slim deck.
    for side, y in (("near", -0.048), ("far", 0.048)):
        deck.visual(
            Box((0.170, 0.014, 0.050)),
            origin=Origin(xyz=(-0.390, y, 0.082)),
            material=dark_metal,
            name=f"rear_dropout_{side}",
        )
    deck.visual(
        Cylinder(radius=0.006, length=0.125),
        origin=Origin(xyz=(-0.445, 0.0, 0.075), rpy=(math.pi / 2, 0.0, 0.0)),
        material=aluminum,
        name="rear_axle",
    )

    steering = model.part("steering")
    steering.visual(
        Cylinder(radius=0.017, length=0.825),
        origin=Origin(xyz=(0.0, 0.0, 0.292)),
        material=aluminum,
        name="stem",
    )
    steering.visual(
        Cylinder(radius=0.034, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.031)),
        material=dark_metal,
        name="lower_bearing_collar",
    )
    steering.visual(
        Cylinder(radius=0.034, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.101)),
        material=dark_metal,
        name="upper_bearing_collar",
    )
    steering.visual(
        Box((0.070, 0.110, 0.038)),
        origin=Origin(xyz=(0.090, 0.0, 0.020)),
        material=dark_metal,
        name="fork_crown",
    )
    steering.visual(
        Cylinder(radius=0.005, length=0.108),
        origin=Origin(xyz=(0.0, 0.0, -0.031), rpy=(math.pi / 2, 0.0, 0.0)),
        material=dark_metal,
        name="fork_cross_tube",
    )
    brace_angle = math.atan2(0.060, 0.036)
    brace_len = math.sqrt(0.060 * 0.060 + 0.036 * 0.036)
    for side, y in (("near", -0.048), ("far", 0.048)):
        steering.visual(
            Cylinder(radius=0.005, length=brace_len),
            origin=Origin(
                xyz=(0.030, y, -0.013),
                rpy=(0.0, brace_angle, 0.0),
            ),
            material=dark_metal,
            name=f"fork_brace_{side}",
        )
    fork_angle = math.atan2(0.055, -0.085)
    fork_len = math.sqrt(0.055 * 0.055 + 0.085 * 0.085)
    for side, y in (("near", -0.042), ("far", 0.042)):
        steering.visual(
            Cylinder(radius=0.010, length=fork_len),
            origin=Origin(
                xyz=(0.0925, y, -0.0375),
                rpy=(0.0, fork_angle, 0.0),
            ),
            material=dark_metal,
            name=f"fork_blade_{side}",
        )
    steering.visual(
        Cylinder(radius=0.006, length=0.112),
        origin=Origin(xyz=(0.120, 0.0, -0.080), rpy=(math.pi / 2, 0.0, 0.0)),
        material=aluminum,
        name="front_axle",
    )
    steering.visual(
        Box((0.046, 0.120, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.700)),
        material=dark_metal,
        name="hinge_clamp",
    )
    steering.visual(
        Cylinder(radius=0.006, length=0.128),
        origin=Origin(xyz=(0.0, 0.0, 0.728), rpy=(math.pi / 2, 0.0, 0.0)),
        material=aluminum,
        name="handlebar_pin",
    )
    for side, y in (("near", -0.050), ("far", 0.050)):
        steering.visual(
            Box((0.038, 0.014, 0.060)),
            origin=Origin(xyz=(0.0, y, 0.728)),
            material=dark_metal,
            name=f"hinge_cheek_{side}",
        )

    handlebar = model.part("handlebar")
    handlebar.visual(
        Cylinder(radius=0.014, length=0.058),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
        material=dark_metal,
        name="hinge_lug",
    )
    handlebar.visual(
        Box((0.078, 0.030, 0.024)),
        origin=Origin(xyz=(0.049, 0.0, 0.0)),
        material=dark_metal,
        name="folding_arm",
    )
    handlebar.visual(
        Cylinder(radius=0.014, length=0.430),
        origin=Origin(xyz=(0.088, 0.0, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
        material=aluminum,
        name="crossbar_tube",
    )
    for side, y in (("near", -0.235), ("far", 0.235)):
        handlebar.visual(
            Cylinder(radius=0.018, length=0.080),
            origin=Origin(xyz=(0.088, y, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
            material=rubber,
            name=f"hand_grip_{side}",
        )

    front_wheel = model.part("front_wheel")
    front_rim, front_tire = _wheel_meshes("front_wheel")
    front_wheel.visual(
        front_rim,
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2)),
        material=aluminum,
        name="rim",
    )
    front_wheel.visual(
        front_tire,
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2)),
        material=rubber,
        name="tire",
    )
    front_wheel.visual(
        Cylinder(radius=0.010, length=0.048),
        origin=Origin(rpy=(math.pi / 2, 0.0, 0.0)),
        material=dark_metal,
        name="bearing_sleeve",
    )

    rear_wheel = model.part("rear_wheel")
    rear_rim, rear_tire = _wheel_meshes("rear_wheel")
    rear_wheel.visual(
        rear_rim,
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2)),
        material=aluminum,
        name="rim",
    )
    rear_wheel.visual(
        rear_tire,
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2)),
        material=rubber,
        name="tire",
    )
    rear_wheel.visual(
        Cylinder(radius=0.010, length=0.048),
        origin=Origin(rpy=(math.pi / 2, 0.0, 0.0)),
        material=dark_metal,
        name="bearing_sleeve",
    )

    model.articulation(
        "deck_to_steering",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=steering,
        origin=Origin(xyz=(0.435, 0.0, 0.155)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-1.05, upper=1.05, effort=8.0, velocity=2.2),
    )
    model.articulation(
        "steering_to_handlebar",
        ArticulationType.REVOLUTE,
        parent=steering,
        child=handlebar,
        origin=Origin(xyz=(0.0, 0.0, 0.728)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.45, effort=4.0, velocity=2.0),
    )
    model.articulation(
        "steering_to_front_wheel",
        ArticulationType.CONTINUOUS,
        parent=steering,
        child=front_wheel,
        origin=Origin(xyz=(0.120, 0.0, -0.080)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=20.0),
    )
    model.articulation(
        "deck_to_rear_wheel",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=rear_wheel,
        origin=Origin(xyz=(-0.445, 0.0, 0.075)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=20.0),
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
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.
    deck = object_model.get_part("deck")
    steering = object_model.get_part("steering")
    handlebar = object_model.get_part("handlebar")
    front_wheel = object_model.get_part("front_wheel")
    rear_wheel = object_model.get_part("rear_wheel")
    steer_joint = object_model.get_articulation("deck_to_steering")
    handle_joint = object_model.get_articulation("steering_to_handlebar")

    ctx.allow_overlap(
        steering,
        handlebar,
        elem_a="handlebar_pin",
        elem_b="hinge_lug",
        reason="The handlebar hinge pin is intentionally captured inside the folding lug.",
    )
    ctx.allow_overlap(
        steering,
        front_wheel,
        elem_a="front_axle",
        elem_b="bearing_sleeve",
        reason="The front wheel bearing sleeve is intentionally captured around the axle.",
    )
    ctx.allow_overlap(
        deck,
        rear_wheel,
        elem_a="rear_axle",
        elem_b="bearing_sleeve",
        reason="The rear wheel bearing sleeve is intentionally captured around the axle.",
    )

    ctx.expect_within(
        steering,
        handlebar,
        axes="xz",
        inner_elem="handlebar_pin",
        outer_elem="hinge_lug",
        margin=0.001,
        name="handlebar pin centered in folding lug",
    )
    ctx.expect_overlap(
        steering,
        handlebar,
        axes="y",
        elem_a="handlebar_pin",
        elem_b="hinge_lug",
        min_overlap=0.050,
        name="handlebar pin spans folding lug",
    )
    ctx.expect_within(
        steering,
        front_wheel,
        axes="xz",
        inner_elem="front_axle",
        outer_elem="bearing_sleeve",
        margin=0.002,
        name="front axle centered in bearing sleeve",
    )
    ctx.expect_overlap(
        steering,
        front_wheel,
        axes="y",
        elem_a="front_axle",
        elem_b="bearing_sleeve",
        min_overlap=0.040,
        name="front axle remains inserted through wheel",
    )
    ctx.expect_within(
        deck,
        rear_wheel,
        axes="xz",
        inner_elem="rear_axle",
        outer_elem="bearing_sleeve",
        margin=0.002,
        name="rear axle centered in bearing sleeve",
    )
    ctx.expect_overlap(
        deck,
        rear_wheel,
        axes="y",
        elem_a="rear_axle",
        elem_b="bearing_sleeve",
        min_overlap=0.040,
        name="rear axle remains inserted through wheel",
    )

    ctx.expect_overlap(
        front_wheel,
        steering,
        axes="z",
        elem_a="tire",
        elem_b="fork_blade_near",
        min_overlap=0.020,
        name="front fork reaches wheel height",
    )
    ctx.expect_overlap(
        rear_wheel,
        deck,
        axes="z",
        elem_a="tire",
        elem_b="rear_dropout_near",
        min_overlap=0.020,
        name="rear dropout reaches wheel height",
    )
    ctx.expect_within(
        steering,
        deck,
        axes="xy",
        inner_elem="stem",
        outer_elem="head_tube",
        margin=0.020,
        name="steering stem runs through head tube",
    )
    ctx.expect_gap(
        handlebar,
        steering,
        axis="z",
        positive_elem="hinge_lug",
        negative_elem="hinge_clamp",
        min_gap=0.0,
        max_gap=0.020,
        name="handlebar lug seated above hinge clamp",
    )

    rest_crossbar = ctx.part_element_world_aabb(handlebar, elem="crossbar_tube")
    with ctx.pose({handle_joint: 1.45}):
        folded_crossbar = ctx.part_element_world_aabb(handlebar, elem="crossbar_tube")
        folded_lug = ctx.part_element_world_aabb(handlebar, elem="hinge_lug")
        ctx.check(
            "folded crossbar drops below hinge",
            folded_crossbar is not None
            and folded_lug is not None
            and (folded_crossbar[1][2] < folded_lug[0][2] - 0.010),
            details=f"crossbar={folded_crossbar}, lug={folded_lug}",
        )
    ctx.check(
        "handlebar folds downward",
        rest_crossbar is not None
        and folded_crossbar is not None
        and ((folded_crossbar[0][2] + folded_crossbar[1][2]) * 0.5)
        < ((rest_crossbar[0][2] + rest_crossbar[1][2]) * 0.5) - 0.080,
        details=f"rest={rest_crossbar}, folded={folded_crossbar}",
    )
    with ctx.pose({steer_joint: 0.55}):
        ctx.expect_origin_distance(
            front_wheel,
            rear_wheel,
            axes="xy",
            min_dist=0.15,
            name="front fork yaws with front wheel clear of rear wheel",
        )
    return ctx.report()


object_model = build_object_model()
