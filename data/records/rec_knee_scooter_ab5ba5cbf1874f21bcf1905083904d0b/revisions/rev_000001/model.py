from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
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
    mesh_from_geometry,
    rounded_rect_profile,
)


WHEEL_RADIUS = 0.095
WHEEL_WIDTH = 0.044
AXLE_Z = 0.105
REAR_X = -0.42
FRONT_X = 0.41
WHEEL_Y = 0.18
STEER_Z = 0.305


def _wheel_visuals(part, hub_mesh, tire_mesh, material_hub: Material, material_tire: Material) -> None:
    wheel_to_world_y = Origin(rpy=(0.0, 0.0, math.pi / 2.0))
    part.visual(tire_mesh, origin=wheel_to_world_y, material=material_tire, name="tire")
    part.visual(hub_mesh, origin=wheel_to_world_y, material=material_hub, name="hub")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rental_knee_scooter")

    frame_mat = model.material("rental_teal_frame", rgba=(0.05, 0.42, 0.48, 1.0))
    metal_mat = model.material("brushed_aluminum", rgba=(0.70, 0.72, 0.70, 1.0))
    dark_metal = model.material("dark_hardware", rgba=(0.08, 0.08, 0.075, 1.0))
    rubber_mat = model.material("black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))
    pad_mat = model.material("matte_black_knee_pad", rgba=(0.015, 0.018, 0.020, 1.0))

    hub_mesh = mesh_from_geometry(
        WheelGeometry(
            0.070,
            WHEEL_WIDTH,
            rim=WheelRim(
                inner_radius=0.047,
                flange_height=0.005,
                flange_thickness=0.003,
                bead_seat_depth=0.002,
            ),
            hub=WheelHub(
                radius=0.024,
                width=0.042,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=4, circle_diameter=0.030, hole_diameter=0.004),
            ),
            face=WheelFace(dish_depth=0.004, front_inset=0.002, rear_inset=0.002),
            spokes=WheelSpokes(style="split_y", count=4, thickness=0.003, window_radius=0.008),
            bore=WheelBore(style="round", diameter=0.015),
        ),
        "compact_scooter_hub",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            WHEEL_RADIUS,
            WHEEL_WIDTH,
            inner_radius=0.062,
            tread=TireTread(style="block", depth=0.0045, count=18, land_ratio=0.58),
            grooves=(TireGroove(center_offset=0.0, width=0.0045, depth=0.002),),
            sidewall=TireSidewall(style="rounded", bulge=0.04),
            shoulder=TireShoulder(width=0.004, radius=0.003),
        ),
        "compact_scooter_tire",
    )
    pad_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(0.45, 0.205, 0.050, corner_segments=10),
            0.055,
            cap=True,
            center=True,
        ),
        "rounded_knee_pad",
    )

    frame = model.part("center_frame")
    frame.visual(
        Cylinder(radius=0.018, length=0.785),
        origin=Origin(xyz=(-0.0125, 0.0, 0.240), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_mat,
        name="center_spine",
    )
    frame.visual(
        Cylinder(radius=0.015, length=0.42),
        origin=Origin(xyz=(REAR_X, 0.0, AXLE_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal_mat,
        name="rear_axle",
    )
    frame.visual(
        Box((0.055, 0.260, 0.125)),
        origin=Origin(xyz=(REAR_X, 0.0, 0.160)),
        material=frame_mat,
        name="rear_dropout",
    )
    frame.visual(
        Box((0.050, 0.210, 0.040)),
        origin=Origin(xyz=(-0.455, 0.0, 0.145)),
        material=frame_mat,
        name="brake_base",
    )
    for side, y in (("near", 0.125), ("far", -0.125)):
        frame.visual(
            Box((0.050, 0.025, 0.115)),
            origin=Origin(xyz=(-0.455, y, 0.205)),
            material=frame_mat,
            name=f"{side}_brake_cheek",
        )
    frame.visual(
        Cylinder(radius=0.010, length=0.34),
        origin=Origin(xyz=(-0.440, 0.0, 0.245), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="brake_pivot",
    )
    frame.visual(
        Cylinder(radius=0.033, length=0.220),
        origin=Origin(xyz=(FRONT_X, 0.0, STEER_Z)),
        material=metal_mat,
        name="head_tube",
    )
    frame.visual(
        Box((0.430, 0.180, 0.020)),
        origin=Origin(xyz=(-0.080, 0.0, 0.376)),
        material=metal_mat,
        name="deck_plate",
    )
    frame.visual(
        Box((0.044, 0.155, 0.124)),
        origin=Origin(xyz=(-0.220, 0.0, 0.312)),
        material=frame_mat,
        name="rear_deck_post",
    )
    frame.visual(
        Box((0.044, 0.155, 0.124)),
        origin=Origin(xyz=(0.115, 0.0, 0.312)),
        material=frame_mat,
        name="front_deck_post",
    )
    frame.visual(
        pad_mesh,
        origin=Origin(xyz=(-0.080, 0.0, 0.410)),
        material=pad_mat,
        name="knee_pad",
    )

    fork = model.part("front_fork")
    fork.visual(
        Cylinder(radius=0.020, length=0.250),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_metal,
        name="steerer",
    )
    fork.visual(
        Box((0.160, 0.060, 0.038)),
        origin=Origin(xyz=(0.0, 0.0, -0.135)),
        material=frame_mat,
        name="fork_crown",
    )
    fork.visual(
        Box((0.050, 0.270, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, -0.145)),
        material=frame_mat,
        name="fork_bridge",
    )
    for side, y in (("near", WHEEL_Y - 0.060), ("far", -WHEEL_Y + 0.060)):
        fork.visual(
            Box((0.035, 0.030, 0.220)),
            origin=Origin(xyz=(0.0, y, -0.240)),
            material=frame_mat,
            name=f"{side}_fork_leg",
        )
    fork.visual(
        Cylinder(radius=0.013, length=0.42),
        origin=Origin(xyz=(0.0, 0.0, AXLE_Z - STEER_Z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal_mat,
        name="front_axle",
    )

    brake = model.part("brake_bar")
    brake.visual(
        Cylinder(radius=0.014, length=0.200),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="pivot_bar",
    )
    brake.visual(
        Box((0.125, 0.044, 0.018)),
        origin=Origin(xyz=(-0.060, 0.0, -0.018)),
        material=dark_metal,
        name="brake_arm",
    )
    brake.visual(
        Box((0.075, 0.270, 0.016)),
        origin=Origin(xyz=(-0.120, 0.0, -0.033)),
        material=rubber_mat,
        name="rubber_shoe",
    )

    wheel_parts = {}
    for name, x, y in (
        ("front_wheel_0", 0.0, WHEEL_Y),
        ("front_wheel_1", 0.0, -WHEEL_Y),
        ("rear_wheel_0", REAR_X, WHEEL_Y),
        ("rear_wheel_1", REAR_X, -WHEEL_Y),
    ):
        wheel = model.part(name)
        _wheel_visuals(wheel, hub_mesh, tire_mesh, metal_mat, rubber_mat)
        wheel_parts[name] = (wheel, x, y)

    model.articulation(
        "front_steer",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=fork,
        origin=Origin(xyz=(FRONT_X, 0.0, STEER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.5, lower=-0.75, upper=0.75),
    )
    model.articulation(
        "front_wheel_0_spin",
        ArticulationType.CONTINUOUS,
        parent=fork,
        child=wheel_parts["front_wheel_0"][0],
        origin=Origin(xyz=(0.0, WHEEL_Y, AXLE_Z - STEER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=20.0),
    )
    model.articulation(
        "front_wheel_1_spin",
        ArticulationType.CONTINUOUS,
        parent=fork,
        child=wheel_parts["front_wheel_1"][0],
        origin=Origin(xyz=(0.0, -WHEEL_Y, AXLE_Z - STEER_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=20.0),
    )
    model.articulation(
        "rear_wheel_0_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel_parts["rear_wheel_0"][0],
        origin=Origin(xyz=(REAR_X, WHEEL_Y, AXLE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=20.0),
    )
    model.articulation(
        "rear_wheel_1_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=wheel_parts["rear_wheel_1"][0],
        origin=Origin(xyz=(REAR_X, -WHEEL_Y, AXLE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=20.0),
    )
    model.articulation(
        "rear_brake_pivot",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=brake,
        origin=Origin(xyz=(-0.440, 0.0, 0.245)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=0.0, upper=0.40),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("center_frame")
    fork = object_model.get_part("front_fork")
    brake = object_model.get_part("brake_bar")
    steer = object_model.get_articulation("front_steer")
    brake_pivot = object_model.get_articulation("rear_brake_pivot")

    ctx.allow_overlap(
        frame,
        fork,
        elem_a="head_tube",
        elem_b="steerer",
        reason="The vertical steerer shaft is intentionally captured inside the frame head tube.",
    )
    ctx.expect_within(
        fork,
        frame,
        axes="xy",
        inner_elem="steerer",
        outer_elem="head_tube",
        margin=0.002,
        name="steerer is centered inside head tube",
    )
    ctx.expect_overlap(
        fork,
        frame,
        axes="z",
        elem_a="steerer",
        elem_b="head_tube",
        min_overlap=0.18,
        name="steerer has retained head-tube engagement",
    )

    for wheel_name, parent_part, axle_elem in (
        ("front_wheel_0", fork, "front_axle"),
        ("front_wheel_1", fork, "front_axle"),
        ("rear_wheel_0", frame, "rear_axle"),
        ("rear_wheel_1", frame, "rear_axle"),
    ):
        wheel = object_model.get_part(wheel_name)
        ctx.allow_overlap(
            parent_part,
            wheel,
            elem_a=axle_elem,
            elem_b="hub",
            reason="The wheel hub is intentionally bored around the fixed axle shaft.",
        )
        ctx.expect_within(
            parent_part,
            wheel,
            axes="xz",
            inner_elem=axle_elem,
            outer_elem="hub",
            margin=0.003,
            name=f"{wheel_name} axle is inside the hub bore",
        )
        ctx.expect_overlap(
            parent_part,
            wheel,
            axes="y",
            elem_a=axle_elem,
            elem_b="hub",
            min_overlap=0.030,
            name=f"{wheel_name} hub stays captured on axle",
        )

    ctx.allow_overlap(
        frame,
        brake,
        elem_a="brake_pivot",
        elem_b="pivot_bar",
        reason="The rear brake pivot pin is intentionally captured through the brake bar barrel.",
    )
    ctx.expect_within(
        frame,
        brake,
        axes="xz",
        inner_elem="brake_pivot",
        outer_elem="pivot_bar",
        margin=0.003,
        name="brake pivot pin runs through the pivot bar",
    )
    ctx.expect_overlap(
        frame,
        brake,
        axes="y",
        elem_a="brake_pivot",
        elem_b="pivot_bar",
        min_overlap=0.18,
        name="brake pivot spans the bar width",
    )

    rest_front = ctx.part_world_position(object_model.get_part("front_wheel_0"))
    with ctx.pose({steer: 0.55}):
        steered_front = ctx.part_world_position(object_model.get_part("front_wheel_0"))
    ctx.check(
        "front fork steers about a vertical axis",
        rest_front is not None
        and steered_front is not None
        and abs(steered_front[0] - rest_front[0]) > 0.035
        and abs(steered_front[2] - rest_front[2]) < 0.002,
        details=f"rest={rest_front}, steered={steered_front}",
    )

    rest_shoe = ctx.part_element_world_aabb(brake, elem="rubber_shoe")
    with ctx.pose({brake_pivot: 0.35}):
        pressed_shoe = ctx.part_element_world_aabb(brake, elem="rubber_shoe")
    ctx.check(
        "rear brake bar swings the rubber shoe downward",
        rest_shoe is not None
        and pressed_shoe is not None
        and pressed_shoe[0][2] < rest_shoe[0][2] - 0.020,
        details=f"rest_shoe={rest_shoe}, pressed_shoe={pressed_shoe}",
    )

    ctx.check(
        "four wheel spin joints are present",
        all(
            object_model.get_articulation(name) is not None
            for name in (
                "front_wheel_0_spin",
                "front_wheel_1_spin",
                "rear_wheel_0_spin",
                "rear_wheel_1_spin",
            )
        ),
    )

    return ctx.report()


object_model = build_object_model()
