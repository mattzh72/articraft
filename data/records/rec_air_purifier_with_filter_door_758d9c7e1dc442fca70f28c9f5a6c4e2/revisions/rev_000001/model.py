from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireSidewall,
    TireTread,
    WheelGeometry,
    WheelHub,
    WheelRim,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_air_scrubber")

    powder_gray = model.material("powder_gray", color=(0.56, 0.60, 0.60, 1.0))
    dark_gray = model.material("dark_gray", color=(0.08, 0.09, 0.09, 1.0))
    black_rubber = model.material("black_rubber", color=(0.015, 0.014, 0.013, 1.0))
    galvanized = model.material("galvanized", color=(0.72, 0.74, 0.73, 1.0))
    warning_yellow = model.material("warning_yellow", color=(0.92, 0.68, 0.12, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((0.66, 0.48, 1.35)),
        origin=Origin(xyz=(0.0, 0.0, 0.805)),
        material=powder_gray,
        name="cabinet_shell",
    )
    cabinet.visual(
        Box((0.62, 0.018, 0.055)),
        origin=Origin(xyz=(0.0, -0.249, 1.455)),
        material=galvanized,
        name="front_top_rail",
    )
    cabinet.visual(
        Box((0.62, 0.018, 0.060)),
        origin=Origin(xyz=(0.0, -0.249, 0.165)),
        material=galvanized,
        name="front_bottom_rail",
    )
    cabinet.visual(
        Box((0.045, 0.018, 1.15)),
        origin=Origin(xyz=(-0.312, -0.235, 0.805)),
        material=galvanized,
        name="front_hinge_jamb",
    )
    cabinet.visual(
        Box((0.045, 0.018, 1.15)),
        origin=Origin(xyz=(0.312, -0.235, 0.805)),
        material=galvanized,
        name="front_latch_jamb",
    )
    cabinet.visual(
        Box((0.46, 0.010, 0.19)),
        origin=Origin(xyz=(0.0, 0.245, 1.24)),
        material=dark_gray,
        name="rear_intake_grille",
    )
    for i, z in enumerate((0.49, 1.24)):
        cabinet.visual(
            Box((0.065, 0.026, 0.210)),
            origin=Origin(xyz=(-0.326, -0.2505, z)),
            material=galvanized,
            name=f"hinge_mount_{i}",
        )
    cabinet.visual(
        Box((0.50, 0.26, 0.008)),
        origin=Origin(xyz=(0.0, 0.02, 1.476)),
        material=dark_gray,
        name="top_exhaust_opening",
    )
    cabinet.visual(
        Box((0.50, 0.030, 0.040)),
        origin=Origin(xyz=(0.0, 0.235, 1.495)),
        material=galvanized,
        name="top_hinge_rail",
    )

    front_door = model.part("front_door")
    front_door.visual(
        Box((0.560, 0.035, 1.040)),
        origin=Origin(xyz=(0.310, 0.0175, 0.520)),
        material=powder_gray,
        name="door_panel",
    )
    front_door.visual(
        Box((0.460, 0.007, 0.620)),
        origin=Origin(xyz=(0.325, -0.004, 0.555)),
        material=dark_gray,
        name="filter_screen",
    )
    for i in range(8):
        front_door.visual(
            Box((0.485, 0.014, 0.018)),
            origin=Origin(xyz=(0.325, -0.011, 0.315 + i * 0.065)),
            material=galvanized,
            name=f"filter_louver_{i}",
        )
    for i, z in enumerate((0.21, 0.96)):
        front_door.visual(
            Cylinder(radius=0.018, length=0.170),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=galvanized,
            name=f"hinge_knuckle_{i}",
        )
        front_door.visual(
            Box((0.090, 0.012, 0.135)),
            origin=Origin(xyz=(0.043, 0.006, z)),
            material=galvanized,
            name=f"hinge_leaf_{i}",
        )
    front_door.visual(
        Box((0.035, 0.012, 0.110)),
        origin=Origin(xyz=(0.535, -0.006, 0.520)),
        material=warning_yellow,
        name="latch_plate",
    )
    for i, z in enumerate((0.405, 0.695)):
        front_door.visual(
            Box((0.040, 0.035, 0.045)),
            origin=Origin(xyz=(0.505, -0.0175, z)),
            material=galvanized,
            name=f"handle_standoff_{i}",
        )
    front_door.visual(
        Cylinder(radius=0.012, length=0.360),
        origin=Origin(xyz=(0.505, -0.038, 0.550)),
        material=galvanized,
        name="vertical_handle",
    )

    door_hinge = model.articulation(
        "cabinet_to_front_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=front_door,
        origin=Origin(xyz=(-0.315, -0.281, 0.280)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.2, lower=0.0, upper=1.75),
    )

    exhaust_baffle = model.part("exhaust_baffle")
    exhaust_baffle.visual(
        Box((0.560, 0.320, 0.035)),
        origin=Origin(xyz=(0.0, -0.160, 0.020)),
        material=powder_gray,
        name="baffle_panel",
    )
    exhaust_baffle.visual(
        Box((0.470, 0.210, 0.006)),
        origin=Origin(xyz=(0.0, -0.155, 0.0395)),
        material=dark_gray,
        name="baffle_screen",
    )
    for i in range(5):
        exhaust_baffle.visual(
            Box((0.470, 0.020, 0.020)),
            origin=Origin(xyz=(0.0, -0.240 + i * 0.045, 0.052)),
            material=galvanized,
            name=f"baffle_louver_{i}",
        )
    for i, x in enumerate((-0.170, 0.170)):
        exhaust_baffle.visual(
            Cylinder(radius=0.013, length=0.130),
            origin=Origin(xyz=(x, 0.0, 0.018), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=galvanized,
            name=f"baffle_knuckle_{i}",
        )
        exhaust_baffle.visual(
            Box((0.120, 0.040, 0.008)),
            origin=Origin(xyz=(x, -0.023, 0.010)),
            material=galvanized,
            name=f"baffle_leaf_{i}",
        )

    baffle_hinge = model.articulation(
        "cabinet_to_exhaust_baffle",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=exhaust_baffle,
        origin=Origin(xyz=(0.0, 0.205, 1.4775)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.0, lower=0.0, upper=1.10),
    )

    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.037,
            0.030,
            rim=WheelRim(inner_radius=0.024, flange_height=0.004, flange_thickness=0.003),
            hub=WheelHub(radius=0.014, width=0.024, cap_style="flat"),
        ),
        "caster_wheel_rim",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.055,
            0.036,
            inner_radius=0.037,
            tread=TireTread(style="ribbed", depth=0.003, count=16, land_ratio=0.60),
            sidewall=TireSidewall(style="rounded", bulge=0.035),
        ),
        "caster_tire",
    )

    caster_positions = (
        (-0.245, -0.160),
        (0.245, -0.160),
        (-0.245, 0.160),
        (0.245, 0.160),
    )
    for i, (x, y) in enumerate(caster_positions):
        caster = model.part(f"caster_{i}")
        caster.visual(
            Box((0.120, 0.095, 0.012)),
            origin=Origin(xyz=(0.0, 0.0, -0.006)),
            material=galvanized,
            name="mount_plate",
        )
        caster.visual(
            Cylinder(radius=0.017, length=0.060),
            origin=Origin(xyz=(0.0, 0.0, -0.041)),
            material=galvanized,
            name="swivel_stem",
        )
        caster.visual(
            Box((0.088, 0.075, 0.020)),
            origin=Origin(xyz=(0.0, 0.0, -0.070)),
            material=galvanized,
            name="fork_crown",
        )
        for side, sx in enumerate((-0.034, 0.034)):
            caster.visual(
                Box((0.009, 0.060, 0.075)),
                origin=Origin(xyz=(sx, 0.0, -0.095)),
                material=galvanized,
                name=f"fork_leg_{side}",
            )
        caster.visual(
            Cylinder(radius=0.006, length=0.082),
            origin=Origin(xyz=(0.0, 0.0, -0.083), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=galvanized,
            name="axle_pin",
        )
        caster.visual(
            tire_mesh,
            origin=Origin(xyz=(0.0, 0.0, -0.083)),
            material=black_rubber,
            name="rubber_tire",
        )
        caster.visual(
            wheel_mesh,
            origin=Origin(xyz=(0.0, 0.0, -0.083)),
            material=galvanized,
            name="metal_wheel",
        )
        model.articulation(
            f"cabinet_to_caster_{i}",
            ArticulationType.FIXED,
            parent=cabinet,
            child=caster,
            origin=Origin(xyz=(x, y, 0.130)),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("front_door")
    baffle = object_model.get_part("exhaust_baffle")
    door_hinge = object_model.get_articulation("cabinet_to_front_door")
    baffle_hinge = object_model.get_articulation("cabinet_to_exhaust_baffle")

    ctx.check(
        "front door has two visible hinge knuckles",
        all(door.get_visual(f"hinge_knuckle_{i}") is not None for i in range(2)),
        details="The access door should show two separate hinge knuckle barrels at its left edge.",
    )
    for i in range(2):
        ctx.allow_overlap(
            cabinet,
            door,
            elem_a=f"hinge_mount_{i}",
            elem_b=f"hinge_knuckle_{i}",
            reason=(
                "Each cabinet-side hinge mount intentionally captures the door "
                "knuckle with a tiny local seated fit so the closed door is "
                "physically supported at the revolute hinge."
            ),
        )
        ctx.expect_gap(
            cabinet,
            door,
            axis="y",
            positive_elem=f"hinge_mount_{i}",
            negative_elem=f"hinge_knuckle_{i}",
            max_penetration=0.002,
            max_gap=0.001,
            name=f"hinge knuckle {i} is locally captured by cabinet mount",
        )
    ctx.expect_gap(
        cabinet,
        door,
        axis="y",
        positive_elem="cabinet_shell",
        negative_elem="door_panel",
        min_gap=0.002,
        max_gap=0.012,
        name="closed door sits just proud of cabinet front",
    )
    ctx.expect_overlap(
        door,
        cabinet,
        axes="xz",
        elem_a="door_panel",
        elem_b="cabinet_shell",
        min_overlap=0.50,
        name="large access door covers cabinet opening",
    )
    ctx.expect_gap(
        baffle,
        cabinet,
        axis="z",
        positive_elem="baffle_panel",
        negative_elem="cabinet_shell",
        min_gap=0.000,
        max_gap=0.020,
        name="closed exhaust baffle rests over the top outlet",
    )

    closed_door_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    with ctx.pose({door_hinge: 1.20}):
        open_door_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    ctx.check(
        "front door swings outward from left hinge",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[0][1] < closed_door_aabb[0][1] - 0.25,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    closed_baffle_aabb = ctx.part_element_world_aabb(baffle, elem="baffle_panel")
    with ctx.pose({baffle_hinge: 0.85}):
        open_baffle_aabb = ctx.part_element_world_aabb(baffle, elem="baffle_panel")
    ctx.check(
        "top exhaust baffle lifts about rear edge",
        closed_baffle_aabb is not None
        and open_baffle_aabb is not None
        and open_baffle_aabb[1][2] > closed_baffle_aabb[1][2] + 0.12,
        details=f"closed={closed_baffle_aabb}, open={open_baffle_aabb}",
    )

    first_caster = object_model.get_part("caster_0")
    ctx.expect_gap(
        cabinet,
        first_caster,
        axis="z",
        positive_elem="cabinet_shell",
        negative_elem="mount_plate",
        min_gap=0.0,
        max_gap=0.002,
        name="caster mounting plate bolts to cabinet underside",
    )

    return ctx.report()


object_model = build_object_model()
