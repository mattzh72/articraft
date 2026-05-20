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
    model = ArticulatedObject(name="heavy_duty_hole_punch")

    steel = model.material("steel", rgba=(0.62, 0.65, 0.69, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.29, 0.31, 0.35, 1.0))
    grip_black = model.material("grip_black", rgba=(0.11, 0.11, 0.12, 1.0))

    base = model.part("base")

    base.visual(
        Box((0.34, 0.14, 0.005)),
        origin=Origin(xyz=(0.0, 0.0, 0.0235)),
        material=steel,
        name="base_deck",
    )
    base.visual(
        Box((0.34, 0.006, 0.021)),
        origin=Origin(xyz=(0.0, 0.067, 0.0105)),
        material=steel,
        name="side_wall_0",
    )
    base.visual(
        Box((0.34, 0.006, 0.021)),
        origin=Origin(xyz=(0.0, -0.067, 0.0105)),
        material=steel,
        name="side_wall_1",
    )
    base.visual(
        Box((0.024, 0.128, 0.006)),
        origin=Origin(xyz=(0.158, 0.0, 0.003)),
        material=steel,
        name="front_lip",
    )
    base.visual(
        Box((0.035, 0.128, 0.006)),
        origin=Origin(xyz=(-0.055, 0.0, 0.006)),
        material=steel,
        name="rear_tray_rail",
    )
    for idx, y in enumerate((0.039, -0.039)):
        base.visual(
            Cylinder(radius=0.0048, length=0.024),
            origin=Origin(xyz=(-0.117, y, 0.008), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"tray_knuckle_{idx}",
        )
        base.visual(
            Box((0.01, 0.013, 0.01)),
            origin=Origin(xyz=(-0.117, 0.0575 if idx == 0 else -0.0575, 0.008)),
            material=dark_steel,
            name=f"tray_hanger_{idx}",
        )
    for idx, y in enumerate((0.044, -0.044)):
        base.visual(
            Box((0.105, 0.022, 0.05)),
            origin=Origin(xyz=(-0.0725, y, 0.051)),
            material=dark_steel,
            name=f"head_cheek_{idx}",
        )
    base.visual(
        Box((0.032, 0.11, 0.02)),
        origin=Origin(xyz=(-0.014, 0.0, 0.041)),
        material=dark_steel,
        name="head_housing",
    )
    base.visual(
        Box((0.065, 0.082, 0.012)),
        origin=Origin(xyz=(-0.028, 0.0, 0.032)),
        material=dark_steel,
        name="die_block",
    )
    for idx, y in enumerate((0.054, -0.054)):
        base.visual(
            Box((0.03, 0.012, 0.07)),
            origin=Origin(xyz=(-0.145, y, 0.061)),
            material=dark_steel,
            name=f"rear_cheek_{idx}",
        )
    for idx, y in enumerate((0.025, -0.025)):
        base.visual(
            Cylinder(radius=0.0065, length=0.016),
            origin=Origin(xyz=(-0.028, y, 0.038), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_steel,
            name=f"die_ring_{idx}",
        )

    base.inertial = Inertial.from_geometry(
        Box((0.34, 0.14, 0.101)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, 0.0505)),
    )

    handle = model.part("handle")
    handle.visual(
        Box((0.29, 0.04, 0.01)),
        origin=Origin(xyz=(0.145, 0.0, 0.0)),
        material=dark_steel,
        name="lever_bar",
    )
    handle.visual(
        Box((0.065, 0.108, 0.018)),
        origin=Origin(xyz=(0.252, 0.0, 0.009)),
        material=grip_black,
        name="grip_pad",
    )
    handle.visual(
        Cylinder(radius=0.014, length=0.116),
        origin=Origin(xyz=(0.285, 0.0, 0.022), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=grip_black,
        name="front_roll",
    )
    handle.visual(
        Cylinder(radius=0.006, length=0.098),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="handle_barrel",
    )
    handle.visual(
        Box((0.062, 0.102, 0.01)),
        origin=Origin(xyz=(0.05, 0.0, -0.009)),
        material=dark_steel,
        name="punch_crosshead",
    )
    for idx, y in enumerate((0.024, -0.024)):
        handle.visual(
            Cylinder(radius=0.0075, length=0.028),
            origin=Origin(xyz=(0.052, y, -0.024)),
            material=dark_steel,
            name=f"punch_stem_{idx}",
        )

    handle.inertial = Inertial.from_geometry(
        Box((0.31, 0.116, 0.06)),
        mass=1.0,
        origin=Origin(xyz=(0.155, 0.0, 0.0)),
    )

    tray_door = model.part("tray_door")
    tray_door.visual(
        Cylinder(radius=0.0045, length=0.054),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="tray_barrel",
    )
    tray_door.visual(
        Box((0.032, 0.05, 0.01)),
        origin=Origin(xyz=(0.016, 0.0, -0.004)),
        material=dark_steel,
        name="tray_flange",
    )
    tray_door.visual(
        Box((0.094, 0.04, 0.006)),
        origin=Origin(xyz=(0.063, 0.0, -0.008)),
        material=dark_steel,
        name="tray_spine",
    )
    tray_door.visual(
        Box((0.126, 0.112, 0.004)),
        origin=Origin(xyz=(0.077, 0.0, -0.01)),
        material=steel,
        name="tray_panel",
    )
    tray_door.visual(
        Box((0.014, 0.112, 0.012)),
        origin=Origin(xyz=(0.137, 0.0, -0.012)),
        material=steel,
        name="tray_pull_lip",
    )
    tray_door.inertial = Inertial.from_geometry(
        Box((0.151, 0.112, 0.03)),
        mass=0.22,
        origin=Origin(xyz=(0.0755, 0.0, -0.006)),
    )

    model.articulation(
        "base_to_handle",
        ArticulationType.REVOLUTE,
        parent=base,
        child=handle,
        origin=Origin(xyz=(-0.145, 0.0, 0.095)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.8,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )
    model.articulation(
        "base_to_tray_door",
        ArticulationType.REVOLUTE,
        parent=base,
        child=tray_door,
        origin=Origin(xyz=(-0.117, 0.0, 0.008)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(110.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    handle = object_model.get_part("handle")
    tray_door = object_model.get_part("tray_door")
    handle_hinge = object_model.get_articulation("base_to_handle")
    tray_hinge = object_model.get_articulation("base_to_tray_door")

    with ctx.pose({handle_hinge: 0.0}):
        ctx.expect_gap(
            handle,
            base,
            axis="z",
            positive_elem="punch_crosshead",
            negative_elem="head_cheek_0",
            min_gap=0.003,
            max_gap=0.012,
            name="crosshead rests just above the punch head",
        )
        ctx.expect_overlap(
            handle,
            base,
            axes="x",
            elem_a="lever_bar",
            elem_b="head_cheek_0",
            min_overlap=0.06,
            name="handle spans over the rear punch head",
        )

    rest_roll = ctx.part_element_world_aabb(handle, elem="front_roll")
    with ctx.pose({handle_hinge: math.radians(60.0)}):
        raised_roll = ctx.part_element_world_aabb(handle, elem="front_roll")

    ctx.check(
        "handle lifts upward when opened",
        rest_roll is not None and raised_roll is not None and raised_roll[1][2] > rest_roll[1][2] + 0.08,
        details=f"rest={rest_roll}, raised={raised_roll}",
    )

    with ctx.pose({tray_hinge: 0.0}):
        ctx.expect_gap(
            base,
            tray_door,
            axis="z",
            positive_elem="rear_tray_rail",
            negative_elem="tray_panel",
            min_gap=0.003,
            max_gap=0.02,
            name="chip tray door tucks just beneath the rear base rail",
        )

    rest_panel = ctx.part_element_world_aabb(tray_door, elem="tray_panel")
    with ctx.pose({tray_hinge: math.radians(85.0)}):
        open_panel = ctx.part_element_world_aabb(tray_door, elem="tray_panel")

    ctx.check(
        "chip tray door swings downward when opened",
        rest_panel is not None and open_panel is not None and open_panel[0][2] < rest_panel[0][2] - 0.05,
        details=f"rest={rest_panel}, open={open_panel}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
