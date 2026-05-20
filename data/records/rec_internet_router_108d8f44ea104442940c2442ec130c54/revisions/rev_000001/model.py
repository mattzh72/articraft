from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rack_router")

    dark_metal = model.material("dark_metal", rgba=(0.035, 0.038, 0.042, 1.0))
    satin_black = model.material("satin_black", rgba=(0.005, 0.006, 0.007, 1.0))
    graphite = model.material("graphite", rgba=(0.12, 0.13, 0.14, 1.0))
    smoked = model.material("smoked_translucent", rgba=(0.03, 0.045, 0.055, 0.58))
    green = model.material("green_led", rgba=(0.0, 0.85, 0.22, 1.0))
    amber = model.material("amber_led", rgba=(1.0, 0.55, 0.04, 1.0))
    blue = model.material("blue_led", rgba=(0.05, 0.35, 1.0, 1.0))

    chassis = model.part("chassis")
    chassis.visual(
        Box((0.440, 0.280, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=dark_metal,
        name="main_shell",
    )
    chassis.visual(
        Box((0.040, 0.028, 0.070)),
        origin=Origin(xyz=(-0.202, 0.151, 0.035)),
        material=graphite,
        name="rack_ear_0",
    )
    chassis.visual(
        Box((0.040, 0.028, 0.070)),
        origin=Origin(xyz=(-0.202, -0.151, 0.035)),
        material=graphite,
        name="rack_ear_1",
    )
    chassis.visual(
        Box((0.0035, 0.224, 0.028)),
        origin=Origin(xyz=(-0.2214, 0.0, 0.031)),
        material=satin_black,
        name="indicator_strip",
    )
    chassis.visual(
        Box((0.006, 0.232, 0.006)),
        origin=Origin(xyz=(-0.2200, 0.0, 0.013)),
        material=graphite,
        name="status_hinge_ledge",
    )

    for i, y in enumerate((-0.070, -0.042, -0.014, 0.014, 0.042, 0.070)):
        chassis.visual(
            Box((0.0040, 0.010, 0.010)),
            origin=Origin(xyz=(-0.2230, y, 0.031)),
            material=(green, green, blue, amber, green, green)[i],
            name=f"indicator_{i}",
        )

    for i, y in enumerate((-0.075, -0.045, -0.015, 0.015, 0.045, 0.075)):
        chassis.visual(
            Box((0.150, 0.008, 0.0020)),
            origin=Origin(xyz=(0.015, y, 0.0506)),
            material=satin_black,
            name=f"top_vent_{i}",
        )

    def add_antenna_mount(side_name: str, y: float) -> None:
        chassis.visual(
            Box((0.048, 0.038, 0.0115)),
            origin=Origin(xyz=(0.156, y, 0.05525)),
            material=graphite,
            name=f"{side_name}_antenna_base",
        )
        for suffix, offset in (("inner", -0.021), ("outer", 0.021)):
            chassis.visual(
                Box((0.013, 0.006, 0.017)),
                origin=Origin(xyz=(0.156, y + offset, 0.0685)),
                material=graphite,
                name=f"{side_name}_yoke_{suffix}",
            )

    add_antenna_mount("left", 0.105)
    add_antenna_mount("right", -0.105)

    status_cover = model.part("status_cover")
    status_cover.visual(
        Cylinder(radius=0.0030, length=0.220),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="door_hinge_barrel",
    )
    status_cover.visual(
        Box((0.0040, 0.224, 0.032)),
        origin=Origin(xyz=(-0.0040, 0.0, 0.0180)),
        material=smoked,
        name="status_panel",
    )

    def add_antenna(side_name: str, y: float):
        antenna = model.part(f"{side_name}_antenna")
        antenna.visual(
            Cylinder(radius=0.0070, length=0.028),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material=graphite,
            name="hinge_barrel",
        )
        antenna.visual(
            Cylinder(radius=0.0065, length=0.020),
            origin=Origin(xyz=(0.0, 0.0, 0.016)),
            material=satin_black,
            name="base_collar",
        )
        antenna.visual(
            Cylinder(radius=0.0045, length=0.150),
            origin=Origin(xyz=(0.0, 0.0, 0.082)),
            material=satin_black,
            name="rubber_stub",
        )
        antenna.visual(
            Sphere(radius=0.0052),
            origin=Origin(xyz=(0.0, 0.0, 0.158)),
            material=satin_black,
            name="rounded_tip",
        )
        model.articulation(
            f"chassis_to_{side_name}_antenna",
            ArticulationType.REVOLUTE,
            parent=chassis,
            child=antenna,
            origin=Origin(xyz=(0.156, y, 0.0677)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=1.4, velocity=2.0, lower=-1.35, upper=0.0),
        )
        return antenna

    add_antenna("left", 0.105)
    add_antenna("right", -0.105)

    model.articulation(
        "chassis_to_status_cover",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=status_cover,
        origin=Origin(xyz=(-0.2257, 0.0, 0.013)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=1.5, lower=0.0, upper=1.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    chassis = object_model.get_part("chassis")
    status_cover = object_model.get_part("status_cover")
    left_antenna = object_model.get_part("left_antenna")
    right_antenna = object_model.get_part("right_antenna")
    door_joint = object_model.get_articulation("chassis_to_status_cover")
    left_joint = object_model.get_articulation("chassis_to_left_antenna")
    right_joint = object_model.get_articulation("chassis_to_right_antenna")

    ctx.allow_overlap(
        chassis,
        status_cover,
        elem_a="status_hinge_ledge",
        elem_b="door_hinge_barrel",
        reason="The front status cover barrel is intentionally captured in the lower hinge clip.",
    )
    for side in ("left", "right"):
        ctx.allow_overlap(
            chassis,
            f"{side}_antenna",
            elem_a=f"{side}_antenna_base",
            elem_b="hinge_barrel",
            reason="The antenna hinge barrel is lightly seated into its small base saddle.",
        )

    ctx.expect_gap(
        chassis,
        status_cover,
        axis="x",
        positive_elem="status_hinge_ledge",
        negative_elem="door_hinge_barrel",
        max_gap=0.0010,
        max_penetration=0.0006,
        name="status cover barrel stays clipped to lower ledge",
    )
    ctx.expect_overlap(
        status_cover,
        chassis,
        axes="y",
        elem_a="door_hinge_barrel",
        elem_b="status_hinge_ledge",
        min_overlap=0.200,
        name="status cover hinge spans the indicator strip",
    )

    for antenna, side in ((left_antenna, "left"), (right_antenna, "right")):
        ctx.expect_gap(
            antenna,
            chassis,
            axis="z",
            positive_elem="hinge_barrel",
            negative_elem=f"{side}_antenna_base",
            max_gap=0.0015,
            max_penetration=0.0006,
            name=f"{side} antenna barrel is seated on its base",
        )

    closed_cover_aabb = ctx.part_element_world_aabb(status_cover, elem="status_panel")
    with ctx.pose({door_joint: 1.20}):
        open_cover_aabb = ctx.part_element_world_aabb(status_cover, elem="status_panel")
        ctx.expect_gap(
            chassis,
            status_cover,
            axis="x",
            positive_elem="status_hinge_ledge",
            negative_elem="door_hinge_barrel",
            max_gap=0.0010,
            max_penetration=0.0006,
            name="status hinge remains attached while cover opens",
        )
    ctx.check(
        "status cover swings outward from the front",
        closed_cover_aabb is not None
        and open_cover_aabb is not None
        and open_cover_aabb[0][0] < closed_cover_aabb[0][0] - 0.008,
        details=f"closed={closed_cover_aabb}, open={open_cover_aabb}",
    )

    for antenna, joint, side in (
        (left_antenna, left_joint, "left"),
        (right_antenna, right_joint, "right"),
    ):
        upright_aabb = ctx.part_element_world_aabb(antenna, elem="rounded_tip")
        with ctx.pose({joint: -1.25}):
            folded_aabb = ctx.part_element_world_aabb(antenna, elem="rounded_tip")
        ctx.check(
            f"{side} antenna folds down from upright",
            upright_aabb is not None
            and folded_aabb is not None
            and folded_aabb[1][2] < upright_aabb[1][2] - 0.050
            and folded_aabb[1][0] > upright_aabb[1][0] + 0.050,
            details=f"upright={upright_aabb}, folded={folded_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
