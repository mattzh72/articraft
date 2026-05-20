from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="toy_suv_split_rear")

    paint = model.material("paint", rgba=(0.80, 0.17, 0.13, 1.0))
    trim = model.material("trim", rgba=(0.12, 0.12, 0.14, 1.0))
    wheel_mat = model.material("wheel", rgba=(0.08, 0.08, 0.09, 1.0))
    hub_mat = model.material("hub", rgba=(0.66, 0.68, 0.72, 1.0))

    body = model.part("body")

    core_width = 0.058
    body.visual(
        Box((0.192, core_width, 0.034)),
        origin=Origin(xyz=(0.0, 0.0, 0.043)),
        material=paint,
        name="lower_shell",
    )
    body.visual(
        Box((0.126, core_width, 0.034)),
        origin=Origin(xyz=(-0.005, 0.0, 0.077)),
        material=paint,
        name="cabin",
    )
    body.visual(
        Box((0.120, 0.056, 0.010)),
        origin=Origin(xyz=(-0.010, 0.0, 0.099)),
        material=paint,
        name="roof",
    )
    body.visual(
        Box((0.028, 0.056, 0.010)),
        origin=Origin(xyz=(-0.082, 0.0, 0.099)),
        material=paint,
        name="rear_header",
    )
    body.visual(
        Box((0.050, core_width, 0.020)),
        origin=Origin(xyz=(0.060, 0.0, 0.070)),
        material=paint,
        name="hood",
    )
    body.visual(
        Box((0.055, 0.050, 0.020)),
        origin=Origin(xyz=(-0.0685, 0.0, 0.070)),
        material=paint,
        name="rear_deck",
    )
    body.visual(
        Box((0.012, 0.052, 0.018)),
        origin=Origin(xyz=(0.099, 0.0, 0.035)),
        material=trim,
        name="front_bumper",
    )
    body.visual(
        Box((0.012, 0.052, 0.018)),
        origin=Origin(xyz=(-0.099, 0.0, 0.035)),
        material=trim,
        name="rear_bumper",
    )

    flare_size = (0.026, 0.014, 0.022)
    for name, x_pos, y_pos in (
        ("left_front_flare", 0.068, 0.036),
        ("right_front_flare", 0.068, -0.036),
        ("left_rear_flare", -0.068, 0.036),
        ("right_rear_flare", -0.068, -0.036),
    ):
        body.visual(
            Box(flare_size),
            origin=Origin(xyz=(x_pos, y_pos, 0.057)),
            material=paint,
            name=name,
        )

    def add_side_door(
        part_name: str,
        hinge_name: str,
        hinge_xyz: tuple[float, float, float],
        door_length: float,
        door_thickness: float,
        door_height: float,
        side_sign: float,
        axis: tuple[float, float, float],
    ) -> None:
        door = model.part(part_name)
        door.visual(
            Box((door_length, door_thickness, door_height)),
            origin=Origin(
                xyz=(
                    -door_length / 2.0,
                    side_sign * door_thickness / 2.0,
                    door_height / 2.0,
                )
            ),
            material=paint,
            name="door_panel",
        )
        model.articulation(
            hinge_name,
            ArticulationType.REVOLUTE,
            parent=body,
            child=door,
            origin=Origin(xyz=hinge_xyz),
            axis=axis,
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=4.0,
                lower=0.0,
                upper=1.15,
            ),
        )

    add_side_door(
        "left_front_door",
        "left_front_door_hinge",
        hinge_xyz=(0.054, 0.029, 0.045),
        door_length=0.044,
        door_thickness=0.004,
        door_height=0.038,
        side_sign=1.0,
        axis=(0.0, 0.0, -1.0),
    )
    add_side_door(
        "right_front_door",
        "right_front_door_hinge",
        hinge_xyz=(0.054, -0.029, 0.045),
        door_length=0.044,
        door_thickness=0.004,
        door_height=0.038,
        side_sign=-1.0,
        axis=(0.0, 0.0, 1.0),
    )
    add_side_door(
        "left_rear_door",
        "left_rear_door_hinge",
        hinge_xyz=(0.004, 0.029, 0.045),
        door_length=0.044,
        door_thickness=0.004,
        door_height=0.038,
        side_sign=1.0,
        axis=(0.0, 0.0, -1.0),
    )
    add_side_door(
        "right_rear_door",
        "right_rear_door_hinge",
        hinge_xyz=(0.004, -0.029, 0.045),
        door_length=0.044,
        door_thickness=0.004,
        door_height=0.038,
        side_sign=-1.0,
        axis=(0.0, 0.0, 1.0),
    )

    def add_wheel(
        part_name: str,
        joint_name: str,
        wheel_xyz: tuple[float, float, float],
    ) -> None:
        wheel = model.part(part_name)
        wheel.visual(
            Cylinder(radius=0.024, length=0.018),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material=wheel_mat,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.014, length=0.020),
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material=hub_mat,
            name="hub",
        )
        model.articulation(
            joint_name,
            ArticulationType.CONTINUOUS,
            parent=body,
            child=wheel,
            origin=Origin(xyz=wheel_xyz),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=20.0),
        )

    add_wheel("left_front_wheel", "left_front_axle", (0.068, 0.041, 0.024))
    add_wheel("right_front_wheel", "right_front_axle", (0.068, -0.041, 0.024))
    add_wheel("left_rear_wheel", "left_rear_axle", (-0.068, 0.041, 0.024))
    add_wheel("right_rear_wheel", "right_rear_axle", (-0.068, -0.041, 0.024))

    rear_hatch = model.part("rear_hatch")
    rear_hatch.visual(
        Box((0.004, 0.050, 0.024)),
        origin=Origin(xyz=(-0.002, 0.0, -0.012)),
        material=trim,
        name="hatch_panel",
    )
    model.articulation(
        "rear_hatch_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=rear_hatch,
        origin=Origin(xyz=(-0.096, 0.0, 0.104)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=3.0,
            lower=0.0,
            upper=1.20,
        ),
    )

    trunk_lid = model.part("trunk_lid")
    trunk_lid.visual(
        Box((0.055, 0.046, 0.004)),
        origin=Origin(xyz=(-0.0275, 0.0, 0.002)),
        material=paint,
        name="lid_panel",
    )
    model.articulation(
        "trunk_lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=trunk_lid,
        origin=Origin(xyz=(-0.041, 0.0, 0.080)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=3.0,
            lower=0.0,
            upper=1.00,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")

    left_front_door = object_model.get_part("left_front_door")
    right_front_door = object_model.get_part("right_front_door")
    left_rear_door = object_model.get_part("left_rear_door")
    right_rear_door = object_model.get_part("right_rear_door")
    rear_hatch = object_model.get_part("rear_hatch")
    trunk_lid = object_model.get_part("trunk_lid")

    left_front_hinge = object_model.get_articulation("left_front_door_hinge")
    right_front_hinge = object_model.get_articulation("right_front_door_hinge")
    left_rear_hinge = object_model.get_articulation("left_rear_door_hinge")
    right_rear_hinge = object_model.get_articulation("right_rear_door_hinge")
    rear_hatch_hinge = object_model.get_articulation("rear_hatch_hinge")
    trunk_lid_hinge = object_model.get_articulation("trunk_lid_hinge")

    ctx.expect_gap(
        left_front_door,
        body,
        axis="y",
        positive_elem="door_panel",
        negative_elem="cabin",
        max_gap=0.002,
        max_penetration=1e-6,
        name="left front door sits flush with body side",
    )
    ctx.expect_gap(
        left_rear_door,
        body,
        axis="y",
        positive_elem="door_panel",
        negative_elem="cabin",
        max_gap=0.002,
        max_penetration=1e-6,
        name="left rear door sits flush with body side",
    )
    ctx.expect_gap(
        body,
        right_front_door,
        axis="y",
        positive_elem="cabin",
        negative_elem="door_panel",
        max_gap=0.002,
        max_penetration=1e-6,
        name="right front door sits flush with body side",
    )
    ctx.expect_gap(
        body,
        right_rear_door,
        axis="y",
        positive_elem="cabin",
        negative_elem="door_panel",
        max_gap=0.002,
        max_penetration=1e-6,
        name="right rear door sits flush with body side",
    )
    ctx.expect_gap(
        trunk_lid,
        body,
        axis="z",
        positive_elem="lid_panel",
        negative_elem="rear_deck",
        max_gap=0.001,
        max_penetration=0.0,
        name="trunk lid remains a separate panel on the rear deck",
    )

    def elem_aabb_zmax(part_name: str, elem_name: str) -> float | None:
        aabb = ctx.part_element_world_aabb(part_name, elem=elem_name)
        return None if aabb is None else aabb[1][2]

    def elem_aabb_zmin(part_name: str, elem_name: str) -> float | None:
        aabb = ctx.part_element_world_aabb(part_name, elem=elem_name)
        return None if aabb is None else aabb[0][2]

    def elem_aabb_y_bounds(part_name: str, elem_name: str) -> tuple[float, float] | None:
        aabb = ctx.part_element_world_aabb(part_name, elem=elem_name)
        return None if aabb is None else (aabb[0][1], aabb[1][1])

    left_front_rest = elem_aabb_y_bounds("left_front_door", "door_panel")
    right_front_rest = elem_aabb_y_bounds("right_front_door", "door_panel")
    left_rear_rest = elem_aabb_y_bounds("left_rear_door", "door_panel")
    right_rear_rest = elem_aabb_y_bounds("right_rear_door", "door_panel")
    hatch_rest_z = elem_aabb_zmin("rear_hatch", "hatch_panel")
    lid_rest_z = elem_aabb_zmax("trunk_lid", "lid_panel")

    left_front_upper = left_front_hinge.motion_limits.upper if left_front_hinge.motion_limits else None
    right_front_upper = right_front_hinge.motion_limits.upper if right_front_hinge.motion_limits else None
    left_rear_upper = left_rear_hinge.motion_limits.upper if left_rear_hinge.motion_limits else None
    right_rear_upper = right_rear_hinge.motion_limits.upper if right_rear_hinge.motion_limits else None
    hatch_upper = rear_hatch_hinge.motion_limits.upper if rear_hatch_hinge.motion_limits else None
    lid_upper = trunk_lid_hinge.motion_limits.upper if trunk_lid_hinge.motion_limits else None

    if left_front_upper is not None:
        with ctx.pose({left_front_hinge: left_front_upper}):
            opened = elem_aabb_y_bounds("left_front_door", "door_panel")
        ctx.check(
            "left front door opens outward",
            left_front_rest is not None and opened is not None and opened[1] > left_front_rest[1] + 0.010,
            details=f"rest={left_front_rest}, opened={opened}",
        )
    if right_front_upper is not None:
        with ctx.pose({right_front_hinge: right_front_upper}):
            opened = elem_aabb_y_bounds("right_front_door", "door_panel")
        ctx.check(
            "right front door opens outward",
            right_front_rest is not None and opened is not None and opened[0] < right_front_rest[0] - 0.010,
            details=f"rest={right_front_rest}, opened={opened}",
        )
    if left_rear_upper is not None:
        with ctx.pose({left_rear_hinge: left_rear_upper}):
            opened = elem_aabb_y_bounds("left_rear_door", "door_panel")
        ctx.check(
            "left rear door opens outward",
            left_rear_rest is not None and opened is not None and opened[1] > left_rear_rest[1] + 0.010,
            details=f"rest={left_rear_rest}, opened={opened}",
        )
    if right_rear_upper is not None:
        with ctx.pose({right_rear_hinge: right_rear_upper}):
            opened = elem_aabb_y_bounds("right_rear_door", "door_panel")
        ctx.check(
            "right rear door opens outward",
            right_rear_rest is not None and opened is not None and opened[0] < right_rear_rest[0] - 0.010,
            details=f"rest={right_rear_rest}, opened={opened}",
        )
    if hatch_upper is not None:
        with ctx.pose({rear_hatch_hinge: hatch_upper}):
            opened_z = elem_aabb_zmin("rear_hatch", "hatch_panel")
        ctx.check(
            "rear hatch lifts upward",
            hatch_rest_z is not None and opened_z is not None and opened_z > hatch_rest_z + 0.015,
            details=f"rest={hatch_rest_z}, opened={opened_z}",
        )
    if lid_upper is not None:
        with ctx.pose({trunk_lid_hinge: lid_upper}):
            opened_z = elem_aabb_zmax("trunk_lid", "lid_panel")
        ctx.check(
            "trunk lid lifts upward",
            lid_rest_z is not None and opened_z is not None and opened_z > lid_rest_z + 0.015,
            details=f"rest={lid_rest_z}, opened={opened_z}",
        )

    return ctx.report()


object_model = build_object_model()
