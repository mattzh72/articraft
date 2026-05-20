from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
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
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="toy_police_sedan")

    white = model.material("police_white", rgba=(0.92, 0.94, 0.92, 1.0))
    black = model.material("gloss_black", rgba=(0.01, 0.012, 0.014, 1.0))
    dark = model.material("dark_trim", rgba=(0.03, 0.035, 0.04, 1.0))
    glass = model.material("smoky_blue_glass", rgba=(0.12, 0.28, 0.45, 0.72))
    red = model.material("red_lens", rgba=(0.95, 0.05, 0.05, 1.0))
    blue = model.material("blue_lens", rgba=(0.04, 0.14, 0.95, 1.0))
    amber = model.material("warm_headlamp", rgba=(1.0, 0.82, 0.28, 1.0))
    silver = model.material("toy_silver", rgba=(0.74, 0.74, 0.70, 1.0))
    rubber = model.material("soft_black_rubber", rgba=(0.006, 0.006, 0.006, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.310, 0.108, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=white,
        name="lower_body",
    )
    body.visual(
        Box((0.108, 0.104, 0.018)),
        origin=Origin(xyz=(0.094, 0.0, 0.0815)),
        material=white,
        name="front_hood",
    )
    body.visual(
        Box((0.076, 0.104, 0.011)),
        origin=Origin(xyz=(-0.108, 0.0, 0.0780)),
        material=white,
        name="rear_deck",
    )
    body.visual(
        Box((0.102, 0.096, 0.052)),
        origin=Origin(xyz=(0.000, 0.0, 0.0985)),
        material=white,
        name="passenger_cabin",
    )
    body.visual(
        Box((0.004, 0.078, 0.028)),
        origin=Origin(xyz=(0.053, 0.0, 0.101)),
        material=glass,
        name="front_windshield",
    )
    body.visual(
        Box((0.004, 0.078, 0.028)),
        origin=Origin(xyz=(-0.053, 0.0, 0.101)),
        material=glass,
        name="rear_windshield",
    )
    body.visual(
        Box((0.050, 0.040, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.1295)),
        material=dark,
        name="roof_light_base",
    )
    body.visual(
        Box((0.022, 0.038, 0.010)),
        origin=Origin(xyz=(0.012, 0.0, 0.1395)),
        material=blue,
        name="blue_roof_lens",
    )
    body.visual(
        Box((0.022, 0.038, 0.010)),
        origin=Origin(xyz=(-0.012, 0.0, 0.1395)),
        material=red,
        name="red_roof_lens",
    )
    body.visual(
        Box((0.006, 0.070, 0.020)),
        origin=Origin(xyz=(0.157, 0.0, 0.060)),
        material=dark,
        name="front_grille",
    )
    body.visual(
        Box((0.004, 0.024, 0.011)),
        origin=Origin(xyz=(0.159, 0.034, 0.066)),
        material=amber,
        name="headlamp_0",
    )
    body.visual(
        Box((0.004, 0.024, 0.011)),
        origin=Origin(xyz=(0.159, -0.034, 0.066)),
        material=amber,
        name="headlamp_1",
    )
    body.visual(
        Box((0.006, 0.078, 0.012)),
        origin=Origin(xyz=(-0.158, 0.0, 0.059)),
        material=dark,
        name="rear_bumper",
    )
    body.visual(
        Box((0.052, 0.003, 0.010)),
        origin=Origin(xyz=(0.106, 0.0545, 0.0775)),
        material=black,
        name="side_stripe_0",
    )
    body.visual(
        Box((0.052, 0.003, 0.010)),
        origin=Origin(xyz=(0.106, -0.0545, 0.0775)),
        material=black,
        name="side_stripe_1",
    )

    def add_front_door(name: str, y: float, axis_z: float, outward: float) -> None:
        door = model.part(name)
        door.visual(
            Box((0.080, 0.006, 0.052)),
            origin=Origin(xyz=(-0.040, 0.0, 0.0)),
            material=white,
            name="door_panel",
        )
        door.visual(
            Box((0.044, 0.003, 0.021)),
            origin=Origin(xyz=(-0.040, outward * 0.0045, 0.011)),
            material=glass,
            name="side_window",
        )
        door.visual(
            Box((0.050, 0.003, 0.008)),
            origin=Origin(xyz=(-0.041, outward * 0.0040, -0.014)),
            material=black,
            name="door_stripe",
        )
        door.visual(
            Box((0.014, 0.003, 0.004)),
            origin=Origin(xyz=(-0.065, outward * 0.0040, -0.007)),
            material=dark,
            name="door_handle",
        )
        door.visual(
            Cylinder(radius=0.0030, length=0.054),
            origin=Origin(xyz=(0.0, 0.0, 0.0)),
            material=dark,
            name="hinge_barrel",
        )
        model.articulation(
            f"body_to_{name}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=door,
            origin=Origin(xyz=(0.055, y, 0.096)),
            axis=(0.0, 0.0, axis_z),
            motion_limits=MotionLimits(effort=1.5, velocity=2.0, lower=0.0, upper=1.10),
        )

    add_front_door("front_door_0", 0.057, -1.0, 1.0)
    add_front_door("front_door_1", -0.057, 1.0, -1.0)

    trunk_lid = model.part("trunk_lid")
    trunk_lid.visual(
        Box((0.081, 0.102, 0.006)),
        origin=Origin(xyz=(0.0405, 0.0, 0.0)),
        material=white,
        name="lid_panel",
    )
    trunk_lid.visual(
        Cylinder(radius=0.0032, length=0.104),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="hinge_barrel",
    )
    trunk_lid.visual(
        Box((0.030, 0.003, 0.004)),
        origin=Origin(xyz=(0.069, -0.0525, -0.001)),
        material=dark,
        name="trunk_handle",
    )
    model.articulation(
        "body_to_trunk_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=trunk_lid,
        origin=Origin(xyz=(-0.151, 0.0, 0.0865)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=1.5, lower=0.0, upper=1.05),
    )

    tire = TireGeometry(
        0.026,
        0.018,
        inner_radius=0.017,
        carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.05),
        tread=TireTread(style="circumferential", depth=0.0012, count=3, land_ratio=0.60),
        grooves=(TireGroove(center_offset=0.0, width=0.0022, depth=0.0010),),
        sidewall=TireSidewall(style="rounded", bulge=0.04),
        shoulder=TireShoulder(width=0.0022, radius=0.0012),
    )
    wheel = WheelGeometry(
        0.018,
        0.014,
        rim=WheelRim(inner_radius=0.012, flange_height=0.0018, flange_thickness=0.0012),
        hub=WheelHub(
            radius=0.0065,
            width=0.012,
            cap_style="domed",
            bolt_pattern=BoltPattern(count=5, circle_diameter=0.008, hole_diameter=0.0012),
        ),
        face=WheelFace(dish_depth=0.0015, front_inset=0.0008, rear_inset=0.0008),
        spokes=WheelSpokes(style="split_y", count=5, thickness=0.0012, window_radius=0.003),
        bore=WheelBore(style="round", diameter=0.003),
    )

    def add_wheel(name: str, x: float, y: float, outward_sign: float) -> None:
        wheel_part = model.part(name)
        wheel_part.visual(
            mesh_from_geometry(tire, f"{name}_tire"),
            material=rubber,
            name="tire",
        )
        wheel_part.visual(
            mesh_from_geometry(wheel, f"{name}_rim"),
            material=silver,
            name="rim",
        )
        model.articulation(
            f"body_to_{name}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=wheel_part,
            origin=Origin(
                xyz=(x, y, 0.029),
                rpy=(0.0, 0.0, outward_sign * math.pi / 2.0),
            ),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=0.8, velocity=20.0),
        )

    add_wheel("front_wheel_0", 0.100, 0.063, 1.0)
    add_wheel("front_wheel_1", 0.100, -0.063, -1.0)
    add_wheel("rear_wheel_0", -0.104, 0.063, 1.0)
    add_wheel("rear_wheel_1", -0.104, -0.063, -1.0)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")

    wheel_joints = [
        object_model.get_articulation("body_to_front_wheel_0"),
        object_model.get_articulation("body_to_front_wheel_1"),
        object_model.get_articulation("body_to_rear_wheel_0"),
        object_model.get_articulation("body_to_rear_wheel_1"),
    ]
    for joint in wheel_joints:
        ctx.check(
            f"{joint.name} is continuous",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            details=f"{joint.name} type={joint.articulation_type}",
        )

    door_0 = object_model.get_part("front_door_0")
    door_1 = object_model.get_part("front_door_1")
    trunk_lid = object_model.get_part("trunk_lid")
    door_0_joint = object_model.get_articulation("body_to_front_door_0")
    door_1_joint = object_model.get_articulation("body_to_front_door_1")
    trunk_joint = object_model.get_articulation("body_to_trunk_lid")

    ctx.check(
        "front doors use vertical hinges",
        door_0_joint.axis == (0.0, 0.0, -1.0) and door_1_joint.axis == (0.0, 0.0, 1.0),
        details=f"axes={door_0_joint.axis}, {door_1_joint.axis}",
    )
    ctx.check(
        "trunk uses rear horizontal hinge",
        trunk_joint.axis == (0.0, -1.0, 0.0),
        details=f"axis={trunk_joint.axis}",
    )

    ctx.expect_gap(
        door_0,
        body,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0001,
        positive_elem="door_panel",
        negative_elem="lower_body",
        name="closed door 0 is seated on body side",
    )
    ctx.expect_gap(
        body,
        door_1,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0001,
        positive_elem="lower_body",
        negative_elem="door_panel",
        name="closed door 1 is seated on body side",
    )
    ctx.expect_gap(
        trunk_lid,
        body,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0001,
        positive_elem="lid_panel",
        negative_elem="rear_deck",
        name="closed trunk lid rests on rear deck",
    )

    for wheel_name in ("front_wheel_0", "rear_wheel_0"):
        ctx.expect_gap(
            object_model.get_part(wheel_name),
            body,
            axis="y",
            max_gap=0.001,
            max_penetration=0.0002,
            positive_elem="tire",
            negative_elem="lower_body",
            name=f"{wheel_name} is exposed beside body",
        )
    for wheel_name in ("front_wheel_1", "rear_wheel_1"):
        ctx.expect_gap(
            body,
            object_model.get_part(wheel_name),
            axis="y",
            max_gap=0.001,
            max_penetration=0.0002,
            positive_elem="lower_body",
            negative_elem="tire",
            name=f"{wheel_name} is exposed beside body",
        )

    def _aabb_center_axis(part, elem: str, axis_index: int) -> float | None:
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        lo, hi = aabb
        return 0.5 * (lo[axis_index] + hi[axis_index])

    closed_door_0_y = _aabb_center_axis(door_0, "door_panel", 1)
    closed_door_1_y = _aabb_center_axis(door_1, "door_panel", 1)
    closed_trunk_z = _aabb_center_axis(trunk_lid, "lid_panel", 2)
    with ctx.pose({door_0_joint: 0.80, door_1_joint: 0.80, trunk_joint: 0.70}):
        open_door_0_y = _aabb_center_axis(door_0, "door_panel", 1)
        open_door_1_y = _aabb_center_axis(door_1, "door_panel", 1)
        open_trunk_z = _aabb_center_axis(trunk_lid, "lid_panel", 2)

    ctx.check(
        "door 0 swings outward",
        closed_door_0_y is not None and open_door_0_y is not None and open_door_0_y > closed_door_0_y + 0.020,
        details=f"closed={closed_door_0_y}, open={open_door_0_y}",
    )
    ctx.check(
        "door 1 swings outward",
        closed_door_1_y is not None and open_door_1_y is not None and open_door_1_y < closed_door_1_y - 0.020,
        details=f"closed={closed_door_1_y}, open={open_door_1_y}",
    )
    ctx.check(
        "trunk lid opens upward",
        closed_trunk_z is not None and open_trunk_z is not None and open_trunk_z > closed_trunk_z + 0.010,
        details=f"closed={closed_trunk_z}, open={open_trunk_z}",
    )

    return ctx.report()


object_model = build_object_model()
