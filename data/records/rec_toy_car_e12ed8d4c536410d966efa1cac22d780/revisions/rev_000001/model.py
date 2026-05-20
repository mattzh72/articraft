from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="toy_boxy_suv")

    red = model.material("toy_red", rgba=(0.82, 0.05, 0.035, 1.0))
    dark_red = model.material("dark_red_plastic", rgba=(0.50, 0.02, 0.02, 1.0))
    black = model.material("soft_black_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    charcoal = model.material("smoked_window", rgba=(0.02, 0.04, 0.055, 1.0))
    silver = model.material("silver_plastic", rgba=(0.72, 0.72, 0.68, 1.0))

    body = model.part("body")

    # A simple, rugged toy casting: a high one-piece boxy lower shell, a blocky
    # cabin, and heavy sills leave clearance for the chunky wheels.
    body.visual(
        Box((0.330, 0.128, 0.048)),
        origin=Origin(xyz=(0.000, 0.000, 0.098)),
        material=red,
        name="lower_shell",
    )
    body.visual(
        Box((0.152, 0.118, 0.052)),
        origin=Origin(xyz=(0.010, 0.000, 0.136)),
        material=red,
        name="cabin_block",
    )
    body.visual(
        Box((0.156, 0.124, 0.012)),
        origin=Origin(xyz=(0.010, 0.000, 0.168)),
        material=dark_red,
        name="flat_roof",
    )
    body.visual(
        Box((0.094, 0.124, 0.014)),
        origin=Origin(xyz=(0.108, 0.000, 0.127)),
        material=red,
        name="hood_deck",
    )
    body.visual(
        Box((0.088, 0.118, 0.010)),
        origin=Origin(xyz=(-0.101, 0.000, 0.122)),
        material=red,
        name="rear_body_deck",
    )
    body.visual(
        Box((0.318, 0.052, 0.014)),
        origin=Origin(xyz=(0.000, 0.000, 0.066)),
        material=dark_red,
        name="underbody_spine",
    )

    # Axle shafts are fixed in the casting; the wheel parts rotate around them.
    for axle_x, axle_name in ((0.094, "front_axle_shaft"), (-0.106, "rear_axle_shaft")):
        body.visual(
            Cylinder(radius=0.0046, length=0.190),
            origin=Origin(xyz=(axle_x, 0.000, 0.036), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=silver,
            name=axle_name,
        )
        body.visual(
            Box((0.024, 0.026, 0.046)),
            origin=Origin(xyz=(axle_x, 0.000, 0.055)),
            material=dark_red,
            name=f"{axle_name}_boss",
        )

    # Blocky flared arches: three connected toy-casting pads around each wheel.
    for axle_x, arch_tag in ((0.094, "front"), (-0.106, "rear")):
        for side_y, side_tag in ((0.071, "side_0"), (-0.071, "side_1")):
            body.visual(
                Box((0.080, 0.018, 0.013)),
                origin=Origin(xyz=(axle_x, side_y, 0.079)),
                material=dark_red,
                name=f"{arch_tag}_{side_tag}_arch_crown",
            )
            body.visual(
                Box((0.010, 0.018, 0.032)),
                origin=Origin(xyz=(axle_x + 0.046, side_y, 0.064)),
                material=dark_red,
                name=f"{arch_tag}_{side_tag}_arch_front_leg",
            )
            body.visual(
                Box((0.010, 0.018, 0.032)),
                origin=Origin(xyz=(axle_x - 0.046, side_y, 0.064)),
                material=dark_red,
                name=f"{arch_tag}_{side_tag}_arch_rear_leg",
            )

    # Simple molded windows on the non-moving shell.
    body.visual(
        Box((0.058, 0.004, 0.027)),
        origin=Origin(xyz=(0.039, 0.061, 0.143)),
        material=charcoal,
        name="side_window_0",
    )
    body.visual(
        Box((0.058, 0.004, 0.027)),
        origin=Origin(xyz=(-0.037, 0.061, 0.143)),
        material=charcoal,
        name="side_window_1",
    )
    body.visual(
        Box((0.058, 0.004, 0.027)),
        origin=Origin(xyz=(0.039, -0.061, 0.143)),
        material=charcoal,
        name="side_window_2",
    )
    body.visual(
        Box((0.058, 0.004, 0.027)),
        origin=Origin(xyz=(-0.037, -0.061, 0.143)),
        material=charcoal,
        name="side_window_3",
    )
    body.visual(
        Box((0.006, 0.090, 0.028)),
        origin=Origin(xyz=(0.085, 0.000, 0.144)),
        material=charcoal,
        name="windshield",
    )
    body.visual(
        Box((0.006, 0.096, 0.030)),
        origin=Origin(xyz=(-0.155, 0.000, 0.096)),
        material=red,
        name="rear_tail_panel",
    )
    for side_y, suffix in ((0.064, "0"), (-0.064, "1")):
        body.visual(
            Box((0.012, 0.008, 0.056)),
            origin=Origin(xyz=(-0.151, side_y, 0.136)),
            material=red,
            name=f"rear_hatch_pillar_{suffix}",
        )

    # Body-side hinge pads that visually carry the four side doors.
    for x_hinge in (0.074, -0.008):
        for side_y, suffix in ((0.064, "side_0"), (-0.064, "side_1")):
            body.visual(
                Box((0.007, 0.004, 0.058)),
                origin=Origin(xyz=(x_hinge, side_y, 0.100)),
                material=dark_red,
                name=f"door_hinge_pad_{suffix}_{x_hinge:+.3f}",
            )

    # Rear upper hatch and trunk-lid hinge pads on the toy casting.
    body.visual(
        Cylinder(radius=0.0026, length=0.128),
        origin=Origin(xyz=(-0.155, 0.000, 0.158), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="hatch_top_hinge_pin",
    )
    body.visual(
        Cylinder(radius=0.0024, length=0.074),
        origin=Origin(xyz=(-0.071, 0.000, 0.129), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="trunk_deck_hinge_pin",
    )

    def add_side_door(
        name: str,
        x_hinge: float,
        y_hinge: float,
        length: float,
        side_sign: float,
        axis_sign: float,
    ) -> None:
        door = model.part(name)
        door.visual(
            Box((length, 0.004, 0.057)),
            origin=Origin(xyz=(-length / 2.0, side_sign * 0.003, 0.000)),
            material=red,
            name="door_panel",
        )
        door.visual(
            Box((length * 0.56, 0.0025, 0.021)),
            origin=Origin(xyz=(-length * 0.52, side_sign * 0.0055, 0.011)),
            material=charcoal,
            name="door_window",
        )
        door.visual(
            Box((0.012, 0.004, 0.004)),
            origin=Origin(xyz=(-length * 0.76, side_sign * 0.0064, -0.010)),
            material=black,
            name="door_handle",
        )
        door.visual(
            Cylinder(radius=0.0022, length=0.059),
            origin=Origin(xyz=(0.000, side_sign * 0.003, 0.000)),
            material=silver,
            name="hinge_barrel",
        )
        model.articulation(
            f"body_to_{name}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=door,
            origin=Origin(xyz=(x_hinge, y_hinge, 0.100)),
            axis=(0.0, 0.0, axis_sign),
            motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=0.0, upper=1.25),
        )

    add_side_door("front_door_0", 0.074, 0.068, 0.074, 1.0, -1.0)
    add_side_door("rear_door_0", -0.008, 0.068, 0.068, 1.0, -1.0)
    add_side_door("front_door_1", 0.074, -0.068, 0.074, -1.0, 1.0)
    add_side_door("rear_door_1", -0.008, -0.068, 0.068, -1.0, 1.0)

    hatch = model.part("rear_hatch")
    hatch.visual(
        Box((0.006, 0.104, 0.036)),
        origin=Origin(xyz=(-0.003, 0.000, -0.018)),
        material=red,
        name="hatch_frame",
    )
    hatch.visual(
        Box((0.0025, 0.084, 0.023)),
        origin=Origin(xyz=(-0.0060, 0.000, -0.019)),
        material=charcoal,
        name="hatch_window",
    )
    hatch.visual(
        Cylinder(radius=0.0022, length=0.096),
        origin=Origin(xyz=(0.000, 0.000, 0.000), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="hatch_hinge_barrel",
    )
    model.articulation(
        "body_to_rear_hatch",
        ArticulationType.REVOLUTE,
        parent=body,
        child=hatch,
        origin=Origin(xyz=(-0.155, 0.000, 0.158)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=1.10),
    )

    trunk_lid = model.part("trunk_lid")
    trunk_lid.visual(
        Box((0.086, 0.112, 0.006)),
        origin=Origin(xyz=(-0.043, 0.000, 0.003)),
        material=red,
        name="lid_panel",
    )
    trunk_lid.visual(
        Box((0.030, 0.004, 0.004)),
        origin=Origin(xyz=(-0.072, 0.000, 0.008)),
        material=black,
        name="lid_pull",
    )
    trunk_lid.visual(
        Cylinder(radius=0.0020, length=0.072),
        origin=Origin(xyz=(0.000, 0.000, 0.002), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="lid_hinge_barrel",
    )
    model.articulation(
        "body_to_trunk_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=trunk_lid,
        origin=Origin(xyz=(-0.071, 0.000, 0.129)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=2.0, lower=0.0, upper=1.05),
    )

    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.033,
            0.032,
            inner_radius=0.022,
            carcass=TireCarcass(belt_width_ratio=0.72, sidewall_bulge=0.05),
            tread=TireTread(style="block", depth=0.0045, count=18, land_ratio=0.55),
            sidewall=TireSidewall(style="square", bulge=0.025),
            shoulder=TireShoulder(width=0.004, radius=0.002),
        ),
        "chunky_toy_tire",
    )
    rim_mesh = mesh_from_geometry(
        WheelGeometry(
            0.023,
            0.034,
            rim=WheelRim(inner_radius=0.015, flange_height=0.003, flange_thickness=0.002),
            hub=WheelHub(
                radius=0.008,
                width=0.026,
                cap_style="flat",
                bolt_pattern=BoltPattern(count=5, circle_diameter=0.012, hole_diameter=0.002),
            ),
            face=WheelFace(dish_depth=0.0025, front_inset=0.001, rear_inset=0.001),
            spokes=WheelSpokes(style="straight", count=5, thickness=0.0025, window_radius=0.004),
            bore=WheelBore(style="round", diameter=0.009),
        ),
        "silver_toy_rim",
    )

    def add_wheel(name: str, x: float, y: float) -> None:
        wheel = model.part(name)
        wheel.visual(tire_mesh, material=black, name="tire")
        wheel.visual(rim_mesh, material=silver, name="rim")
        model.articulation(
            f"body_to_{name}",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=wheel,
            origin=Origin(xyz=(x, y, 0.036), rpy=(0.0, 0.0, pi / 2.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=12.0),
        )

    add_wheel("wheel_0", 0.094, 0.086)
    add_wheel("wheel_1", 0.094, -0.086)
    add_wheel("wheel_2", -0.106, 0.086)
    add_wheel("wheel_3", -0.106, -0.086)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    door_joints = [
        object_model.get_articulation("body_to_front_door_0"),
        object_model.get_articulation("body_to_rear_door_0"),
        object_model.get_articulation("body_to_front_door_1"),
        object_model.get_articulation("body_to_rear_door_1"),
    ]
    wheel_joints = [object_model.get_articulation(f"body_to_wheel_{i}") for i in range(4)]
    hatch_joint = object_model.get_articulation("body_to_rear_hatch")
    trunk_joint = object_model.get_articulation("body_to_trunk_lid")

    ctx.check("four side doors are hinged", len(door_joints) == 4)
    ctx.check("four wheels rotate continuously", len(wheel_joints) == 4)
    ctx.check("rear hatch is hinged", hatch_joint.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("trunk lid is hinged", trunk_joint.articulation_type == ArticulationType.REVOLUTE)

    for wheel_name, axle_elem in (
        ("wheel_0", "front_axle_shaft"),
        ("wheel_1", "front_axle_shaft"),
        ("wheel_2", "rear_axle_shaft"),
        ("wheel_3", "rear_axle_shaft"),
    ):
        ctx.allow_overlap(
            "body",
            wheel_name,
            elem_a=axle_elem,
            elem_b="rim",
            reason="The fixed toy axle shaft is intentionally captured inside the wheel hub bore.",
        )
        ctx.expect_overlap(
            "body",
            wheel_name,
            axes="y",
            elem_a=axle_elem,
            elem_b="rim",
            min_overlap=0.020,
            name=f"{wheel_name} retained on axle shaft",
        )
        ctx.expect_within(
            "body",
            wheel_name,
            axes="xz",
            inner_elem=axle_elem,
            outer_elem="rim",
            margin=0.004,
            name=f"{wheel_name} axle centered in hub",
        )

    front_door_0 = object_model.get_part("front_door_0")
    front_door_1 = object_model.get_part("front_door_1")
    rear_hatch = object_model.get_part("rear_hatch")
    trunk_lid = object_model.get_part("trunk_lid")

    closed_left = ctx.part_element_world_aabb(front_door_0, elem="door_panel")
    with ctx.pose({door_joints[0]: 1.0}):
        open_left = ctx.part_element_world_aabb(front_door_0, elem="door_panel")
    ctx.check(
        "left side door swings outward",
        closed_left is not None
        and open_left is not None
        and open_left[1][1] > closed_left[1][1] + 0.020,
        details=f"closed={closed_left}, open={open_left}",
    )

    closed_right = ctx.part_element_world_aabb(front_door_1, elem="door_panel")
    with ctx.pose({door_joints[2]: 1.0}):
        open_right = ctx.part_element_world_aabb(front_door_1, elem="door_panel")
    ctx.check(
        "right side door swings outward",
        closed_right is not None
        and open_right is not None
        and open_right[0][1] < closed_right[0][1] - 0.020,
        details=f"closed={closed_right}, open={open_right}",
    )

    closed_hatch = ctx.part_element_world_aabb(rear_hatch, elem="hatch_frame")
    with ctx.pose({hatch_joint: 0.9}):
        open_hatch = ctx.part_element_world_aabb(rear_hatch, elem="hatch_frame")
    ctx.check(
        "rear hatch lifts upward",
        closed_hatch is not None
        and open_hatch is not None
        and open_hatch[0][2] > closed_hatch[0][2] + 0.010,
        details=f"closed={closed_hatch}, open={open_hatch}",
    )

    closed_lid = ctx.part_element_world_aabb(trunk_lid, elem="lid_panel")
    with ctx.pose({trunk_joint: 0.8}):
        open_lid = ctx.part_element_world_aabb(trunk_lid, elem="lid_panel")
    ctx.check(
        "trunk lid lifts from rear deck",
        closed_lid is not None
        and open_lid is not None
        and open_lid[1][2] > closed_lid[1][2] + 0.020,
        details=f"closed={closed_lid}, open={open_lid}",
    )

    return ctx.report()


object_model = build_object_model()
