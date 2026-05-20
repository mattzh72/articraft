from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
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


def _car_side_prism() -> MeshGeometry:
    """One connected, die-cast-style coupe body shell extruded across the car."""
    half_width = 0.0165
    profile = [
        (-0.0390, 0.0070),  # front valance bottom
        (0.0390, 0.0070),  # rear valance bottom
        (0.0390, 0.0180),  # rear fascia
        (0.0350, 0.0220),  # trunk rear edge
        (0.0200, 0.0232),  # separate low trunk deck under the lid
        (0.0120, 0.0288),  # rear roof shoulder
        (-0.0060, 0.0302),  # roof crown
        (-0.0145, 0.0230),  # windshield base / cowl
        (-0.0325, 0.0200),  # short hood
        (-0.0390, 0.0150),  # nose
    ]

    geom = MeshGeometry()
    left_ids = [geom.add_vertex(x, half_width, z) for x, z in profile]
    right_ids = [geom.add_vertex(x, -half_width, z) for x, z in profile]
    count = len(profile)

    for index in range(1, count - 1):
        geom.add_face(left_ids[0], left_ids[index], left_ids[index + 1])
        geom.add_face(right_ids[0], right_ids[index + 1], right_ids[index])

    for index in range(count):
        j = (index + 1) % count
        geom.add_face(left_ids[index], right_ids[index], right_ids[j])
        geom.add_face(left_ids[index], right_ids[j], left_ids[j])

    return geom


def _add_side_door(
    model: ArticulatedObject,
    *,
    name: str,
    parent,
    y: float,
    axis_sign: float,
    body_paint: Material,
    seam_black: Material,
    glass_blue: Material,
    chrome: Material,
) -> None:
    door = model.part(name)
    door.visual(
        Box((0.0260, 0.0010, 0.0128)),
        origin=Origin(xyz=(0.0130, 0.0, 0.0)),
        material=body_paint,
        name="door_panel",
    )
    door.visual(
        Box((0.0158, 0.0012, 0.0052)),
        origin=Origin(xyz=(0.0108, axis_sign * 0.00015, 0.0044)),
        material=glass_blue,
        name="side_window",
    )
    door.visual(
        Box((0.0010, 0.00125, 0.0129)),
        origin=Origin(xyz=(0.0007, axis_sign * 0.00020, 0.0)),
        material=seam_black,
        name="front_cut",
    )
    door.visual(
        Box((0.0010, 0.00125, 0.0115)),
        origin=Origin(xyz=(0.0254, axis_sign * 0.00020, -0.0007)),
        material=seam_black,
        name="rear_cut",
    )
    door.visual(
        Box((0.0260, 0.00125, 0.0009)),
        origin=Origin(xyz=(0.0130, axis_sign * 0.00020, -0.0060)),
        material=seam_black,
        name="lower_cut",
    )
    door.visual(
        Cylinder(radius=0.0009, length=0.0145),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=chrome,
        name="hinge_barrel",
    )
    door.visual(
        Box((0.0032, 0.0008, 0.0010)),
        origin=Origin(xyz=(0.0185, axis_sign * 0.00075, -0.0015)),
        material=chrome,
        name="door_handle",
    )

    model.articulation(
        f"body_to_{name}",
        ArticulationType.REVOLUTE,
        parent=parent,
        child=door,
        origin=Origin(xyz=(-0.0160, y, 0.0182)),
        axis=(0.0, 0.0, axis_sign),
        motion_limits=MotionLimits(effort=0.2, velocity=2.0, lower=0.0, upper=1.22),
    )


def _add_wheel(
    model: ArticulatedObject,
    *,
    name: str,
    parent,
    x: float,
    y: float,
    z: float,
    tire_mesh,
    rim_mesh,
    rubber: Material,
    chrome: Material,
) -> None:
    wheel = model.part(name)
    wheel.visual(tire_mesh, material=rubber, name="tire")
    wheel.visual(rim_mesh, material=chrome, name="rim")
    wheel.visual(
        Cylinder(radius=0.0046, length=0.0056),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="bearing_sleeve",
    )
    model.articulation(
        f"body_to_{name}",
        ArticulationType.CONTINUOUS,
        parent=parent,
        child=wheel,
        origin=Origin(xyz=(x, y, z), rpy=(0.0, 0.0, math.pi / 2.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.05, velocity=30.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="diecast_toy_coupe")

    body_paint = model.material("toy_red_enamel", rgba=(0.72, 0.03, 0.02, 1.0))
    seam_black = model.material("black_panel_gaps", rgba=(0.015, 0.014, 0.013, 1.0))
    glass_blue = model.material("blue_tinted_glass", rgba=(0.25, 0.55, 0.78, 0.55))
    rubber = model.material("soft_black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))
    chrome = model.material("bright_diecast_chrome", rgba=(0.82, 0.84, 0.86, 1.0))
    chassis_black = model.material("black_chassis", rgba=(0.08, 0.08, 0.075, 1.0))
    tail_red = model.material("tail_lens_red", rgba=(0.9, 0.02, 0.01, 1.0))
    headlight_clear = model.material("clear_headlight", rgba=(0.92, 0.92, 0.82, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_geometry(_car_side_prism(), "coupe_body_shell"),
        material=body_paint,
        name="body_shell",
    )
    body.visual(
        Box((0.0750, 0.0260, 0.0022)),
        origin=Origin(xyz=(0.0000, 0.0, 0.0062)),
        material=chassis_black,
        name="base_plate",
    )
    body.visual(
        Box((0.0208, 0.0272, 0.00045)),
        origin=Origin(xyz=(0.0272, 0.0, 0.02345)),
        material=seam_black,
        name="rear_deck_gasket",
    )
    body.visual(
        Box((0.0011, 0.0285, 0.0008)),
        origin=Origin(xyz=(0.0170, 0.0, 0.0240)),
        material=seam_black,
        name="trunk_front_seam",
    )
    body.visual(
        Box((0.0010, 0.0270, 0.0008)),
        origin=Origin(xyz=(0.0370, 0.0, 0.0229)),
        material=seam_black,
        name="trunk_rear_seam",
    )
    body.visual(
        Box((0.0200, 0.0009, 0.0008)),
        origin=Origin(xyz=(0.0272, 0.0140, 0.0233)),
        material=seam_black,
        name="trunk_side_seam_0",
    )
    body.visual(
        Box((0.0200, 0.0009, 0.0008)),
        origin=Origin(xyz=(0.0272, -0.0140, 0.0233)),
        material=seam_black,
        name="trunk_side_seam_1",
    )
    body.visual(
        Box((0.0170, 0.0008, 0.0065)),
        origin=Origin(xyz=(0.0025, 0.0169, 0.0245)),
        material=glass_blue,
        name="left_cabin_glass",
    )
    body.visual(
        Box((0.0170, 0.0008, 0.0065)),
        origin=Origin(xyz=(0.0025, -0.0169, 0.0245)),
        material=glass_blue,
        name="right_cabin_glass",
    )
    body.visual(
        Box((0.0100, 0.0220, 0.0008)),
        origin=Origin(xyz=(-0.0125, 0.0, 0.0240), rpy=(0.0, math.radians(38.0), 0.0)),
        material=glass_blue,
        name="windshield",
    )
    body.visual(
        Box((0.0090, 0.0210, 0.0008)),
        origin=Origin(xyz=(0.0155, 0.0, 0.0250), rpy=(0.0, math.radians(-35.0), 0.0)),
        material=glass_blue,
        name="rear_window",
    )
    body.visual(
        Box((0.0018, 0.0180, 0.0020)),
        origin=Origin(xyz=(-0.0397, 0.0, 0.0145)),
        material=headlight_clear,
        name="headlights",
    )
    body.visual(
        Box((0.0018, 0.0200, 0.0020)),
        origin=Origin(xyz=(0.0397, 0.0, 0.0150)),
        material=tail_red,
        name="tail_lights",
    )

    for side_name, y, side_sign in (
        ("left", 0.0185, 1.0),
        ("right", -0.0185, -1.0),
    ):
        body.visual(
            Box((0.0022, 0.00152, 0.0148)),
            origin=Origin(xyz=(-0.0160, side_sign * 0.01726, 0.0182)),
            material=chrome,
            name=f"{side_name}_door_hinge_bracket",
        )
        body.visual(
            Cylinder(radius=0.00048, length=0.0152),
            origin=Origin(xyz=(-0.0160, y, 0.0182)),
            material=chrome,
            name=f"{side_name}_door_pin",
        )

    body.visual(
        Box((0.0040, 0.0180, 0.0015)),
        origin=Origin(xyz=(0.0170, 0.0, 0.02355)),
        material=chrome,
        name="trunk_hinge_pedestal",
    )
    body.visual(
        Cylinder(radius=0.00048, length=0.0200),
        origin=Origin(xyz=(0.0170, 0.0, 0.02455), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="trunk_pin",
    )
    body.visual(
        Cylinder(radius=0.00093, length=0.0450),
        origin=Origin(xyz=(-0.0255, 0.0, 0.0076), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="front_axle",
    )
    body.visual(
        Cylinder(radius=0.00093, length=0.0450),
        origin=Origin(xyz=(0.0260, 0.0, 0.0076), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="rear_axle",
    )

    _add_side_door(
        model,
        name="left_door",
        parent=body,
        y=0.0185,
        axis_sign=1.0,
        body_paint=body_paint,
        seam_black=seam_black,
        glass_blue=glass_blue,
        chrome=chrome,
    )
    _add_side_door(
        model,
        name="right_door",
        parent=body,
        y=-0.0185,
        axis_sign=-1.0,
        body_paint=body_paint,
        seam_black=seam_black,
        glass_blue=glass_blue,
        chrome=chrome,
    )

    trunk_lid = model.part("trunk_lid")
    trunk_lid.visual(
        Box((0.0202, 0.0262, 0.0012)),
        origin=Origin(xyz=(0.0101, 0.0, 0.0006)),
        material=body_paint,
        name="lid_panel",
    )
    trunk_lid.visual(
        Box((0.0188, 0.0007, 0.0010)),
        origin=Origin(xyz=(0.0106, 0.01305, 0.0009)),
        material=seam_black,
        name="lid_side_gap_0",
    )
    trunk_lid.visual(
        Box((0.0188, 0.0007, 0.0010)),
        origin=Origin(xyz=(0.0106, -0.01305, 0.0009)),
        material=seam_black,
        name="lid_side_gap_1",
    )
    trunk_lid.visual(
        Cylinder(radius=0.00085, length=0.0205),
        origin=Origin(xyz=(0.0, 0.0, 0.0000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="hinge_barrel",
    )
    model.articulation(
        "body_to_trunk_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=trunk_lid,
        origin=Origin(xyz=(0.0170, 0.0, 0.02455)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.18, velocity=1.6, lower=0.0, upper=1.10),
    )

    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.0072,
            0.0052,
            inner_radius=0.0047,
            carcass=TireCarcass(belt_width_ratio=0.72, sidewall_bulge=0.05),
            tread=TireTread(style="block", depth=0.00045, count=20, land_ratio=0.58),
            grooves=(TireGroove(center_offset=0.0, width=0.00045, depth=0.00025),),
            sidewall=TireSidewall(style="rounded", bulge=0.04),
            shoulder=TireShoulder(width=0.00045, radius=0.00025),
        ),
        "toy_coupe_tire",
    )
    rim_mesh = mesh_from_geometry(
        WheelGeometry(
            0.0050,
            0.0050,
            rim=WheelRim(inner_radius=0.0033, flange_height=0.00045, flange_thickness=0.00025),
            hub=WheelHub(
                radius=0.0017,
                width=0.0040,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=5, circle_diameter=0.0024, hole_diameter=0.00028),
            ),
            face=WheelFace(dish_depth=0.0006, front_inset=0.00025, rear_inset=0.00020),
            spokes=WheelSpokes(style="split_y", count=5, thickness=0.00025, window_radius=0.00075),
            bore=WheelBore(style="round", diameter=0.0018),
        ),
        "toy_coupe_rim",
    )
    for wheel_name, x, y in (
        ("front_left_wheel", -0.0255, 0.0205),
        ("front_right_wheel", -0.0255, -0.0205),
        ("rear_left_wheel", 0.0260, 0.0205),
        ("rear_right_wheel", 0.0260, -0.0205),
    ):
        _add_wheel(
            model,
            name=wheel_name,
            parent=body,
            x=x,
            y=y,
            z=0.0076,
            tire_mesh=tire_mesh,
            rim_mesh=rim_mesh,
            rubber=rubber,
            chrome=chrome,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")
    trunk_lid = object_model.get_part("trunk_lid")

    for side, door in (("left", left_door), ("right", right_door)):
        ctx.allow_overlap(
            body,
            door,
            elem_a=f"{side}_door_pin",
            elem_b="hinge_barrel",
            reason="The tiny metal pin is intentionally captured inside the toy door hinge barrel.",
        )
        ctx.expect_within(
            body,
            door,
            axes="xy",
            inner_elem=f"{side}_door_pin",
            outer_elem="hinge_barrel",
            margin=0.00025,
            name=f"{side} door pin is centered in hinge barrel",
        )
        ctx.expect_overlap(
            body,
            door,
            axes="z",
            elem_a=f"{side}_door_pin",
            elem_b="hinge_barrel",
            min_overlap=0.012,
            name=f"{side} door hinge has vertical engagement",
        )

    ctx.allow_overlap(
        body,
        trunk_lid,
        elem_a="trunk_pin",
        elem_b="hinge_barrel",
        reason="The rear-deck hinge pin is intentionally captured inside the trunk-lid barrel.",
    )
    ctx.expect_within(
        body,
        trunk_lid,
        axes="xz",
        inner_elem="trunk_pin",
        outer_elem="hinge_barrel",
        margin=0.00025,
        name="trunk pin is centered in lid barrel",
    )
    ctx.expect_overlap(
        body,
        trunk_lid,
        axes="y",
        elem_a="trunk_pin",
        elem_b="hinge_barrel",
        min_overlap=0.018,
        name="trunk hinge has cross-car engagement",
    )
    ctx.expect_gap(
        trunk_lid,
        body,
        axis="z",
        positive_elem="lid_panel",
        negative_elem="rear_deck_gasket",
        min_gap=0.00025,
        max_gap=0.0015,
        name="trunk lid sits visibly separate above rear deck",
    )

    for wheel_name, axle_elem in (
        ("front_left_wheel", "front_axle"),
        ("front_right_wheel", "front_axle"),
        ("rear_left_wheel", "rear_axle"),
        ("rear_right_wheel", "rear_axle"),
    ):
        wheel = object_model.get_part(wheel_name)
        ctx.allow_overlap(
            body,
            wheel,
            elem_a=axle_elem,
            elem_b="bearing_sleeve",
            reason="The toy wheel's simple bearing sleeve is intentionally captured on the metal axle.",
        )
        ctx.expect_within(
            body,
            wheel,
            axes="xz",
            inner_elem=axle_elem,
            outer_elem="bearing_sleeve",
            margin=0.0002,
            name=f"{wheel_name} axle is centered in bearing sleeve",
        )
        ctx.expect_overlap(
            body,
            wheel,
            axes="y",
            elem_a=axle_elem,
            elem_b="bearing_sleeve",
            min_overlap=0.004,
            name=f"{wheel_name} remains retained on axle",
        )

    for joint_name in (
        "body_to_front_left_wheel",
        "body_to_front_right_wheel",
        "body_to_rear_left_wheel",
        "body_to_rear_right_wheel",
    ):
        joint = object_model.get_articulation(joint_name)
        ctx.check(
            f"{joint_name} is continuous",
            joint.articulation_type == ArticulationType.CONTINUOUS,
            details=f"{joint_name} type={joint.articulation_type}",
        )

    closed_left = ctx.part_world_aabb(left_door)
    closed_right = ctx.part_world_aabb(right_door)
    closed_trunk = ctx.part_world_aabb(trunk_lid)
    with ctx.pose({"body_to_left_door": 0.85, "body_to_right_door": 0.85, "body_to_trunk_lid": 0.80}):
        open_left = ctx.part_world_aabb(left_door)
        open_right = ctx.part_world_aabb(right_door)
        open_trunk = ctx.part_world_aabb(trunk_lid)

    ctx.check(
        "left door opens outward",
        closed_left is not None and open_left is not None and open_left[1][1] > closed_left[1][1] + 0.010,
        details=f"closed={closed_left}, open={open_left}",
    )
    ctx.check(
        "right door opens outward",
        closed_right is not None and open_right is not None and open_right[0][1] < closed_right[0][1] - 0.010,
        details=f"closed={closed_right}, open={open_right}",
    )
    ctx.check(
        "trunk lid rotates upward",
        closed_trunk is not None and open_trunk is not None and open_trunk[1][2] > closed_trunk[1][2] + 0.006,
        details=f"closed={closed_trunk}, open={open_trunk}",
    )

    return ctx.report()


object_model = build_object_model()
