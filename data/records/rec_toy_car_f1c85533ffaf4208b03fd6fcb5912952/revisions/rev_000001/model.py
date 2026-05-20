from __future__ import annotations

from math import pi

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

CAR_LENGTH = 0.148
BODY_WIDTH = 0.052
HALF_BODY_WIDTH = BODY_WIDTH * 0.5
WHEEL_RADIUS = 0.0125
WHEEL_WIDTH = 0.0086
AXLE_Z = WHEEL_RADIUS
FRONT_AXLE_X = -0.034
REAR_AXLE_X = 0.037
WHEEL_Y = 0.031
DOOR_HINGE_X = -0.014
DOOR_HINGE_Z = 0.022
DOOR_Y = 0.024
TRUNK_HINGE_X = 0.0305
TRUNK_HINGE_Z = 0.0302


def _add_wheel_visuals(part, mesh_prefix: str, wheel_finish, tire_finish) -> None:
    part.visual(
        mesh_from_geometry(
            WheelGeometry(
                0.0100,
                0.0076,
                rim=WheelRim(
                    inner_radius=0.0068,
                    flange_height=0.0010,
                    flange_thickness=0.0008,
                    bead_seat_depth=0.0006,
                ),
                hub=WheelHub(
                    radius=0.0028,
                    width=0.0058,
                    cap_style="domed",
                    bolt_pattern=BoltPattern(
                        count=5,
                        circle_diameter=0.0040,
                        hole_diameter=0.0006,
                    ),
                ),
                face=WheelFace(dish_depth=0.0008, front_inset=0.0004, rear_inset=0.0003),
                spokes=WheelSpokes(style="straight", count=5, thickness=0.0009, window_radius=0.0018),
                bore=WheelBore(style="round", diameter=0.0016),
            ),
            f"{mesh_prefix}_wheel",
        ),
        material=wheel_finish,
        name="wheel",
    )
    part.visual(
        mesh_from_geometry(
            TireGeometry(
                WHEEL_RADIUS,
                WHEEL_WIDTH,
                inner_radius=0.0101,
                carcass=TireCarcass(belt_width_ratio=0.72, sidewall_bulge=0.04),
                tread=TireTread(style="circumferential", depth=0.0009, count=3),
                grooves=(TireGroove(center_offset=0.0, width=0.0008, depth=0.0004),),
                sidewall=TireSidewall(style="rounded", bulge=0.04),
                shoulder=TireShoulder(width=0.0008, radius=0.0005),
            ),
            f"{mesh_prefix}_tire",
        ),
        material=tire_finish,
        name="tire",
    )
    part.visual(
        Cylinder(radius=0.0030, length=0.0086),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=wheel_finish,
        name="spindle",
    )


def _add_door_visuals(part, *, side_sign: float, body_finish, glass_finish, trim_finish) -> None:
    part.visual(
        Box((0.044, 0.0026, 0.0178)),
        origin=Origin(xyz=(0.0222, side_sign * 0.0013, -0.0005)),
        material=body_finish,
        name="panel",
    )
    part.visual(
        Box((0.029, 0.0012, 0.0084)),
        origin=Origin(xyz=(0.0240, 0.0, 0.0098)),
        material=glass_finish,
        name="glass",
    )
    part.visual(
        Box((0.0060, 0.0012, 0.0018)),
        origin=Origin(xyz=(0.0310, side_sign * 0.0019, 0.0010)),
        material=trim_finish,
        name="handle",
    )
    part.visual(
        Cylinder(radius=0.0010, length=0.0150),
        origin=Origin(xyz=(0.0010, 0.0, 0.0)),
        material=trim_finish,
        name="hinge_barrel",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="toy_coupe")

    body_finish = model.material("body_finish", rgba=(0.79, 0.11, 0.14, 1.0))
    base_finish = model.material("base_finish", rgba=(0.11, 0.11, 0.12, 1.0))
    trim_finish = model.material("trim_finish", rgba=(0.68, 0.69, 0.71, 1.0))
    glass_finish = model.material("glass_finish", rgba=(0.18, 0.22, 0.28, 0.88))
    tire_finish = model.material("tire_finish", rgba=(0.05, 0.05, 0.05, 1.0))
    wheel_finish = model.material("wheel_finish", rgba=(0.76, 0.78, 0.80, 1.0))
    lamp_finish = model.material("lamp_finish", rgba=(0.77, 0.12, 0.11, 1.0))
    interior_finish = model.material("interior_finish", rgba=(0.16, 0.16, 0.17, 1.0))

    body = model.part("body")
    body.visual(Box((0.136, 0.046, 0.004)), origin=Origin(xyz=(0.0, 0.0, 0.009)), material=base_finish, name="baseplate")
    body.visual(Box((0.018, 0.046, 0.008)), origin=Origin(xyz=(-0.064, 0.0, 0.015)), material=body_finish, name="nose")
    body.visual(Box((0.050, 0.048, 0.005)), origin=Origin(xyz=(-0.039, 0.0, 0.026)), material=body_finish, name="hood")
    body.visual(Box((0.008, 0.046, 0.005)), origin=Origin(xyz=(-0.011, 0.0, 0.0305)), material=body_finish, name="cowl")
    body.visual(Box((0.052, 0.048, 0.005)), origin=Origin(xyz=(0.010, 0.0, 0.039)), material=body_finish, name="roof")
    body.visual(Box((0.004, 0.042, 0.004)), origin=Origin(xyz=(0.0285, 0.0, 0.031)), material=body_finish, name="trunk_front_strip")
    body.visual(Box((0.031, 0.004, 0.004)), origin=Origin(xyz=(0.046, 0.0195, 0.029)), material=body_finish, name="left_deck_strip")
    body.visual(Box((0.031, 0.004, 0.004)), origin=Origin(xyz=(0.046, -0.0195, 0.029)), material=body_finish, name="right_deck_strip")
    body.visual(Box((0.014, 0.046, 0.010)), origin=Origin(xyz=(0.067, 0.0, 0.017)), material=body_finish, name="tail")

    for side_name, side_sign in (("left", 1.0), ("right", -1.0)):
        y_outer = side_sign * 0.0245
        y_pillar = side_sign * 0.0216
        y_shoulder = side_sign * 0.0225
        body.visual(
            Box((0.132, 0.003, 0.006)),
            origin=Origin(xyz=(0.0, y_outer, 0.013)),
            material=body_finish,
            name=f"{side_name}_rocker",
        )
        body.visual(
            Box((0.020, 0.003, 0.014)),
            origin=Origin(xyz=(-0.058, y_outer, 0.020)),
            material=body_finish,
            name=f"{side_name}_front_fender",
        )
        body.visual(
            Box((0.028, 0.003, 0.010)),
            origin=Origin(xyz=(-0.034, y_outer, 0.028)),
            material=body_finish,
            name=f"{side_name}_front_arch",
        )
        body.visual(
            Box((0.012, 0.0022, 0.018)),
            origin=Origin(xyz=(-0.014, y_pillar, 0.029)),
            material=body_finish,
            name=f"{side_name}_a_pillar",
        )
        body.visual(
            Box((0.012, 0.0022, 0.015)),
            origin=Origin(xyz=(0.038, y_pillar, 0.031)),
            material=body_finish,
            name=f"{side_name}_c_pillar",
        )
        body.visual(
            Box((0.030, 0.003, 0.011)),
            origin=Origin(xyz=(0.037, y_outer, 0.028)),
            material=body_finish,
            name=f"{side_name}_rear_arch",
        )
        body.visual(
            Box((0.020, 0.003, 0.014)),
            origin=Origin(xyz=(0.060, y_outer, 0.020)),
            material=body_finish,
            name=f"{side_name}_rear_quarter",
        )
        body.visual(
            Box((0.018, 0.006, 0.007)),
            origin=Origin(xyz=(0.044, y_shoulder, 0.032)),
            material=body_finish,
            name=f"{side_name}_shoulder",
        )

    body.visual(
        Box((0.021, 0.038, 0.0016)),
        origin=Origin(xyz=(-0.013, 0.0, 0.032), rpy=(0.0, -0.68, 0.0)),
        material=glass_finish,
        name="windshield",
    )
    body.visual(
        Box((0.021, 0.036, 0.0016)),
        origin=Origin(xyz=(0.039, 0.0, 0.033), rpy=(0.0, 0.72, 0.0)),
        material=glass_finish,
        name="rear_glass",
    )
    body.visual(Box((0.006, 0.044, 0.003)), origin=Origin(xyz=(-0.070, 0.0, 0.014)), material=trim_finish, name="front_bumper")
    body.visual(Box((0.004, 0.020, 0.003)), origin=Origin(xyz=(-0.069, 0.0, 0.017)), material=base_finish, name="grille")
    body.visual(Box((0.006, 0.044, 0.003)), origin=Origin(xyz=(0.070, 0.0, 0.014)), material=trim_finish, name="rear_bumper")
    body.visual(Box((0.004, 0.022, 0.002)), origin=Origin(xyz=(0.069, 0.0, 0.019)), material=lamp_finish, name="tail_lamp")

    body.visual(
        Cylinder(radius=0.0038, length=0.0015),
        origin=Origin(xyz=(FRONT_AXLE_X, WHEEL_Y - 0.0050, AXLE_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=trim_finish,
        name="front_left_axle",
    )
    body.visual(
        Cylinder(radius=0.0038, length=0.0015),
        origin=Origin(xyz=(FRONT_AXLE_X, -WHEEL_Y + 0.0050, AXLE_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=trim_finish,
        name="front_right_axle",
    )
    body.visual(
        Cylinder(radius=0.0038, length=0.0015),
        origin=Origin(xyz=(REAR_AXLE_X, WHEEL_Y - 0.0050, AXLE_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=trim_finish,
        name="rear_left_axle",
    )
    body.visual(
        Cylinder(radius=0.0038, length=0.0015),
        origin=Origin(xyz=(REAR_AXLE_X, -WHEEL_Y + 0.0050, AXLE_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=trim_finish,
        name="rear_right_axle",
    )

    interior = model.part("interior")
    interior.visual(Box((0.074, 0.034, 0.009)), origin=Origin(xyz=(0.008, 0.0, 0.0165)), material=interior_finish, name="floor")
    interior.visual(Box((0.020, 0.030, 0.006)), origin=Origin(xyz=(-0.008, 0.0, 0.022)), material=interior_finish, name="dashboard")
    interior.visual(Box((0.016, 0.010, 0.008)), origin=Origin(xyz=(0.006, 0.009, 0.020)), material=interior_finish, name="left_front_cushion")
    interior.visual(Box((0.016, 0.010, 0.008)), origin=Origin(xyz=(0.006, -0.009, 0.020)), material=interior_finish, name="right_front_cushion")
    interior.visual(Box((0.006, 0.010, 0.012)), origin=Origin(xyz=(0.011, 0.009, 0.026)), material=interior_finish, name="left_front_back")
    interior.visual(Box((0.006, 0.010, 0.012)), origin=Origin(xyz=(0.011, -0.009, 0.026)), material=interior_finish, name="right_front_back")
    interior.visual(Box((0.020, 0.028, 0.008)), origin=Origin(xyz=(0.027, 0.0, 0.020)), material=interior_finish, name="rear_bench")
    interior.visual(Box((0.018, 0.028, 0.004)), origin=Origin(xyz=(0.040, 0.0, 0.024)), material=interior_finish, name="rear_shelf")

    left_door = model.part("left_door")
    _add_door_visuals(left_door, side_sign=1.0, body_finish=body_finish, glass_finish=glass_finish, trim_finish=trim_finish)

    right_door = model.part("right_door")
    _add_door_visuals(right_door, side_sign=-1.0, body_finish=body_finish, glass_finish=glass_finish, trim_finish=trim_finish)

    trunk_lid = model.part("trunk_lid")
    trunk_lid.visual(
        Box((0.029, 0.033, 0.002)),
        origin=Origin(xyz=(0.0145, 0.0, 0.0005)),
        material=body_finish,
        name="panel",
    )
    trunk_lid.visual(
        Cylinder(radius=0.0010, length=0.0080),
        origin=Origin(xyz=(0.0015, 0.0105, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=trim_finish,
        name="left_hinge_barrel",
    )
    trunk_lid.visual(
        Cylinder(radius=0.0010, length=0.0080),
        origin=Origin(xyz=(0.0015, -0.0105, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=trim_finish,
        name="right_hinge_barrel",
    )
    trunk_lid.visual(
        Box((0.006, 0.002, 0.0012)),
        origin=Origin(xyz=(0.0245, 0.0, -0.0003)),
        material=trim_finish,
        name="pull",
    )

    for wheel_name in ("front_left_wheel", "front_right_wheel", "rear_left_wheel", "rear_right_wheel"):
        _add_wheel_visuals(model.part(wheel_name), wheel_name, wheel_finish, tire_finish)

    model.articulation("interior_mount", ArticulationType.FIXED, parent=body, child=interior, origin=Origin())
    model.articulation(
        "left_door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=left_door,
        origin=Origin(xyz=(DOOR_HINGE_X, DOOR_Y, DOOR_HINGE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=4.0, lower=0.0, upper=1.15),
    )
    model.articulation(
        "right_door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=right_door,
        origin=Origin(xyz=(DOOR_HINGE_X, -DOOR_Y, DOOR_HINGE_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=4.0, lower=0.0, upper=1.15),
    )
    model.articulation(
        "trunk_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=trunk_lid,
        origin=Origin(xyz=(TRUNK_HINGE_X, 0.0, TRUNK_HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.3, velocity=3.0, lower=0.0, upper=1.05),
    )

    for joint_name, wheel_name, axle_x, axle_y in (
        ("front_left_spin", "front_left_wheel", FRONT_AXLE_X, WHEEL_Y),
        ("front_right_spin", "front_right_wheel", FRONT_AXLE_X, -WHEEL_Y),
        ("rear_left_spin", "rear_left_wheel", REAR_AXLE_X, WHEEL_Y),
        ("rear_right_spin", "rear_right_wheel", REAR_AXLE_X, -WHEEL_Y),
    ):
        model.articulation(
            joint_name,
            ArticulationType.CONTINUOUS,
            parent=body,
            child=wheel_name,
            origin=Origin(xyz=(axle_x, axle_y, AXLE_Z), rpy=(0.0, 0.0, pi / 2.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=0.3, velocity=40.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")
    trunk_lid = object_model.get_part("trunk_lid")
    front_left_wheel = object_model.get_part("front_left_wheel")
    front_right_wheel = object_model.get_part("front_right_wheel")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    rear_right_wheel = object_model.get_part("rear_right_wheel")

    left_door_hinge = object_model.get_articulation("left_door_hinge")
    right_door_hinge = object_model.get_articulation("right_door_hinge")
    trunk_hinge = object_model.get_articulation("trunk_hinge")

    with ctx.pose({left_door_hinge: 0.0, right_door_hinge: 0.0, trunk_hinge: 0.0}):
        ctx.expect_overlap(left_door, body, axes="xz", min_overlap=0.012, name="left door closes into the body side opening")
        ctx.expect_overlap(right_door, body, axes="xz", min_overlap=0.012, name="right door closes into the body side opening")
        ctx.expect_overlap(trunk_lid, body, axes="xy", min_overlap=0.020, name="trunk lid covers the rear deck opening")
        ctx.expect_gap(
            front_left_wheel,
            body,
            axis="y",
            positive_elem="spindle",
            negative_elem="front_left_axle",
            max_gap=0.0012,
            max_penetration=0.0002,
            name="front left wheel reaches the body axle support",
        )
        ctx.expect_gap(
            rear_left_wheel,
            body,
            axis="y",
            positive_elem="spindle",
            negative_elem="rear_left_axle",
            max_gap=0.0012,
            max_penetration=0.0002,
            name="rear left wheel reaches the body axle support",
        )
        ctx.expect_gap(
            body,
            front_right_wheel,
            axis="y",
            positive_elem="front_right_axle",
            negative_elem="spindle",
            max_gap=0.0012,
            max_penetration=0.0002,
            name="front right wheel reaches the body axle support",
        )
        ctx.expect_gap(
            body,
            rear_right_wheel,
            axis="y",
            positive_elem="rear_right_axle",
            negative_elem="spindle",
            max_gap=0.0012,
            max_penetration=0.0002,
            name="rear right wheel reaches the body axle support",
        )

        closed_left_handle = ctx.part_element_world_aabb(left_door, elem="handle")
        closed_right_handle = ctx.part_element_world_aabb(right_door, elem="handle")
        closed_trunk_panel = ctx.part_element_world_aabb(trunk_lid, elem="panel")

    with ctx.pose({left_door_hinge: 1.15}):
        open_left_handle = ctx.part_element_world_aabb(left_door, elem="handle")
        ctx.check(
            "left door swings outward",
            open_left_handle is not None
            and closed_left_handle is not None
            and open_left_handle[1][1] > closed_left_handle[1][1] + 0.010,
            details=f"closed={closed_left_handle!r}, open={open_left_handle!r}",
        )

    with ctx.pose({right_door_hinge: 1.15}):
        open_right_handle = ctx.part_element_world_aabb(right_door, elem="handle")
        ctx.check(
            "right door swings outward",
            open_right_handle is not None
            and closed_right_handle is not None
            and open_right_handle[0][1] < closed_right_handle[0][1] - 0.010,
            details=f"closed={closed_right_handle!r}, open={open_right_handle!r}",
        )

    with ctx.pose({trunk_hinge: 1.05}):
        open_trunk_panel = ctx.part_element_world_aabb(trunk_lid, elem="panel")
        ctx.check(
            "trunk lid lifts upward",
            open_trunk_panel is not None
            and closed_trunk_panel is not None
            and open_trunk_panel[1][2] > closed_trunk_panel[1][2] + 0.010,
            details=f"closed={closed_trunk_panel!r}, open={open_trunk_panel!r}",
        )

    return ctx.report()


object_model = build_object_model()
