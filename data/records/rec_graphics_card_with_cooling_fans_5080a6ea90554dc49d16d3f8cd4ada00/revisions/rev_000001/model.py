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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


CARD_LENGTH = 0.338
CARD_HEIGHT = 0.140
CARD_THICKNESS = 0.062

SHROUD_LENGTH = 0.314
SHROUD_HEIGHT = 0.128
SHROUD_DEPTH = 0.028
FAN_CENTER_Y = 0.0115
FAN_RADIUS = 0.0405
FAN_HOLE_RADIUS = 0.044
FAN_DEPTH = 0.010
FAN_CENTERS_X = (-0.102, 0.0, 0.102)
LEG_HINGE_ORIGIN = (0.126, 0.022, -0.071)


def _circle_profile(radius: float, segments: int = 40) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * i) / segments),
            radius * math.sin((2.0 * math.pi * i) / segments),
        )
        for i in range(segments)
    ]


def _shroud_mesh():
    shell = ExtrudeWithHolesGeometry(
        rounded_rect_profile(SHROUD_LENGTH, SHROUD_HEIGHT, radius=0.017, corner_segments=10),
        [
            [(x + cx, z) for x, z in _circle_profile(FAN_HOLE_RADIUS, segments=48)]
            for cx in FAN_CENTERS_X
        ],
        SHROUD_DEPTH,
        center=True,
    )
    shell.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(shell, "gpu_outer_shroud")


def _fan_frame_mesh():
    frame = ExtrudeWithHolesGeometry(
        _circle_profile(0.045, segments=48),
        [_circle_profile(0.037, segments=48)],
        0.008,
        center=True,
    )
    frame.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(frame, "gpu_fan_frame")


def _add_fan_geometry(fan_part, dark_plastic) -> None:
    fan_part.visual(
        Cylinder(radius=0.011, length=0.013),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_plastic,
        name="fan_hub",
    )
    blade_count = 9
    blade_radius = 0.022
    for index in range(blade_count):
        angle = (2.0 * math.pi * index) / blade_count
        fan_part.visual(
            Box((0.038, 0.006, 0.010)),
            origin=Origin(
                xyz=(blade_radius * math.cos(angle), 0.0, blade_radius * math.sin(angle)),
                rpy=(0.0, angle + 0.28, 0.0),
            ),
            material=dark_plastic,
            name=f"blade_{index:02d}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="triple_fan_graphics_card")

    shroud_black = model.material("shroud_black", rgba=(0.13, 0.14, 0.15, 1.0))
    matte_black = model.material("matte_black", rgba=(0.08, 0.09, 0.10, 1.0))
    heatsink_gray = model.material("heatsink_gray", rgba=(0.57, 0.60, 0.63, 1.0))
    pcb_green = model.material("pcb_green", rgba=(0.12, 0.24, 0.15, 1.0))
    steel = model.material("steel", rgba=(0.72, 0.74, 0.76, 1.0))
    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.80, 0.81, 0.83, 1.0))

    shroud_mesh = _shroud_mesh()
    fan_frame_mesh = _fan_frame_mesh()

    card_body = model.part("card_body")
    card_body.visual(
        shroud_mesh,
        origin=Origin(xyz=(0.0, 0.017, 0.0)),
        material=shroud_black,
        name="outer_shroud",
    )
    card_body.visual(
        Box((0.304, 0.028, 0.112)),
        origin=Origin(xyz=(-0.002, -0.011, 0.0)),
        material=heatsink_gray,
        name="heatsink_core",
    )
    card_body.visual(
        Box((0.316, 0.005, 0.126)),
        origin=Origin(xyz=(0.0, -0.0285, 0.0)),
        material=brushed_aluminum,
        name="backplate_panel",
    )
    card_body.visual(
        Box((0.306, 0.0024, 0.118)),
        origin=Origin(xyz=(-0.002, -0.0252, 0.0)),
        material=pcb_green,
        name="pcb_board",
    )
    card_body.visual(
        Box((0.017, 0.030, 0.122)),
        origin=Origin(xyz=(-0.1635, -0.010, 0.0)),
        material=steel,
        name="io_bracket",
    )
    card_body.visual(
        Box((0.032, 0.018, 0.014)),
        origin=Origin(xyz=(0.074, -0.002, 0.067)),
        material=matte_black,
        name="power_connector",
    )
    card_body.visual(
        Box((0.018, 0.020, 0.006)),
        origin=Origin(xyz=(0.126, 0.022, -0.063)),
        material=steel,
        name="support_hinge_pad",
    )
    for slot_name, x_pos in zip(("left", "center", "right"), FAN_CENTERS_X):
        card_body.visual(
            fan_frame_mesh,
            origin=Origin(xyz=(x_pos, 0.021, 0.0)),
            material=matte_black,
            name=f"fan_frame_{slot_name}",
        )
        card_body.visual(
            Cylinder(radius=0.0055, length=0.006),
            origin=Origin(xyz=(x_pos, 0.002, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"fan_bearing_{slot_name}",
        )
        for strut_index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
            card_body.visual(
                Box((0.046, 0.004, 0.006)),
                origin=Origin(
                    xyz=(x_pos + 0.023 * math.cos(angle), 0.004, 0.023 * math.sin(angle)),
                    rpy=(0.0, angle, 0.0),
                ),
                material=shroud_black,
                name=f"fan_stator_{slot_name}_{strut_index}",
            )
    card_body.inertial = Inertial.from_geometry(
        Box((CARD_LENGTH, CARD_THICKNESS, CARD_HEIGHT)),
        mass=1.85,
    )

    for slot_name in ("fan_left", "fan_center", "fan_right"):
        fan = model.part(slot_name)
        _add_fan_geometry(fan, matte_black)
        fan.inertial = Inertial.from_geometry(
            Cylinder(radius=FAN_RADIUS, length=FAN_DEPTH),
            mass=0.10,
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        )

    support_leg = model.part("support_leg")
    support_leg.visual(
        Cylinder(radius=0.005, length=0.022),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="leg_hinge_barrel",
    )
    support_leg.visual(
        Box((0.086, 0.010, 0.010)),
        origin=Origin(xyz=(-0.043, 0.0, -0.008)),
        material=matte_black,
        name="leg_arm",
    )
    support_leg.visual(
        Box((0.020, 0.016, 0.014)),
        origin=Origin(xyz=(-0.082, 0.0, -0.012)),
        material=steel,
        name="leg_foot_pad",
    )
    support_leg.visual(
        Box((0.024, 0.010, 0.010)),
        origin=Origin(xyz=(-0.068, 0.0, -0.016)),
        material=matte_black,
        name="leg_brace",
    )
    support_leg.inertial = Inertial.from_geometry(
        Box((0.090, 0.018, 0.028)),
        mass=0.08,
        origin=Origin(xyz=(-0.040, 0.0, -0.010)),
    )

    for slot_name, x_pos in zip(("fan_left", "fan_center", "fan_right"), FAN_CENTERS_X):
        model.articulation(
            f"body_to_{slot_name}",
            ArticulationType.CONTINUOUS,
            parent=card_body,
            child=slot_name,
            origin=Origin(xyz=(x_pos, FAN_CENTER_Y, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=40.0),
        )

    model.articulation(
        "body_to_support_leg",
        ArticulationType.REVOLUTE,
        parent=card_body,
        child=support_leg,
        origin=Origin(xyz=LEG_HINGE_ORIGIN),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(98.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("card_body")
    leg = object_model.get_part("support_leg")
    leg_joint = object_model.get_articulation("body_to_support_leg")
    fan_parts = [
        object_model.get_part("fan_left"),
        object_model.get_part("fan_center"),
        object_model.get_part("fan_right"),
    ]
    fan_joints = [
        object_model.get_articulation("body_to_fan_left"),
        object_model.get_articulation("body_to_fan_center"),
        object_model.get_articulation("body_to_fan_right"),
    ]

    for fan, joint, bearing_name in zip(
        fan_parts,
        fan_joints,
        ("fan_bearing_left", "fan_bearing_center", "fan_bearing_right"),
    ):
        ctx.check(
            f"{joint.name} is a continuous fan joint",
            joint.articulation_type == ArticulationType.CONTINUOUS
            and joint.motion_limits is not None
            and joint.motion_limits.lower is None
            and joint.motion_limits.upper is None
            and abs(joint.axis[1]) > 0.99,
            details=f"type={joint.articulation_type}, axis={joint.axis}, limits={joint.motion_limits}",
        )
        ctx.expect_overlap(
            fan,
            body,
            axes="xz",
            min_overlap=0.060,
            name=f"{fan.name} stays within the card face",
        )
        ctx.expect_contact(
            fan,
            body,
            elem_a="fan_hub",
            elem_b=bearing_name,
            contact_tol=0.001,
            name=f"{fan.name} stays carried by its center bearing",
        )

    ctx.expect_contact(
        leg,
        body,
        elem_a="leg_hinge_barrel",
        elem_b="support_hinge_pad",
        contact_tol=0.0015,
        name="support leg stays seated on its hinge pad",
    )
    ctx.expect_origin_gap(
        leg,
        body,
        axis="x",
        min_gap=0.10,
        max_gap=0.15,
        name="support leg mounts near the far end",
    )

    closed_aabb = ctx.part_world_aabb(leg)
    upper = leg_joint.motion_limits.upper if leg_joint.motion_limits is not None else None
    with ctx.pose({leg_joint: upper}):
        open_aabb = ctx.part_world_aabb(leg)
        ctx.expect_contact(
            leg,
            body,
            elem_a="leg_hinge_barrel",
            elem_b="support_hinge_pad",
            contact_tol=0.0015,
            name="support leg keeps hinge contact when deployed",
        )

    ctx.check(
        "support leg folds below the card when deployed",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[0][2] < closed_aabb[0][2] - 0.045,
        details=f"closed_aabb={closed_aabb}, open_aabb={open_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
