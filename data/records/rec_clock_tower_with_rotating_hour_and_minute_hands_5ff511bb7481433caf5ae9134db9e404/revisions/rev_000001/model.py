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


CLOCK_Z = 10.9
SHAFT_HALF = 1.05
BEZEL_CENTER = 1.07
DIAL_CENTER = 1.105
AXLE_CAP_CENTER = 1.092
HOUR_JOINT_OFFSET = 1.128
MINUTE_JOINT_OFFSET = 1.143

FACE_SPECS = {
    "north": {
        "axis_name": "y",
        "sign": 1.0,
        "joint_axis": (0.0, 1.0, 0.0),
        "bezel_name": "north_bezel",
        "dial_name": "north_dial",
        "axle_name": "north_axle_cap",
    },
    "east": {
        "axis_name": "x",
        "sign": 1.0,
        "joint_axis": (1.0, 0.0, 0.0),
        "bezel_name": "east_bezel",
        "dial_name": "east_dial",
        "axle_name": "east_axle_cap",
    },
    "south": {
        "axis_name": "y",
        "sign": -1.0,
        "joint_axis": (0.0, -1.0, 0.0),
        "bezel_name": "south_bezel",
        "dial_name": "south_dial",
        "axle_name": "south_axle_cap",
    },
    "west": {
        "axis_name": "x",
        "sign": -1.0,
        "joint_axis": (-1.0, 0.0, 0.0),
        "bezel_name": "west_bezel",
        "dial_name": "west_dial",
        "axle_name": "west_axle_cap",
    },
}


def _face_center(axis_name: str, sign: float, offset: float) -> tuple[float, float, float]:
    if axis_name == "y":
        return (0.0, sign * offset, CLOCK_Z)
    return (sign * offset, 0.0, CLOCK_Z)


def _cylinder_rpy(axis_name: str) -> tuple[float, float, float]:
    if axis_name == "y":
        return (math.pi / 2.0, 0.0, 0.0)
    return (0.0, math.pi / 2.0, 0.0)


def _add_clock_face(
    tower,
    *,
    face_name: str,
    axis_name: str,
    sign: float,
    bezel_name: str,
    dial_name: str,
    axle_name: str,
    steel,
    dial_material,
    marker_material,
) -> None:
    face_center = _face_center(axis_name, sign, BEZEL_CENTER)
    dial_center = _face_center(axis_name, sign, DIAL_CENTER)
    axle_center = _face_center(axis_name, sign, AXLE_CAP_CENTER)
    face_rpy = _cylinder_rpy(axis_name)

    tower.visual(
        Cylinder(radius=0.90, length=0.10),
        origin=Origin(xyz=face_center, rpy=face_rpy),
        material=steel,
        name=bezel_name,
    )
    tower.visual(
        Cylinder(radius=0.79, length=0.018),
        origin=Origin(xyz=dial_center, rpy=face_rpy),
        material=dial_material,
        name=dial_name,
    )
    tower.visual(
        Cylinder(radius=0.10, length=0.030),
        origin=Origin(xyz=axle_center, rpy=face_rpy),
        material=steel,
        name=axle_name,
    )

    marker_depth = 0.020
    if axis_name == "y":
        tower.visual(
            Box((0.10, marker_depth, 0.24)),
            origin=Origin(xyz=(0.0, sign * DIAL_CENTER, CLOCK_Z + 0.56)),
            material=marker_material,
            name=f"{face_name}_marker_12",
        )
        tower.visual(
            Box((0.10, marker_depth, 0.24)),
            origin=Origin(xyz=(0.0, sign * DIAL_CENTER, CLOCK_Z - 0.56)),
            material=marker_material,
            name=f"{face_name}_marker_6",
        )
        tower.visual(
            Box((0.24, marker_depth, 0.10)),
            origin=Origin(xyz=(0.56, sign * DIAL_CENTER, CLOCK_Z)),
            material=marker_material,
            name=f"{face_name}_marker_3",
        )
        tower.visual(
            Box((0.24, marker_depth, 0.10)),
            origin=Origin(xyz=(-0.56, sign * DIAL_CENTER, CLOCK_Z)),
            material=marker_material,
            name=f"{face_name}_marker_9",
        )
        return

    tower.visual(
        Box((marker_depth, 0.10, 0.24)),
        origin=Origin(xyz=(sign * DIAL_CENTER, 0.0, CLOCK_Z + 0.56)),
        material=marker_material,
        name=f"{face_name}_marker_12",
    )
    tower.visual(
        Box((marker_depth, 0.10, 0.24)),
        origin=Origin(xyz=(sign * DIAL_CENTER, 0.0, CLOCK_Z - 0.56)),
        material=marker_material,
        name=f"{face_name}_marker_6",
    )
    tower.visual(
        Box((marker_depth, 0.24, 0.10)),
        origin=Origin(xyz=(sign * DIAL_CENTER, -0.56, CLOCK_Z)),
        material=marker_material,
        name=f"{face_name}_marker_3",
    )
    tower.visual(
        Box((marker_depth, 0.24, 0.10)),
        origin=Origin(xyz=(sign * DIAL_CENTER, 0.56, CLOCK_Z)),
        material=marker_material,
        name=f"{face_name}_marker_9",
    )


def _add_hand_geometry(
    part,
    *,
    axis_name: str,
    hand_length: float,
    hand_width: float,
    hand_depth: float,
    tail_length: float,
    tail_width: float,
    hub_radius: float,
    hub_length: float,
    material,
) -> None:
    pointer_z = hand_length * 0.5 - 0.04
    tail_z = -(tail_length * 0.5 + 0.05)
    hub_rpy = _cylinder_rpy(axis_name)

    if axis_name == "y":
        part.visual(
            Box((hand_width, hand_depth, hand_length)),
            origin=Origin(xyz=(0.0, 0.0, pointer_z)),
            material=material,
            name="pointer",
        )
        part.visual(
            Box((tail_width, hand_depth, tail_length)),
            origin=Origin(xyz=(0.0, 0.0, tail_z)),
            material=material,
            name="counterweight",
        )
    else:
        part.visual(
            Box((hand_depth, hand_width, hand_length)),
            origin=Origin(xyz=(0.0, 0.0, pointer_z)),
            material=material,
            name="pointer",
        )
        part.visual(
            Box((hand_depth, tail_width, tail_length)),
            origin=Origin(xyz=(0.0, 0.0, tail_z)),
            material=material,
            name="counterweight",
        )

    part.visual(
        Cylinder(radius=hub_radius, length=hub_length),
        origin=Origin(rpy=hub_rpy),
        material=material,
        name="hub",
    )
    part.inertial = Inertial.from_geometry(
        Box((0.20, 0.20, hand_length + tail_length + 0.08)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="modern_clock_tower")

    concrete = model.material("concrete", rgba=(0.63, 0.63, 0.62, 1.0))
    steel = model.material("steel", rgba=(0.34, 0.37, 0.40, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.16, 0.18, 0.20, 1.0))
    glass = model.material("glass", rgba=(0.58, 0.78, 0.88, 0.28))
    dial_white = model.material("dial_white", rgba=(0.96, 0.97, 0.98, 1.0))
    marker_black = model.material("marker_black", rgba=(0.11, 0.12, 0.13, 1.0))
    minute_steel = model.material("minute_steel", rgba=(0.78, 0.81, 0.83, 1.0))
    hour_graphite = model.material("hour_graphite", rgba=(0.09, 0.10, 0.11, 1.0))

    tower = model.part("tower")
    tower.visual(
        Box((4.6, 4.6, 0.85)),
        origin=Origin(xyz=(0.0, 0.0, 0.425)),
        material=concrete,
        name="base_plinth",
    )
    tower.visual(
        Box((3.4, 3.4, 0.45)),
        origin=Origin(xyz=(0.0, 0.0, 1.075)),
        material=dark_steel,
        name="upper_podium",
    )

    column_height = 12.4
    column_center_z = 1.30 + column_height * 0.5
    for sx in (-SHAFT_HALF, SHAFT_HALF):
        for sy in (-SHAFT_HALF, SHAFT_HALF):
            tower.visual(
                Box((0.20, 0.20, column_height)),
                origin=Origin(xyz=(sx, sy, column_center_z)),
                material=steel,
                name=f"column_{'p' if sx > 0 else 'n'}x_{'p' if sy > 0 else 'n'}y",
            )

    glass_height = 12.1
    glass_center_z = 1.30 + glass_height * 0.5
    tower.visual(
        Box((1.90, 0.04, glass_height)),
        origin=Origin(xyz=(0.0, SHAFT_HALF, glass_center_z)),
        material=glass,
        name="north_glass",
    )
    tower.visual(
        Box((1.90, 0.04, glass_height)),
        origin=Origin(xyz=(0.0, -SHAFT_HALF, glass_center_z)),
        material=glass,
        name="south_glass",
    )
    tower.visual(
        Box((0.04, 1.90, glass_height)),
        origin=Origin(xyz=(SHAFT_HALF, 0.0, glass_center_z)),
        material=glass,
        name="east_glass",
    )
    tower.visual(
        Box((0.04, 1.90, glass_height)),
        origin=Origin(xyz=(-SHAFT_HALF, 0.0, glass_center_z)),
        material=glass,
        name="west_glass",
    )

    tower.visual(
        Box((2.55, 2.55, 0.22)),
        origin=Origin(xyz=(0.0, 0.0, 13.81)),
        material=dark_steel,
        name="roof_cap",
    )
    tower.visual(
        Box((1.50, 1.50, 0.90)),
        origin=Origin(xyz=(0.0, 0.0, 14.37)),
        material=glass,
        name="lantern_glass",
    )
    for sx in (-0.69, 0.69):
        for sy in (-0.69, 0.69):
            tower.visual(
                Box((0.12, 0.12, 0.90)),
                origin=Origin(xyz=(sx, sy, 14.37)),
                material=steel,
                name=f"lantern_post_{'p' if sx > 0 else 'n'}x_{'p' if sy > 0 else 'n'}y",
            )
    tower.visual(
        Box((1.86, 1.86, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 14.88)),
        material=dark_steel,
        name="lantern_cap",
    )
    tower.visual(
        Cylinder(radius=0.06, length=1.20),
        origin=Origin(xyz=(0.0, 0.0, 15.54)),
        material=steel,
        name="spire",
    )

    for face_name, spec in FACE_SPECS.items():
        _add_clock_face(
            tower,
            face_name=face_name,
            axis_name=spec["axis_name"],
            sign=spec["sign"],
            bezel_name=spec["bezel_name"],
            dial_name=spec["dial_name"],
            axle_name=spec["axle_name"],
            steel=steel,
            dial_material=dial_white,
            marker_material=marker_black,
        )

    tower.inertial = Inertial.from_geometry(
        Box((4.6, 4.6, 16.2)),
        mass=6000.0,
        origin=Origin(xyz=(0.0, 0.0, 8.1)),
    )

    for face_name, spec in FACE_SPECS.items():
        hour_hand = model.part(f"{face_name}_hour_hand")
        _add_hand_geometry(
            hour_hand,
            axis_name=spec["axis_name"],
            hand_length=0.46,
            hand_width=0.09,
            hand_depth=0.014,
            tail_length=0.15,
            tail_width=0.05,
            hub_radius=0.08,
            hub_length=0.016,
            material=hour_graphite,
        )

        minute_hand = model.part(f"{face_name}_minute_hand")
        _add_hand_geometry(
            minute_hand,
            axis_name=spec["axis_name"],
            hand_length=0.68,
            hand_width=0.06,
            hand_depth=0.012,
            tail_length=0.18,
            tail_width=0.03,
            hub_radius=0.065,
            hub_length=0.014,
            material=minute_steel,
        )

        model.articulation(
            f"{face_name}_hour_rotation",
            ArticulationType.CONTINUOUS,
            parent=tower,
            child=hour_hand,
            origin=Origin(xyz=_face_center(spec["axis_name"], spec["sign"], HOUR_JOINT_OFFSET)),
            axis=spec["joint_axis"],
            motion_limits=MotionLimits(effort=12.0, velocity=2.5),
        )
        model.articulation(
            f"{face_name}_minute_rotation",
            ArticulationType.CONTINUOUS,
            parent=tower,
            child=minute_hand,
            origin=Origin(xyz=_face_center(spec["axis_name"], spec["sign"], MINUTE_JOINT_OFFSET)),
            axis=spec["joint_axis"],
            motion_limits=MotionLimits(effort=12.0, velocity=6.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")

    for face_name, spec in FACE_SPECS.items():
        hour_hand = object_model.get_part(f"{face_name}_hour_hand")
        minute_hand = object_model.get_part(f"{face_name}_minute_hand")
        hour_joint = object_model.get_articulation(f"{face_name}_hour_rotation")
        minute_joint = object_model.get_articulation(f"{face_name}_minute_rotation")

        ctx.check(
            f"{face_name} hour joint is continuous on the face normal",
            hour_joint.articulation_type == ArticulationType.CONTINUOUS
            and tuple(hour_joint.axis) == spec["joint_axis"]
            and hour_joint.motion_limits is not None
            and hour_joint.motion_limits.lower is None
            and hour_joint.motion_limits.upper is None,
            details=f"joint={hour_joint.name}, axis={hour_joint.axis}, limits={hour_joint.motion_limits}",
        )
        ctx.check(
            f"{face_name} minute joint is continuous on the face normal",
            minute_joint.articulation_type == ArticulationType.CONTINUOUS
            and tuple(minute_joint.axis) == spec["joint_axis"]
            and minute_joint.motion_limits is not None
            and minute_joint.motion_limits.lower is None
            and minute_joint.motion_limits.upper is None,
            details=f"joint={minute_joint.name}, axis={minute_joint.axis}, limits={minute_joint.motion_limits}",
        )

        shared_axes = "xz" if spec["axis_name"] == "y" else "yz"
        gap_axis = "y" if spec["axis_name"] == "y" else "x"
        positive = minute_hand if spec["sign"] > 0.0 else hour_hand
        negative = hour_hand if spec["sign"] > 0.0 else minute_hand

        ctx.expect_origin_distance(
            minute_hand,
            hour_hand,
            axes=shared_axes,
            max_dist=1e-6,
            name=f"{face_name} hands share one axle center",
        )
        ctx.expect_origin_gap(
            positive,
            negative,
            axis=gap_axis,
            min_gap=0.015,
            max_gap=0.025,
            name=f"{face_name} hour and minute hands are stacked on the same axis",
        )

    north_minute = object_model.get_part("north_minute_hand")
    north_joint = object_model.get_articulation("north_minute_rotation")
    ctx.expect_overlap(
        north_minute,
        tower,
        axes="xz",
        elem_a="pointer",
        elem_b="north_dial",
        min_overlap=0.05,
        name="north minute hand sits over the north dial",
    )
    north_rest = ctx.part_element_world_aabb(north_minute, elem="pointer")
    with ctx.pose({north_joint: math.pi / 2.0}):
        north_turned = ctx.part_element_world_aabb(north_minute, elem="pointer")
    ctx.check(
        "north minute hand rotates across the north clock face",
        north_rest is not None
        and north_turned is not None
        and north_rest[1][2] > CLOCK_Z + 0.55
        and north_turned[1][0] > 0.45
        and north_turned[1][2] < CLOCK_Z + 0.10,
        details=f"rest={north_rest}, turned={north_turned}",
    )

    east_minute = object_model.get_part("east_minute_hand")
    east_joint = object_model.get_articulation("east_minute_rotation")
    ctx.expect_overlap(
        east_minute,
        tower,
        axes="yz",
        elem_a="pointer",
        elem_b="east_dial",
        min_overlap=0.05,
        name="east minute hand sits over the east dial",
    )
    east_rest = ctx.part_element_world_aabb(east_minute, elem="pointer")
    with ctx.pose({east_joint: math.pi / 2.0}):
        east_turned = ctx.part_element_world_aabb(east_minute, elem="pointer")
    ctx.check(
        "east minute hand rotates across the east clock face",
        east_rest is not None
        and east_turned is not None
        and east_rest[1][2] > CLOCK_Z + 0.55
        and east_turned[0][1] < -0.45
        and east_turned[1][2] < CLOCK_Z + 0.10,
        details=f"rest={east_rest}, turned={east_turned}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
