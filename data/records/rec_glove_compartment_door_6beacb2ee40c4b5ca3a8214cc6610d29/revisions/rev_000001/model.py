from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, cos, sin

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


OUTER_W = 0.38
OUTER_D = 0.12
OUTER_H = 0.20
WALL = 0.008

DOOR_W = 0.356
DOOR_H = 0.172
DOOR_THICK = 0.004

HINGE_Y = 0.020
HINGE_Z = 0.078
DOOR_OPEN_ANGLE = 0.55

SIDE_PIVOT_X = OUTER_W * 0.5 - WALL - 0.018
PIVOT_RADIUS = 0.009
LINK_THICK = 0.004
WASHER_THICK = 0.002
WASHER_OFFSET_X = LINK_THICK * 0.5 + WASHER_THICK * 0.5

DOOR_PIVOT_LOCAL_Y = -0.005
DOOR_PIVOT_LOCAL_Z = -0.120
LINK_BISECTOR_T = -0.75


def _rotate_yz(y: float, z: float, angle: float) -> tuple[float, float]:
    c = cos(angle)
    s = sin(angle)
    return (y * c - z * s, y * s + z * c)


_door_pivot_open_y, _door_pivot_open_z = _rotate_yz(
    DOOR_PIVOT_LOCAL_Y,
    DOOR_PIVOT_LOCAL_Z,
    DOOR_OPEN_ANGLE,
)
_mid_y = 0.5 * (DOOR_PIVOT_LOCAL_Y + _door_pivot_open_y)
_mid_z = 0.5 * (DOOR_PIVOT_LOCAL_Z + _door_pivot_open_z)
_chord_y = _door_pivot_open_y - DOOR_PIVOT_LOCAL_Y
_chord_z = _door_pivot_open_z - DOOR_PIVOT_LOCAL_Z
LINK_WALL_REL_Y = _mid_y + LINK_BISECTOR_T * (-_chord_z)
LINK_WALL_REL_Z = _mid_z + LINK_BISECTOR_T * _chord_y
LINK_WALL_Y = HINGE_Y + LINK_WALL_REL_Y
LINK_WALL_Z = HINGE_Z + LINK_WALL_REL_Z

LINK_CLOSED_VEC_Y = DOOR_PIVOT_LOCAL_Y - LINK_WALL_REL_Y
LINK_CLOSED_VEC_Z = DOOR_PIVOT_LOCAL_Z - LINK_WALL_REL_Z
LINK_OPEN_VEC_Y = _door_pivot_open_y - LINK_WALL_REL_Y
LINK_OPEN_VEC_Z = _door_pivot_open_z - LINK_WALL_REL_Z
LINK_LENGTH = (LINK_CLOSED_VEC_Y**2 + LINK_CLOSED_VEC_Z**2) ** 0.5
LINK_BAR_LENGTH = max(LINK_LENGTH - 2.0 * PIVOT_RADIUS, 0.012)
LINK_BAR_ROLL = atan2(-LINK_CLOSED_VEC_Y, LINK_CLOSED_VEC_Z)
LINK_OPEN_ANGLE = atan2(LINK_OPEN_VEC_Z, LINK_OPEN_VEC_Y) - atan2(
    LINK_CLOSED_VEC_Z,
    LINK_CLOSED_VEC_Y,
)


def _add_captured_pivot_pair(
    part,
    *,
    inner_name: str,
    outer_name: str,
    x: float,
    y: float,
    z: float,
    outward_sign: float,
    material,
) -> None:
    part.visual(
        Cylinder(radius=PIVOT_RADIUS, length=WASHER_THICK),
        origin=Origin(
            xyz=(x - outward_sign * WASHER_OFFSET_X, y, z),
            rpy=(0.0, 1.5707963267948966, 0.0),
        ),
        material=material,
        name=inner_name,
    )
    part.visual(
        Cylinder(radius=PIVOT_RADIUS, length=WASHER_THICK),
        origin=Origin(
            xyz=(x + outward_sign * WASHER_OFFSET_X, y, z),
            rpy=(0.0, 1.5707963267948966, 0.0),
        ),
        material=material,
        name=outer_name,
    )


def _add_link_geometry(part, material) -> None:
    part.visual(
        Cylinder(radius=PIVOT_RADIUS, length=LINK_THICK),
        origin=Origin(rpy=(0.0, 1.5707963267948966, 0.0)),
        material=material,
        name="lower_pad",
    )
    part.visual(
        Cylinder(radius=PIVOT_RADIUS, length=LINK_THICK),
        origin=Origin(
            xyz=(0.0, LINK_CLOSED_VEC_Y, LINK_CLOSED_VEC_Z),
            rpy=(0.0, 1.5707963267948966, 0.0),
        ),
        material=material,
        name="upper_pad",
    )
    part.visual(
        Box((LINK_THICK, 0.010, LINK_BAR_LENGTH)),
        origin=Origin(
            xyz=(0.0, LINK_CLOSED_VEC_Y * 0.5, LINK_CLOSED_VEC_Z * 0.5),
            rpy=(LINK_BAR_ROLL, 0.0, 0.0),
        ),
        material=material,
        name="link_bar",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="utility_truck_glove_compartment")

    dark_plastic = model.material("dark_plastic", rgba=(0.14, 0.15, 0.16, 1.0))
    panel_plastic = model.material("panel_plastic", rgba=(0.18, 0.19, 0.20, 1.0))
    textured_black = model.material("textured_black", rgba=(0.10, 0.10, 0.11, 1.0))
    zinc_steel = model.material("zinc_steel", rgba=(0.66, 0.68, 0.70, 1.0))

    bin_part = model.part("storage_bin")
    bin_part.inertial = Inertial.from_geometry(
        Box((OUTER_W, OUTER_D, OUTER_H)),
        mass=2.8,
    )
    bin_part.visual(
        Box((OUTER_W, WALL, OUTER_H)),
        origin=Origin(xyz=(0.0, -OUTER_D * 0.5 + WALL * 0.5, 0.0)),
        material=dark_plastic,
        name="bin_shell_back",
    )
    bin_part.visual(
        Box((OUTER_W, OUTER_D, WALL)),
        origin=Origin(xyz=(0.0, 0.0, OUTER_H * 0.5 - WALL * 0.5)),
        material=dark_plastic,
        name="bin_shell_top",
    )
    bin_part.visual(
        Box((OUTER_W, OUTER_D, WALL)),
        origin=Origin(xyz=(0.0, 0.0, -OUTER_H * 0.5 + WALL * 0.5)),
        material=dark_plastic,
        name="bin_shell_bottom",
    )
    bin_part.visual(
        Box((WALL, OUTER_D, OUTER_H - 2.0 * WALL)),
        origin=Origin(xyz=(OUTER_W * 0.5 - WALL * 0.5, 0.0, 0.0)),
        material=dark_plastic,
        name="bin_shell_right",
    )
    bin_part.visual(
        Box((WALL, OUTER_D, OUTER_H - 2.0 * WALL)),
        origin=Origin(xyz=(-OUTER_W * 0.5 + WALL * 0.5, 0.0, 0.0)),
        material=dark_plastic,
        name="bin_shell_left",
    )
    bin_part.visual(
        Box((OUTER_W - 0.024, 0.040, 0.018)),
        origin=Origin(xyz=(0.0, 0.032, HINGE_Z + 0.021)),
        material=panel_plastic,
        name="header_housing",
    )
    bin_part.visual(
        Box((OUTER_W - 0.030, 0.014, 0.016)),
        origin=Origin(xyz=(0.0, 0.050, -OUTER_H * 0.5 + 0.016)),
        material=panel_plastic,
        name="opening_sill",
    )
    bin_part.visual(
        Box((WALL, 0.014, OUTER_H - 0.040)),
        origin=Origin(xyz=(OUTER_W * 0.5 - WALL * 0.5, 0.050, -0.004)),
        material=panel_plastic,
        name="opening_jamb_right",
    )
    bin_part.visual(
        Box((WALL, 0.014, OUTER_H - 0.040)),
        origin=Origin(xyz=(-OUTER_W * 0.5 + WALL * 0.5, 0.050, -0.004)),
        material=panel_plastic,
        name="opening_jamb_left",
    )
    bin_part.visual(
        Box((OUTER_W - 0.030, 0.014, 0.016)),
        origin=Origin(xyz=(0.0, 0.050, HINGE_Z + 0.012)),
        material=panel_plastic,
        name="opening_header",
    )
    bin_part.visual(
        Box((0.124, 0.012, 0.020)),
        origin=Origin(xyz=(0.0, 0.022, 0.088)),
        material=panel_plastic,
        name="hinge_mount_spine",
    )
    bin_part.visual(
        Cylinder(radius=0.006, length=0.090),
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z), rpy=(0.0, 1.5707963267948966, 0.0)),
        material=zinc_steel,
        name="hinge_barrel_center",
    )
    _add_captured_pivot_pair(
        bin_part,
        inner_name="left_pivot_inner",
        outer_name="left_pivot_outer",
        x=-SIDE_PIVOT_X,
        y=LINK_WALL_Y,
        z=LINK_WALL_Z,
        outward_sign=-1.0,
        material=zinc_steel,
    )
    _add_captured_pivot_pair(
        bin_part,
        inner_name="right_pivot_inner",
        outer_name="right_pivot_outer",
        x=SIDE_PIVOT_X,
        y=LINK_WALL_Y,
        z=LINK_WALL_Z,
        outward_sign=1.0,
        material=zinc_steel,
    )
    bin_part.visual(
        Box((0.032, 0.018, 0.024)),
        origin=Origin(xyz=(-SIDE_PIVOT_X - 0.005, LINK_WALL_Y, LINK_WALL_Z)),
        material=panel_plastic,
        name="left_pivot_support",
    )
    bin_part.visual(
        Box((0.032, 0.018, 0.024)),
        origin=Origin(xyz=(SIDE_PIVOT_X + 0.005, LINK_WALL_Y, LINK_WALL_Z)),
        material=panel_plastic,
        name="right_pivot_support",
    )

    door = model.part("door")
    door.inertial = Inertial.from_geometry(
        Box((DOOR_W, 0.042, DOOR_H)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.028, -DOOR_H * 0.5)),
    )
    door.visual(
        Box((DOOR_W, DOOR_THICK, DOOR_H)),
        origin=Origin(xyz=(0.0, 0.041, -DOOR_H * 0.5)),
        material=panel_plastic,
        name="door_panel",
    )
    door.visual(
        Box((DOOR_W, 0.036, 0.016)),
        origin=Origin(xyz=(0.0, 0.023, -0.008)),
        material=panel_plastic,
        name="door_top_flange",
    )
    door.visual(
        Box((0.220, 0.010, 0.014)),
        origin=Origin(xyz=(0.0, 0.046, -DOOR_H + 0.018)),
        material=textured_black,
        name="door_pull_lip",
    )
    door.visual(
        Box((0.270, 0.014, 0.024)),
        origin=Origin(xyz=(0.0, 0.033, -DOOR_H + 0.040)),
        material=textured_black,
        name="door_inner_stiffener",
    )
    door.visual(
        Cylinder(radius=0.006, length=0.060),
        origin=Origin(xyz=(-0.105, 0.006, 0.0), rpy=(0.0, 1.5707963267948966, 0.0)),
        material=zinc_steel,
        name="door_hinge_barrel_left",
    )
    door.visual(
        Cylinder(radius=0.006, length=0.060),
        origin=Origin(xyz=(0.105, 0.006, 0.0), rpy=(0.0, 1.5707963267948966, 0.0)),
        material=zinc_steel,
        name="door_hinge_barrel_right",
    )
    _add_captured_pivot_pair(
        door,
        inner_name="left_door_ear_inner",
        outer_name="left_door_ear_outer",
        x=-SIDE_PIVOT_X,
        y=DOOR_PIVOT_LOCAL_Y,
        z=DOOR_PIVOT_LOCAL_Z,
        outward_sign=-1.0,
        material=zinc_steel,
    )
    _add_captured_pivot_pair(
        door,
        inner_name="right_door_ear_inner",
        outer_name="right_door_ear_outer",
        x=SIDE_PIVOT_X,
        y=DOOR_PIVOT_LOCAL_Y,
        z=DOOR_PIVOT_LOCAL_Z,
        outward_sign=1.0,
        material=zinc_steel,
    )
    door.visual(
        Box((0.022, 0.048, 0.032)),
        origin=Origin(
            xyz=(-SIDE_PIVOT_X, 0.019, DOOR_PIVOT_LOCAL_Z + 0.010),
        ),
        material=panel_plastic,
        name="left_link_bracket",
    )
    door.visual(
        Box((0.022, 0.048, 0.032)),
        origin=Origin(
            xyz=(SIDE_PIVOT_X, 0.019, DOOR_PIVOT_LOCAL_Z + 0.010),
        ),
        material=panel_plastic,
        name="right_link_bracket",
    )

    left_link = model.part("left_limiter_link")
    left_link.inertial = Inertial.from_geometry(
        Box((LINK_THICK, 0.018, LINK_LENGTH)),
        mass=0.10,
        origin=Origin(xyz=(0.0, LINK_CLOSED_VEC_Y * 0.5, LINK_CLOSED_VEC_Z * 0.5)),
    )
    _add_link_geometry(left_link, zinc_steel)

    right_link = model.part("right_limiter_link")
    right_link.inertial = Inertial.from_geometry(
        Box((LINK_THICK, 0.018, LINK_LENGTH)),
        mass=0.10,
        origin=Origin(xyz=(0.0, LINK_CLOSED_VEC_Y * 0.5, LINK_CLOSED_VEC_Z * 0.5)),
    )
    _add_link_geometry(right_link, zinc_steel)

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=bin_part,
        child=door,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=0.0,
            upper=0.70,
        ),
    )
    model.articulation(
        "left_link_pivot",
        ArticulationType.REVOLUTE,
        parent=bin_part,
        child=left_link,
        origin=Origin(xyz=(-SIDE_PIVOT_X, LINK_WALL_Y, LINK_WALL_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=4.0,
            lower=-1.30,
            upper=0.15,
        ),
    )
    model.articulation(
        "right_link_pivot",
        ArticulationType.REVOLUTE,
        parent=bin_part,
        child=right_link,
        origin=Origin(xyz=(SIDE_PIVOT_X, LINK_WALL_Y, LINK_WALL_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=4.0,
            lower=-1.30,
            upper=0.15,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bin_part = object_model.get_part("storage_bin")
    door = object_model.get_part("door")
    left_link = object_model.get_part("left_limiter_link")
    right_link = object_model.get_part("right_limiter_link")
    door_hinge = object_model.get_articulation("door_hinge")
    left_link_pivot = object_model.get_articulation("left_link_pivot")
    right_link_pivot = object_model.get_articulation("right_link_pivot")

    ctx.expect_gap(
        door,
        bin_part,
        axis="y",
        min_gap=0.001,
        max_gap=0.008,
        positive_elem="door_panel",
        negative_elem="opening_header",
        name="door sits just ahead of the glove box frame",
    )
    ctx.expect_overlap(
        door,
        bin_part,
        axes="xz",
        elem_a="door_panel",
        min_overlap=0.15,
        name="door covers the front opening footprint",
    )
    ctx.expect_contact(
        left_link,
        bin_part,
        elem_a="lower_pad",
        elem_b="left_pivot_outer",
        name="left limiter link is captured at the box wall pivot",
    )
    ctx.expect_contact(
        right_link,
        bin_part,
        elem_a="lower_pad",
        elem_b="right_pivot_outer",
        name="right limiter link is captured at the box wall pivot",
    )
    ctx.expect_contact(
        left_link,
        door,
        elem_a="upper_pad",
        elem_b="left_door_ear_outer",
        name="left limiter link is captured at the door pivot",
    )
    ctx.expect_contact(
        right_link,
        door,
        elem_a="upper_pad",
        elem_b="right_door_ear_outer",
        name="right limiter link is captured at the door pivot",
    )

    closed_panel = ctx.part_element_world_aabb(door, elem="door_panel")
    with ctx.pose(
        {
            door_hinge: DOOR_OPEN_ANGLE,
            left_link_pivot: LINK_OPEN_ANGLE,
            right_link_pivot: LINK_OPEN_ANGLE,
        }
    ):
        open_panel = ctx.part_element_world_aabb(door, elem="door_panel")
        ctx.expect_contact(
            left_link,
            door,
            elem_a="upper_pad",
            elem_b="left_door_ear_outer",
            name="left limiter link stays engaged when the door is opened",
        )
        ctx.expect_contact(
            right_link,
            door,
            elem_a="upper_pad",
            elem_b="right_door_ear_outer",
            name="right limiter link stays engaged when the door is opened",
        )

    door_swings_up = (
        closed_panel is not None
        and open_panel is not None
        and open_panel[0][2] > closed_panel[0][2] + 0.035
        and open_panel[1][1] > closed_panel[1][1] + 0.015
    )
    ctx.check(
        "door opens upward around the top hinge",
        door_swings_up,
        details=f"closed_panel={closed_panel}, open_panel={open_panel}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
