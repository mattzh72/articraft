from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

import cadquery as cq

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
    mesh_from_cadquery,
)


SLIDE_AXIS = (1.0, 0.0, 0.0)

OUTER_LENGTH = 0.62
OUTER_WIDTH = 0.13
OUTER_BASE_T = 0.018
OUTER_SIDE_H = 0.056
OUTER_WALL_T = 0.014
OUTER_FOOT_L = 0.095
OUTER_FOOT_W = 0.150
OUTER_FOOT_T = 0.008
OUTER_GUIDE_LENGTH = 0.54
OUTER_GUIDE_WIDTH = 0.016
OUTER_GUIDE_HEIGHT = 0.010
OUTER_GUIDE_Y = 0.036
OUTER_GUIDE_TOP_Z = OUTER_BASE_T + OUTER_GUIDE_HEIGHT
OUTER_STAGE_HOME_X = 0.08
OUTER_STAGE_TRAVEL = 0.18

MIDDLE_LENGTH = 0.42
MIDDLE_WIDTH = 0.090
MIDDLE_BODY_BASE_Z = 0.010
MIDDLE_BODY_H = 0.044
MIDDLE_SHOE_LENGTH = 0.28
MIDDLE_SHOE_WIDTH = 0.016
MIDDLE_SHOE_HEIGHT = 0.012
MIDDLE_SHOE_X = 0.21
MIDDLE_STAGE2_GUIDE_LENGTH = 0.34
MIDDLE_STAGE2_GUIDE_WIDTH = 0.012
MIDDLE_STAGE2_GUIDE_HEIGHT = 0.008
MIDDLE_STAGE2_GUIDE_X = 0.20
MIDDLE_STAGE2_GUIDE_Y = 0.027
MIDDLE_STAGE2_GUIDE_BASE_Z = 0.046
MIDDLE_STAGE2_GUIDE_TOP_Z = MIDDLE_STAGE2_GUIDE_BASE_Z + MIDDLE_STAGE2_GUIDE_HEIGHT
MIDDLE_TO_INNER_HOME_X = 0.09
MIDDLE_TO_INNER_TRAVEL = 0.12

INNER_LENGTH = 0.26
INNER_WIDTH = 0.068
INNER_BODY_BASE_Z = 0.010
INNER_BODY_H = 0.032
INNER_SHOE_LENGTH = 0.18
INNER_SHOE_WIDTH = 0.012
INNER_SHOE_HEIGHT = 0.010
INNER_SHOE_X = 0.13
INNER_SHOE_Y = 0.027
INNER_STAGE3_GUIDE_LENGTH = 0.18
INNER_STAGE3_GUIDE_WIDTH = 0.010
INNER_STAGE3_GUIDE_HEIGHT = 0.008
INNER_STAGE3_GUIDE_X = 0.13
INNER_STAGE3_GUIDE_Y = 0.018
INNER_STAGE3_GUIDE_BASE_Z = 0.038
INNER_STAGE3_GUIDE_TOP_Z = INNER_STAGE3_GUIDE_BASE_Z + INNER_STAGE3_GUIDE_HEIGHT
INNER_TO_NOSE_HOME_X = 0.10
INNER_TO_NOSE_TRAVEL = 0.06

NOSE_SHOE_LENGTH = 0.10
NOSE_SHOE_WIDTH = 0.010
NOSE_SHOE_HEIGHT = 0.010
NOSE_SHOE_Y = 0.018
NOSE_BLOCK_LENGTH = 0.10
NOSE_BLOCK_WIDTH = 0.050
NOSE_BLOCK_BASE_Z = 0.008
NOSE_BLOCK_H = 0.028
NOSE_PLATE_T = 0.018
NOSE_PLATE_WIDTH = 0.092
NOSE_PLATE_HEIGHT = 0.084


def _cq_box(length: float, width: float, height: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(length, width, height).translate(center)


def _make_outer_body_shape() -> cq.Workplane:
    rear_foot = _cq_box(
        OUTER_FOOT_L,
        OUTER_FOOT_W,
        OUTER_FOOT_T,
        (0.012 + OUTER_FOOT_L / 2.0, 0.0, OUTER_FOOT_T / 2.0),
    )
    front_foot = _cq_box(
        OUTER_FOOT_L,
        OUTER_FOOT_W,
        OUTER_FOOT_T,
        (OUTER_LENGTH - 0.012 - OUTER_FOOT_L / 2.0, 0.0, OUTER_FOOT_T / 2.0),
    )
    base = _cq_box(
        OUTER_LENGTH,
        OUTER_WIDTH,
        OUTER_BASE_T,
        (OUTER_LENGTH / 2.0, 0.0, OUTER_BASE_T / 2.0),
    )
    left_wall = _cq_box(
        OUTER_LENGTH,
        OUTER_WALL_T,
        OUTER_SIDE_H,
        (OUTER_LENGTH / 2.0, OUTER_WIDTH / 2.0 - OUTER_WALL_T / 2.0, OUTER_SIDE_H / 2.0),
    )
    right_wall = _cq_box(
        OUTER_LENGTH,
        OUTER_WALL_T,
        OUTER_SIDE_H,
        (OUTER_LENGTH / 2.0, -OUTER_WIDTH / 2.0 + OUTER_WALL_T / 2.0, OUTER_SIDE_H / 2.0),
    )
    rear_bridge = _cq_box(0.040, OUTER_WIDTH - 0.010, 0.016, (0.030, 0.0, OUTER_BASE_T + 0.008))
    front_bridge = _cq_box(
        0.030,
        OUTER_WIDTH - 0.010,
        0.014,
        (OUTER_LENGTH - 0.025, 0.0, OUTER_BASE_T + 0.007),
    )
    body = rear_foot.union(front_foot).union(base).union(left_wall).union(right_wall).union(rear_bridge).union(front_bridge)

    side_window = _cq_box(0.41, OUTER_WALL_T + 0.003, 0.022, (0.315, 0.0, 0.036))
    body = body.cut(side_window.translate((0.0, OUTER_WIDTH / 2.0 - OUTER_WALL_T / 2.0, 0.0)))
    body = body.cut(side_window.translate((0.0, -OUTER_WIDTH / 2.0 + OUTER_WALL_T / 2.0, 0.0)))
    body = body.edges("|Z").fillet(0.0025)
    return body


def _make_middle_carriage_shape() -> cq.Workplane:
    core = _cq_box(
        MIDDLE_LENGTH,
        MIDDLE_WIDTH,
        MIDDLE_BODY_H,
        (MIDDLE_LENGTH / 2.0, 0.0, MIDDLE_BODY_BASE_Z + MIDDLE_BODY_H / 2.0),
    )
    front_pad = _cq_box(0.060, MIDDLE_WIDTH, 0.014, (MIDDLE_LENGTH - 0.030, 0.0, 0.019))
    rear_pad = _cq_box(0.050, MIDDLE_WIDTH, 0.012, (0.030, 0.0, 0.018))
    body = core.union(front_pad).union(rear_pad)

    top_pocket = _cq_box(0.250, 0.044, 0.018, (0.220, 0.0, 0.047))
    side_relief = _cq_box(0.180, 0.012, 0.020, (0.220, 0.0, 0.032))
    body = body.cut(top_pocket)
    body = body.cut(side_relief.translate((0.0, 0.039, 0.0)))
    body = body.cut(side_relief.translate((0.0, -0.039, 0.0)))
    body = body.edges("|Z").fillet(0.002)
    return body


def _make_inner_carriage_shape() -> cq.Workplane:
    core = _cq_box(
        INNER_LENGTH,
        INNER_WIDTH,
        INNER_BODY_H,
        (INNER_LENGTH / 2.0, 0.0, INNER_BODY_BASE_Z + INNER_BODY_H / 2.0),
    )
    front_nose = _cq_box(0.038, INNER_WIDTH, 0.014, (INNER_LENGTH - 0.020, 0.0, 0.018))
    rear_pad = _cq_box(0.036, INNER_WIDTH, 0.012, (0.020, 0.0, 0.017))
    body = core.union(front_nose).union(rear_pad)

    top_pocket = _cq_box(0.120, 0.032, 0.014, (0.135, 0.0, 0.036))
    side_relief = _cq_box(0.110, 0.010, 0.016, (0.135, 0.0, 0.026))
    body = body.cut(top_pocket)
    body = body.cut(side_relief.translate((0.0, 0.029, 0.0)))
    body = body.cut(side_relief.translate((0.0, -0.029, 0.0)))
    body = body.edges("|Z").fillet(0.0018)
    return body


def _make_nose_plate_shape() -> cq.Workplane:
    slide_block = _cq_box(
        NOSE_BLOCK_LENGTH,
        NOSE_BLOCK_WIDTH,
        NOSE_BLOCK_H,
        (NOSE_BLOCK_LENGTH / 2.0, 0.0, NOSE_BLOCK_BASE_Z + NOSE_BLOCK_H / 2.0),
    )
    plate = _cq_box(
        NOSE_PLATE_T,
        NOSE_PLATE_WIDTH,
        NOSE_PLATE_HEIGHT,
        (NOSE_BLOCK_LENGTH + NOSE_PLATE_T / 2.0, 0.0, NOSE_PLATE_HEIGHT / 2.0),
    )
    rib = _cq_box(0.020, 0.050, 0.048, (NOSE_BLOCK_LENGTH - 0.006, 0.0, 0.024))
    body = slide_block.union(plate).union(rib)
    body = body.cut(_cq_box(0.008, 0.044, 0.042, (NOSE_BLOCK_LENGTH + 0.010, 0.0, 0.044)))
    body = body.edges("|Z").fillet(0.0018)
    return body


def _add_vertical_cap_screw(
    part,
    *,
    x: float,
    y: float,
    mount_z: float,
    shank_len: float,
    shank_radius: float,
    head_height: float,
    head_radius: float,
    material: str,
) -> None:
    part.visual(
        Cylinder(radius=shank_radius, length=shank_len),
        origin=Origin(xyz=(x, y, mount_z - shank_len / 2.0 + 0.001)),
        material=material,
    )
    part.visual(
        Cylinder(radius=head_radius, length=head_height),
        origin=Origin(xyz=(x, y, mount_z + head_height / 2.0 - 0.0005)),
        material=material,
    )


def _add_buffer_block(
    part,
    *,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material: str,
) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_extension_module")

    model.material("outer_aluminum", rgba=(0.72, 0.75, 0.78, 1.0))
    model.material("middle_anodized", rgba=(0.34, 0.37, 0.42, 1.0))
    model.material("inner_aluminum", rgba=(0.63, 0.66, 0.70, 1.0))
    model.material("nose_gray", rgba=(0.47, 0.49, 0.53, 1.0))
    model.material("guide_steel", rgba=(0.55, 0.57, 0.60, 1.0))
    model.material("buffer_polymer", rgba=(0.88, 0.49, 0.16, 1.0))
    model.material("cap_screw_black", rgba=(0.08, 0.08, 0.09, 1.0))

    outer = model.part("outer_body")
    outer.visual(
        Box((OUTER_LENGTH, OUTER_WIDTH, OUTER_BASE_T)),
        origin=Origin(xyz=(OUTER_LENGTH / 2.0, 0.0, OUTER_BASE_T / 2.0)),
        material="outer_aluminum",
        name="body_shell",
    )
    outer.visual(
        Box((OUTER_FOOT_L, OUTER_FOOT_W, OUTER_FOOT_T)),
        origin=Origin(xyz=(0.012 + OUTER_FOOT_L / 2.0, 0.0, OUTER_FOOT_T / 2.0)),
        material="outer_aluminum",
    )
    outer.visual(
        Box((OUTER_FOOT_L, OUTER_FOOT_W, OUTER_FOOT_T)),
        origin=Origin(
            xyz=(OUTER_LENGTH - 0.012 - OUTER_FOOT_L / 2.0, 0.0, OUTER_FOOT_T / 2.0)
        ),
        material="outer_aluminum",
    )
    outer.visual(
        Box((OUTER_LENGTH, OUTER_WALL_T, OUTER_SIDE_H)),
        origin=Origin(
            xyz=(OUTER_LENGTH / 2.0, OUTER_WIDTH / 2.0 - OUTER_WALL_T / 2.0, OUTER_SIDE_H / 2.0)
        ),
        material="outer_aluminum",
    )
    outer.visual(
        Box((OUTER_LENGTH, OUTER_WALL_T, OUTER_SIDE_H)),
        origin=Origin(
            xyz=(OUTER_LENGTH / 2.0, -OUTER_WIDTH / 2.0 + OUTER_WALL_T / 2.0, OUTER_SIDE_H / 2.0)
        ),
        material="outer_aluminum",
    )
    outer.visual(
        Box((0.040, 0.040, 0.016)),
        origin=Origin(xyz=(0.030, 0.0, OUTER_BASE_T + 0.008)),
        material="outer_aluminum",
    )
    outer.visual(
        Box((0.030, 0.034, 0.014)),
        origin=Origin(xyz=(OUTER_LENGTH - 0.025, 0.0, OUTER_BASE_T + 0.007)),
        material="outer_aluminum",
    )
    outer.visual(
        Box((OUTER_GUIDE_LENGTH, OUTER_GUIDE_WIDTH, OUTER_GUIDE_HEIGHT)),
        origin=Origin(xyz=(OUTER_LENGTH / 2.0, OUTER_GUIDE_Y, OUTER_BASE_T + OUTER_GUIDE_HEIGHT / 2.0)),
        material="guide_steel",
        name="left_guide_strip",
    )
    outer.visual(
        Box((OUTER_GUIDE_LENGTH, OUTER_GUIDE_WIDTH, OUTER_GUIDE_HEIGHT)),
        origin=Origin(xyz=(OUTER_LENGTH / 2.0, -OUTER_GUIDE_Y, OUTER_BASE_T + OUTER_GUIDE_HEIGHT / 2.0)),
        material="guide_steel",
        name="right_guide_strip",
    )
    for x_pos in (0.08, 0.20, 0.42, 0.54):
        for y_pos in (0.057, -0.057):
            _add_vertical_cap_screw(
                outer,
                x=x_pos,
                y=y_pos,
                mount_z=OUTER_SIDE_H,
                shank_len=0.010,
                shank_radius=0.0024,
                head_height=0.004,
                head_radius=0.005,
                material="cap_screw_black",
            )
    for x_pos in (0.036, OUTER_LENGTH - 0.036):
        for y_pos in (0.056, -0.056):
            _add_buffer_block(
                outer,
                size=(0.020, 0.008, 0.014),
                xyz=(x_pos, 0.047 if y_pos > 0.0 else -0.047, 0.025),
                material="buffer_polymer",
            )
    outer.inertial = Inertial.from_geometry(
        Box((OUTER_LENGTH, OUTER_FOOT_W, OUTER_SIDE_H)),
        mass=8.8,
        origin=Origin(xyz=(OUTER_LENGTH / 2.0, 0.0, OUTER_SIDE_H / 2.0)),
    )

    middle = model.part("middle_carriage")
    middle.visual(
        Box((MIDDLE_LENGTH, MIDDLE_WIDTH, MIDDLE_BODY_H)),
        origin=Origin(
            xyz=(MIDDLE_LENGTH / 2.0, 0.0, MIDDLE_BODY_BASE_Z + MIDDLE_BODY_H / 2.0)
        ),
        material="middle_anodized",
        name="body_shell",
    )
    middle.visual(
        Box((0.250, 0.036, 0.012)),
        origin=Origin(xyz=(0.220, 0.0, MIDDLE_BODY_BASE_Z + MIDDLE_BODY_H + 0.006)),
        material="middle_anodized",
    )
    middle.visual(
        Box((0.060, MIDDLE_WIDTH, 0.014)),
        origin=Origin(xyz=(MIDDLE_LENGTH - 0.030, 0.0, 0.019)),
        material="middle_anodized",
    )
    middle.visual(
        Box((0.050, MIDDLE_WIDTH, 0.012)),
        origin=Origin(xyz=(0.030, 0.0, 0.018)),
        material="middle_anodized",
    )
    middle.visual(
        Box((MIDDLE_SHOE_LENGTH, MIDDLE_SHOE_WIDTH, MIDDLE_SHOE_HEIGHT)),
        origin=Origin(xyz=(MIDDLE_SHOE_X, OUTER_GUIDE_Y, MIDDLE_SHOE_HEIGHT / 2.0)),
        material="guide_steel",
        name="stage1_left_shoe",
    )
    middle.visual(
        Box((MIDDLE_SHOE_LENGTH, MIDDLE_SHOE_WIDTH, MIDDLE_SHOE_HEIGHT)),
        origin=Origin(xyz=(MIDDLE_SHOE_X, -OUTER_GUIDE_Y, MIDDLE_SHOE_HEIGHT / 2.0)),
        material="guide_steel",
        name="stage1_right_shoe",
    )
    middle.visual(
        Box((MIDDLE_STAGE2_GUIDE_LENGTH, MIDDLE_STAGE2_GUIDE_WIDTH, MIDDLE_STAGE2_GUIDE_HEIGHT)),
        origin=Origin(
            xyz=(
                MIDDLE_STAGE2_GUIDE_X,
                MIDDLE_STAGE2_GUIDE_Y,
                MIDDLE_STAGE2_GUIDE_BASE_Z + MIDDLE_STAGE2_GUIDE_HEIGHT / 2.0,
            )
        ),
        material="guide_steel",
        name="stage2_left_guide",
    )
    middle.visual(
        Box((MIDDLE_STAGE2_GUIDE_LENGTH, MIDDLE_STAGE2_GUIDE_WIDTH, MIDDLE_STAGE2_GUIDE_HEIGHT)),
        origin=Origin(
            xyz=(
                MIDDLE_STAGE2_GUIDE_X,
                -MIDDLE_STAGE2_GUIDE_Y,
                MIDDLE_STAGE2_GUIDE_BASE_Z + MIDDLE_STAGE2_GUIDE_HEIGHT / 2.0,
            )
        ),
        material="guide_steel",
        name="stage2_right_guide",
    )
    for x_pos in (0.07, 0.17, 0.27, 0.37):
        for y_pos in (0.040, -0.040):
            _add_vertical_cap_screw(
                middle,
                x=x_pos,
                y=y_pos,
                mount_z=MIDDLE_BODY_BASE_Z + MIDDLE_BODY_H,
                shank_len=0.008,
                shank_radius=0.0022,
                head_height=0.0036,
                head_radius=0.0046,
                material="cap_screw_black",
            )
    for x_pos in (0.028, MIDDLE_LENGTH - 0.028):
        for y_pos in (0.031, -0.031):
            _add_buffer_block(
                middle,
                size=(0.016, 0.008, 0.012),
                xyz=(x_pos, 0.041 if y_pos > 0.0 else -0.041, 0.022),
                material="buffer_polymer",
            )
    middle.inertial = Inertial.from_geometry(
        Box((MIDDLE_LENGTH, MIDDLE_WIDTH, MIDDLE_BODY_BASE_Z + MIDDLE_BODY_H)),
        mass=3.6,
        origin=Origin(xyz=(MIDDLE_LENGTH / 2.0, 0.0, (MIDDLE_BODY_BASE_Z + MIDDLE_BODY_H) / 2.0)),
    )

    inner = model.part("inner_carriage")
    inner.visual(
        Box((INNER_LENGTH, INNER_WIDTH, INNER_BODY_H)),
        origin=Origin(xyz=(INNER_LENGTH / 2.0, 0.0, INNER_BODY_BASE_Z + INNER_BODY_H / 2.0)),
        material="inner_aluminum",
        name="body_shell",
    )
    inner.visual(
        Box((0.140, 0.018, 0.010)),
        origin=Origin(xyz=(0.135, 0.0, INNER_BODY_BASE_Z + INNER_BODY_H + 0.005)),
        material="inner_aluminum",
    )
    inner.visual(
        Box((0.038, INNER_WIDTH, 0.014)),
        origin=Origin(xyz=(INNER_LENGTH - 0.020, 0.0, 0.018)),
        material="inner_aluminum",
    )
    inner.visual(
        Box((0.036, INNER_WIDTH, 0.012)),
        origin=Origin(xyz=(0.020, 0.0, 0.017)),
        material="inner_aluminum",
    )
    inner.visual(
        Box((INNER_SHOE_LENGTH, INNER_SHOE_WIDTH, INNER_SHOE_HEIGHT)),
        origin=Origin(xyz=(INNER_SHOE_X, INNER_SHOE_Y, INNER_SHOE_HEIGHT / 2.0)),
        material="guide_steel",
        name="stage2_left_shoe",
    )
    inner.visual(
        Box((INNER_SHOE_LENGTH, INNER_SHOE_WIDTH, INNER_SHOE_HEIGHT)),
        origin=Origin(xyz=(INNER_SHOE_X, -INNER_SHOE_Y, INNER_SHOE_HEIGHT / 2.0)),
        material="guide_steel",
        name="stage2_right_shoe",
    )
    inner.visual(
        Box((INNER_STAGE3_GUIDE_LENGTH, INNER_STAGE3_GUIDE_WIDTH, INNER_STAGE3_GUIDE_HEIGHT)),
        origin=Origin(
            xyz=(
                INNER_STAGE3_GUIDE_X,
                INNER_STAGE3_GUIDE_Y,
                INNER_STAGE3_GUIDE_BASE_Z + INNER_STAGE3_GUIDE_HEIGHT / 2.0,
            )
        ),
        material="guide_steel",
        name="stage3_left_guide",
    )
    inner.visual(
        Box((INNER_STAGE3_GUIDE_LENGTH, INNER_STAGE3_GUIDE_WIDTH, INNER_STAGE3_GUIDE_HEIGHT)),
        origin=Origin(
            xyz=(
                INNER_STAGE3_GUIDE_X,
                -INNER_STAGE3_GUIDE_Y,
                INNER_STAGE3_GUIDE_BASE_Z + INNER_STAGE3_GUIDE_HEIGHT / 2.0,
            )
        ),
        material="guide_steel",
        name="stage3_right_guide",
    )
    for x_pos in (0.06, 0.13, 0.20):
        for y_pos in (0.030, -0.030):
            _add_vertical_cap_screw(
                inner,
                x=x_pos,
                y=y_pos,
                mount_z=INNER_BODY_BASE_Z + INNER_BODY_H,
                shank_len=0.006,
                shank_radius=0.0019,
                head_height=0.003,
                head_radius=0.0038,
                material="cap_screw_black",
            )
    for x_pos in (0.020, INNER_LENGTH - 0.020):
        for y_pos in (0.023, -0.023):
            _add_buffer_block(
                inner,
                size=(0.014, 0.007, 0.010),
                xyz=(x_pos, 0.0305 if y_pos > 0.0 else -0.0305, 0.019),
                material="buffer_polymer",
            )
    inner.inertial = Inertial.from_geometry(
        Box((INNER_LENGTH, INNER_WIDTH, INNER_BODY_BASE_Z + INNER_BODY_H)),
        mass=1.8,
        origin=Origin(xyz=(INNER_LENGTH / 2.0, 0.0, (INNER_BODY_BASE_Z + INNER_BODY_H) / 2.0)),
    )

    nose = model.part("nose_plate")
    nose.visual(
        Box((NOSE_BLOCK_LENGTH, NOSE_BLOCK_WIDTH, NOSE_BLOCK_H)),
        origin=Origin(
            xyz=(NOSE_BLOCK_LENGTH / 2.0, 0.0, NOSE_BLOCK_BASE_Z + NOSE_BLOCK_H / 2.0)
        ),
        material="nose_gray",
        name="plate_shell",
    )
    nose.visual(
        Box((NOSE_PLATE_T, NOSE_PLATE_WIDTH, NOSE_PLATE_HEIGHT)),
        origin=Origin(
            xyz=(NOSE_BLOCK_LENGTH + NOSE_PLATE_T / 2.0, 0.0, 0.012 + NOSE_PLATE_HEIGHT / 2.0)
        ),
        material="nose_gray",
    )
    nose.visual(
        Box((0.020, 0.038, 0.036)),
        origin=Origin(xyz=(NOSE_BLOCK_LENGTH - 0.006, 0.0, 0.012 + 0.018)),
        material="nose_gray",
    )
    nose.visual(
        Box((NOSE_SHOE_LENGTH, NOSE_SHOE_WIDTH, NOSE_SHOE_HEIGHT)),
        origin=Origin(xyz=(NOSE_SHOE_LENGTH / 2.0, NOSE_SHOE_Y, NOSE_SHOE_HEIGHT / 2.0)),
        material="guide_steel",
        name="stage3_left_shoe",
    )
    nose.visual(
        Box((NOSE_SHOE_LENGTH, NOSE_SHOE_WIDTH, NOSE_SHOE_HEIGHT)),
        origin=Origin(xyz=(NOSE_SHOE_LENGTH / 2.0, -NOSE_SHOE_Y, NOSE_SHOE_HEIGHT / 2.0)),
        material="guide_steel",
        name="stage3_right_shoe",
    )
    for y_pos in (0.024, -0.024):
        nose.visual(
            Cylinder(radius=0.0032, length=0.008),
            origin=Origin(
                xyz=(NOSE_BLOCK_LENGTH + NOSE_PLATE_T / 2.0, y_pos, NOSE_PLATE_HEIGHT * 0.64),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material="cap_screw_black",
        )
        nose.visual(
            Cylinder(radius=0.0032, length=0.008),
            origin=Origin(
                xyz=(NOSE_BLOCK_LENGTH + NOSE_PLATE_T / 2.0, y_pos, NOSE_PLATE_HEIGHT * 0.30),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material="cap_screw_black",
        )
    _add_buffer_block(
        nose,
        size=(0.010, 0.026, 0.012),
        xyz=(NOSE_BLOCK_LENGTH + NOSE_PLATE_T + 0.002, 0.0, 0.018),
        material="buffer_polymer",
    )
    nose.inertial = Inertial.from_geometry(
        Box((NOSE_BLOCK_LENGTH + NOSE_PLATE_T, NOSE_PLATE_WIDTH, NOSE_PLATE_HEIGHT)),
        mass=0.9,
        origin=Origin(
            xyz=((NOSE_BLOCK_LENGTH + NOSE_PLATE_T) / 2.0, 0.0, NOSE_PLATE_HEIGHT / 2.0)
        ),
    )

    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=middle,
        origin=Origin(xyz=(OUTER_STAGE_HOME_X, 0.0, OUTER_GUIDE_TOP_Z)),
        axis=SLIDE_AXIS,
        motion_limits=MotionLimits(
            effort=1800.0,
            velocity=0.45,
            lower=0.0,
            upper=OUTER_STAGE_TRAVEL,
        ),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=inner,
        origin=Origin(xyz=(MIDDLE_TO_INNER_HOME_X, 0.0, MIDDLE_STAGE2_GUIDE_TOP_Z)),
        axis=SLIDE_AXIS,
        motion_limits=MotionLimits(
            effort=900.0,
            velocity=0.40,
            lower=0.0,
            upper=MIDDLE_TO_INNER_TRAVEL,
        ),
    )
    model.articulation(
        "inner_to_nose",
        ArticulationType.PRISMATIC,
        parent=inner,
        child=nose,
        origin=Origin(xyz=(INNER_TO_NOSE_HOME_X, 0.0, INNER_STAGE3_GUIDE_TOP_Z)),
        axis=SLIDE_AXIS,
        motion_limits=MotionLimits(
            effort=350.0,
            velocity=0.35,
            lower=0.0,
            upper=INNER_TO_NOSE_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_body")
    middle = object_model.get_part("middle_carriage")
    inner = object_model.get_part("inner_carriage")
    nose = object_model.get_part("nose_plate")
    stage1 = object_model.get_articulation("outer_to_middle")
    stage2 = object_model.get_articulation("middle_to_inner")
    stage3 = object_model.get_articulation("inner_to_nose")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    joints = (stage1, stage2, stage3)
    ctx.check(
        "aligned_prismatic_axes",
        all(tuple(joint.axis) == SLIDE_AXIS for joint in joints),
        details="All three stages should slide along the same +X guide direction.",
    )

    ctx.expect_contact(
        middle,
        outer,
        elem_a="stage1_left_shoe",
        elem_b="left_guide_strip",
        name="stage1_left_shoe_supported",
    )
    ctx.expect_contact(
        middle,
        outer,
        elem_a="stage1_right_shoe",
        elem_b="right_guide_strip",
        name="stage1_right_shoe_supported",
    )
    ctx.expect_contact(
        inner,
        middle,
        elem_a="stage2_left_shoe",
        elem_b="stage2_left_guide",
        name="stage2_left_shoe_supported",
    )
    ctx.expect_contact(
        inner,
        middle,
        elem_a="stage2_right_shoe",
        elem_b="stage2_right_guide",
        name="stage2_right_shoe_supported",
    )
    ctx.expect_contact(
        nose,
        inner,
        elem_a="stage3_left_shoe",
        elem_b="stage3_left_guide",
        name="stage3_left_shoe_supported",
    )
    ctx.expect_contact(
        nose,
        inner,
        elem_a="stage3_right_shoe",
        elem_b="stage3_right_guide",
        name="stage3_right_shoe_supported",
    )

    with ctx.pose(
        {
            stage1: OUTER_STAGE_TRAVEL,
            stage2: MIDDLE_TO_INNER_TRAVEL,
            stage3: INNER_TO_NOSE_TRAVEL,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_full_extension")
        ctx.expect_contact(
            middle,
            outer,
            elem_a="stage1_left_shoe",
            elem_b="left_guide_strip",
            name="stage1_stays_on_guides_fully_extended",
        )
        ctx.expect_contact(
            inner,
            middle,
            elem_a="stage2_left_shoe",
            elem_b="stage2_left_guide",
            name="stage2_stays_on_guides_fully_extended",
        )
        ctx.expect_contact(
            nose,
            inner,
            elem_a="stage3_left_shoe",
            elem_b="stage3_left_guide",
            name="stage3_stays_on_guides_fully_extended",
        )
        ctx.expect_overlap(
            middle,
            outer,
            axes="x",
            elem_a="stage1_left_shoe",
            elem_b="left_guide_strip",
            min_overlap=0.24,
            name="stage1_keeps_long_engagement",
        )
        ctx.expect_overlap(
            inner,
            middle,
            axes="x",
            elem_a="stage2_left_shoe",
            elem_b="stage2_left_guide",
            min_overlap=0.11,
            name="stage2_keeps_long_engagement",
        )
        ctx.expect_overlap(
            nose,
            inner,
            axes="x",
            elem_a="stage3_left_shoe",
            elem_b="stage3_left_guide",
            min_overlap=0.055,
            name="stage3_keeps_compact_engagement",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
