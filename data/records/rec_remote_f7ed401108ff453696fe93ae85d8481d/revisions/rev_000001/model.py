from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


CASE_WIDTH = 0.62
CASE_DEPTH = 0.32
CASE_HEIGHT = 0.078
TOP_THICKNESS = 0.007
TOP_EMBED = 0.001
CASE_TOP = CASE_HEIGHT - TOP_EMBED + TOP_THICKNESS

JOYSTICK_X = -0.19
JOYSTICK_Y = 0.0
COLLAR_HEIGHT = 0.014
JOYSTICK_JOINT_Z = CASE_TOP + COLLAR_HEIGHT

BUTTON_OUTER_R = 0.034
BUTTON_INNER_R = 0.028
BUTTON_HOUSING_H = 0.010
BUTTON_HOUSING_BASE_Z = CASE_TOP - 0.001
BUTTON_JOINT_Z = BUTTON_HOUSING_BASE_Z + BUTTON_HOUSING_H
BUTTON_POSITIONS = (
    (0.105, 0.058),
    (0.185, 0.058),
    (0.265, 0.058),
    (0.125, -0.045),
    (0.205, -0.045),
    (0.285, -0.045),
)


def _circle_profile(radius: float, segments: int = 64) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * i / segments),
            radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="arcade_fight_stick")

    body_mat = model.material("dark_plastic", rgba=(0.035, 0.038, 0.045, 1.0))
    top_mat = model.material("satin_black_panel", rgba=(0.08, 0.085, 0.095, 1.0))
    trim_mat = model.material("black_trim", rgba=(0.008, 0.008, 0.009, 1.0))
    metal_mat = model.material("brushed_steel", rgba=(0.72, 0.72, 0.68, 1.0))
    rubber_mat = model.material("matte_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    red_mat = model.material("red_button", rgba=(0.86, 0.04, 0.03, 1.0))
    blue_mat = model.material("blue_button", rgba=(0.04, 0.20, 0.85, 1.0))
    yellow_mat = model.material("yellow_button", rgba=(0.95, 0.76, 0.04, 1.0))
    green_mat = model.material("green_button", rgba=(0.02, 0.55, 0.14, 1.0))
    white_mat = model.material("white_button", rgba=(0.90, 0.90, 0.84, 1.0))
    purple_mat = model.material("purple_button", rgba=(0.46, 0.12, 0.74, 1.0))

    case = model.part("case")
    case_body_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(CASE_WIDTH, CASE_DEPTH, 0.035, corner_segments=10),
            CASE_HEIGHT,
        ),
        "rounded_case_body",
    )
    top_panel_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(
                CASE_WIDTH - 0.034,
                CASE_DEPTH - 0.030,
                0.026,
                corner_segments=10,
            ),
            TOP_THICKNESS,
        ),
        "recessed_top_panel",
    )
    button_housing_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(BUTTON_OUTER_R),
            [_circle_profile(BUTTON_INNER_R)],
            BUTTON_HOUSING_H,
            center=False,
        ),
        "button_housing_ring",
    )

    case.visual(case_body_mesh, material=body_mat, name="case_shell")
    case.visual(
        top_panel_mesh,
        origin=Origin(xyz=(0.0, 0.0, CASE_HEIGHT - TOP_EMBED)),
        material=top_mat,
        name="top_panel",
    )

    # A broad dust washer and raised collar make the joystick pivot location
    # visibly mechanical without occupying the moving shaft's volume.
    case.visual(
        Cylinder(radius=0.060, length=0.005),
        origin=Origin(xyz=(JOYSTICK_X, JOYSTICK_Y, CASE_TOP + 0.0025)),
        material=trim_mat,
        name="dust_washer",
    )
    case.visual(
        Cylinder(radius=0.033, length=COLLAR_HEIGHT),
        origin=Origin(
            xyz=(JOYSTICK_X, JOYSTICK_Y, CASE_TOP + COLLAR_HEIGHT * 0.5)
        ),
        material=rubber_mat,
        name="joystick_collar",
    )
    for lug_index, y_sign in enumerate((-1.0, 1.0)):
        case.visual(
            Box((0.042, 0.014, 0.020)),
            origin=Origin(
                xyz=(JOYSTICK_X, y_sign * 0.049, CASE_TOP + 0.010)
            ),
            material=trim_mat,
            name=f"pivot_lug_{lug_index}",
        )

    for button_index, (x, y) in enumerate(BUTTON_POSITIONS):
        case.visual(
            button_housing_mesh,
            origin=Origin(xyz=(x, y, BUTTON_HOUSING_BASE_Z)),
            material=trim_mat,
            name=f"button_housing_{button_index}",
        )

    for screw_index, (x, y) in enumerate(
        (
            (-0.275, -0.125),
            (-0.275, 0.125),
            (0.275, -0.125),
            (0.275, 0.125),
        )
    ):
        case.visual(
            Cylinder(radius=0.007, length=0.002),
            origin=Origin(xyz=(x, y, CASE_TOP + 0.001)),
            material=metal_mat,
            name=f"screw_{screw_index}",
        )

    joystick = model.part("joystick")
    joystick.visual(
        Cylinder(radius=0.014, length=0.015),
        origin=Origin(xyz=(0.0, 0.0, 0.0075)),
        material=metal_mat,
        name="shaft_root",
    )
    joystick.visual(
        Cylinder(radius=0.0075, length=0.108),
        origin=Origin(xyz=(0.0, 0.0, 0.069)),
        material=metal_mat,
        name="shaft",
    )
    joystick.visual(
        Sphere(radius=0.033),
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        material=red_mat,
        name="ball_top",
    )
    model.articulation(
        "case_to_joystick",
        ArticulationType.REVOLUTE,
        parent=case,
        child=joystick,
        origin=Origin(xyz=(JOYSTICK_X, JOYSTICK_Y, JOYSTICK_JOINT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0, velocity=4.0, lower=-0.45, upper=0.45
        ),
    )

    button_materials = (
        red_mat,
        blue_mat,
        yellow_mat,
        green_mat,
        white_mat,
        purple_mat,
    )
    for button_index, ((x, y), button_mat) in enumerate(
        zip(BUTTON_POSITIONS, button_materials)
    ):
        button = model.part(f"button_{button_index}")
        button.visual(
            Cylinder(radius=0.023, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, 0.0030)),
            material=button_mat,
            name="button_stem",
        )
        button.visual(
            Cylinder(radius=0.031, length=0.009),
            origin=Origin(xyz=(0.0, 0.0, 0.0105)),
            material=button_mat,
            name="button_cap",
        )
        model.articulation(
            f"case_to_button_{button_index}",
            ArticulationType.PRISMATIC,
            parent=case,
            child=button,
            origin=Origin(xyz=(x, y, BUTTON_JOINT_Z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=1.5, velocity=0.15, lower=0.0, upper=0.006
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    case = object_model.get_part("case")
    joystick = object_model.get_part("joystick")
    joystick_joint = object_model.get_articulation("case_to_joystick")

    ctx.check(
        "case is wide and flat",
        CASE_WIDTH > 1.7 * CASE_DEPTH and CASE_DEPTH > 3.0 * CASE_HEIGHT,
        details=f"case dimensions={(CASE_WIDTH, CASE_DEPTH, CASE_HEIGHT)}",
    )
    ctx.check(
        "joystick is on the left half",
        JOYSTICK_X < -0.10 and JOYSTICK_X < 0.0,
        details=f"joystick_x={JOYSTICK_X}",
    )
    ctx.check(
        "six housings occupy right half",
        len(BUTTON_POSITIONS) == 6 and all(x > 0.0 for x, _ in BUTTON_POSITIONS),
        details=f"button_positions={BUTTON_POSITIONS}",
    )
    ctx.check(
        "joystick has horizontal revolute axis",
        joystick_joint.articulation_type == ArticulationType.REVOLUTE
        and abs(joystick_joint.axis[2]) < 1e-9
        and math.hypot(joystick_joint.axis[0], joystick_joint.axis[1]) > 0.99,
        details=f"axis={joystick_joint.axis}",
    )
    ctx.expect_gap(
        joystick,
        case,
        axis="z",
        max_gap=0.001,
        max_penetration=0.00001,
        positive_elem="shaft_root",
        negative_elem="joystick_collar",
        name="shaft base sits on collar",
    )
    ctx.expect_overlap(
        joystick,
        case,
        axes="xy",
        min_overlap=0.020,
        elem_a="shaft_root",
        elem_b="joystick_collar",
        name="shaft base is centered in collar footprint",
    )

    rest_ball_aabb = ctx.part_element_world_aabb(joystick, elem="ball_top")
    with ctx.pose({joystick_joint: 0.35}):
        tilted_ball_aabb = ctx.part_element_world_aabb(joystick, elem="ball_top")
    if rest_ball_aabb is not None and tilted_ball_aabb is not None:
        rest_ball_x = (rest_ball_aabb[0][0] + rest_ball_aabb[1][0]) * 0.5
        tilted_ball_x = (tilted_ball_aabb[0][0] + tilted_ball_aabb[1][0]) * 0.5
    else:
        rest_ball_x = tilted_ball_x = None
    ctx.check(
        "joystick shaft visibly pivots",
        rest_ball_x is not None
        and tilted_ball_x is not None
        and tilted_ball_x > rest_ball_x + 0.035,
        details=f"rest_ball_x={rest_ball_x}, tilted_ball_x={tilted_ball_x}",
    )

    for button_index in range(6):
        button = object_model.get_part(f"button_{button_index}")
        joint = object_model.get_articulation(f"case_to_button_{button_index}")
        ctx.check(
            f"button_{button_index} depresses vertically",
            joint.articulation_type == ArticulationType.PRISMATIC
            and joint.axis == (0.0, 0.0, -1.0),
            details=f"axis={joint.axis}, type={joint.articulation_type}",
        )
        ctx.expect_within(
            button,
            case,
            axes="xy",
            inner_elem="button_stem",
            outer_elem=f"button_housing_{button_index}",
            margin=0.002,
            name=f"button_{button_index} stem is captured by housing",
        )
        ctx.expect_gap(
            button,
            case,
            axis="z",
            min_gap=0.002,
            max_gap=0.007,
            positive_elem="button_cap",
            negative_elem=f"button_housing_{button_index}",
            name=f"button_{button_index} cap stands proud of housing",
        )

    first_button = object_model.get_part("button_0")
    first_joint = object_model.get_articulation("case_to_button_0")
    rest_button_pos = ctx.part_world_position(first_button)
    with ctx.pose({first_joint: 0.006}):
        pressed_button_pos = ctx.part_world_position(first_button)
    ctx.check(
        "button press moves downward",
        rest_button_pos is not None
        and pressed_button_pos is not None
        and pressed_button_pos[2] < rest_button_pos[2] - 0.005,
        details=f"rest={rest_button_pos}, pressed={pressed_button_pos}",
    )

    return ctx.report()


object_model = build_object_model()
