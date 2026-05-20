from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


CASE_W = 0.170
CASE_D = 0.112
FLOOR_H = 0.010
PLATE_H = 0.006
PLATE_Z = 0.026
PLATE_TOP_Z = PLATE_Z + PLATE_H / 2.0

ROWS = 4
COLS = 5
KEY_PITCH = 0.020
KEY_START_X = -0.045
KEY_START_Y = -0.030
KEY_TRAVEL = 0.006

KNOB_X = 0.064
KNOB_Y = 0.036
KNOB_Z = 0.046
KNOB_DIAMETER = 0.028
KNOB_LENGTH = 0.026


def _translated_profile(profile, dx: float, dy: float):
    return [(x + dx, y + dy) for x, y in profile]


def _profile_at_z(profile, z: float):
    return [(x, y, z) for x, y in profile]


def _key_center(row: int, col: int) -> tuple[float, float]:
    return (KEY_START_X + col * KEY_PITCH, KEY_START_Y + row * KEY_PITCH)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_keyboard_with_corner_knob")

    case_mat = model.material("anodized_charcoal", rgba=(0.055, 0.060, 0.065, 1.0))
    plate_mat = model.material("brushed_gunmetal", rgba=(0.22, 0.23, 0.24, 1.0))
    cheek_mat = model.material("black_anodized_bracket", rgba=(0.015, 0.017, 0.020, 1.0))
    key_mat = model.material("warm_gray_keycaps", rgba=(0.78, 0.77, 0.72, 1.0))
    accent_key_mat = model.material("charcoal_keycaps", rgba=(0.12, 0.13, 0.14, 1.0))
    switch_mat = model.material("black_switch_stems", rgba=(0.02, 0.018, 0.017, 1.0))
    legend_mat = model.material("off_white_legends", rgba=(0.96, 0.94, 0.86, 1.0))
    knob_mat = model.material("matte_black_knurled", rgba=(0.01, 0.010, 0.012, 1.0))
    metal_mat = model.material("dark_steel_axle", rgba=(0.38, 0.39, 0.40, 1.0))

    # A continuous top plate with real through-openings for the switch stems.
    outer_plate = rounded_rect_profile(CASE_W, CASE_D, 0.008, corner_segments=8)
    switch_opening = rounded_rect_profile(0.0145, 0.0145, 0.0016, corner_segments=3)
    hole_profiles = []
    for row in range(ROWS):
        for col in range(COLS):
            x, y = _key_center(row, col)
            hole_profiles.append(_translated_profile(switch_opening, x, y))

    top_plate_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            outer_plate,
            hole_profiles,
            PLATE_H,
            center=True,
        ),
        "top_plate_with_switch_openings",
    )

    # A sloped keycap: wider lower skirt, smaller flat top.
    cap_bottom = rounded_rect_profile(0.0176, 0.0176, 0.0022, corner_segments=5)
    cap_top = rounded_rect_profile(0.0152, 0.0152, 0.0020, corner_segments=5)
    keycap_mesh = mesh_from_geometry(
        section_loft(
            [
                _profile_at_z(cap_bottom, 0.0),
                _profile_at_z(cap_top, 0.009),
            ]
        ),
        "sloped_keycap",
    )

    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            KNOB_DIAMETER,
            KNOB_LENGTH,
            body_style="cylindrical",
            edge_radius=0.0007,
            grip=KnobGrip(style="knurled", count=36, depth=0.0007, helix_angle_deg=18.0),
            indicator=KnobIndicator(style="dot", mode="raised", angle_deg=0.0),
        ),
        "corner_rotary_knob",
    )

    case = model.part("case")
    case.visual(
        Box((CASE_W, CASE_D, FLOOR_H)),
        origin=Origin(xyz=(0.0, 0.0, FLOOR_H / 2.0)),
        material=case_mat,
        name="floor",
    )
    wall_h = PLATE_TOP_Z - 0.004
    wall_z = wall_h / 2.0
    wall_t = 0.006
    case.visual(
        Box((wall_t, CASE_D, wall_h)),
        origin=Origin(xyz=(-(CASE_W - wall_t) / 2.0, 0.0, wall_z)),
        material=case_mat,
        name="side_wall_0",
    )
    case.visual(
        Box((wall_t, CASE_D, wall_h)),
        origin=Origin(xyz=((CASE_W - wall_t) / 2.0, 0.0, wall_z)),
        material=case_mat,
        name="side_wall_1",
    )
    case.visual(
        Box((CASE_W, wall_t, wall_h)),
        origin=Origin(xyz=(0.0, -(CASE_D - wall_t) / 2.0, wall_z)),
        material=case_mat,
        name="front_wall",
    )
    case.visual(
        Box((CASE_W, wall_t, wall_h)),
        origin=Origin(xyz=(0.0, (CASE_D - wall_t) / 2.0, wall_z)),
        material=case_mat,
        name="rear_wall",
    )
    case.visual(
        top_plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, PLATE_Z)),
        material=plate_mat,
        name="top_plate",
    )

    # Fork-like cheeks make the horizontal corner volume knob read as mounted.
    cheek_y_gap = KNOB_LENGTH / 2.0 + 0.003
    for suffix, y in (("front", KNOB_Y - cheek_y_gap), ("rear", KNOB_Y + cheek_y_gap)):
        case.visual(
            Box((0.036, 0.004, 0.034)),
            origin=Origin(xyz=(KNOB_X, y, 0.043)),
            material=cheek_mat,
            name=f"knob_{suffix}_cheek",
        )
        case.visual(
            Cylinder(radius=0.0065, length=0.0015),
            origin=Origin(
                xyz=(KNOB_X, y + (0.0026 if suffix == "front" else -0.0026), KNOB_Z),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=metal_mat,
            name=f"knob_{suffix}_bearing",
        )

    for row in range(ROWS):
        for col in range(COLS):
            x, y = _key_center(row, col)
            key = model.part(f"key_{row}_{col}")
            material: Material = accent_key_mat if (row == ROWS - 1 and col in (3, 4)) else key_mat
            key.visual(
                Box((0.0065, 0.0065, 0.016)),
                origin=Origin(xyz=(0.0, 0.0, 0.0)),
                material=switch_mat,
                name="stem",
            )
            key.visual(
                keycap_mesh,
                origin=Origin(xyz=(0.0, 0.0, 0.0078)),
                material=material,
                name="cap",
            )
            # A small raised legend stripe gives each cap a manufactured face
            # without making the legends separate floating parts.
            key.visual(
                Box((0.0060, 0.0012, 0.0006)),
                origin=Origin(xyz=(0.0, -0.0025, 0.0170)),
                material=legend_mat,
                name="legend",
            )
            model.articulation(
                f"case_to_key_{row}_{col}",
                ArticulationType.PRISMATIC,
                parent=case,
                child=key,
                origin=Origin(xyz=(x, y, PLATE_TOP_Z)),
                axis=(0.0, 0.0, -1.0),
                motion_limits=MotionLimits(
                    lower=0.0,
                    upper=KEY_TRAVEL,
                    effort=2.0,
                    velocity=0.20,
                ),
            )

    knob = model.part("corner_knob")
    knob.visual(
        knob_mesh,
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=knob_mat,
        name="knob_cap",
    )
    knob.visual(
        Cylinder(radius=0.0032, length=0.030),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=metal_mat,
        name="axle",
    )
    model.articulation(
        "case_to_corner_knob",
        ArticulationType.REVOLUTE,
        parent=case,
        child=knob,
        origin=Origin(xyz=(KNOB_X, KNOB_Y, KNOB_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-math.pi,
            upper=math.pi,
            effort=0.8,
            velocity=8.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    case = object_model.get_part("case")
    key = object_model.get_part("key_0_0")
    key_joint = object_model.get_articulation("case_to_key_0_0")
    knob_joint = object_model.get_articulation("case_to_corner_knob")

    key_joints = [
        object_model.get_articulation(f"case_to_key_{row}_{col}")
        for row in range(ROWS)
        for col in range(COLS)
    ]
    ctx.check(
        "every keycap has a prismatic slider",
        len(key_joints) == ROWS * COLS
        and all(j.articulation_type == ArticulationType.PRISMATIC for j in key_joints),
        details=f"found {len(key_joints)} key slider joints",
    )
    ctx.check(
        "key sliders travel vertically into openings",
        all(abs(j.axis[2] + 1.0) < 1e-6 for j in key_joints),
        details=[j.axis for j in key_joints],
    )
    ctx.check(
        "knob rotates front to back",
        knob_joint.articulation_type == ArticulationType.REVOLUTE
        and abs(knob_joint.axis[1] - 1.0) < 1e-6,
        details=f"type={knob_joint.articulation_type}, axis={knob_joint.axis}",
    )

    ctx.expect_gap(
        key,
        case,
        axis="z",
        positive_elem="cap",
        negative_elem="top_plate",
        min_gap=0.006,
        max_gap=0.010,
        name="rest keycap clears top plate",
    )
    ctx.expect_overlap(
        key,
        case,
        axes="xy",
        elem_a="stem",
        elem_b="top_plate",
        min_overlap=0.006,
        name="stem enters switch opening footprint",
    )

    rest_pos = ctx.part_world_position(key)
    with ctx.pose({key_joint: KEY_TRAVEL}):
        ctx.expect_gap(
            key,
            case,
            axis="z",
            positive_elem="cap",
            negative_elem="top_plate",
            min_gap=0.001,
            max_gap=0.004,
            name="pressed keycap stops above plate",
        )
        pressed_pos = ctx.part_world_position(key)
    ctx.check(
        "key slider moves downward",
        rest_pos is not None and pressed_pos is not None and pressed_pos[2] < rest_pos[2] - 0.005,
        details=f"rest={rest_pos}, pressed={pressed_pos}",
    )

    return ctx.report()


object_model = build_object_model()
