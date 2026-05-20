from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    VentGrilleFrame,
    VentGrilleGeometry,
    VentGrilleSlats,
    VentGrilleSleeve,
    mesh_from_geometry,
)

WIDTH = 1.07
DEPTH = 0.50
HEIGHT = 0.42
FRONT_THICKNESS = 0.024
WALL_THICKNESS = 0.018
REAR_WALL_THICKNESS = 0.020
JOIN = 0.002

INTAKE_WIDTH = 0.78
INTAKE_HEIGHT = 0.19
INTAKE_GRILLE_WIDTH = INTAKE_WIDTH + 0.028
INTAKE_GRILLE_HEIGHT = INTAKE_HEIGHT + 0.020
INTAKE_X = -0.11
INTAKE_Z = 0.277

OUTLET_WIDTH = 0.72
OUTLET_HEIGHT = 0.060
OUTLET_X = -0.14
OUTLET_Z = 0.110
OUTLET_DUCT_DEPTH = 0.460

CONTROL_WIDTH = 0.17
CONTROL_HEIGHT = 0.275
CONTROL_X = 0.405
CONTROL_Z = 0.220
CONTROL_RECESS_DEPTH = 0.008
SEPARATOR_WIDTH = 0.060
SEPARATOR_X = 0.290

BUTTON_WIDTH = 0.038
BUTTON_HEIGHT = 0.024
BUTTON_CAP_DEPTH = 0.010
BUTTON_STEM_DEPTH = 0.030
BUTTON_TRAVEL = 0.006
BUTTON_HOLE_WIDTH = 0.041
BUTTON_HOLE_HEIGHT = 0.027
BUTTON_X = CONTROL_X
BUTTON_ZS = (0.315, 0.276, 0.237, 0.198)

FLAP_WIDTH = OUTLET_WIDTH - 0.014
FLAP_HEIGHT = 0.056
FLAP_DEPTH = 0.018
FLAP_BARREL_RADIUS = 0.005
FLAP_FRONT_Y = 0.0015

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="through_wall_air_conditioner")

    cabinet = model.material("cabinet", rgba=(0.84, 0.83, 0.78, 1.0))
    grille = model.material("grille", rgba=(0.90, 0.89, 0.84, 1.0))
    shadow = model.material("shadow", rgba=(0.19, 0.20, 0.22, 1.0))
    button = model.material("button", rgba=(0.56, 0.57, 0.59, 1.0))
    trim = model.material("trim", rgba=(0.72, 0.72, 0.68, 1.0))

    body = model.part("body")
    body.visual(
        Box((WIDTH, DEPTH, WALL_THICKNESS)),
        origin=Origin(xyz=(0.0, DEPTH / 2.0, WALL_THICKNESS / 2.0)),
        material=cabinet,
        name="bottom_shell",
    )
    body.visual(
        Box((WIDTH, DEPTH, WALL_THICKNESS)),
        origin=Origin(xyz=(0.0, DEPTH / 2.0, HEIGHT - WALL_THICKNESS / 2.0)),
        material=cabinet,
        name="top_shell",
    )
    body.visual(
        Box((WALL_THICKNESS, DEPTH, HEIGHT - 2.0 * WALL_THICKNESS + JOIN)),
        origin=Origin(
            xyz=(
                -WIDTH / 2.0 + WALL_THICKNESS / 2.0,
                DEPTH / 2.0,
                HEIGHT / 2.0,
            )
        ),
        material=cabinet,
        name="left_shell",
    )
    body.visual(
        Box((WALL_THICKNESS, DEPTH, HEIGHT - 2.0 * WALL_THICKNESS + JOIN)),
        origin=Origin(
            xyz=(
                WIDTH / 2.0 - WALL_THICKNESS / 2.0,
                DEPTH / 2.0,
                HEIGHT / 2.0,
            )
        ),
        material=cabinet,
        name="right_shell",
    )
    body.visual(
        Box((WIDTH - 2.0 * WALL_THICKNESS + JOIN, REAR_WALL_THICKNESS, HEIGHT - 2.0 * WALL_THICKNESS + JOIN)),
        origin=Origin(
            xyz=(
                0.0,
                DEPTH - REAR_WALL_THICKNESS / 2.0,
                HEIGHT / 2.0,
            )
        ),
        material=cabinet,
        name="back_shell",
    )
    body.visual(
        Box((WIDTH - 2.0 * WALL_THICKNESS + JOIN, FRONT_THICKNESS, HEIGHT - (INTAKE_Z + INTAKE_HEIGHT / 2.0) + JOIN)),
        origin=Origin(
            xyz=(
                0.0,
                FRONT_THICKNESS / 2.0,
                (HEIGHT + INTAKE_Z + INTAKE_HEIGHT / 2.0) / 2.0,
            )
        ),
        material=cabinet,
        name="top_band",
    )
    body.visual(
        Box((WIDTH - 2.0 * WALL_THICKNESS + JOIN, FRONT_THICKNESS, OUTLET_Z - OUTLET_HEIGHT / 2.0 + JOIN)),
        origin=Origin(
            xyz=(
                0.0,
                FRONT_THICKNESS / 2.0,
                (OUTLET_Z - OUTLET_HEIGHT / 2.0 + JOIN) / 2.0,
            )
        ),
        material=cabinet,
        name="bottom_band",
    )
    body.visual(
        Box((WIDTH - 2.0 * WALL_THICKNESS + JOIN, FRONT_THICKNESS, INTAKE_Z - INTAKE_HEIGHT / 2.0 - (OUTLET_Z + OUTLET_HEIGHT / 2.0) + JOIN)),
        origin=Origin(
            xyz=(
                0.0,
                FRONT_THICKNESS / 2.0,
                ((INTAKE_Z - INTAKE_HEIGHT / 2.0) + (OUTLET_Z + OUTLET_HEIGHT / 2.0)) / 2.0,
            )
        ),
        material=cabinet,
        name="mid_band",
    )
    body.visual(
        Box((SEPARATOR_WIDTH, FRONT_THICKNESS, (INTAKE_Z + INTAKE_HEIGHT / 2.0) - (OUTLET_Z - OUTLET_HEIGHT / 2.0) + JOIN)),
        origin=Origin(
            xyz=(
                SEPARATOR_X,
                FRONT_THICKNESS / 2.0,
                ((INTAKE_Z + INTAKE_HEIGHT / 2.0) + (OUTLET_Z - OUTLET_HEIGHT / 2.0)) / 2.0,
            )
        ),
        material=cabinet,
        name="control_separator",
    )
    body.visual(
        mesh_from_geometry(
            VentGrilleGeometry(
                (INTAKE_GRILLE_WIDTH, INTAKE_GRILLE_HEIGHT),
                frame=0.013,
                face_thickness=0.004,
                duct_depth=0.032,
                duct_wall=0.003,
                slat_pitch=0.018,
                slat_width=0.009,
                slat_angle_deg=34.0,
                slats=VentGrilleSlats(
                    profile="airfoil",
                    direction="up",
                    inset=0.002,
                    divider_count=2,
                    divider_width=0.004,
                ),
                frame_profile=VentGrilleFrame(style="beveled", depth=0.002),
                sleeve=VentGrilleSleeve(style="short", depth=0.032, wall=0.003),
                corner_radius=0.006,
                center=False,
            ),
            "ac_intake_grille",
        ),
        origin=Origin(xyz=(INTAKE_X, 0.001, INTAKE_Z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=grille,
        name="intake_grille",
    )
    body.visual(
        Box((OUTLET_WIDTH - 0.030, OUTLET_DUCT_DEPTH, OUTLET_HEIGHT - 0.010)),
        origin=Origin(
            xyz=(
                OUTLET_X,
                FRONT_THICKNESS + OUTLET_DUCT_DEPTH / 2.0 - 0.001,
                OUTLET_Z,
            )
        ),
        material=shadow,
        name="outlet_cavity",
    )
    control_backer_depth = 0.440
    body.visual(
        Box((CONTROL_WIDTH - 0.030, control_backer_depth, CONTROL_HEIGHT - 0.020)),
        origin=Origin(
            xyz=(
                CONTROL_X,
                0.0405 + control_backer_depth / 2.0,
                CONTROL_Z,
            )
        ),
        material=trim,
        name="control_backer",
    )

    flap = model.part("flap")
    flap.visual(
        Box((FLAP_WIDTH, FLAP_DEPTH, FLAP_HEIGHT)),
        origin=Origin(xyz=(0.0, FLAP_DEPTH / 2.0, -FLAP_HEIGHT / 2.0 - 0.001)),
        material=grille,
        name="flap_panel",
    )
    flap.visual(
        Cylinder(radius=FLAP_BARREL_RADIUS, length=FLAP_WIDTH - 0.016),
        origin=Origin(xyz=(0.0, FLAP_DEPTH / 2.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grille,
        name="flap_barrel",
    )
    flap_hinge = model.articulation(
        "body_to_flap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=flap,
        origin=Origin(xyz=(OUTLET_X, FLAP_FRONT_Y, OUTLET_Z + OUTLET_HEIGHT / 2.0 - 0.004)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=1.5,
            lower=0.0,
            upper=0.95,
        ),
    )

    for index, button_z in enumerate(BUTTON_ZS):
        button_part = model.part(f"button_{index}")
        button_part.visual(
            Box((BUTTON_WIDTH, BUTTON_CAP_DEPTH, BUTTON_HEIGHT)),
            origin=Origin(xyz=(0.0, BUTTON_CAP_DEPTH / 2.0, 0.0)),
            material=button,
            name="button_cap",
        )
        button_part.visual(
            Box((BUTTON_WIDTH - 0.010, BUTTON_STEM_DEPTH, BUTTON_HEIGHT - 0.010)),
            origin=Origin(
                xyz=(
                    0.0,
                    BUTTON_CAP_DEPTH + BUTTON_STEM_DEPTH / 2.0 - 0.001,
                    0.0,
                )
            ),
            material=button,
            name="button_stem",
        )
        model.articulation(
            f"body_to_button_{index}",
            ArticulationType.PRISMATIC,
            parent=body,
            child=button_part,
            origin=Origin(xyz=(BUTTON_X, 0.0015, button_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=0.04,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    flap.meta["qc_samples"] = [0.0, 0.45, flap_hinge.motion_limits.upper]

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    flap = object_model.get_part("flap")
    flap_hinge = object_model.get_articulation("body_to_flap")

    button_parts = [object_model.get_part(f"button_{index}") for index in range(4)]
    button_joints = [object_model.get_articulation(f"body_to_button_{index}") for index in range(4)]

    ctx.check("four_buttons_present", len(button_parts) == 4, details=str([part.name for part in button_parts]))

    ctx.expect_gap(
        body,
        flap,
        axis="y",
        positive_elem="outlet_cavity",
        negative_elem="flap_panel",
        min_gap=0.002,
        max_gap=0.012,
        name="closed_flap_sits_just_ahead_of_outlet_cavity",
    )

    flap_rest_aabb = ctx.part_world_aabb(flap)
    with ctx.pose({flap_hinge: flap_hinge.motion_limits.upper}):
        flap_open_aabb = ctx.part_world_aabb(flap)
    ctx.check(
        "flap_opens_outward",
        flap_rest_aabb is not None
        and flap_open_aabb is not None
        and flap_open_aabb[0][1] < flap_rest_aabb[0][1] - 0.03,
        details=f"rest={flap_rest_aabb}, open={flap_open_aabb}",
    )

    rest_positions = [ctx.part_world_position(part) for part in button_parts]
    pressed_positions: list[tuple[float, float, float] | None] = []
    with ctx.pose({button_joints[0]: BUTTON_TRAVEL}):
        pressed_positions = [ctx.part_world_position(part) for part in button_parts]

    ctx.check(
        "button_0_presses_inward",
        rest_positions[0] is not None
        and pressed_positions[0] is not None
        and pressed_positions[0][1] > rest_positions[0][1] + BUTTON_TRAVEL * 0.8,
        details=f"rest={rest_positions[0]}, pressed={pressed_positions[0]}",
    )
    ctx.check(
        "buttons_move_independently",
        rest_positions[1] is not None
        and pressed_positions[1] is not None
        and abs(pressed_positions[1][1] - rest_positions[1][1]) < 1e-6,
        details=f"rest={rest_positions[1]}, pressed={pressed_positions[1]}",
    )

    for index, button_joint in enumerate(button_joints):
        rest_position = ctx.part_world_position(button_parts[index])
        with ctx.pose({button_joint: BUTTON_TRAVEL}):
            pressed_position = ctx.part_world_position(button_parts[index])
        ctx.check(
            f"button_{index}_has_short_travel",
            rest_position is not None
            and pressed_position is not None
            and BUTTON_TRAVEL * 0.8 < pressed_position[1] - rest_position[1] < 0.012,
            details=f"rest={rest_position}, pressed={pressed_position}",
        )

    return ctx.report()


object_model = build_object_model()
