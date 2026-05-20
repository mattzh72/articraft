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
)

PANEL_SIZE = 0.95
PANEL_THICKNESS = 0.032
PANEL_HALF = PANEL_SIZE * 0.5
BORDER_WIDTH = 0.085
INNER_BORDER = PANEL_HALF - BORDER_WIDTH

INTAKE_OUTER = 0.56
INTAKE_OUTER_HALF = INTAKE_OUTER * 0.5
INTAKE_FRAME_WIDTH = 0.06
INTAKE_INNER = INTAKE_OUTER - (2.0 * INTAKE_FRAME_WIDTH)
INTAKE_INNER_HALF = INTAKE_INNER * 0.5

CORNER_BRIDGE = 0.18
CORNER_CENTER = 0.335

OUTLET_SPAN = 0.47
OUTLET_DEPTH = 0.102
FLAP_THICKNESS = 0.010
FLAP_BARREL_RADIUS = 0.006
FLAP_OPEN_ANGLE = 0.95

BUTTON_TRAVEL = 0.0025


def _add_box(part, *, size, xyz, material, name):
    return part.visual(
        Box(size),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def _add_flap(
    model: ArticulatedObject,
    panel,
    *,
    name: str,
    joint_name: str,
    joint_origin: tuple[float, float, float],
    axis: tuple[float, float, float],
    plate_size: tuple[float, float, float],
    plate_origin: tuple[float, float, float],
    barrel_length: float,
    barrel_rpy: tuple[float, float, float],
    material,
) -> None:
    flap = model.part(name)
    flap.visual(
        Cylinder(radius=FLAP_BARREL_RADIUS, length=barrel_length),
        origin=Origin(xyz=(0.0, 0.0, -FLAP_BARREL_RADIUS), rpy=barrel_rpy),
        material=material,
        name="hinge_barrel",
    )
    flap.visual(
        Box(plate_size),
        origin=Origin(xyz=plate_origin),
        material=material,
        name="flap_blade",
    )
    flap.visual(
        Box(
            (
                plate_size[0],
                0.014 if plate_size[1] < plate_size[0] else plate_size[1],
                0.020,
            )
            if plate_size[0] > plate_size[1]
            else (
                0.014,
                plate_size[1],
                0.020,
            )
        ),
        origin=Origin(
            xyz=(
                plate_origin[0] * 1.88 if plate_size[0] < plate_size[1] else 0.0,
                plate_origin[1] * 1.88 if plate_size[0] > plate_size[1] else 0.0,
                -0.012,
            )
        ),
        material=material,
        name="deflector_rib",
    )

    model.articulation(
        joint_name,
        ArticulationType.REVOLUTE,
        parent=panel,
        child=flap,
        origin=Origin(xyz=joint_origin),
        axis=axis,
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.2,
            lower=0.0,
            upper=FLAP_OPEN_ANGLE,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ceiling_cassette_air_conditioner")

    panel_white = model.material("panel_white", rgba=(0.93, 0.94, 0.92, 1.0))
    flap_white = model.material("flap_white", rgba=(0.89, 0.90, 0.88, 1.0))
    intake_dark = model.material("intake_dark", rgba=(0.36, 0.39, 0.41, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.70, 0.72, 0.74, 1.0))
    button_gray = model.material("button_gray", rgba=(0.63, 0.66, 0.68, 1.0))

    panel = model.part("panel")

    for name, xyz, size in (
        (
            "north_border",
            (0.0, PANEL_HALF - (BORDER_WIDTH * 0.5), PANEL_THICKNESS * 0.5),
            (PANEL_SIZE, BORDER_WIDTH, PANEL_THICKNESS),
        ),
        (
            "south_border",
            (0.0, -PANEL_HALF + (BORDER_WIDTH * 0.5), PANEL_THICKNESS * 0.5),
            (PANEL_SIZE, BORDER_WIDTH, PANEL_THICKNESS),
        ),
        (
            "east_border",
            (PANEL_HALF - (BORDER_WIDTH * 0.5), 0.0, PANEL_THICKNESS * 0.5),
            (BORDER_WIDTH, PANEL_SIZE, PANEL_THICKNESS),
        ),
        (
            "west_border",
            (-PANEL_HALF + (BORDER_WIDTH * 0.5), 0.0, PANEL_THICKNESS * 0.5),
            (BORDER_WIDTH, PANEL_SIZE, PANEL_THICKNESS),
        ),
    ):
        _add_box(panel, size=size, xyz=xyz, material=panel_white, name=name)

    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            _add_box(
                panel,
                size=(CORNER_BRIDGE, CORNER_BRIDGE, 0.028),
                xyz=(x_sign * CORNER_CENTER, y_sign * CORNER_CENTER, 0.014),
                material=panel_white,
                name=f"corner_bridge_{int((x_sign + 1.0) * 0.5)}_{int((y_sign + 1.0) * 0.5)}",
            )

    for name, xyz, size in (
        (
            "intake_top",
            (0.0, INTAKE_OUTER_HALF - (INTAKE_FRAME_WIDTH * 0.5), 0.016),
            (INTAKE_OUTER, INTAKE_FRAME_WIDTH, 0.024),
        ),
        (
            "intake_bottom",
            (0.0, -INTAKE_OUTER_HALF + (INTAKE_FRAME_WIDTH * 0.5), 0.016),
            (INTAKE_OUTER, INTAKE_FRAME_WIDTH, 0.024),
        ),
        (
            "intake_right",
            (INTAKE_OUTER_HALF - (INTAKE_FRAME_WIDTH * 0.5), 0.0, 0.016),
            (INTAKE_FRAME_WIDTH, INTAKE_OUTER, 0.024),
        ),
        (
            "intake_left",
            (-INTAKE_OUTER_HALF + (INTAKE_FRAME_WIDTH * 0.5), 0.0, 0.016),
            (INTAKE_FRAME_WIDTH, INTAKE_OUTER, 0.024),
        ),
    ):
        _add_box(panel, size=size, xyz=xyz, material=trim_gray, name=name)

    for index, y_pos in enumerate((-0.085, 0.085)):
        _add_box(
            panel,
            size=(INTAKE_INNER, 0.022, 0.014),
            xyz=(0.0, y_pos, 0.014),
            material=intake_dark,
            name=f"intake_crossbar_{index}",
        )
    for index, x_pos in enumerate((-0.085, 0.085)):
        _add_box(
            panel,
            size=(0.022, INTAKE_INNER, 0.014),
            xyz=(x_pos, 0.0, 0.014),
            material=intake_dark,
            name=f"intake_longbar_{index}",
        )
    _add_box(
        panel,
        size=(0.150, 0.150, 0.012),
        xyz=(0.0, 0.0, 0.013),
        material=intake_dark,
        name="intake_center_hub",
    )

    _add_box(
        panel,
        size=(0.090, 0.036, 0.003),
        xyz=(0.345, -0.095, 0.0015),
        material=trim_gray,
        name="service_label",
    )

    _add_flap(
        model,
        panel,
        name="cross_flap_0",
        joint_name="panel_to_cross_flap_0",
        joint_origin=(0.0, INNER_BORDER, 0.0),
        axis=(1.0, 0.0, 0.0),
        plate_size=(OUTLET_SPAN, OUTLET_DEPTH, FLAP_THICKNESS),
        plate_origin=(0.0, -(OUTLET_DEPTH * 0.5), -0.007),
        barrel_length=OUTLET_SPAN,
        barrel_rpy=(0.0, math.pi * 0.5, 0.0),
        material=flap_white,
    )
    _add_flap(
        model,
        panel,
        name="cross_flap_1",
        joint_name="panel_to_cross_flap_1",
        joint_origin=(0.0, -INNER_BORDER, 0.0),
        axis=(-1.0, 0.0, 0.0),
        plate_size=(OUTLET_SPAN, OUTLET_DEPTH, FLAP_THICKNESS),
        plate_origin=(0.0, OUTLET_DEPTH * 0.5, -0.007),
        barrel_length=OUTLET_SPAN,
        barrel_rpy=(0.0, math.pi * 0.5, 0.0),
        material=flap_white,
    )
    _add_flap(
        model,
        panel,
        name="service_flap",
        joint_name="panel_to_service_flap",
        joint_origin=(INNER_BORDER, 0.0, 0.0),
        axis=(0.0, -1.0, 0.0),
        plate_size=(OUTLET_DEPTH, OUTLET_SPAN, FLAP_THICKNESS),
        plate_origin=(-(OUTLET_DEPTH * 0.5), 0.0, -0.007),
        barrel_length=OUTLET_SPAN,
        barrel_rpy=(math.pi * 0.5, 0.0, 0.0),
        material=flap_white,
    )
    _add_flap(
        model,
        panel,
        name="opposite_flap",
        joint_name="panel_to_opposite_flap",
        joint_origin=(-INNER_BORDER, 0.0, 0.0),
        axis=(0.0, 1.0, 0.0),
        plate_size=(OUTLET_DEPTH, OUTLET_SPAN, FLAP_THICKNESS),
        plate_origin=(OUTLET_DEPTH * 0.5, 0.0, -0.007),
        barrel_length=OUTLET_SPAN,
        barrel_rpy=(math.pi * 0.5, 0.0, 0.0),
        material=flap_white,
    )

    for index, y_pos in enumerate((-0.115, -0.080)):
        button = model.part(f"service_button_{index}")
        button.visual(
            Box((0.028, 0.014, 0.005)),
            origin=Origin(xyz=(0.0, 0.0, -0.0055)),
            material=button_gray,
            name="button_cap",
        )
        button.visual(
            Box((0.016, 0.008, 0.004)),
            origin=Origin(xyz=(0.0, 0.0, -0.0020)),
            material=trim_gray,
            name="button_stem",
        )
        model.articulation(
            f"panel_to_service_button_{index}",
            ArticulationType.PRISMATIC,
            parent=panel,
            child=button,
            origin=Origin(xyz=(0.425, y_pos, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=12.0,
                velocity=0.03,
                lower=0.0,
                upper=BUTTON_TRAVEL,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    panel = object_model.get_part("panel")
    flap_names = (
        "cross_flap_0",
        "cross_flap_1",
        "service_flap",
        "opposite_flap",
    )
    flap_joint_names = (
        "panel_to_cross_flap_0",
        "panel_to_cross_flap_1",
        "panel_to_service_flap",
        "panel_to_opposite_flap",
    )

    for flap_name in flap_names:
        flap = object_model.get_part(flap_name)
        ctx.expect_gap(
            panel,
            flap,
            axis="z",
            max_gap=0.0005,
            max_penetration=0.00005,
            name=f"{flap_name} sits at the panel face when closed",
        )
        ctx.expect_overlap(
            panel,
            flap,
            axes="xy",
            min_overlap=0.080,
            name=f"{flap_name} stays under the panel footprint",
        )

    for joint_name, flap_name in zip(flap_joint_names, flap_names):
        joint = object_model.get_articulation(joint_name)
        flap = object_model.get_part(flap_name)
        rest_aabb = ctx.part_world_aabb(flap)
        with ctx.pose({joint: FLAP_OPEN_ANGLE}):
            open_aabb = ctx.part_world_aabb(flap)
        ctx.check(
            f"{flap_name} opens downward",
            rest_aabb is not None
            and open_aabb is not None
            and open_aabb[0][2] < rest_aabb[0][2] - 0.030,
            details=f"rest_aabb={rest_aabb}, open_aabb={open_aabb}",
        )

    for index in range(2):
        button = object_model.get_part(f"service_button_{index}")
        joint = object_model.get_articulation(f"panel_to_service_button_{index}")

        ctx.allow_overlap(
            panel,
            button,
            elem_b="button_stem",
            reason="The button stem is modeled as entering a simplified internal switch cavity behind the visible panel skin.",
        )

        ctx.expect_gap(
            panel,
            button,
            axis="z",
            min_gap=0.0004,
            max_gap=0.0032,
            negative_elem="button_cap",
            name=f"service_button_{index} protrudes slightly below the panel",
        )
        ctx.expect_overlap(
            panel,
            button,
            axes="xy",
            min_overlap=0.010,
            name=f"service_button_{index} stays within the service area",
        )

        rest_pos = ctx.part_world_position(button)
        with ctx.pose({joint: BUTTON_TRAVEL}):
            pressed_pos = ctx.part_world_position(button)
        ctx.check(
            f"service_button_{index} presses upward",
            rest_pos is not None
            and pressed_pos is not None
            and pressed_pos[2] > rest_pos[2] + 0.0015,
            details=f"rest_pos={rest_pos}, pressed_pos={pressed_pos}",
        )

    return ctx.report()


object_model = build_object_model()
