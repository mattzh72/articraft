from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    LoftGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


HOUSING_WIDTH = 0.230
HOUSING_DEPTH = 0.160
BODY_HEIGHT = 0.024
DECK_HEIGHT = 0.002
TOP_Z = BODY_HEIGHT + DECK_HEIGHT


def _rounded_box_mesh(
    name: str,
    *,
    width: float,
    depth: float,
    height: float,
    radius: float,
):
    profile = rounded_rect_profile(width, depth, radius, corner_segments=10)
    return mesh_from_geometry(ExtrudeGeometry.from_z0(profile, height), name)


def _keycap_mesh(name: str):
    lower = rounded_rect_profile(0.030, 0.030, 0.0042, corner_segments=6)
    upper = rounded_rect_profile(0.026, 0.026, 0.0032, corner_segments=6)
    geom = LoftGeometry(
        [
            [(x, y, 0.0) for x, y in lower],
            [(x, y, 0.010) for x, y in upper],
        ],
        cap=True,
        closed=True,
    )
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="programmable_macro_keypad")

    body_mat = model.material("matte_charcoal", rgba=(0.055, 0.058, 0.065, 1.0))
    deck_mat = model.material("satin_black_deck", rgba=(0.015, 0.017, 0.020, 1.0))
    key_mat = model.material("warm_grey_keycaps", rgba=(0.78, 0.79, 0.77, 1.0))
    legend_mat = model.material("pale_legend", rgba=(0.95, 0.96, 0.93, 1.0))
    stem_mat = model.material("black_switch_stem", rgba=(0.035, 0.036, 0.038, 1.0))
    knob_mat = model.material("knurled_black_knobs", rgba=(0.09, 0.095, 0.10, 1.0))
    metal_mat = model.material("dark_hinge_metal", rgba=(0.18, 0.19, 0.20, 1.0))
    rubber_mat = model.material("rubber_foot", rgba=(0.025, 0.026, 0.027, 1.0))

    housing = model.part("housing")
    housing.visual(
        _rounded_box_mesh(
            "rounded_macro_keypad_housing",
            width=HOUSING_WIDTH,
            depth=HOUSING_DEPTH,
            height=BODY_HEIGHT,
            radius=0.014,
        ),
        material=body_mat,
        name="body_shell",
    )
    housing.visual(
        _rounded_box_mesh(
            "slightly_inset_top_deck",
            width=HOUSING_WIDTH - 0.012,
            depth=HOUSING_DEPTH - 0.012,
            height=DECK_HEIGHT,
            radius=0.010,
        ),
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT)),
        material=deck_mat,
        name="top_deck",
    )
    housing.visual(
        Box((0.038, 0.006, 0.010)),
        origin=Origin(xyz=(0.0, HOUSING_DEPTH * 0.5 + 0.001, BODY_HEIGHT * 0.45)),
        material=deck_mat,
        name="usb_recess",
    )
    for index, x in enumerate((-0.066, 0.066)):
        housing.visual(
            Box((0.022, 0.006, 0.008)),
            origin=Origin(xyz=(x, HOUSING_DEPTH * 0.5, -0.004)),
            material=metal_mat,
            name=f"hinge_ear_{index}",
        )
    for index, x in enumerate((-0.090, 0.090)):
        housing.visual(
            Box((0.024, 0.014, 0.003)),
            origin=Origin(xyz=(x, -0.058, -0.0015)),
            material=rubber_mat,
            name=f"front_pad_{index}",
        )

    key_mesh = _keycap_mesh("single_tapered_keycap")
    key_positions: list[tuple[int, int, float, float]] = []
    x_positions = (-0.060, -0.020, 0.020, 0.060)
    y_positions = (-0.050, -0.010, 0.030)
    for row, y in enumerate(y_positions):
        for col, x in enumerate(x_positions):
            key_positions.append((row, col, x, y))

    for row, col, x, y in key_positions:
        key = model.part(f"key_{row}_{col}")
        key.visual(
            Box((0.010, 0.010, 0.004)),
            origin=Origin(xyz=(0.0, 0.0, 0.002)),
            material=stem_mat,
            name="stem",
        )
        key.visual(
            key_mesh,
            origin=Origin(xyz=(0.0, 0.0, 0.004)),
            material=key_mat,
            name="cap",
        )
        key.visual(
            Box((0.012, 0.0022, 0.0005)),
            origin=Origin(xyz=(0.0, 0.0, 0.01425)),
            material=legend_mat,
            name="legend_bar",
        )
        model.articulation(
            f"housing_to_key_{row}_{col}",
            ArticulationType.PRISMATIC,
            parent=housing,
            child=key,
            origin=Origin(xyz=(x, y, TOP_Z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=2.0, velocity=0.08, lower=0.0, upper=0.0035),
        )

    encoder_mesh = mesh_from_geometry(
        KnobGeometry(
            0.027,
            0.018,
            body_style="cylindrical",
            edge_radius=0.001,
            grip=KnobGrip(style="knurled", count=36, depth=0.00075, helix_angle_deg=18.0),
            indicator=KnobIndicator(style="dot", mode="raised", angle_deg=0.0),
            center=False,
        ),
        "knurled_encoder_knob",
    )
    for index, x in enumerate((-0.075, 0.075)):
        encoder = model.part(f"encoder_{index}")
        encoder.visual(
            Cylinder(radius=0.006, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, 0.003)),
            material=metal_mat,
            name="shaft",
        )
        encoder.visual(
            encoder_mesh,
            origin=Origin(xyz=(0.0, 0.0, 0.006)),
            material=knob_mat,
            name="knob",
        )
        model.articulation(
            f"housing_to_encoder_{index}",
            ArticulationType.CONTINUOUS,
            parent=housing,
            child=encoder,
            origin=Origin(xyz=(x, 0.062, TOP_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.12, velocity=8.0),
        )

    tilt_leg = model.part("tilt_leg")
    tilt_leg.visual(
        Cylinder(radius=0.003, length=0.110),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_mat,
        name="hinge_barrel",
    )
    for index, x in enumerate((-0.045, 0.045)):
        tilt_leg.visual(
            Box((0.012, 0.094, 0.004)),
            origin=Origin(xyz=(x, -0.047, -0.003)),
            material=metal_mat,
            name=f"arm_{index}",
        )
    tilt_leg.visual(
        Box((0.120, 0.014, 0.006)),
        origin=Origin(xyz=(0.0, -0.094, -0.004)),
        material=rubber_mat,
        name="foot",
    )
    model.articulation(
        "housing_to_tilt_leg",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=tilt_leg,
        origin=Origin(xyz=(0.0, HOUSING_DEPTH * 0.5 + 0.006, -0.004)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=1.5, lower=0.0, upper=1.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    key = object_model.get_part("key_1_1")
    key_joint = object_model.get_articulation("housing_to_key_1_1")
    encoder_0 = object_model.get_articulation("housing_to_encoder_0")
    encoder_1 = object_model.get_articulation("housing_to_encoder_1")
    leg = object_model.get_part("tilt_leg")
    leg_joint = object_model.get_articulation("housing_to_tilt_leg")

    key_parts = [part for part in object_model.parts if part.name.startswith("key_")]
    ctx.check("twelve keycaps", len(key_parts) == 12, details=f"found {len(key_parts)}")
    ctx.check(
        "two continuous encoder knobs",
        encoder_0.articulation_type == ArticulationType.CONTINUOUS
        and encoder_1.articulation_type == ArticulationType.CONTINUOUS
        and encoder_0.axis == (0.0, 0.0, 1.0)
        and encoder_1.axis == (0.0, 0.0, 1.0),
        details=f"encoder_0={encoder_0.articulation_type}/{encoder_0.axis}, "
        f"encoder_1={encoder_1.articulation_type}/{encoder_1.axis}",
    )
    ctx.expect_gap(
        key,
        housing,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem="stem",
        name="key stem sits on deck",
    )
    ctx.expect_within(
        key,
        housing,
        axes="xy",
        margin=0.0,
        inner_elem="cap",
        outer_elem="top_deck",
        name="sample key is inside top deck footprint",
    )

    rest_key_position = ctx.part_world_position(key)
    with ctx.pose({key_joint: 0.0035}):
        pressed_key_position = ctx.part_world_position(key)
    ctx.check(
        "key press travels downward",
        rest_key_position is not None
        and pressed_key_position is not None
        and pressed_key_position[2] < rest_key_position[2] - 0.003,
        details=f"rest={rest_key_position}, pressed={pressed_key_position}",
    )

    folded_foot = ctx.part_element_world_aabb(leg, elem="foot")
    with ctx.pose({leg_joint: 1.0}):
        deployed_foot = ctx.part_element_world_aabb(leg, elem="foot")
    ctx.check(
        "tilt leg deploys below housing",
        folded_foot is not None
        and deployed_foot is not None
        and deployed_foot[0][2] < folded_foot[0][2] - 0.040,
        details=f"folded={folded_foot}, deployed={deployed_foot}",
    )

    return ctx.report()


object_model = build_object_model()
