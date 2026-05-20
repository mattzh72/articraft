from __future__ import annotations

from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BlowerWheelGeometry,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    VentGrilleGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)


BASE_RADIUS = 0.16
BASE_FOOT_HEIGHT = 0.012
BASE_PLATTER_HEIGHT = 0.026
BASE_PILLAR_HEIGHT = 0.020
BASE_TOTAL_HEIGHT = BASE_FOOT_HEIGHT + BASE_PLATTER_HEIGHT + BASE_PILLAR_HEIGHT

BODY_WIDTH = 0.13
BODY_DEPTH = 0.17
BODY_HEIGHT = 0.88
BODY_WALL = 0.004
BODY_CORNER_RADIUS = 0.020
BODY_BOTTOM_Z = 0.028

GRILLE_WIDTH = 0.096
GRILLE_HEIGHT = 0.735
GRILLE_FACE_THICKNESS = 0.003

NECK_COLLAR_RADIUS = 0.046
NECK_COLLAR_HEIGHT = 0.028

BLOWER_OUTER_RADIUS = 0.042
BLOWER_INNER_RADIUS = 0.018
BLOWER_WIDTH = 0.720

DIAL_RADIUS = 0.020
DIAL_HEIGHT = 0.014
DIAL_TOP_CAP_HEIGHT = 0.003
DIAL_POINTER_LENGTH = 0.014
DIAL_POINTER_OFFSET = 0.011


def _aabb_center(aabb):
    if aabb is None:
        return None
    (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
    return (
        0.5 * (min_x + max_x),
        0.5 * (min_y + max_y),
        0.5 * (min_z + max_z),
    )


def _build_housing_shell() -> cq.Workplane:
    outer = cq.Workplane("XY").box(BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT)
    outer = outer.edges("|Z").fillet(BODY_CORNER_RADIUS)

    inner = cq.Workplane("XY").box(
        BODY_WIDTH - 2.0 * BODY_WALL,
        BODY_DEPTH - 2.0 * BODY_WALL,
        BODY_HEIGHT - 2.0 * BODY_WALL,
    )
    shell = outer.cut(inner)

    front_opening = cq.Workplane("XY").box(
        GRILLE_WIDTH,
        BODY_WALL + 0.018,
        GRILLE_HEIGHT,
    )
    front_opening = front_opening.translate((0.0, BODY_DEPTH * 0.5 - BODY_WALL * 0.5, 0.0))

    return shell.cut(front_opening)


def _dial_part(model: ArticulatedObject, name: str, knob_material, accent_material):
    dial = model.part(name)
    dial.visual(
        Cylinder(radius=DIAL_RADIUS, length=DIAL_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, DIAL_HEIGHT * 0.5)),
        material=knob_material,
        name="dial_knob",
    )
    dial.visual(
        Cylinder(radius=DIAL_RADIUS * 0.82, length=DIAL_TOP_CAP_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, DIAL_HEIGHT + DIAL_TOP_CAP_HEIGHT * 0.5)),
        material=accent_material,
        name="dial_cap",
    )
    dial.visual(
        Box((0.004, DIAL_POINTER_LENGTH, 0.0025)),
        origin=Origin(
            xyz=(0.0, DIAL_POINTER_OFFSET, DIAL_HEIGHT + DIAL_TOP_CAP_HEIGHT + 0.00125)
        ),
        material=accent_material,
        name="dial_pointer",
    )
    return dial


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tower_fan")

    base_black = model.material("base_black", rgba=(0.11, 0.12, 0.13, 1.0))
    housing_white = model.material("housing_white", rgba=(0.90, 0.91, 0.89, 1.0))
    grille_dark = model.material("grille_dark", rgba=(0.28, 0.30, 0.33, 1.0))
    dial_charcoal = model.material("dial_charcoal", rgba=(0.18, 0.19, 0.20, 1.0))
    accent_silver = model.material("accent_silver", rgba=(0.67, 0.68, 0.70, 1.0))
    wheel_grey = model.material("wheel_grey", rgba=(0.40, 0.42, 0.45, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=BASE_RADIUS, length=BASE_FOOT_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_FOOT_HEIGHT * 0.5)),
        material=base_black,
        name="base_foot",
    )
    base.visual(
        Cylinder(radius=0.145, length=BASE_PLATTER_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_FOOT_HEIGHT + BASE_PLATTER_HEIGHT * 0.5)),
        material=base_black,
        name="base_platter",
    )
    base.visual(
        Cylinder(radius=0.052, length=BASE_PILLAR_HEIGHT),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                BASE_FOOT_HEIGHT + BASE_PLATTER_HEIGHT + BASE_PILLAR_HEIGHT * 0.5,
            )
        ),
        material=base_black,
        name="base_pillar",
    )

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_housing_shell(), "tower_fan_housing_shell"),
        origin=Origin(xyz=(0.0, 0.0, BODY_BOTTOM_Z + BODY_HEIGHT * 0.5)),
        material=housing_white,
        name="housing_shell",
    )
    body.visual(
        mesh_from_geometry(
            VentGrilleGeometry(
                (GRILLE_WIDTH + 0.010, GRILLE_HEIGHT + 0.010),
                frame=0.010,
                face_thickness=GRILLE_FACE_THICKNESS,
                duct_depth=0.010,
                duct_wall=0.0025,
                slat_pitch=0.0135,
                slat_width=0.0060,
                slat_angle_deg=16.0,
                corner_radius=0.006,
            ),
            "tower_fan_front_grille",
        ),
        origin=Origin(
            xyz=(
                0.0,
                BODY_DEPTH * 0.5 - GRILLE_FACE_THICKNESS * 0.5 - 0.0005,
                BODY_BOTTOM_Z + BODY_HEIGHT * 0.5,
            ),
            rpy=(-pi * 0.5, 0.0, 0.0),
        ),
        material=grille_dark,
        name="front_grille",
    )
    body.visual(
        Cylinder(radius=NECK_COLLAR_RADIUS, length=NECK_COLLAR_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, NECK_COLLAR_HEIGHT * 0.5)),
        material=housing_white,
        name="neck_collar",
    )
    body.visual(
        Cylinder(radius=0.016, length=0.016),
        origin=Origin(xyz=(0.0, 0.018, 0.036)),
        material=grille_dark,
        name="bottom_bearing",
    )
    body.visual(
        Cylinder(radius=0.016, length=0.016),
        origin=Origin(xyz=(0.0, 0.018, BODY_BOTTOM_Z + BODY_HEIGHT - 0.004)),
        material=grille_dark,
        name="top_bearing",
    )

    blower = model.part("blower")
    blower.visual(
        mesh_from_geometry(
            BlowerWheelGeometry(
                BLOWER_OUTER_RADIUS,
                BLOWER_INNER_RADIUS,
                BLOWER_WIDTH,
                34,
                blade_thickness=0.0016,
                blade_sweep_deg=30.0,
                backplate=True,
                shroud=True,
            ),
            "tower_fan_blower_wheel",
        ),
        material=wheel_grey,
        name="wheel",
    )
    blower.visual(
        Cylinder(radius=0.022, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.350)),
        material=wheel_grey,
        name="bottom_hub",
    )
    blower.visual(
        Cylinder(radius=0.022, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.350)),
        material=wheel_grey,
        name="top_hub",
    )
    blower.visual(
        Cylinder(radius=0.0055, length=0.0612),
        origin=Origin(xyz=(0.0, 0.0, -0.3846)),
        material=accent_silver,
        name="bottom_shaft",
    )
    blower.visual(
        Cylinder(radius=0.0055, length=0.0828),
        origin=Origin(xyz=(0.0, 0.0, 0.3954)),
        material=accent_silver,
        name="top_shaft",
    )
    blower.visual(
        Box((0.018, 0.006, 0.014)),
        origin=Origin(xyz=(0.035, 0.0, BLOWER_WIDTH * 0.5 - 0.007)),
        material=accent_silver,
        name="wheel_balance",
    )

    dial_0 = _dial_part(model, "dial_0", dial_charcoal, accent_silver)
    dial_1 = _dial_part(model, "dial_1", dial_charcoal, accent_silver)

    body_to_body = model.articulation(
        "base_to_body",
        ArticulationType.REVOLUTE,
        parent=base,
        child=body,
        origin=Origin(xyz=(0.0, 0.0, BASE_TOTAL_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.5, lower=-0.90, upper=0.90),
    )
    body_to_blower = model.articulation(
        "body_to_blower",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=blower,
        origin=Origin(xyz=(0.0, 0.018, BODY_BOTTOM_Z + BODY_HEIGHT * 0.49)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.2, velocity=30.0),
    )
    body_to_dial_0 = model.articulation(
        "body_to_dial_0",
        ArticulationType.REVOLUTE,
        parent=body,
        child=dial_0,
        origin=Origin(xyz=(-0.030, 0.010, BODY_BOTTOM_Z + BODY_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.15, velocity=3.0, lower=-2.4, upper=2.4),
    )
    body_to_dial_1 = model.articulation(
        "body_to_dial_1",
        ArticulationType.REVOLUTE,
        parent=body,
        child=dial_1,
        origin=Origin(xyz=(0.030, 0.010, BODY_BOTTOM_Z + BODY_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.15, velocity=3.0, lower=-2.4, upper=2.4),
    )

    body_to_body.meta["qc_samples"] = [-0.60, 0.0, 0.60]
    body_to_blower.meta["qc_samples"] = [0.0, 1.2, 2.4]
    body_to_dial_0.meta["qc_samples"] = [-1.0, 0.0, 1.0]
    body_to_dial_1.meta["qc_samples"] = [-1.0, 0.0, 1.0]

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    body = object_model.get_part("body")
    blower = object_model.get_part("blower")
    dial_0 = object_model.get_part("dial_0")
    dial_1 = object_model.get_part("dial_1")

    base_to_body = object_model.get_articulation("base_to_body")
    body_to_blower = object_model.get_articulation("body_to_blower")
    body_to_dial_0 = object_model.get_articulation("body_to_dial_0")

    ctx.expect_gap(
        body,
        base,
        axis="z",
        positive_elem="neck_collar",
        negative_elem="base_pillar",
        max_gap=0.001,
        max_penetration=1e-6,
        name="body collar seats cleanly on the pedestal pillar",
    )
    ctx.expect_overlap(
        body,
        base,
        axes="xy",
        elem_a="neck_collar",
        elem_b="base_pillar",
        min_overlap=0.080,
        name="oscillation collar remains centered over the pedestal pillar",
    )
    ctx.expect_origin_distance(
        dial_0,
        dial_1,
        axes="x",
        min_dist=0.050,
        max_dist=0.070,
        name="top dials are spaced side by side on the cap",
    )
    ctx.expect_gap(
        dial_0,
        body,
        axis="z",
        positive_elem="dial_knob",
        negative_elem="housing_shell",
        max_gap=0.001,
        max_penetration=0.0,
        name="first dial sits on the top cap",
    )
    ctx.expect_gap(
        dial_1,
        body,
        axis="z",
        positive_elem="dial_knob",
        negative_elem="housing_shell",
        max_gap=0.001,
        max_penetration=0.0,
        name="second dial sits on the top cap",
    )

    grille_center_rest = _aabb_center(ctx.part_element_world_aabb(body, elem="front_grille"))
    with ctx.pose({base_to_body: 0.60}):
        grille_center_turned = _aabb_center(ctx.part_element_world_aabb(body, elem="front_grille"))
    ctx.check(
        "body oscillation yaws the grille toward negative x for positive rotation",
        grille_center_rest is not None
        and grille_center_turned is not None
        and grille_center_turned[0] < grille_center_rest[0] - 0.035,
        details=f"rest={grille_center_rest}, turned={grille_center_turned}",
    )

    balance_center_rest = _aabb_center(ctx.part_element_world_aabb(blower, elem="wheel_balance"))
    with ctx.pose({body_to_blower: 1.30}):
        balance_center_spun = _aabb_center(ctx.part_element_world_aabb(blower, elem="wheel_balance"))
    ctx.check(
        "blower wheel spins about the vertical center axis",
        balance_center_rest is not None
        and balance_center_spun is not None
        and balance_center_spun[0] < balance_center_rest[0] - 0.020,
        details=f"rest={balance_center_rest}, spun={balance_center_spun}",
    )

    pointer_center_rest = _aabb_center(ctx.part_element_world_aabb(dial_0, elem="dial_pointer"))
    with ctx.pose({body_to_dial_0: 1.10}):
        pointer_center_turned = _aabb_center(ctx.part_element_world_aabb(dial_0, elem="dial_pointer"))
    ctx.check(
        "dial pointer rotates around its shaft",
        pointer_center_rest is not None
        and pointer_center_turned is not None
        and pointer_center_turned[0] < pointer_center_rest[0] - 0.008,
        details=f"rest={pointer_center_rest}, turned={pointer_center_turned}",
    )

    return ctx.report()


object_model = build_object_model()
