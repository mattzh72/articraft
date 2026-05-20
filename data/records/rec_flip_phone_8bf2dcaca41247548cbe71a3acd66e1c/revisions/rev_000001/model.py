from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CapsuleGeometry,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
)
import cadquery as cq


LOWER_LENGTH = 0.110
UPPER_LENGTH = 0.104
BODY_WIDTH = 0.044
LOWER_THICKNESS = 0.010
UPPER_THICKNESS = 0.009
HINGE_Z = 0.008
OPEN_ANGLE = 1.10
ANTENNA_TRAVEL = 0.045


def rounded_slab_mesh(length: float, width: float, thickness: float, radius: float, name: str):
    profile = rounded_rect_profile(length, width, radius, corner_segments=8)
    return mesh_from_geometry(
        ExtrudeGeometry.centered(profile, thickness, cap=True, closed=True),
        name,
    )


def add_key(
    model: ArticulatedObject,
    lower_body,
    *,
    name: str,
    xyz: tuple[float, float, float],
    size: tuple[float, float, float],
    material,
    round_key: bool = False,
):
    key = model.part(name)
    if round_key:
        key.visual(
            Cylinder(radius=size[0] * 0.5, length=size[2]),
            origin=Origin(xyz=(0.0, 0.0, size[2] * 0.5)),
            material=material,
            name="keycap",
        )
    else:
        key.visual(
            Box(size),
            origin=Origin(xyz=(0.0, 0.0, size[2] * 0.5)),
            material=material,
            name="keycap",
        )
    model.articulation(
        f"lower_to_{name}",
        ArticulationType.PRISMATIC,
        parent=lower_body,
        child=key,
        origin=Origin(xyz=xyz),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=0.04, lower=0.0, upper=0.0015),
    )
    return key


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slim_clamshell_flip_phone")

    satin_black = model.material("satin_black", rgba=(0.015, 0.015, 0.018, 1.0))
    graphite = model.material("graphite", rgba=(0.12, 0.13, 0.14, 1.0))
    hinge_metal = model.material("dark_hinge_metal", rgba=(0.28, 0.29, 0.30, 1.0))
    soft_rubber = model.material("soft_rubber_keys", rgba=(0.035, 0.037, 0.040, 1.0))
    screen_glass = model.material("blue_black_glass", rgba=(0.015, 0.055, 0.085, 1.0))
    pale_mark = model.material("pale_key_marks", rgba=(0.72, 0.76, 0.78, 1.0))
    antenna_material = model.material("matte_black_antenna", rgba=(0.02, 0.02, 0.018, 1.0))

    lower_body = model.part("lower_body")
    lower_body.visual(
        rounded_slab_mesh(LOWER_LENGTH, BODY_WIDTH, LOWER_THICKNESS, 0.006, "lower_body_shell"),
        origin=Origin(xyz=(-LOWER_LENGTH * 0.5 - 0.003, 0.0, 0.0)),
        material=satin_black,
        name="lower_shell",
    )
    lower_body.visual(
        Box((0.078, 0.034, 0.0012)),
        origin=Origin(xyz=(-0.062, 0.0, LOWER_THICKNESS * 0.5 + 0.0006)),
        material=graphite,
        name="keypad_deck",
    )
    lower_body.visual(
        Cylinder(radius=0.0052, length=0.029),
        origin=Origin(xyz=(0.0, 0.0, HINGE_Z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=hinge_metal,
        name="center_barrel",
    )

    upper_body = model.part("upper_body")
    upper_body.visual(
        rounded_slab_mesh(UPPER_LENGTH, BODY_WIDTH, UPPER_THICKNESS, 0.006, "upper_body_shell"),
        origin=Origin(xyz=(UPPER_LENGTH * 0.5 + 0.007, 0.0, 0.0)),
        material=satin_black,
        name="upper_shell",
    )
    upper_body.visual(
        Box((0.066, 0.032, 0.0010)),
        origin=Origin(xyz=(0.060, 0.0, UPPER_THICKNESS * 0.5 + 0.0005)),
        material=screen_glass,
        name="display_glass",
    )
    upper_body.visual(
        Box((0.021, 0.0030, 0.0011)),
        origin=Origin(xyz=(0.096, 0.0, UPPER_THICKNESS * 0.5 + 0.0007)),
        material=graphite,
        name="earpiece_slot",
    )
    for y in (-0.017, 0.017):
        upper_body.visual(
            Cylinder(radius=0.0040, length=0.0060),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=hinge_metal,
            name=f"hinge_lug_{'minus' if y < 0 else 'plus'}",
        )
        upper_body.visual(
            Box((0.018, 0.0060, 0.0050)),
            origin=Origin(xyz=(0.007, y, 0.0)),
            material=satin_black,
            name=f"hinge_ear_{'minus' if y < 0 else 'plus'}",
        )
    upper_body.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.0032, tube=0.00075, radial_segments=16, tubular_segments=32).rotate_y(math.pi / 2.0),
            "antenna_collar",
        ),
        origin=Origin(xyz=(UPPER_LENGTH + 0.007, 0.014, UPPER_THICKNESS * 0.5 - 0.0005)),
        material=hinge_metal,
        name="antenna_collar",
    )

    hinge = model.articulation(
        "lower_to_upper",
        ArticulationType.REVOLUTE,
        parent=lower_body,
        child=upper_body,
        origin=Origin(xyz=(0.0, 0.0, HINGE_Z), rpy=(0.0, -OPEN_ANGLE, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=-0.35, upper=1.85),
    )

    antenna_mast = model.part("antenna_mast")
    antenna_mast.visual(
        Cylinder(radius=0.0030, length=0.000936),
        origin=Origin(xyz=(0.001182, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=antenna_material,
        name="base_flange",
    )
    antenna_mast.visual(
        Cylinder(radius=0.00125, length=0.052),
        origin=Origin(xyz=(0.02765, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=antenna_material,
        name="mast_rod",
    )
    antenna_mast.visual(
        Sphere(radius=0.0022),
        origin=Origin(xyz=(0.055, 0.0, 0.0)),
        material=antenna_material,
        name="rounded_tip",
    )
    model.articulation(
        "upper_to_antenna",
        ArticulationType.PRISMATIC,
        parent=upper_body,
        child=antenna_mast,
        origin=Origin(xyz=(UPPER_LENGTH + 0.007, 0.014, UPPER_THICKNESS * 0.5 - 0.0005)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=0.20, lower=0.0, upper=ANTENNA_TRAVEL),
    )

    key_z = LOWER_THICKNESS * 0.5 + 0.00005
    add_key(
        model,
        lower_body,
        name="nav_key",
        xyz=(-0.024, 0.0, key_z),
        size=(0.018, 0.018, 0.0024),
        material=soft_rubber,
        round_key=True,
    )
    for idx, y in enumerate((-0.013, 0.013)):
        add_key(
            model,
            lower_body,
            name=f"soft_key_{idx}",
            xyz=(-0.034, y, key_z),
            size=(0.014, 0.007, 0.0022),
            material=graphite,
        )

    row_xs = (-0.049, -0.064, -0.079, -0.094)
    col_ys = (-0.013, 0.0, 0.013)
    for r, x in enumerate(row_xs):
        for c, y in enumerate(col_ys):
            key = add_key(
                model,
                lower_body,
                name=f"key_{r}_{c}",
                xyz=(x, y, key_z),
                size=(0.0105, 0.0075, 0.0022),
                material=soft_rubber,
            )
            key.visual(
                Box((0.0040, 0.0010, 0.00025)),
                origin=Origin(xyz=(0.0, 0.0, 0.002325)),
                material=pale_mark,
                name="key_mark",
            )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower = object_model.get_part("lower_body")
    upper = object_model.get_part("upper_body")
    mast = object_model.get_part("antenna_mast")
    hinge = object_model.get_articulation("lower_to_upper")
    antenna_slide = object_model.get_articulation("upper_to_antenna")

    ctx.check(
        "single center barrel hinge is revolute",
        hinge.articulation_type == ArticulationType.REVOLUTE,
        details=f"hinge type={hinge.articulation_type}",
    )
    ctx.check(
        "antenna mast is prismatic",
        antenna_slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"antenna type={antenna_slide.articulation_type}",
    )
    ctx.expect_contact(
        lower,
        object_model.get_part("nav_key"),
        elem_a="keypad_deck",
        elem_b="keycap",
        contact_tol=0.0002,
        name="navigation key sits on keypad deck",
    )

    rest_upper_aabb = ctx.part_element_world_aabb(upper, elem="upper_shell")
    with ctx.pose({hinge: 0.55}):
        lifted_upper_aabb = ctx.part_element_world_aabb(upper, elem="upper_shell")
    ctx.check(
        "display body rotates upward about the barrel",
        rest_upper_aabb is not None
        and lifted_upper_aabb is not None
        and lifted_upper_aabb[1][2] > rest_upper_aabb[1][2] + 0.008,
        details=f"rest={rest_upper_aabb}, lifted={lifted_upper_aabb}",
    )

    rest_tip = ctx.part_element_world_aabb(mast, elem="rounded_tip")
    with ctx.pose({antenna_slide: ANTENNA_TRAVEL}):
        extended_tip = ctx.part_element_world_aabb(mast, elem="rounded_tip")
    ctx.check(
        "antenna tip pushes out from the upper top edge",
        rest_tip is not None
        and extended_tip is not None
        and extended_tip[1][0] > rest_tip[1][0] + 0.015
        and extended_tip[1][2] > rest_tip[1][2] + 0.015,
        details=f"rest={rest_tip}, extended={extended_tip}",
    )

    return ctx.report()


object_model = build_object_model()
