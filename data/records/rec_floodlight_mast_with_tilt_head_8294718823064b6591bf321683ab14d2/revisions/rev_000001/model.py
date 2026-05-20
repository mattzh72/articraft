from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


CONCRETE = Material("pale cast concrete", rgba=(0.58, 0.56, 0.52, 1.0))
GALVANIZED = Material("dull galvanized steel", rgba=(0.46, 0.49, 0.50, 1.0))
DARK_METAL = Material("black powder coated aluminum", rgba=(0.035, 0.038, 0.04, 1.0))
GLASS = Material("warm translucent glass", rgba=(1.0, 0.86, 0.42, 0.72))
LED = Material("warm led emitters", rgba=(1.0, 0.95, 0.72, 1.0))


BASE_SIZE = 1.20
BASE_HEIGHT = 0.35
ANCHOR_PLATE_SIZE = 0.46
ANCHOR_PLATE_HEIGHT = 0.05
POLE_Z0 = BASE_HEIGHT + 0.035
POLE_HEIGHT = 5.35
POLE_TOP_Z = POLE_Z0 + POLE_HEIGHT
HINGE_Z = POLE_TOP_Z - 0.06
BRACKET_TIP_X = 0.72
YOKE_INNER_HALF_WIDTH = 0.085
YOKE_PLATE_THICKNESS = 0.030


def _octagonal_pole() -> cq.Workplane:
    """Tapered octagonal parking-light pole, authored in world coordinates."""

    return (
        cq.Workplane("XY")
        .polygon(8, 0.24)
        .workplane(offset=POLE_HEIGHT)
        .polygon(8, 0.18)
        .loft(combine=True)
        .translate((0.0, 0.0, POLE_Z0))
    )


def _concrete_base() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(BASE_SIZE, BASE_SIZE, BASE_HEIGHT)
        .edges("|Z")
        .chamfer(0.035)
        .translate((0.0, 0.0, BASE_HEIGHT / 2.0))
    )


def _hex_nut() -> cq.Workplane:
    height = 0.026
    return cq.Workplane("XY").polygon(6, 0.070).extrude(height).translate((0.0, 0.0, -height / 2.0))


def _head_body() -> cq.Workplane:
    """Rounded rectangular flood head housing in the child hinge frame."""

    return (
        cq.Workplane("XY")
        .box(0.18, 0.45, 0.30)
        .edges("|X")
        .fillet(0.018)
        .translate((0.25, 0.0, -0.07))
    )


def _add_cylinder_between(part, *, name: str, p0, p1, radius: float, material: Material) -> None:
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    if abs(dy) > 1e-9:
        raise ValueError("This floodlight helper only handles XZ-plane cylinders.")
    length = math.sqrt(dx * dx + dz * dz)
    theta_y = math.atan2(dx, dz)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((p0[0] + p1[0]) / 2.0, (p0[1] + p1[1]) / 2.0, (p0[2] + p1[2]) / 2.0),
            rpy=(0.0, theta_y, 0.0),
        ),
        material=material,
        name=name,
    )


def _add_fixed_bracket(support, *, index: int, side: float) -> None:
    arm_length = BRACKET_TIP_X - 0.08
    support.visual(
        Box((arm_length, 0.18, 0.08)),
        origin=Origin(xyz=(side * arm_length / 2.0, 0.0, HINGE_Z)),
        material=GALVANIZED,
        name=f"bracket_{index}_arm",
    )
    for y_sign, y_name in ((1.0, "pos_y"), (-1.0, "neg_y")):
        support.visual(
            Box((0.18, YOKE_PLATE_THICKNESS, 0.22)),
            origin=Origin(
                xyz=(
                    side * BRACKET_TIP_X,
                    y_sign * (YOKE_INNER_HALF_WIDTH + YOKE_PLATE_THICKNESS / 2.0),
                    HINGE_Z,
                )
            ),
            material=GALVANIZED,
            name=f"bracket_{index}_yoke_plate_{y_name}",
        )
    support.visual(
        Box((0.12, 0.23, 0.040)),
        origin=Origin(xyz=(side * (BRACKET_TIP_X - 0.015), 0.0, HINGE_Z + 0.095)),
        material=GALVANIZED,
        name=f"bracket_{index}_yoke_bridge",
    )
    _add_cylinder_between(
        support,
        name=f"bracket_{index}_diagonal_strut",
        p0=(side * 0.075, 0.0, HINGE_Z - 0.46),
        p1=(side * (BRACKET_TIP_X - 0.17), 0.0, HINGE_Z - 0.020),
        radius=0.024,
        material=GALVANIZED,
    )


def _add_flood_head(head, *, index: int) -> None:
    head.visual(
        Cylinder(radius=0.058, length=YOKE_INNER_HALF_WIDTH * 2.0),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=GALVANIZED,
        name="trunnion",
    )
    head.visual(
        Box((0.18, 0.11, 0.12)),
        origin=Origin(xyz=(0.105, 0.0, -0.020)),
        material=DARK_METAL,
        name="hinge_lug",
    )
    head.visual(mesh_from_cadquery(_head_body(), f"head_{index}_body"), material=DARK_METAL, name="housing")
    head.visual(
        Box((0.016, 0.36, 0.22)),
        origin=Origin(xyz=(0.342, 0.0, -0.070)),
        material=GLASS,
        name="lens",
    )
    head.visual(
        Box((0.030, 0.45, 0.034)),
        origin=Origin(xyz=(0.358, 0.0, 0.066)),
        material=DARK_METAL,
        name="bezel_top",
    )
    head.visual(
        Box((0.030, 0.45, 0.034)),
        origin=Origin(xyz=(0.358, 0.0, -0.206)),
        material=DARK_METAL,
        name="bezel_bottom",
    )
    for y_sign, name in ((1.0, "bezel_side_0"), (-1.0, "bezel_side_1")):
        head.visual(
            Box((0.030, 0.034, 0.30)),
            origin=Origin(xyz=(0.358, y_sign * 0.208, -0.070)),
            material=DARK_METAL,
            name=name,
        )
    for row, z in enumerate((-0.125, -0.070, -0.015)):
        for col, y in enumerate((-0.105, 0.0, 0.105)):
            head.visual(
                Cylinder(radius=0.017, length=0.012),
                origin=Origin(xyz=(0.350, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=LED,
                name=f"led_{row}_{col}",
            )
    for fin, y in enumerate((-0.160, -0.095, -0.030, 0.035, 0.100, 0.165)):
        head.visual(
            Box((0.060, 0.018, 0.255)),
            origin=Origin(xyz=(0.132, y, -0.070)),
            material=DARK_METAL,
            name=f"cooling_fin_{fin}",
        )
    head.visual(
        Box((0.22, 0.49, 0.030)),
        origin=Origin(xyz=(0.272, 0.0, 0.095)),
        material=DARK_METAL,
        name="sun_hood",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="parking_lot_floodlight")

    support = model.part("support")
    support.visual(mesh_from_cadquery(_concrete_base(), "concrete_base"), material=CONCRETE, name="concrete_base")
    support.visual(
        Box((ANCHOR_PLATE_SIZE, ANCHOR_PLATE_SIZE, ANCHOR_PLATE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT + ANCHOR_PLATE_HEIGHT / 2.0)),
        material=GALVANIZED,
        name="anchor_plate",
    )
    support.visual(mesh_from_cadquery(_octagonal_pole(), "octagonal_pole"), material=GALVANIZED, name="octagonal_pole")
    support.visual(
        Cylinder(radius=0.145, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, POLE_Z0 + 0.03)),
        material=GALVANIZED,
        name="base_collar",
    )
    support.visual(
        Cylinder(radius=0.115, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, HINGE_Z)),
        material=GALVANIZED,
        name="top_collar",
    )

    nut_mesh = mesh_from_cadquery(_hex_nut(), "anchor_nut")
    for bolt_index, (x, y) in enumerate(
        (
            (-0.165, -0.165),
            (-0.165, 0.165),
            (0.165, -0.165),
            (0.165, 0.165),
        )
    ):
        support.visual(
            Cylinder(radius=0.013, length=0.095),
            origin=Origin(xyz=(x, y, BASE_HEIGHT + ANCHOR_PLATE_HEIGHT + 0.032)),
            material=GALVANIZED,
            name=f"anchor_bolt_{bolt_index}",
        )
        support.visual(
            nut_mesh,
            origin=Origin(xyz=(x, y, BASE_HEIGHT + ANCHOR_PLATE_HEIGHT + 0.013)),
            material=GALVANIZED,
            name=f"anchor_nut_{bolt_index}",
        )

    _add_fixed_bracket(support, index=0, side=1.0)
    _add_fixed_bracket(support, index=1, side=-1.0)

    tilt_limits = MotionLimits(effort=30.0, velocity=1.0, lower=-0.35, upper=0.80)
    for index, side in enumerate((1.0, -1.0)):
        head = model.part(f"head_{index}")
        _add_flood_head(head, index=index)
        model.articulation(
            f"support_to_head_{index}",
            ArticulationType.REVOLUTE,
            parent=support,
            child=head,
            origin=Origin(
                xyz=(side * BRACKET_TIP_X, 0.0, HINGE_Z),
                rpy=(0.0, 0.0, 0.0 if side > 0 else math.pi),
            ),
            axis=(0.0, 1.0, 0.0),
            motion_limits=tilt_limits,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    support = object_model.get_part("support")
    heads = [object_model.get_part("head_0"), object_model.get_part("head_1")]
    joints = [object_model.get_articulation("support_to_head_0"), object_model.get_articulation("support_to_head_1")]

    ctx.check("one concrete-pole support and two tilting heads", support is not None and all(heads) and all(joints))
    for index, head in enumerate(heads):
        ctx.expect_contact(
            head,
            support,
            elem_a="trunnion",
            elem_b=f"bracket_{index}_yoke_plate_pos_y",
            contact_tol=0.0015,
            name=f"head_{index} trunnion seats in clevis",
        )

        rest_aabb = ctx.part_element_world_aabb(head, elem="lens")
        with ctx.pose({joints[index]: 0.60}):
            tilted_aabb = ctx.part_element_world_aabb(head, elem="lens")

        if rest_aabb is None or tilted_aabb is None:
            ctx.fail(f"head_{index} lens pose measurable", "lens AABB was unavailable")
        else:
            rest_z = (rest_aabb[0][2] + rest_aabb[1][2]) * 0.5
            tilted_z = (tilted_aabb[0][2] + tilted_aabb[1][2]) * 0.5
            ctx.check(
                f"head_{index} positive tilt aims downward",
                tilted_z < rest_z - 0.045,
                details=f"rest lens z={rest_z:.3f}, tilted lens z={tilted_z:.3f}",
            )

    return ctx.report()


object_model = build_object_model()
