from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _x_cylinder(part, radius: float, length: float, xyz, material, name: str) -> None:
    """Cylinder whose axis is the binocular fore-aft X axis."""
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _y_cylinder(part, radius: float, length: float, xyz, material, name: str) -> None:
    """Cylinder whose axis is the crosswise Y axis."""
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _add_optical_side(part, *, y: float, sign: float, rubber, black, glass, name_prefix: str) -> None:
    """One straight roof-prism binocular barrel with objective, eyecup, and armor."""
    # Main straight tube: roof-prism binoculars keep the optical axes in line.
    _x_cylinder(part, 0.020, 0.122, (0.000, y, 0.000), rubber, "main_tube")
    _x_cylinder(part, 0.0215, 0.012, (0.028, y, 0.000), black, "front_armor_band")
    _x_cylinder(part, 0.0215, 0.012, (-0.033, y, 0.000), black, "rear_armor_band")

    # 42 mm-class objective end and smaller rear eyecup.
    _x_cylinder(part, 0.026, 0.026, (0.064, y, 0.000), black, "objective_housing")
    _x_cylinder(part, 0.018, 0.028, (-0.071, y, 0.000), black, "eyecup")
    _x_cylinder(part, 0.018, 0.003, (0.076, y, 0.000), glass, "objective_glass")
    _x_cylinder(part, 0.013, 0.003, (-0.084, y, 0.000), glass, "ocular_glass")

    # Angular prism shoulder and raised roof line, typical of a roof-prism body.
    part.visual(
        Box((0.072, 0.044, 0.030)),
        origin=Origin(xyz=(-0.010, y, 0.020)),
        material=rubber,
        name="prism_housing",
    )
    part.visual(
        Box((0.058, 0.018, 0.011)),
        origin=Origin(xyz=(-0.010, y, 0.040)),
        material=black,
        name="roof_ridge",
    )

    # Raised outside palm pad; small diagonal ribs make the hunting rubber armor read.
    part.visual(
        Box((0.058, 0.008, 0.026)),
        origin=Origin(xyz=(0.006, y + sign * 0.022, 0.006)),
        material=black,
        name="side_grip_pad",
    )
    for i, x in enumerate((-0.018, -0.002, 0.014, 0.030)):
        part.visual(
            Box((0.004, 0.010, 0.028)),
            origin=Origin(
                xyz=(x, y + sign * 0.026, 0.008),
                rpy=(0.0, 0.0, sign * 0.45),
            ),
            material=rubber,
            name=f"{name_prefix}_grip_rib_{i}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="8x42_roof_prism_hunting_binocular")

    rubber = Material("mat_olive_rubber", rgba=(0.05, 0.075, 0.055, 1.0))
    black = Material("mat_black_rubber", rgba=(0.006, 0.007, 0.006, 1.0))
    glass = Material("mat_green_coated_glass", rgba=(0.12, 0.36, 0.30, 0.72))
    metal = Material("mat_dark_metal", rgba=(0.12, 0.12, 0.11, 1.0))
    marking = Material("mat_tan_marking", rgba=(0.72, 0.62, 0.40, 1.0))

    left_barrel = model.part("left_barrel")
    _add_optical_side(
        left_barrel,
        y=0.040,
        sign=1.0,
        rubber=rubber,
        black=black,
        glass=glass,
        name_prefix="left",
    )

    # The fixed half carries the continuous central hinge pin and a one-sided
    # support yoke for the focus axle.
    _x_cylinder(left_barrel, 0.006, 0.138, (0.000, 0.000, 0.000), metal, "hinge_pin")
    left_barrel.visual(
        Box((0.030, 0.034, 0.012)),
        origin=Origin(xyz=(0.000, 0.022, 0.006)),
        material=rubber,
        name="center_bridge",
    )
    left_barrel.visual(
        Box((0.022, 0.016, 0.028)),
        origin=Origin(xyz=(-0.028, 0.026, 0.043)),
        material=rubber,
        name="focus_yoke",
    )
    _y_cylinder(left_barrel, 0.004, 0.052, (-0.028, 0.000, 0.055), metal, "focus_axle")
    left_barrel.visual(
        Box((0.022, 0.002, 0.010)),
        origin=Origin(xyz=(-0.018, 0.063, 0.023)),
        material=marking,
        name="8x42_badge",
    )

    right_barrel = model.part("right_barrel")
    _add_optical_side(
        right_barrel,
        y=-0.040,
        sign=-1.0,
        rubber=rubber,
        black=black,
        glass=glass,
        name_prefix="right",
    )

    # Movable hinge knuckles wrap the fixed pin. They are solid proxy sleeves so
    # their local pin capture is allowed and proven in run_tests().
    for x, lug_name, bridge_name in (
        (0.045, "front_hinge_lug", "front_bridge"),
        (-0.045, "rear_hinge_lug", "rear_bridge"),
    ):
        _x_cylinder(right_barrel, 0.010, 0.026, (x, 0.000, 0.000), metal, lug_name)
        right_barrel.visual(
            Box((0.026, 0.032, 0.012)),
            origin=Origin(xyz=(x, -0.024, 0.006)),
            material=rubber,
            name=bridge_name,
        )

    focus_knob = model.part("focus_knob")
    knob_geometry = KnobGeometry(
        0.036,
        0.030,
        body_style="cylindrical",
        edge_radius=0.0015,
        grip=KnobGrip(style="ribbed", count=32, depth=0.0014, width=0.002),
    )
    focus_knob.visual(
        mesh_from_geometry(knob_geometry, "ribbed_focus_knob"),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="knob_shell",
    )

    model.articulation(
        "barrel_pivot",
        ArticulationType.REVOLUTE,
        parent=left_barrel,
        child=right_barrel,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.0, lower=-0.18, upper=0.18),
    )
    model.articulation(
        "focus_rotation",
        ArticulationType.REVOLUTE,
        parent=left_barrel,
        child=focus_knob,
        origin=Origin(xyz=(-0.028, 0.000, 0.055)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.35, velocity=4.0, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    left_barrel = object_model.get_part("left_barrel")
    right_barrel = object_model.get_part("right_barrel")
    focus_knob = object_model.get_part("focus_knob")
    pivot = object_model.get_articulation("barrel_pivot")
    focus = object_model.get_articulation("focus_rotation")

    ctx.check(
        "roof-prism binocular has two barrel assemblies and center focus",
        left_barrel is not None and right_barrel is not None and focus_knob is not None,
    )
    ctx.check(
        "primary mechanisms are revolute",
        pivot.articulation_type == ArticulationType.REVOLUTE
        and focus.articulation_type == ArticulationType.REVOLUTE,
    )

    # Captured hinge and focus-shaft proxy overlaps are local and mechanical:
    # solid sleeve/shaft stand-ins keep the visual mechanism legible.
    for lug_name in ("front_hinge_lug", "rear_hinge_lug"):
        ctx.allow_overlap(
            left_barrel,
            right_barrel,
            elem_a="hinge_pin",
            elem_b=lug_name,
            reason="The movable hinge lug is intentionally shown captured around the central pivot pin.",
        )
        ctx.expect_overlap(
            left_barrel,
            right_barrel,
            axes="x",
            elem_a="hinge_pin",
            elem_b=lug_name,
            min_overlap=0.020,
            name=f"{lug_name} has retained length on hinge pin",
        )
        ctx.expect_overlap(
            left_barrel,
            right_barrel,
            axes="yz",
            elem_a="hinge_pin",
            elem_b=lug_name,
            min_overlap=0.010,
            name=f"{lug_name} surrounds hinge pin in cross-section",
        )

    ctx.allow_overlap(
        left_barrel,
        focus_knob,
        elem_a="focus_axle",
        elem_b="knob_shell",
        reason="The focus knob is intentionally captured on a shaft proxy through its rubber wheel.",
    )
    ctx.expect_overlap(
        left_barrel,
        focus_knob,
        axes="y",
        elem_a="focus_axle",
        elem_b="knob_shell",
        min_overlap=0.025,
        name="focus knob remains on crosswise axle",
    )
    ctx.expect_overlap(
        left_barrel,
        focus_knob,
        axes="xz",
        elem_a="focus_axle",
        elem_b="knob_shell",
        min_overlap=0.006,
        name="focus axle passes through knob core",
    )

    # The binocular hinge should move the opposite barrel around the central pin.
    rest_aabb = ctx.part_element_world_aabb(right_barrel, elem="main_tube")
    with ctx.pose({pivot: 0.16}):
        folded_aabb = ctx.part_element_world_aabb(right_barrel, elem="main_tube")
        ctx.expect_overlap(
            left_barrel,
            right_barrel,
            axes="x",
            elem_a="hinge_pin",
            elem_b="front_hinge_lug",
            min_overlap=0.018,
            name="folded barrel stays retained on front hinge lug",
        )
    if rest_aabb is not None and folded_aabb is not None:
        rest_center_z = (rest_aabb[0][2] + rest_aabb[1][2]) * 0.5
        folded_center_z = (folded_aabb[0][2] + folded_aabb[1][2]) * 0.5
        ctx.check(
            "barrel pivot visibly folds one barrel",
            abs(folded_center_z - rest_center_z) > 0.004,
            details=f"rest_z={rest_center_z:.4f}, folded_z={folded_center_z:.4f}",
        )

    return ctx.report()


object_model = build_object_model()
