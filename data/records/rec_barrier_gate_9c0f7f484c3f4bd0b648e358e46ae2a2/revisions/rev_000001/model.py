from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    Mimic,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


LEAF_WIDTH = 1.87
HINGE_X = 1.90
HINGE_Y = -0.045
LOWER_HINGE_Z = 0.55
UPPER_HINGE_Z = 1.35
PANEL_BOTTOM_Z = 0.24
PANEL_TOP_Z = 1.58


def _bar_between(part, name, p0, p1, thickness, material):
    """Add a rectangular wrought-iron bar between two local XZ points."""
    x0, z0 = p0
    x1, z1 = p1
    dx = x1 - x0
    dz = z1 - z0
    length = math.hypot(dx, dz)
    theta_y = -math.atan2(dz, dx)
    part.visual(
        Box((length, thickness, thickness)),
        origin=Origin(
            xyz=((x0 + x1) * 0.5, 0.0, (z0 + z1) * 0.5),
            rpy=(0.0, theta_y, 0.0),
        ),
        material=material,
        name=name,
    )


def _add_panel_geometry(part, side: int, material, ring_mesh) -> None:
    """Build one rectangular ornamental swing-gate leaf around its hinge axis."""
    bottom = PANEL_BOTTOM_Z - LOWER_HINGE_Z
    top = PANEL_TOP_Z - LOWER_HINGE_Z
    height = top - bottom
    center_z = (top + bottom) * 0.5

    stile_thick = 0.060
    rail_thick = 0.060
    y_thick = 0.055
    hinge_stile_x = side * 0.080
    free_stile_x = side * (LEAF_WIDTH - 0.040)
    rail_start = 0.080
    rail_end = LEAF_WIDTH - 0.040
    rail_center = side * ((rail_start + rail_end) * 0.5)
    rail_length = rail_end - rail_start

    part.visual(
        Box((stile_thick, y_thick, height)),
        origin=Origin(xyz=(hinge_stile_x, 0.0, center_z)),
        material=material,
        name="hinge_stile",
    )
    part.visual(
        Box((stile_thick, y_thick, height)),
        origin=Origin(xyz=(free_stile_x, 0.0, center_z)),
        material=material,
        name="free_stile",
    )
    for name, z in (("bottom_rail", bottom + rail_thick * 0.5), ("top_rail", top - rail_thick * 0.5)):
        part.visual(
            Box((rail_length, y_thick, rail_thick)),
            origin=Origin(xyz=(rail_center, 0.0, z)),
            material=material,
            name=name,
        )
    part.visual(
        Box((rail_length * 0.86, 0.045, 0.045)),
        origin=Origin(xyz=(side * (LEAF_WIDTH * 0.50), 0.0, (bottom + top) * 0.5)),
        material=material,
        name="middle_rail",
    )

    picket_bottom = bottom + 0.060
    picket_top = top - 0.060
    picket_height = picket_top - picket_bottom
    for idx, x_abs in enumerate((0.38, 0.67, 0.96, 1.25, 1.54)):
        x = side * x_abs
        part.visual(
            Cylinder(radius=0.014, length=picket_height),
            origin=Origin(xyz=(x, 0.0, (picket_bottom + picket_top) * 0.5)),
            material=material,
            name=f"picket_{idx}",
        )
        part.visual(
            Sphere(radius=0.032),
            origin=Origin(xyz=(x, 0.0, picket_top + 0.025)),
            material=material,
            name=f"finial_{idx}",
        )

    low = bottom + 0.180
    high = top - 0.180
    _bar_between(
        part,
        "rising_scroll_bar",
        (side * 0.210, low),
        (side * (LEAF_WIDTH - 0.210), high),
        0.028,
        material,
    )
    _bar_between(
        part,
        "falling_scroll_bar",
        (side * 0.210, high),
        (side * (LEAF_WIDTH - 0.210), low),
        0.028,
        material,
    )

    part.visual(
        ring_mesh,
        origin=Origin(
            xyz=(side * (LEAF_WIDTH * 0.50), -0.002, (bottom + top) * 0.5),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=material,
        name="central_ring",
    )

    # Lower leaf of the pintle hinge, bolted to the panel and revolving with it.
    part.visual(
        Cylinder(radius=0.034, length=0.150),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=material,
        name="lower_barrel",
    )
    part.visual(
        Box((0.150, 0.030, 0.055)),
        origin=Origin(xyz=(side * 0.075, -0.0425, 0.0)),
        material=material,
        name="lower_strap",
    )


def _add_upper_leaf(part, side: int, material) -> None:
    """Build the visible upper hinge leaf that mimics the same leaf swing."""
    part.visual(
        Cylinder(radius=0.034, length=0.150),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=material,
        name="upper_barrel",
    )
    # The strap kisses the front face of the panel stile so it reads bolted-on
    # without creating an unintended inter-part volume overlap.
    part.visual(
        Box((0.150, 0.030, 0.055)),
        origin=Origin(xyz=(side * 0.075, -0.0425, 0.0)),
        material=material,
        name="upper_strap",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="double_leaf_decorative_driveway_gate")

    stone = model.material("cast_stone", rgba=(0.58, 0.54, 0.47, 1.0))
    cap_stone = model.material("stone_cap", rgba=(0.66, 0.62, 0.55, 1.0))
    iron = model.material("black_wrought_iron", rgba=(0.015, 0.014, 0.012, 1.0))
    root_metal = model.material("dark_pintle_steel", rgba=(0.025, 0.023, 0.021, 1.0))

    ring_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.210, tube=0.011, radial_segments=18, tubular_segments=48),
        "ornamental_gate_ring",
    )

    pillars = model.part("pillars")
    pillars.visual(
        Box((4.62, 0.46, 0.120)),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=stone,
        name="ground_sill",
    )
    for side_name, x in (("left", -2.12), ("right", 2.12)):
        pillars.visual(
            Box((0.340, 0.420, 1.900)),
            origin=Origin(xyz=(x, 0.0, 0.950)),
            material=stone,
            name=f"{side_name}_pillar",
        )
        pillars.visual(
            Box((0.440, 0.500, 0.110)),
            origin=Origin(xyz=(x, 0.0, 1.955)),
            material=cap_stone,
            name=f"{side_name}_cap",
        )

    for side_name, hinge_x, bracket_x in (
        ("left", -HINGE_X, -1.935),
        ("right", HINGE_X, 1.935),
    ):
        for level_name, z in (("lower", LOWER_HINGE_Z), ("upper", UPPER_HINGE_Z)):
            pillars.visual(
                Cylinder(radius=0.020, length=0.190),
                origin=Origin(xyz=(hinge_x, HINGE_Y, z)),
                material=root_metal,
                name=f"{side_name}_{level_name}_pin",
            )
            pillars.visual(
                Box((0.060, 0.075, 0.040)),
                origin=Origin(xyz=(bracket_x, HINGE_Y, z + 0.105)),
                material=root_metal,
                name=f"{side_name}_{level_name}_bracket",
            )

    left_panel = model.part("left_panel")
    _add_panel_geometry(left_panel, +1, iron, ring_mesh)

    right_panel = model.part("right_panel")
    _add_panel_geometry(right_panel, -1, iron, ring_mesh)

    left_upper_leaf = model.part("left_upper_leaf")
    _add_upper_leaf(left_upper_leaf, +1, iron)

    right_upper_leaf = model.part("right_upper_leaf")
    _add_upper_leaf(right_upper_leaf, -1, iron)

    swing_limits = MotionLimits(effort=120.0, velocity=0.7, lower=0.0, upper=math.radians(92.0))
    model.articulation(
        "left_lower_pintle",
        ArticulationType.REVOLUTE,
        parent=pillars,
        child=left_panel,
        origin=Origin(xyz=(-HINGE_X, HINGE_Y, LOWER_HINGE_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=swing_limits,
    )
    model.articulation(
        "left_upper_pintle",
        ArticulationType.REVOLUTE,
        parent=pillars,
        child=left_upper_leaf,
        origin=Origin(xyz=(-HINGE_X, HINGE_Y, UPPER_HINGE_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=swing_limits,
        mimic=Mimic("left_lower_pintle"),
    )
    model.articulation(
        "right_lower_pintle",
        ArticulationType.REVOLUTE,
        parent=pillars,
        child=right_panel,
        origin=Origin(xyz=(HINGE_X, HINGE_Y, LOWER_HINGE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=swing_limits,
    )
    model.articulation(
        "right_upper_pintle",
        ArticulationType.REVOLUTE,
        parent=pillars,
        child=right_upper_leaf,
        origin=Origin(xyz=(HINGE_X, HINGE_Y, UPPER_HINGE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=swing_limits,
        mimic=Mimic("right_lower_pintle"),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pillars = object_model.get_part("pillars")
    left_panel = object_model.get_part("left_panel")
    right_panel = object_model.get_part("right_panel")
    left_upper_leaf = object_model.get_part("left_upper_leaf")
    right_upper_leaf = object_model.get_part("right_upper_leaf")

    left_lower = object_model.get_articulation("left_lower_pintle")
    left_upper = object_model.get_articulation("left_upper_pintle")
    right_lower = object_model.get_articulation("right_lower_pintle")
    right_upper = object_model.get_articulation("right_upper_pintle")

    for joint in (left_lower, left_upper, right_lower, right_upper):
        ctx.check(
            f"{joint.name} is revolute",
            joint.articulation_type == ArticulationType.REVOLUTE,
            details=f"{joint.name} type={joint.articulation_type}",
        )
        ctx.check(
            f"{joint.name} has vertical pintle axis",
            abs(joint.axis[2]) > 0.99 and abs(joint.axis[0]) < 1e-6 and abs(joint.axis[1]) < 1e-6,
            details=f"{joint.name} axis={joint.axis}",
        )

    # The visible hinge barrels are solid proxy cylinders coaxial with the
    # pillar-mounted pintle pins; their interpenetration represents a captured
    # pin inside a hinge eye, not an accidental collision.
    hinge_pairs = (
        (left_panel, pillars, "lower_barrel", "left_lower_pin"),
        (left_upper_leaf, pillars, "upper_barrel", "left_upper_pin"),
        (pillars, right_panel, "right_lower_pin", "lower_barrel"),
        (pillars, right_upper_leaf, "right_upper_pin", "upper_barrel"),
    )
    for link_a, link_b, elem_a, elem_b in hinge_pairs:
        ctx.allow_overlap(
            link_a,
            link_b,
            elem_a=elem_a,
            elem_b=elem_b,
            reason="Solid proxy hinge eye is intentionally captured around the pillar pintle pin.",
        )
        ctx.expect_overlap(
            link_a,
            link_b,
            axes="xy",
            elem_a=elem_a,
            elem_b=elem_b,
            min_overlap=0.025,
            name=f"{elem_a} is coaxial with {elem_b}",
        )
        ctx.expect_overlap(
            link_a,
            link_b,
            axes="z",
            elem_a=elem_a,
            elem_b=elem_b,
            min_overlap=0.120,
            name=f"{elem_a} is vertically captured by {elem_b}",
        )

    ctx.expect_gap(
        right_panel,
        left_panel,
        axis="x",
        positive_elem="free_stile",
        negative_elem="free_stile",
        min_gap=0.035,
        max_gap=0.080,
        name="closed leaves meet with a narrow center gap",
    )
    ctx.expect_contact(
        left_upper_leaf,
        left_panel,
        elem_a="upper_strap",
        elem_b="hinge_stile",
        contact_tol=0.002,
        name="left upper hinge leaf bears on panel stile",
    )
    ctx.expect_contact(
        right_upper_leaf,
        right_panel,
        elem_a="upper_strap",
        elem_b="hinge_stile",
        contact_tol=0.002,
        name="right upper hinge leaf bears on panel stile",
    )

    rest_left = ctx.part_world_aabb(left_panel)
    rest_right = ctx.part_world_aabb(right_panel)
    with ctx.pose({left_lower: 1.15, right_lower: 1.15}):
        ctx.expect_contact(
            left_upper_leaf,
            left_panel,
            elem_a="upper_strap",
            elem_b="hinge_stile",
            contact_tol=0.003,
            name="left upper hinge leaf tracks open panel",
        )
        ctx.expect_contact(
            right_upper_leaf,
            right_panel,
            elem_a="upper_strap",
            elem_b="hinge_stile",
            contact_tol=0.003,
            name="right upper hinge leaf tracks open panel",
        )
        open_left = ctx.part_world_aabb(left_panel)
        open_right = ctx.part_world_aabb(right_panel)
    ctx.check(
        "both leaves swing outward toward driveway side",
        rest_left is not None
        and rest_right is not None
        and open_left is not None
        and open_right is not None
        and open_left[0][1] < rest_left[0][1] - 0.55
        and open_right[0][1] < rest_right[0][1] - 0.55,
        details=f"rest_left={rest_left}, open_left={open_left}, rest_right={rest_right}, open_right={open_right}",
    )

    return ctx.report()


object_model = build_object_model()
