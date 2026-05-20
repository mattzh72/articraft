from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _add_square_tube(
    part,
    *,
    prefix: str,
    outer_size: float,
    wall: float,
    length: float,
    base_z: float,
    material,
) -> None:
    center_z = base_z + 0.5 * length
    side_x = 0.5 * outer_size - 0.5 * wall
    side_y = 0.5 * outer_size - 0.5 * wall
    part.visual(
        Box((wall, outer_size, length)),
        origin=Origin(xyz=(-side_x, 0.0, center_z)),
        material=material,
        name=f"{prefix}_left_wall",
    )
    part.visual(
        Box((wall, outer_size, length)),
        origin=Origin(xyz=(side_x, 0.0, center_z)),
        material=material,
        name=f"{prefix}_right_wall",
    )
    part.visual(
        Box((outer_size, wall, length)),
        origin=Origin(xyz=(0.0, side_y, center_z)),
        material=material,
        name=f"{prefix}_front_wall",
    )
    part.visual(
        Box((outer_size, wall, length)),
        origin=Origin(xyz=(0.0, -side_y, center_z)),
        material=material,
        name=f"{prefix}_back_wall",
    )


def _build_leg(part, *, metal, rubber) -> None:
    part.visual(
        Box((0.070, 0.060, 0.050)),
        origin=Origin(xyz=(0.075, 0.0, -0.010)),
        material=metal,
        name="hinge_block",
    )
    part.visual(
        Box((0.500, 0.050, 0.035)),
        origin=Origin(xyz=(0.250, 0.0, -0.040)),
        material=metal,
        name="main_beam",
    )
    _add_member(
        part,
        (0.050, 0.0, 0.010),
        (0.240, 0.0, -0.060),
        0.012,
        metal,
        name="brace_tube",
    )
    part.visual(
        Box((0.055, 0.055, 0.055)),
        origin=Origin(xyz=(0.450, 0.0, -0.063)),
        material=metal,
        name="foot_neck",
    )
    part.visual(
        Box((0.090, 0.100, 0.025)),
        origin=Origin(xyz=(0.490, 0.0, -0.082)),
        material=metal,
        name="foot_pad",
    )
    part.visual(
        Sphere(radius=0.018),
        origin=Origin(xyz=(0.505, 0.0, -0.087)),
        material=rubber,
        name="rubber_foot",
    )
    part.inertial = Inertial.from_geometry(
        Box((0.560, 0.120, 0.110)),
        mass=1.2,
        origin=Origin(xyz=(0.280, 0.0, -0.045)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="construction_tower_light")

    safety_yellow = model.material("safety_yellow", rgba=(0.90, 0.76, 0.12, 1.0))
    charcoal = model.material("charcoal", rgba=(0.19, 0.20, 0.22, 1.0))
    steel = model.material("steel", rgba=(0.55, 0.57, 0.60, 1.0))
    lens = model.material("lens", rgba=(0.86, 0.93, 0.97, 0.55))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    base = model.part("base_collar")
    base.visual(
        Box((0.240, 0.240, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=charcoal,
        name="base_spider",
    )
    base.visual(
        Cylinder(radius=0.100, length=0.140),
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        material=charcoal,
        name="center_collar",
    )
    base.visual(
        Cylinder(radius=0.080, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.190)),
        material=steel,
        name="mast_receiver_cap",
    )
    for index, angle in enumerate((0.0, 0.5 * math.pi, math.pi, -0.5 * math.pi)):
        c = math.cos(angle)
        s = math.sin(angle)
        base.visual(
            Box((0.060, 0.070, 0.050)),
            origin=Origin(
                xyz=(0.115 * c, 0.115 * s, 0.090),
                rpy=(0.0, 0.0, angle),
            ),
            material=steel,
            name=f"hinge_pad_{index}",
        )
    base.inertial = Inertial.from_geometry(
        Box((0.320, 0.320, 0.230)),
        mass=10.0,
        origin=Origin(xyz=(0.0, 0.0, 0.115)),
    )

    outer_mast = model.part("outer_mast")
    outer_mast.visual(
        Box((0.120, 0.120, 0.080)),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=safety_yellow,
        name="base_socket",
    )
    _add_square_tube(
        outer_mast,
        prefix="outer",
        outer_size=0.100,
        wall=0.008,
        length=0.760,
        base_z=0.080,
        material=safety_yellow,
    )
    _add_square_tube(
        outer_mast,
        prefix="top_guide",
        outer_size=0.120,
        wall=0.024,
        length=0.040,
        base_z=0.840,
        material=steel,
    )
    outer_mast.inertial = Inertial.from_geometry(
        Box((0.140, 0.140, 0.900)),
        mass=5.5,
        origin=Origin(xyz=(0.0, 0.0, 0.450)),
    )

    inner_mast = model.part("inner_mast")
    _add_square_tube(
        inner_mast,
        prefix="inner",
        outer_size=0.072,
        wall=0.006,
        length=1.260,
        base_z=-0.560,
        material=safety_yellow,
    )
    inner_mast.visual(
        Box((0.090, 0.090, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.730)),
        material=steel,
        name="mast_head_block",
    )
    inner_mast.visual(
        Box((0.060, 0.050, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.765)),
        material=steel,
        name="yoke_stem",
    )
    inner_mast.visual(
        Box((0.700, 0.050, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.790)),
        material=steel,
        name="yoke_crossbar",
    )
    inner_mast.visual(
        Box((0.030, 0.070, 0.180)),
        origin=Origin(xyz=(-0.345, 0.0, 0.730)),
        material=steel,
        name="left_yoke_cheek",
    )
    inner_mast.visual(
        Box((0.030, 0.070, 0.180)),
        origin=Origin(xyz=(0.345, 0.0, 0.730)),
        material=steel,
        name="right_yoke_cheek",
    )
    inner_mast.inertial = Inertial.from_geometry(
        Box((0.720, 0.140, 1.440)),
        mass=6.0,
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
    )

    lamp_panel = model.part("lamp_panel")
    lamp_panel.visual(
        Cylinder(radius=0.018, length=0.042),
        origin=Origin(xyz=(-0.309, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="left_trunnion",
    )
    lamp_panel.visual(
        Cylinder(radius=0.018, length=0.042),
        origin=Origin(xyz=(0.309, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="right_trunnion",
    )
    lamp_panel.visual(
        Box((0.620, 0.060, 0.260)),
        origin=Origin(xyz=(0.0, 0.100, -0.040)),
        material=charcoal,
        name="back_shell",
    )
    lamp_panel.visual(
        Box((0.580, 0.010, 0.220)),
        origin=Origin(xyz=(0.0, 0.135, -0.040)),
        material=lens,
        name="lens",
    )
    lamp_panel.visual(
        Box((0.030, 0.100, 0.064)),
        origin=Origin(xyz=(-0.282, 0.070, -0.024)),
        material=charcoal,
        name="left_side_bracket",
    )
    lamp_panel.visual(
        Box((0.030, 0.050, 0.035)),
        origin=Origin(xyz=(-0.288, 0.025, -0.028)),
        material=charcoal,
        name="left_lower_gusset",
    )
    lamp_panel.visual(
        Box((0.030, 0.100, 0.064)),
        origin=Origin(xyz=(0.282, 0.070, -0.024)),
        material=charcoal,
        name="right_side_bracket",
    )
    lamp_panel.visual(
        Box((0.030, 0.050, 0.035)),
        origin=Origin(xyz=(0.288, 0.025, -0.028)),
        material=charcoal,
        name="right_lower_gusset",
    )
    lamp_panel.visual(
        Box((0.660, 0.050, 0.040)),
        origin=Origin(xyz=(0.0, 0.098, 0.110)),
        material=charcoal,
        name="top_hood",
    )
    lamp_panel.visual(
        Box((0.620, 0.055, 0.030)),
        origin=Origin(xyz=(0.0, 0.098, -0.160)),
        material=charcoal,
        name="lower_bezel",
    )
    for index, x in enumerate((-0.180, 0.0, 0.180)):
        lamp_panel.visual(
            Box((0.018, 0.020, 0.200)),
            origin=Origin(xyz=(x, 0.060, -0.040)),
            material=charcoal,
            name=f"heat_sink_fin_{index}",
        )
    lamp_panel.inertial = Inertial.from_geometry(
        Box((0.700, 0.120, 0.340)),
        mass=3.0,
        origin=Origin(xyz=(0.0, 0.025, -0.040)),
    )

    model.articulation(
        "base_to_outer_mast",
        ArticulationType.FIXED,
        parent=base,
        child=outer_mast,
        origin=Origin(xyz=(0.0, 0.0, 0.210)),
    )
    model.articulation(
        "outer_to_inner",
        ArticulationType.PRISMATIC,
        parent=outer_mast,
        child=inner_mast,
        origin=Origin(xyz=(0.0, 0.0, 0.840)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.25,
            lower=0.0,
            upper=0.340,
        ),
    )
    model.articulation(
        "inner_to_lamp_panel",
        ArticulationType.REVOLUTE,
        parent=inner_mast,
        child=lamp_panel,
        origin=Origin(xyz=(0.0, 0.0, 0.730)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.2,
            lower=0.0,
            upper=1.000,
        ),
    )

    for name, angle in (
        ("leg_front", 0.0),
        ("leg_left", 0.5 * math.pi),
        ("leg_rear", math.pi),
        ("leg_right", -0.5 * math.pi),
    ):
        leg = model.part(name)
        _build_leg(leg, metal=charcoal, rubber=rubber)
        model.articulation(
            f"base_to_{name}",
            ArticulationType.REVOLUTE,
            parent=base,
            child=leg,
            origin=Origin(
                xyz=(0.110 * math.cos(angle), 0.110 * math.sin(angle), 0.090),
                rpy=(0.0, 0.0, angle),
            ),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=45.0,
                velocity=1.2,
                lower=0.0,
                upper=1.180,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    outer_mast = object_model.get_part("outer_mast")
    inner_mast = object_model.get_part("inner_mast")
    lamp_panel = object_model.get_part("lamp_panel")
    leg_front = object_model.get_part("leg_front")

    mast_slide = object_model.get_articulation("outer_to_inner")
    lamp_tilt = object_model.get_articulation("inner_to_lamp_panel")
    front_leg_hinge = object_model.get_articulation("base_to_leg_front")

    ctx.expect_gap(
        outer_mast,
        inner_mast,
        axis="x",
        positive_elem="outer_right_wall",
        negative_elem="inner_right_wall",
        min_gap=0.004,
        max_gap=0.012,
        name="inner mast clears right sleeve wall",
    )
    ctx.expect_gap(
        inner_mast,
        outer_mast,
        axis="x",
        positive_elem="inner_left_wall",
        negative_elem="outer_left_wall",
        min_gap=0.004,
        max_gap=0.012,
        name="inner mast clears left sleeve wall",
    )
    ctx.expect_gap(
        outer_mast,
        inner_mast,
        axis="y",
        positive_elem="outer_front_wall",
        negative_elem="inner_front_wall",
        min_gap=0.004,
        max_gap=0.012,
        name="inner mast clears front sleeve wall",
    )
    ctx.expect_gap(
        inner_mast,
        outer_mast,
        axis="y",
        positive_elem="inner_back_wall",
        negative_elem="outer_back_wall",
        min_gap=0.004,
        max_gap=0.012,
        name="inner mast clears rear sleeve wall",
    )
    ctx.expect_overlap(
        inner_mast,
        outer_mast,
        axes="z",
        elem_a="inner_front_wall",
        elem_b="outer_front_wall",
        min_overlap=0.50,
        name="collapsed inner mast stays deeply inserted",
    )

    rest_inner_pos = ctx.part_world_position(inner_mast)
    with ctx.pose({mast_slide: 0.340}):
        ctx.expect_overlap(
            inner_mast,
            outer_mast,
            axes="z",
            elem_a="inner_front_wall",
            elem_b="outer_front_wall",
            min_overlap=0.20,
            name="extended inner mast retains insertion",
        )
        extended_inner_pos = ctx.part_world_position(inner_mast)

    ctx.check(
        "inner mast extends upward",
        rest_inner_pos is not None
        and extended_inner_pos is not None
        and extended_inner_pos[2] > rest_inner_pos[2] + 0.30,
        details=f"rest={rest_inner_pos}, extended={extended_inner_pos}",
    )

    rest_panel_aabb = ctx.part_element_world_aabb(lamp_panel, elem="lens")
    with ctx.pose({lamp_tilt: 0.650}):
        tilted_panel_aabb = ctx.part_element_world_aabb(lamp_panel, elem="lens")
    ctx.check(
        "lamp panel tilts downward",
        rest_panel_aabb is not None
        and tilted_panel_aabb is not None
        and tilted_panel_aabb[0][2] < rest_panel_aabb[0][2] - 0.050,
        details=f"rest={rest_panel_aabb}, tilted={tilted_panel_aabb}",
    )

    rest_leg_aabb = ctx.part_element_world_aabb(leg_front, elem="foot_pad")
    with ctx.pose({front_leg_hinge: 1.000}):
        folded_leg_aabb = ctx.part_element_world_aabb(leg_front, elem="foot_pad")
    ctx.check(
        "front leg folds upward toward the collar",
        rest_leg_aabb is not None
        and folded_leg_aabb is not None
        and folded_leg_aabb[0][2] > rest_leg_aabb[0][2] + 0.18,
        details=f"rest={rest_leg_aabb}, folded={folded_leg_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
