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
    MotionProperties,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _annular_cylinder(
    outer_radius: float,
    inner_radius: float,
    height: float,
    *,
    chamfer: float = 0.0,
) -> cq.Workplane:
    """Centered annular bearing/seal geometry with a true open bore."""
    ring = cq.Workplane("XY").circle(outer_radius).circle(inner_radius).extrude(height)
    if chamfer > 0.0:
        ring = ring.edges().chamfer(chamfer)
    return ring.translate((0.0, 0.0, -height / 2.0))


def _radial_origin(radius: float, z: float, angle: float) -> Origin:
    return Origin(
        xyz=(radius * math.cos(angle), radius * math.sin(angle), z),
        rpy=(0.0, math.pi / 2.0, angle),
    )


def _axis_rpy(axis: str) -> tuple[float, float, float]:
    if axis == "x":
        return (0.0, math.pi / 2.0, 0.0)
    if axis == "y":
        return (0.0, math.pi / 2.0, math.pi / 2.0)
    return (0.0, 0.0, 0.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_bearing_turnstile_gate")

    matte_graphite = Material("matte_graphite", rgba=(0.055, 0.058, 0.062, 1.0))
    satin_steel = Material("satin_stainless", rgba=(0.70, 0.68, 0.62, 1.0))
    brushed_dark = Material("brushed_dark_metal", rgba=(0.18, 0.19, 0.20, 1.0))
    black_rubber = Material("black_rubber_seals", rgba=(0.012, 0.012, 0.011, 1.0))
    floor_metal = Material("warm_floor_plate", rgba=(0.30, 0.30, 0.285, 1.0))
    glass_black = Material("black_glass", rgba=(0.012, 0.026, 0.034, 1.0))
    status_green = Material("soft_status_green", rgba=(0.15, 0.80, 0.40, 1.0))
    bronze = Material("restrained_bronze_accent", rgba=(0.70, 0.52, 0.30, 1.0))

    fixed = model.part("fixed_frame")
    fixed.visual(
        Box((2.08, 2.18, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=floor_metal,
        name="floor_plinth",
    )
    fixed.visual(
        Box((1.86, 0.018, 0.006)),
        origin=Origin(xyz=(0.0, 0.98, 0.062)),
        material=black_rubber,
        name="rear_floor_seam",
    )
    fixed.visual(
        Box((1.86, 0.018, 0.006)),
        origin=Origin(xyz=(0.0, -0.98, 0.062)),
        material=black_rubber,
        name="front_floor_seam",
    )
    fixed.visual(
        Box((0.018, 1.86, 0.006)),
        origin=Origin(xyz=(0.98, 0.0, 0.062)),
        material=black_rubber,
        name="side_floor_seam_0",
    )
    fixed.visual(
        Box((0.018, 1.86, 0.006)),
        origin=Origin(xyz=(-0.98, 0.0, 0.062)),
        material=black_rubber,
        name="side_floor_seam_1",
    )

    # Four structural posts tied by side rails and top beams.
    for ix, x in enumerate((-0.88, 0.88)):
        for iy, y in enumerate((-0.92, 0.92)):
            fixed.visual(
                Cylinder(0.045, 1.62),
                origin=Origin(xyz=(x, y, 0.86)),
                material=matte_graphite,
                name=f"corner_post_{ix}_{iy}",
            )
            fixed.visual(
                Cylinder(0.051, 0.018),
                origin=Origin(xyz=(x, y, 1.675)),
                material=satin_steel,
                name=f"post_cap_{ix}_{iy}",
            )

    for y_name, y in (("front", -0.92), ("rear", 0.92)):
        for level_name, z, radius in (
            ("lower", 0.42, 0.024),
            ("waist", 0.92, 0.027),
            ("upper", 1.34, 0.024),
        ):
            fixed.visual(
                Cylinder(radius, 1.80),
                origin=Origin(xyz=(0.0, y, z), rpy=_axis_rpy("x")),
                material=matte_graphite,
                name=f"{y_name}_{level_name}_rail",
            )
        fixed.visual(
            Box((1.84, 0.070, 0.065)),
            origin=Origin(xyz=(0.0, y, 1.60)),
            material=brushed_dark,
            name=f"{y_name}_top_beam",
        )

    for x_name, x in (("side_0", -0.88), ("side_1", 0.88)):
        fixed.visual(
            Box((0.070, 1.90, 0.065)),
            origin=Origin(xyz=(x, 0.0, 1.60)),
            material=brushed_dark,
            name=f"{x_name}_top_beam",
        )

    # Lower bearing plinths keep the annular lower bearing visibly tied to the floor plate.
    fixed.visual(
        Box((0.060, 0.100, 0.090)),
        origin=Origin(xyz=(0.130, 0.0, 0.095)),
        material=brushed_dark,
        name="lower_bearing_boss_0",
    )
    fixed.visual(
        Box((0.060, 0.100, 0.090)),
        origin=Origin(xyz=(-0.130, 0.0, 0.095)),
        material=brushed_dark,
        name="lower_bearing_boss_1",
    )
    fixed.visual(
        Box((0.100, 0.060, 0.090)),
        origin=Origin(xyz=(0.0, 0.130, 0.095)),
        material=brushed_dark,
        name="lower_bearing_boss_2",
    )
    fixed.visual(
        Box((0.100, 0.060, 0.090)),
        origin=Origin(xyz=(0.0, -0.130, 0.095)),
        material=brushed_dark,
        name="lower_bearing_boss_3",
    )
    fixed.visual(
        mesh_from_cadquery(
            _annular_cylinder(0.170, 0.066, 0.080, chamfer=0.004),
            "lower_bearing_collar",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
        material=satin_steel,
        name="lower_bearing_collar",
    )
    fixed.visual(
        mesh_from_cadquery(
            _annular_cylinder(0.078, 0.049, 0.014, chamfer=0.0015),
            "lower_bearing_seal",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.207)),
        material=black_rubber,
        name="lower_bearing_seal",
    )

    # Split yoke arms stop short of the bore, so the rotating spindle has a clear path.
    fixed.visual(
        Box((0.090, 0.780, 0.060)),
        origin=Origin(xyz=(0.0, 0.535, 1.600)),
        material=brushed_dark,
        name="upper_yoke_arm_0",
    )
    fixed.visual(
        Box((0.090, 0.780, 0.060)),
        origin=Origin(xyz=(0.0, -0.535, 1.600)),
        material=brushed_dark,
        name="upper_yoke_arm_1",
    )
    fixed.visual(
        mesh_from_cadquery(
            _annular_cylinder(0.170, 0.066, 0.080, chamfer=0.004),
            "upper_bearing_collar",
        ),
        origin=Origin(xyz=(0.0, 0.0, 1.600)),
        material=satin_steel,
        name="upper_bearing_collar",
    )
    fixed.visual(
        mesh_from_cadquery(
            _annular_cylinder(0.078, 0.049, 0.014, chamfer=0.0015),
            "upper_bearing_seal",
        ),
        origin=Origin(xyz=(0.0, 0.0, 1.553)),
        material=black_rubber,
        name="upper_bearing_seal",
    )

    # Restrained access-control pod, mounted to the fixed frame instead of floating.
    fixed.visual(
        Box((0.135, 0.070, 0.280)),
        origin=Origin(xyz=(-0.88, -0.975, 1.06)),
        material=matte_graphite,
        name="reader_housing",
    )
    fixed.visual(
        Box((0.098, 0.008, 0.160)),
        origin=Origin(xyz=(-0.88, -1.014, 1.085)),
        material=glass_black,
        name="reader_glass",
    )
    fixed.visual(
        Sphere(0.018),
        origin=Origin(xyz=(-0.88, -1.020, 1.175)),
        material=status_green,
        name="reader_indicator",
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(0.050, 1.55),
        origin=Origin(xyz=(0.0, 0.0, 0.860)),
        material=satin_steel,
        name="spindle",
    )
    rotor.visual(
        Cylinder(0.105, 0.840),
        origin=Origin(xyz=(0.0, 0.0, 0.875)),
        material=satin_steel,
        name="central_post",
    )
    rotor.visual(
        Cylinder(0.118, 0.150),
        origin=Origin(xyz=(0.0, 0.0, 0.550)),
        material=brushed_dark,
        name="lower_arm_hub",
    )
    rotor.visual(
        Cylinder(0.125, 0.180),
        origin=Origin(xyz=(0.0, 0.0, 0.850)),
        material=brushed_dark,
        name="center_arm_hub",
    )
    rotor.visual(
        Cylinder(0.118, 0.150),
        origin=Origin(xyz=(0.0, 0.0, 1.150)),
        material=brushed_dark,
        name="upper_arm_hub",
    )
    rotor.visual(
        Cylinder(0.108, 0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.435)),
        material=black_rubber,
        name="lower_hub_seam",
    )
    rotor.visual(
        Cylinder(0.108, 0.018),
        origin=Origin(xyz=(0.0, 0.0, 1.315)),
        material=black_rubber,
        name="upper_hub_seam",
    )
    rotor.visual(
        Cylinder(0.092, 0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.245)),
        material=bronze,
        name="lower_thrust_collar",
    )
    rotor.visual(
        Cylinder(0.092, 0.040),
        origin=Origin(xyz=(0.0, 0.0, 1.515)),
        material=bronze,
        name="upper_thrust_collar",
    )

    arm_levels = (0.55, 0.85, 1.15)
    for wing_index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        rotor.visual(
            Cylinder(0.026, 0.900),
            origin=Origin(
                xyz=(0.755 * math.cos(angle), 0.755 * math.sin(angle), 0.875),
                rpy=(0.0, 0.0, 0.0),
            ),
            material=satin_steel,
            name=f"outer_rail_{wing_index}",
        )
        rotor.visual(
            Sphere(0.031),
            origin=Origin(xyz=(0.755 * math.cos(angle), 0.755 * math.sin(angle), 1.325)),
            material=satin_steel,
            name=f"outer_top_cap_{wing_index}",
        )
        rotor.visual(
            Sphere(0.031),
            origin=Origin(xyz=(0.755 * math.cos(angle), 0.755 * math.sin(angle), 0.425)),
            material=satin_steel,
            name=f"outer_lower_cap_{wing_index}",
        )
        for level_index, z in enumerate(arm_levels):
            rotor.visual(
                Cylinder(0.022, 0.700),
                origin=_radial_origin(0.405, z, angle),
                material=satin_steel,
                name=f"radial_arm_{wing_index}_{level_index}",
            )
            rotor.visual(
                Cylinder(0.028, 0.030),
                origin=_radial_origin(0.735, z, angle),
                material=brushed_dark,
                name=f"arm_end_collar_{wing_index}_{level_index}",
            )

    model.articulation(
        "bearing_axis",
        ArticulationType.CONTINUOUS,
        parent=fixed,
        child=rotor,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.2),
        motion_properties=MotionProperties(damping=0.12, friction=0.04),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fixed = object_model.get_part("fixed_frame")
    rotor = object_model.get_part("rotor")
    bearing_axis = object_model.get_articulation("bearing_axis")

    ctx.allow_overlap(
        rotor,
        fixed,
        elem_a="spindle",
        elem_b="lower_bearing_seal",
        reason=(
            "The rubber bearing seal is intentionally modeled with slight "
            "compression around the rotating spindle so the bearing support reads as seated."
        ),
    )
    ctx.allow_overlap(
        rotor,
        fixed,
        elem_a="spindle",
        elem_b="upper_bearing_seal",
        reason=(
            "The upper rubber bearing seal is intentionally compressed locally "
            "around the rotating spindle."
        ),
    )

    ctx.check(
        "single vertical rotary bearing axis",
        bearing_axis.axis == (0.0, 0.0, 1.0),
        details=f"axis={bearing_axis.axis}",
    )
    ctx.expect_within(
        rotor,
        fixed,
        axes="xy",
        inner_elem="spindle",
        outer_elem="lower_bearing_collar",
        margin=0.0,
        name="spindle centered in lower bearing footprint",
    )
    ctx.expect_within(
        rotor,
        fixed,
        axes="xy",
        inner_elem="spindle",
        outer_elem="upper_bearing_collar",
        margin=0.0,
        name="spindle centered in upper bearing footprint",
    )
    ctx.expect_overlap(
        rotor,
        fixed,
        axes="z",
        elem_a="spindle",
        elem_b="lower_bearing_collar",
        min_overlap=0.050,
        name="spindle passes through lower bearing",
    )
    ctx.expect_overlap(
        rotor,
        fixed,
        axes="z",
        elem_a="spindle",
        elem_b="upper_bearing_collar",
        min_overlap=0.050,
        name="spindle passes through upper bearing",
    )
    ctx.expect_overlap(
        rotor,
        fixed,
        axes="z",
        elem_a="spindle",
        elem_b="lower_bearing_seal",
        min_overlap=0.010,
        name="lower seal captures spindle height",
    )
    ctx.expect_overlap(
        rotor,
        fixed,
        axes="z",
        elem_a="spindle",
        elem_b="upper_bearing_seal",
        min_overlap=0.010,
        name="upper seal captures spindle height",
    )
    ctx.expect_gap(
        rotor,
        fixed,
        axis="z",
        positive_elem="lower_thrust_collar",
        negative_elem="lower_bearing_collar",
        min_gap=0.010,
        max_gap=0.040,
        name="lower thrust collar has tight bearing seam",
    )
    ctx.expect_gap(
        fixed,
        rotor,
        axis="z",
        positive_elem="upper_bearing_collar",
        negative_elem="upper_thrust_collar",
        min_gap=0.010,
        max_gap=0.040,
        name="upper thrust collar has tight bearing seam",
    )

    def _aabb_center_xy(aabb):
        return ((aabb[0][0] + aabb[1][0]) / 2.0, (aabb[0][1] + aabb[1][1]) / 2.0)

    rest_outer = ctx.part_element_world_aabb(rotor, elem="outer_rail_0")
    with ctx.pose({bearing_axis: math.pi / 2.0}):
        turned_outer = ctx.part_element_world_aabb(rotor, elem="outer_rail_0")
    if rest_outer is not None and turned_outer is not None:
        rest_xy = _aabb_center_xy(rest_outer)
        turned_xy = _aabb_center_xy(turned_outer)
        travel = math.hypot(turned_xy[0] - rest_xy[0], turned_xy[1] - rest_xy[1])
    else:
        rest_xy = None
        turned_xy = None
        travel = 0.0
    ctx.check(
        "radial gate wing rotates around the supported spindle",
        travel > 0.85,
        details=f"rest={rest_xy}, turned={turned_xy}, travel={travel:.3f}",
    )

    return ctx.report()


object_model = build_object_model()
