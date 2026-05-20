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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


HOUSING_HALF_SPAN = 0.72
HUB_DEPTH = 0.15
AXLE_FLANGE_THICKNESS = 0.012
AXLE_ROD_RADIUS = 0.017


def _x_shell_mesh(name: str, outer_profile: list[tuple[float, float]], inner_profile: list[tuple[float, float]]):
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(outer_profile, inner_profile, segments=64).rotate_y(math.pi / 2.0),
        name,
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(mins, maxs))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="full_floating_rear_axle")

    cast_iron = model.material("cast_iron", rgba=(0.25, 0.26, 0.28, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.18, 0.19, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.61, 0.63, 0.66, 1.0))
    oily_black = model.material("oily_black", rgba=(0.08, 0.08, 0.09, 1.0))

    housing_outer = [
        (0.056, -HOUSING_HALF_SPAN),
        (0.056, -0.59),
        (0.062, -0.56),
        (0.076, -0.47),
        (0.135, -0.24),
        (0.168, -0.08),
        (0.176, 0.00),
        (0.168, 0.08),
        (0.135, 0.24),
        (0.076, 0.47),
        (0.062, 0.56),
        (0.056, 0.59),
        (0.056, HOUSING_HALF_SPAN),
    ]
    housing_inner = [
        (0.034, -HOUSING_HALF_SPAN),
        (0.034, -0.58),
        (0.040, -0.54),
        (0.050, -0.45),
        (0.083, -0.22),
        (0.100, -0.08),
        (0.106, 0.00),
        (0.100, 0.08),
        (0.083, 0.22),
        (0.050, 0.45),
        (0.040, 0.54),
        (0.034, 0.58),
        (0.034, HOUSING_HALF_SPAN),
    ]
    housing_shell = _x_shell_mesh("axle_housing_shell", housing_outer, housing_inner)

    hub_outer = [
        (0.070, 0.000),
        (0.082, 0.014),
        (0.146, 0.032),
        (0.156, 0.080),
        (0.156, 0.118),
        (0.102, 0.134),
        (0.102, HUB_DEPTH),
    ]
    hub_inner = [
        (0.030, 0.000),
        (0.030, 0.030),
        (0.030, 0.120),
        (0.040, 0.134),
        (0.050, HUB_DEPTH),
    ]
    hub_shell = _x_shell_mesh("hub_drum_shell", hub_outer, hub_inner)

    housing = model.part("axle_housing")
    housing.visual(housing_shell, material=cast_iron, name="housing_shell")
    housing.visual(
        Cylinder(radius=0.115, length=0.040),
        origin=Origin(xyz=(0.0, 0.124, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=cast_iron,
        name="differential_cover",
    )
    housing.visual(
        Cylinder(radius=0.040, length=0.100),
        origin=Origin(xyz=(0.0, -0.135, -0.010), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=cast_iron,
        name="pinion_nose",
    )
    housing.visual(
        Cylinder(radius=0.022, length=0.070),
        origin=Origin(xyz=(0.0, -0.205, -0.010), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pinion_yoke_stub",
    )
    for index in range(10):
        angle = 2.0 * math.pi * index / 10.0
        housing.visual(
            Cylinder(radius=0.007, length=0.018),
            origin=Origin(
                xyz=(0.090 * math.cos(angle), 0.150, 0.090 * math.sin(angle)),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=machined_steel,
            name=f"cover_bolt_{index}",
        )
    housing.inertial = Inertial.from_geometry(
        Box((1.52, 0.40, 0.40)),
        mass=112.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    left_hub = model.part("left_hub_drum")
    left_hub.visual(
        hub_shell,
        origin=Origin(rpy=(0.0, 0.0, math.pi)),
        material=dark_steel,
        name="left_hub_shell",
    )
    for index in range(5):
        angle = 2.0 * math.pi * index / 5.0 + 0.18
        left_hub.visual(
            Cylinder(radius=0.008, length=0.022),
            origin=Origin(
                xyz=(-0.161, 0.092 * math.cos(angle), 0.092 * math.sin(angle)),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=machined_steel,
            name=f"left_hub_stud_{index}",
        )
    left_hub.inertial = Inertial.from_geometry(
        Cylinder(radius=0.156, length=HUB_DEPTH),
        mass=21.0,
        origin=Origin(xyz=(-0.075, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    right_hub = model.part("right_hub_drum")
    right_hub.visual(hub_shell, material=dark_steel, name="right_hub_shell")
    for index in range(5):
        angle = 2.0 * math.pi * index / 5.0 + 0.18
        right_hub.visual(
            Cylinder(radius=0.008, length=0.022),
            origin=Origin(
                xyz=(0.161, 0.092 * math.cos(angle), 0.092 * math.sin(angle)),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=machined_steel,
            name=f"right_hub_stud_{index}",
        )
    right_hub.inertial = Inertial.from_geometry(
        Cylinder(radius=0.156, length=HUB_DEPTH),
        mass=21.0,
        origin=Origin(xyz=(0.075, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    left_axle = model.part("left_axle_shaft")
    left_axle.visual(
        Cylinder(radius=AXLE_ROD_RADIUS, length=0.86),
        origin=Origin(xyz=(-0.44, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=oily_black,
        name="left_axle_rod",
    )
    left_axle.visual(
        Cylinder(radius=0.056, length=AXLE_FLANGE_THICKNESS),
        origin=Origin(xyz=(-0.876, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="left_axle_flange",
    )
    for index in range(6):
        angle = 2.0 * math.pi * index / 6.0
        left_axle.visual(
            Cylinder(radius=0.006, length=0.012),
            origin=Origin(
                xyz=(-0.888, 0.034 * math.cos(angle), 0.034 * math.sin(angle)),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=machined_steel,
            name=f"left_axle_bolt_{index}",
        )
    left_axle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.056, length=0.89),
        mass=7.5,
        origin=Origin(xyz=(-0.445, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    right_axle = model.part("right_axle_shaft")
    right_axle.visual(
        Cylinder(radius=AXLE_ROD_RADIUS, length=0.86),
        origin=Origin(xyz=(0.44, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=oily_black,
        name="right_axle_rod",
    )
    right_axle.visual(
        Cylinder(radius=0.056, length=AXLE_FLANGE_THICKNESS),
        origin=Origin(xyz=(0.876, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="right_axle_flange",
    )
    for index in range(6):
        angle = 2.0 * math.pi * index / 6.0
        right_axle.visual(
            Cylinder(radius=0.006, length=0.012),
            origin=Origin(
                xyz=(0.888, 0.034 * math.cos(angle), 0.034 * math.sin(angle)),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=machined_steel,
            name=f"right_axle_bolt_{index}",
        )
    right_axle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.056, length=0.89),
        mass=7.5,
        origin=Origin(xyz=(0.445, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "left_hub_spin",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=left_hub,
        origin=Origin(xyz=(-HOUSING_HALF_SPAN, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=20.0),
    )
    model.articulation(
        "right_hub_spin",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=right_hub,
        origin=Origin(xyz=(HOUSING_HALF_SPAN, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=20.0),
    )
    model.articulation(
        "left_axle_spin",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=left_axle,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=35.0),
    )
    model.articulation(
        "right_axle_spin",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=right_axle,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=35.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    housing = object_model.get_part("axle_housing")
    left_hub = object_model.get_part("left_hub_drum")
    right_hub = object_model.get_part("right_hub_drum")
    left_axle = object_model.get_part("left_axle_shaft")
    right_axle = object_model.get_part("right_axle_shaft")

    left_hub_spin = object_model.get_articulation("left_hub_spin")
    right_hub_spin = object_model.get_articulation("right_hub_spin")
    left_axle_spin = object_model.get_articulation("left_axle_spin")
    right_axle_spin = object_model.get_articulation("right_axle_spin")

    ctx.expect_gap(
        right_hub,
        housing,
        axis="x",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem="right_hub_shell",
        negative_elem="housing_shell",
        name="right hub drum seats at the tube end",
    )
    ctx.expect_gap(
        housing,
        left_hub,
        axis="x",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem="housing_shell",
        negative_elem="left_hub_shell",
        name="left hub drum seats at the tube end",
    )
    ctx.expect_gap(
        right_axle,
        right_hub,
        axis="x",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem="right_axle_flange",
        negative_elem="right_hub_shell",
        name="right axle flange meets the removable hub face",
    )
    ctx.expect_gap(
        left_hub,
        left_axle,
        axis="x",
        max_gap=0.002,
        max_penetration=0.0,
        positive_elem="left_hub_shell",
        negative_elem="left_axle_flange",
        name="left axle flange meets the removable hub face",
    )
    ctx.expect_within(
        left_axle,
        housing,
        axes="yz",
        inner_elem="left_axle_rod",
        outer_elem="housing_shell",
        margin=0.0,
        name="left axle shaft stays centered inside the housing envelope",
    )
    ctx.expect_within(
        right_axle,
        housing,
        axes="yz",
        inner_elem="right_axle_rod",
        outer_elem="housing_shell",
        margin=0.0,
        name="right axle shaft stays centered inside the housing envelope",
    )
    ctx.expect_overlap(
        left_axle,
        housing,
        axes="x",
        elem_a="left_axle_rod",
        elem_b="housing_shell",
        min_overlap=0.68,
        name="left axle shaft remains deeply inserted through the tube",
    )
    ctx.expect_overlap(
        right_axle,
        housing,
        axes="x",
        elem_a="right_axle_rod",
        elem_b="housing_shell",
        min_overlap=0.68,
        name="right axle shaft remains deeply inserted through the tube",
    )

    with ctx.pose({left_hub_spin: 0.0}):
        left_hub_rest = _aabb_center(ctx.part_element_world_aabb(left_hub, elem="left_hub_stud_0"))
    with ctx.pose({left_hub_spin: math.pi / 2.0}):
        left_hub_turned = _aabb_center(ctx.part_element_world_aabb(left_hub, elem="left_hub_stud_0"))

    if left_hub_rest is None or left_hub_turned is None:
        ctx.fail("left hub spin exposes a rotating stud", "missing left_hub_stud_0 probe geometry")
    else:
        rest_radius = math.hypot(left_hub_rest[1], left_hub_rest[2])
        turn_radius = math.hypot(left_hub_turned[1], left_hub_turned[2])
        ctx.check(
            "left hub spin exposes a rotating stud",
            abs(left_hub_rest[0] - left_hub_turned[0]) < 0.002
            and abs(rest_radius - turn_radius) < 0.002
            and abs(left_hub_rest[1]) > abs(left_hub_rest[2]) + 0.03
            and abs(left_hub_turned[2]) > abs(left_hub_turned[1]) + 0.03,
            details=f"rest={left_hub_rest}, turned={left_hub_turned}",
        )

    with ctx.pose({right_axle_spin: 0.0}):
        right_axle_rest = _aabb_center(ctx.part_element_world_aabb(right_axle, elem="right_axle_bolt_0"))
    with ctx.pose({right_axle_spin: math.pi / 2.0}):
        right_axle_turned = _aabb_center(ctx.part_element_world_aabb(right_axle, elem="right_axle_bolt_0"))

    if right_axle_rest is None or right_axle_turned is None:
        ctx.fail("right axle shaft visibly spins inside the tube", "missing right_axle_bolt_0 probe geometry")
    else:
        rest_radius = math.hypot(right_axle_rest[1], right_axle_rest[2])
        turn_radius = math.hypot(right_axle_turned[1], right_axle_turned[2])
        ctx.check(
            "right axle shaft visibly spins inside the tube",
            abs(right_axle_rest[0] - right_axle_turned[0]) < 0.002
            and abs(rest_radius - turn_radius) < 0.002
            and abs(right_axle_rest[1]) > abs(right_axle_rest[2]) + 0.015
            and abs(right_axle_turned[2]) > abs(right_axle_turned[1]) + 0.015,
            details=f"rest={right_axle_rest}, turned={right_axle_turned}",
        )

    with ctx.pose({right_hub_spin: 0.7, left_axle_spin: 1.1}):
        ctx.expect_gap(
            right_hub,
            housing,
            axis="x",
            max_gap=0.002,
            max_penetration=0.0,
            positive_elem="right_hub_shell",
            negative_elem="housing_shell",
            name="right hub stays seated while spinning",
        )
        ctx.expect_overlap(
            left_axle,
            housing,
            axes="x",
            elem_a="left_axle_rod",
            elem_b="housing_shell",
            min_overlap=0.68,
            name="left axle shaft stays retained while spinning",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
