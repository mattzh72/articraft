from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import hypot

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_FOOT_RADIUS = 0.22
BASE_FOOT_HEIGHT = 0.03
BASE_HOUSING_RADIUS = 0.17
BASE_HOUSING_HEIGHT = 0.045
BASE_SUPPORT_COLLAR_RADIUS = 0.11
BASE_SUPPORT_COLLAR_HEIGHT = 0.008
SPINDLE_RADIUS = 0.027
SPINDLE_HEIGHT = 0.175
SPINDLE_TOP_Z = BASE_FOOT_HEIGHT + SPINDLE_HEIGHT
RETAINER_RADIUS = 0.036
RETAINER_HEIGHT = 0.009

LOWER_OUTER_RADIUS = 0.20
LOWER_BORE_RADIUS = 0.032
LOWER_THICKNESS = 0.045
LOWER_UPPER_COLLAR_RADIUS = 0.112
LOWER_UPPER_COLLAR_HEIGHT = 0.008

MIDDLE_OUTER_RADIUS = 0.155
MIDDLE_INNER_RADIUS = 0.10
MIDDLE_THICKNESS = 0.03
MIDDLE_UPPER_COLLAR_RADIUS = 0.055
MIDDLE_UPPER_COLLAR_HEIGHT = 0.006

TOP_OUTER_RADIUS = 0.082
TOP_BORE_RADIUS = 0.032
TOP_THICKNESS = 0.02

BASE_TO_LOWER_Z = BASE_FOOT_HEIGHT + BASE_HOUSING_HEIGHT + BASE_SUPPORT_COLLAR_HEIGHT
LOWER_TO_MIDDLE_Z = LOWER_THICKNESS + LOWER_UPPER_COLLAR_HEIGHT
MIDDLE_TO_TOP_Z = MIDDLE_THICKNESS + MIDDLE_UPPER_COLLAR_HEIGHT


def _annular_plate(outer_radius: float, inner_radius: float, thickness: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(thickness)
    )


def _disk_with_bore(outer_radius: float, inner_radius: float, thickness: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(thickness)
    )


def _middle_stage_body() -> cq.Workplane:
    hub_outer_radius = 0.06
    spoke_width = 0.024
    spoke_span = MIDDLE_INNER_RADIUS - hub_outer_radius

    body = _annular_plate(MIDDLE_OUTER_RADIUS, MIDDLE_INNER_RADIUS, MIDDLE_THICKNESS)
    body = body.union(_annular_plate(hub_outer_radius, TOP_BORE_RADIUS, MIDDLE_THICKNESS))

    for angle_deg in (0.0, 120.0, 240.0):
        spoke = (
            cq.Workplane("XY")
            .box(spoke_span, spoke_width, MIDDLE_THICKNESS)
            .translate((hub_outer_radius + (spoke_span * 0.5), 0.0, MIDDLE_THICKNESS * 0.5))
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)
        )
        body = body.union(spoke)

    return body


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))


def _xy_distance(
    a: tuple[float, float, float] | None,
    b: tuple[float, float, float] | None,
) -> float | None:
    if a is None or b is None:
        return None
    return hypot(a[0] - b[0], a[1] - b[1])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="coaxial_indexing_stack")

    model.material("cast_iron", rgba=(0.21, 0.22, 0.24, 1.0))
    model.material("anodized_gray", rgba=(0.60, 0.63, 0.67, 1.0))
    model.material("machined_aluminum", rgba=(0.76, 0.78, 0.81, 1.0))
    model.material("bearing_steel", rgba=(0.70, 0.72, 0.75, 1.0))
    model.material("index_black", rgba=(0.12, 0.12, 0.13, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=BASE_FOOT_RADIUS, length=BASE_FOOT_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_FOOT_HEIGHT * 0.5)),
        material="cast_iron",
        name="base_foot",
    )
    base.visual(
        Cylinder(radius=BASE_HOUSING_RADIUS, length=BASE_HOUSING_HEIGHT),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                BASE_FOOT_HEIGHT + (BASE_HOUSING_HEIGHT * 0.5),
            )
        ),
        material="cast_iron",
        name="base_housing",
    )
    base.visual(
        Cylinder(radius=BASE_SUPPORT_COLLAR_RADIUS, length=BASE_SUPPORT_COLLAR_HEIGHT),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                BASE_FOOT_HEIGHT + BASE_HOUSING_HEIGHT + (BASE_SUPPORT_COLLAR_HEIGHT * 0.5),
            )
        ),
        material="bearing_steel",
        name="base_support_collar",
    )
    base.visual(
        Cylinder(radius=SPINDLE_RADIUS, length=SPINDLE_HEIGHT),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                BASE_FOOT_HEIGHT + (SPINDLE_HEIGHT * 0.5),
            )
        ),
        material="bearing_steel",
        name="center_spindle",
    )
    base.visual(
        Cylinder(radius=RETAINER_RADIUS, length=RETAINER_HEIGHT),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                SPINDLE_TOP_Z - (RETAINER_HEIGHT * 0.5),
            )
        ),
        material="bearing_steel",
        name="spindle_retainer",
    )
    base.visual(
        Box((0.032, 0.016, 0.02)),
        origin=Origin(
            xyz=(
                BASE_HOUSING_RADIUS - 0.006,
                0.0,
                BASE_FOOT_HEIGHT + BASE_HOUSING_HEIGHT - 0.01,
            )
        ),
        material="index_black",
        name="base_index_block",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=BASE_FOOT_RADIUS, length=BASE_FOOT_HEIGHT + BASE_HOUSING_HEIGHT),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.0, (BASE_FOOT_HEIGHT + BASE_HOUSING_HEIGHT) * 0.5)),
    )

    lower_stage = model.part("lower_stage")
    lower_stage.visual(
        mesh_from_cadquery(
            _disk_with_bore(LOWER_OUTER_RADIUS, LOWER_BORE_RADIUS, LOWER_THICKNESS),
            "lower_stage_plate",
        ),
        material="anodized_gray",
        name="lower_plate",
    )
    lower_stage.visual(
        mesh_from_cadquery(
            _annular_plate(LOWER_UPPER_COLLAR_RADIUS, LOWER_BORE_RADIUS, LOWER_UPPER_COLLAR_HEIGHT),
            "lower_stage_upper_collar",
        ),
        origin=Origin(xyz=(0.0, 0.0, LOWER_THICKNESS)),
        material="bearing_steel",
        name="lower_upper_collar",
    )
    lower_stage.visual(
        Box((0.05, 0.022, 0.018)),
        origin=Origin(
            xyz=(
                LOWER_OUTER_RADIUS - 0.022,
                0.0,
                LOWER_THICKNESS - 0.011,
            )
        ),
        material="index_black",
        name="lower_pointer",
    )
    lower_stage.inertial = Inertial.from_geometry(
        Cylinder(radius=LOWER_OUTER_RADIUS, length=LOWER_THICKNESS + LOWER_UPPER_COLLAR_HEIGHT),
        mass=11.0,
        origin=Origin(xyz=(0.0, 0.0, (LOWER_THICKNESS + LOWER_UPPER_COLLAR_HEIGHT) * 0.5)),
    )

    middle_stage = model.part("middle_stage")
    middle_stage.visual(
        mesh_from_cadquery(
            _middle_stage_body(),
            "middle_stage_ring",
        ),
        material="machined_aluminum",
        name="middle_ring",
    )
    middle_stage.visual(
        mesh_from_cadquery(
            _annular_plate(MIDDLE_UPPER_COLLAR_RADIUS, TOP_BORE_RADIUS, MIDDLE_UPPER_COLLAR_HEIGHT),
            "middle_stage_upper_collar",
        ),
        origin=Origin(xyz=(0.0, 0.0, MIDDLE_THICKNESS)),
        material="bearing_steel",
        name="middle_upper_collar",
    )
    middle_stage.visual(
        Box((0.018, 0.04, 0.014)),
        origin=Origin(
            xyz=(
                0.0,
                MIDDLE_OUTER_RADIUS - 0.017,
                MIDDLE_THICKNESS - 0.008,
            )
        ),
        material="index_black",
        name="middle_pointer",
    )
    middle_stage.inertial = Inertial.from_geometry(
        Cylinder(radius=MIDDLE_OUTER_RADIUS, length=MIDDLE_THICKNESS + MIDDLE_UPPER_COLLAR_HEIGHT),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.0, (MIDDLE_THICKNESS + MIDDLE_UPPER_COLLAR_HEIGHT) * 0.5)),
    )

    top_stage = model.part("top_stage")
    top_stage.visual(
        mesh_from_cadquery(
            _disk_with_bore(TOP_OUTER_RADIUS, TOP_BORE_RADIUS, TOP_THICKNESS),
            "top_stage_flange",
        ),
        material="machined_aluminum",
        name="top_flange",
    )
    top_stage.visual(
        Box((0.038, 0.014, 0.012)),
        origin=Origin(
            xyz=(
                -(TOP_OUTER_RADIUS - 0.018),
                0.0,
                TOP_THICKNESS - 0.006,
            )
        ),
        material="index_black",
        name="top_pointer",
    )
    top_stage.inertial = Inertial.from_geometry(
        Cylinder(radius=TOP_OUTER_RADIUS, length=TOP_THICKNESS),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, TOP_THICKNESS * 0.5)),
    )

    model.articulation(
        "base_to_lower",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_stage,
        origin=Origin(xyz=(0.0, 0.0, BASE_TO_LOWER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-3.14159,
            upper=3.14159,
            effort=120.0,
            velocity=1.2,
        ),
    )
    model.articulation(
        "lower_to_middle",
        ArticulationType.REVOLUTE,
        parent=lower_stage,
        child=middle_stage,
        origin=Origin(xyz=(0.0, 0.0, LOWER_TO_MIDDLE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-2.35,
            upper=2.35,
            effort=70.0,
            velocity=1.6,
        ),
    )
    model.articulation(
        "middle_to_top",
        ArticulationType.REVOLUTE,
        parent=middle_stage,
        child=top_stage,
        origin=Origin(xyz=(0.0, 0.0, MIDDLE_TO_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=-2.0,
            upper=2.0,
            effort=35.0,
            velocity=2.2,
        ),
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

    base = object_model.get_part("base")
    lower_stage = object_model.get_part("lower_stage")
    middle_stage = object_model.get_part("middle_stage")
    top_stage = object_model.get_part("top_stage")

    base_to_lower = object_model.get_articulation("base_to_lower")
    lower_to_middle = object_model.get_articulation("lower_to_middle")
    middle_to_top = object_model.get_articulation("middle_to_top")

    ctx.check(
        "all three rotary joints are vertical and coaxial",
        all(joint.axis == (0.0, 0.0, 1.0) for joint in (base_to_lower, lower_to_middle, middle_to_top))
        and all(joint.origin.xyz[:2] == (0.0, 0.0) for joint in (base_to_lower, lower_to_middle, middle_to_top)),
        details=(
            f"axes={[base_to_lower.axis, lower_to_middle.axis, middle_to_top.axis]}, "
            f"origins={[base_to_lower.origin.xyz, lower_to_middle.origin.xyz, middle_to_top.origin.xyz]}"
        ),
    )

    ctx.expect_origin_distance(
        lower_stage,
        base,
        axes="xy",
        max_dist=0.001,
        name="lower stage stays centered on the base spindle",
    )
    ctx.expect_origin_distance(
        middle_stage,
        lower_stage,
        axes="xy",
        max_dist=0.001,
        name="middle ring stays centered on the lower stage",
    )
    ctx.expect_origin_distance(
        top_stage,
        middle_stage,
        axes="xy",
        max_dist=0.001,
        name="top flange stays centered on the middle ring",
    )

    with ctx.pose({base_to_lower: 0.0, lower_to_middle: 0.0, middle_to_top: 0.0}):
        ctx.expect_gap(
            lower_stage,
            base,
            axis="z",
            positive_elem="lower_plate",
            negative_elem="base_support_collar",
            max_gap=0.001,
            max_penetration=1e-5,
            name="lower stage seats on the base collar",
        )
        ctx.expect_overlap(
            lower_stage,
            base,
            axes="xy",
            elem_a="lower_plate",
            elem_b="base_support_collar",
            min_overlap=0.10,
            name="lower stage footprint covers the base collar",
        )
        ctx.expect_gap(
            middle_stage,
            lower_stage,
            axis="z",
            positive_elem="middle_ring",
            negative_elem="lower_upper_collar",
            max_gap=0.001,
            max_penetration=1e-5,
            name="middle ring seats on the lower bearing collar",
        )
        ctx.expect_overlap(
            middle_stage,
            lower_stage,
            axes="xy",
            elem_a="middle_ring",
            elem_b="lower_upper_collar",
            min_overlap=0.01,
            name="lower bearing collar supports the middle ring",
        )
        ctx.expect_gap(
            top_stage,
            middle_stage,
            axis="z",
            positive_elem="top_flange",
            negative_elem="middle_upper_collar",
            max_gap=0.001,
            max_penetration=1e-5,
            name="top flange seats on the middle bearing collar",
        )
        ctx.expect_overlap(
            top_stage,
            middle_stage,
            axes="xy",
            elem_a="top_flange",
            elem_b="middle_upper_collar",
            min_overlap=0.02,
            name="middle bearing collar supports the top flange",
        )

        lower_pointer_rest = _aabb_center(ctx.part_element_world_aabb(lower_stage, elem="lower_pointer"))
        middle_pointer_rest = _aabb_center(ctx.part_element_world_aabb(middle_stage, elem="middle_pointer"))
        top_pointer_rest = _aabb_center(ctx.part_element_world_aabb(top_stage, elem="top_pointer"))

    with ctx.pose({base_to_lower: 0.8, lower_to_middle: 0.0, middle_to_top: 0.0}):
        lower_pointer_rotated = _aabb_center(ctx.part_element_world_aabb(lower_stage, elem="lower_pointer"))
    lower_pointer_shift = _xy_distance(lower_pointer_rest, lower_pointer_rotated)
    ctx.check(
        "lower stage visibly rotates about the spindle",
        lower_pointer_shift is not None and lower_pointer_shift > 0.05,
        details=f"rest={lower_pointer_rest}, moved={lower_pointer_rotated}, shift={lower_pointer_shift}",
    )

    with ctx.pose({base_to_lower: 0.0, lower_to_middle: 0.7, middle_to_top: 0.0}):
        lower_pointer_same = _aabb_center(ctx.part_element_world_aabb(lower_stage, elem="lower_pointer"))
        middle_pointer_rotated = _aabb_center(ctx.part_element_world_aabb(middle_stage, elem="middle_pointer"))
    lower_drift = _xy_distance(lower_pointer_rest, lower_pointer_same)
    middle_shift = _xy_distance(middle_pointer_rest, middle_pointer_rotated)
    ctx.check(
        "middle ring rotates independently above the lower stage",
        lower_drift is not None
        and middle_shift is not None
        and lower_drift < 0.002
        and middle_shift > 0.04,
        details=(
            f"lower_rest={lower_pointer_rest}, lower_pose={lower_pointer_same}, lower_drift={lower_drift}, "
            f"middle_rest={middle_pointer_rest}, middle_pose={middle_pointer_rotated}, middle_shift={middle_shift}"
        ),
    )

    with ctx.pose({base_to_lower: 0.0, lower_to_middle: 0.0, middle_to_top: 0.9}):
        middle_pointer_same = _aabb_center(ctx.part_element_world_aabb(middle_stage, elem="middle_pointer"))
        top_pointer_rotated = _aabb_center(ctx.part_element_world_aabb(top_stage, elem="top_pointer"))
    middle_drift = _xy_distance(middle_pointer_rest, middle_pointer_same)
    top_shift = _xy_distance(top_pointer_rest, top_pointer_rotated)
    ctx.check(
        "top flange rotates independently above the middle ring",
        middle_drift is not None
        and top_shift is not None
        and middle_drift < 0.002
        and top_shift > 0.03,
        details=(
            f"middle_rest={middle_pointer_rest}, middle_pose={middle_pointer_same}, middle_drift={middle_drift}, "
            f"top_rest={top_pointer_rest}, top_pose={top_pointer_rotated}, top_shift={top_shift}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
