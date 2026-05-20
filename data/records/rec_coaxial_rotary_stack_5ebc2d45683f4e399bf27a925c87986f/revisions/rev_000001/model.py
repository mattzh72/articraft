from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_RADIUS = 0.120
BASE_HEIGHT = 0.026

STAGE_BORE_RADIUS = 0.0075
INTER_STAGE_GAP = 0.009

LOWER_SUPPORT_RADIUS = 0.020
LOWER_SUPPORT_THICKNESS = 0.0035
MIDDLE_SUPPORT_RADIUS = 0.0155
MIDDLE_SUPPORT_THICKNESS = 0.0035
UPPER_SUPPORT_RADIUS = 0.0115
UPPER_SUPPORT_THICKNESS = 0.003

LOWER_STAGE_OUTER_RADIUS = 0.095
LOWER_STAGE_RING_INNER_RADIUS = 0.060
LOWER_STAGE_HUB_RADIUS = 0.028
LOWER_STAGE_THICKNESS = 0.012
LOWER_STAGE_Z = LOWER_SUPPORT_THICKNESS

MIDDLE_STAGE_OUTER_RADIUS = 0.068
MIDDLE_STAGE_RING_INNER_RADIUS = 0.040
MIDDLE_STAGE_HUB_RADIUS = 0.0205
MIDDLE_STAGE_THICKNESS = 0.010
MIDDLE_STAGE_Z = LOWER_STAGE_Z + LOWER_STAGE_THICKNESS + INTER_STAGE_GAP

UPPER_STAGE_OUTER_RADIUS = 0.044
UPPER_STAGE_THICKNESS = 0.007
UPPER_STAGE_Z = MIDDLE_STAGE_Z + MIDDLE_STAGE_THICKNESS + INTER_STAGE_GAP

SPINE_SHAFT_RADIUS = 0.006
SPINE_HEIGHT = UPPER_STAGE_Z + UPPER_STAGE_THICKNESS + 0.004


def _cylinder(radius: float, height: float, z0: float = 0.0) -> cq.Workplane:
    return cq.Workplane("XY").workplane(offset=z0).circle(radius).extrude(height)


def _build_base_shape() -> cq.Workplane:
    return _cylinder(BASE_RADIUS, BASE_HEIGHT)


def _build_spine_shape() -> cq.Workplane:
    spine = _cylinder(SPINE_SHAFT_RADIUS, SPINE_HEIGHT)
    spine = spine.union(_cylinder(LOWER_SUPPORT_RADIUS, LOWER_SUPPORT_THICKNESS, 0.0))
    spine = spine.union(
        _cylinder(
            MIDDLE_SUPPORT_RADIUS,
            MIDDLE_SUPPORT_THICKNESS,
            MIDDLE_STAGE_Z - MIDDLE_SUPPORT_THICKNESS,
        )
    )
    spine = spine.union(
        _cylinder(
            UPPER_SUPPORT_RADIUS,
            UPPER_SUPPORT_THICKNESS,
            UPPER_STAGE_Z - UPPER_SUPPORT_THICKNESS,
        )
    )
    return spine


def _build_spoked_stage_shape(
    outer_radius: float,
    ring_inner_radius: float,
    hub_radius: float,
    bore_radius: float,
    thickness: float,
    *,
    spoke_width: float,
    spoke_count: int,
) -> cq.Workplane:
    stage = cq.Workplane("XY").circle(outer_radius).circle(ring_inner_radius).extrude(thickness)
    stage = (
        stage.union(
            cq.Workplane("XY").circle(hub_radius).circle(bore_radius).extrude(thickness)
        )
    )

    spoke_overlap = 0.002
    spoke_span = ring_inner_radius - hub_radius
    spoke_length = spoke_span + (2.0 * spoke_overlap)
    spoke_center_x = hub_radius + (spoke_span * 0.5)
    for index in range(spoke_count):
        angle_deg = index * (360.0 / spoke_count)
        spoke = (
            cq.Workplane("XY")
            .center(spoke_center_x, 0.0)
            .rect(spoke_length, spoke_width)
            .extrude(thickness)
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg)
        )
        stage = stage.union(spoke)

    return stage


def _build_faceplate_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(UPPER_STAGE_OUTER_RADIUS)
        .circle(STAGE_BORE_RADIUS)
        .extrude(UPPER_STAGE_THICKNESS)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="coaxial_turntable_module")

    model.material("base_anodized", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("spine_steel", rgba=(0.42, 0.44, 0.47, 1.0))
    model.material("lower_ring_finish", rgba=(0.70, 0.72, 0.75, 1.0))
    model.material("middle_ring_finish", rgba=(0.60, 0.63, 0.67, 1.0))
    model.material("upper_faceplate_finish", rgba=(0.82, 0.83, 0.85, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_build_base_shape(), "base_body"),
        material="base_anodized",
        name="base_body",
    )

    spine = model.part("spine")
    spine.visual(
        mesh_from_cadquery(_build_spine_shape(), "spine_body"),
        material="spine_steel",
        name="spine_body",
    )

    lower_stage = model.part("lower_stage")
    lower_stage.visual(
        mesh_from_cadquery(
            _build_spoked_stage_shape(
                LOWER_STAGE_OUTER_RADIUS,
                LOWER_STAGE_RING_INNER_RADIUS,
                LOWER_STAGE_HUB_RADIUS,
                STAGE_BORE_RADIUS,
                LOWER_STAGE_THICKNESS,
                spoke_width=0.012,
                spoke_count=3,
            ),
            "lower_stage",
        ),
        material="lower_ring_finish",
        name="lower_stage_shell",
    )

    middle_stage = model.part("middle_stage")
    middle_stage.visual(
        mesh_from_cadquery(
            _build_spoked_stage_shape(
                MIDDLE_STAGE_OUTER_RADIUS,
                MIDDLE_STAGE_RING_INNER_RADIUS,
                MIDDLE_STAGE_HUB_RADIUS,
                STAGE_BORE_RADIUS,
                MIDDLE_STAGE_THICKNESS,
                spoke_width=0.009,
                spoke_count=3,
            ),
            "middle_stage",
        ),
        material="middle_ring_finish",
        name="middle_stage_shell",
    )

    upper_stage = model.part("upper_stage")
    upper_stage.visual(
        mesh_from_cadquery(
            _build_faceplate_shape(),
            "upper_stage",
        ),
        material="upper_faceplate_finish",
        name="upper_faceplate",
    )

    model.articulation(
        "base_to_spine",
        ArticulationType.FIXED,
        parent=base,
        child=spine,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT)),
    )
    model.articulation(
        "spine_to_lower_stage",
        ArticulationType.REVOLUTE,
        parent=spine,
        child=lower_stage,
        origin=Origin(xyz=(0.0, 0.0, LOWER_STAGE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=2.5,
            lower=-pi,
            upper=pi,
        ),
    )
    model.articulation(
        "spine_to_middle_stage",
        ArticulationType.REVOLUTE,
        parent=spine,
        child=middle_stage,
        origin=Origin(xyz=(0.0, 0.0, MIDDLE_STAGE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=3.0,
            lower=-pi,
            upper=pi,
        ),
    )
    model.articulation(
        "spine_to_upper_stage",
        ArticulationType.REVOLUTE,
        parent=spine,
        child=upper_stage,
        origin=Origin(xyz=(0.0, 0.0, UPPER_STAGE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=3.5,
            lower=-pi,
            upper=pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    spine = object_model.get_part("spine")
    lower_stage = object_model.get_part("lower_stage")
    middle_stage = object_model.get_part("middle_stage")
    upper_stage = object_model.get_part("upper_stage")
    lower_joint = object_model.get_articulation("spine_to_lower_stage")
    middle_joint = object_model.get_articulation("spine_to_middle_stage")
    upper_joint = object_model.get_articulation("spine_to_upper_stage")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.allow_overlap(
        lower_stage,
        spine,
        reason=(
            "Coaxial shoulder seat intentionally nests at the lower ring hub; "
            "exact contact and stand-off checks below define the real support relationship."
        ),
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    def span(part) -> tuple[float, float, float]:
        aabb = ctx.part_world_aabb(part)
        assert aabb is not None
        minimum, maximum = aabb
        return (
            maximum[0] - minimum[0],
            maximum[1] - minimum[1],
            maximum[2] - minimum[2],
        )

    lower_span = span(lower_stage)
    middle_span = span(middle_stage)
    upper_span = span(upper_stage)

    joints = (lower_joint, middle_joint, upper_joint)
    shared_axis = all(joint.axis == (0.0, 0.0, 1.0) for joint in joints)
    centered_axis = all(
        abs(joint.origin.xyz[0]) < 1e-9 and abs(joint.origin.xyz[1]) < 1e-9 for joint in joints
    )

    ctx.check(
        "coaxial z-axis articulations",
        shared_axis and centered_axis,
        details=(
            f"axes={[joint.axis for joint in joints]} "
            f"origins={[joint.origin.xyz for joint in joints]}"
        ),
    )
    ctx.check(
        "stage diameters step down upward",
        lower_span[0] > middle_span[0] > upper_span[0],
        details=f"diameters={[lower_span[0], middle_span[0], upper_span[0]]}",
    )
    ctx.check(
        "stage thicknesses step down upward",
        lower_span[2] > middle_span[2] > upper_span[2],
        details=f"thicknesses={[lower_span[2], middle_span[2], upper_span[2]]}",
    )

    ctx.expect_origin_distance(spine, base, axes="xy", max_dist=1e-6, name="spine centered")
    ctx.expect_origin_distance(lower_stage, spine, axes="xy", max_dist=1e-6, name="lower centered")
    ctx.expect_origin_distance(middle_stage, spine, axes="xy", max_dist=1e-6, name="middle centered")
    ctx.expect_origin_distance(upper_stage, spine, axes="xy", max_dist=1e-6, name="upper centered")

    ctx.expect_contact(base, spine, name="spine mounted on base")
    ctx.expect_contact(lower_stage, spine, name="lower stage supported by spine")
    ctx.expect_contact(middle_stage, spine, name="middle stage supported by spine")
    ctx.expect_contact(upper_stage, spine, name="upper stage supported by spine")

    ctx.expect_gap(
        lower_stage,
        base,
        axis="z",
        min_gap=0.003,
        max_gap=0.005,
        name="lower stage stands off above base deck",
    )

    ctx.expect_gap(
        middle_stage,
        lower_stage,
        axis="z",
        min_gap=0.008,
        max_gap=0.012,
        name="middle stage clears lower stage",
    )
    ctx.expect_gap(
        upper_stage,
        middle_stage,
        axis="z",
        min_gap=0.008,
        max_gap=0.011,
        name="upper stage clears middle stage",
    )

    ctx.expect_within(
        middle_stage,
        lower_stage,
        axes="xy",
        margin=0.0,
        name="middle stage footprint sits within lower stage",
    )
    ctx.expect_within(
        upper_stage,
        middle_stage,
        axes="xy",
        margin=0.0,
        name="upper faceplate footprint sits within middle stage",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
