from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
)


def _rect_loop(
    width: float,
    depth: float,
    z: float,
    *,
    center_x: float = 0.0,
    center_y: float = 0.0,
):
    half_w = width / 2.0
    half_d = depth / 2.0
    return (
        (center_x - half_w, center_y - half_d, z),
        (center_x + half_w, center_y - half_d, z),
        (center_x + half_w, center_y + half_d, z),
        (center_x - half_w, center_y + half_d, z),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_range_hood")

    brushed_steel = model.material("brushed_steel", rgba=(0.72, 0.74, 0.76, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.62, 0.64, 0.67, 1.0))
    charcoal = model.material("charcoal", rgba=(0.18, 0.19, 0.20, 1.0))
    soft_black = model.material("soft_black", rgba=(0.10, 0.10, 0.11, 1.0))

    hood_body = model.part("hood_body")

    tapered_shell = section_loft(
        [
            _rect_loop(0.894, 0.477, 0.042, center_y=-0.006),
            _rect_loop(0.700, 0.305, 0.258, center_y=-0.092),
        ]
    )
    hood_body.visual(
        mesh_from_geometry(tapered_shell, "tapered_shell"),
        material=brushed_steel,
        name="tapered_shell",
    )

    hood_body.visual(
        Box((0.860, 0.454, 0.012)),
        origin=Origin(xyz=(0.0, -0.004, 0.054)),
        material=satin_steel,
        name="canopy_top",
    )
    hood_body.visual(
        Box((0.018, 0.454, 0.050)),
        origin=Origin(xyz=(-0.421, -0.004, 0.025)),
        material=satin_steel,
        name="left_skirt",
    )
    hood_body.visual(
        Box((0.018, 0.454, 0.050)),
        origin=Origin(xyz=(0.421, -0.004, 0.025)),
        material=satin_steel,
        name="right_skirt",
    )
    hood_body.visual(
        Box((0.860, 0.018, 0.050)),
        origin=Origin(xyz=(0.0, -0.222, 0.025)),
        material=satin_steel,
        name="rear_skirt",
    )
    hood_body.visual(
        Box((0.860, 0.024, 0.060)),
        origin=Origin(xyz=(0.0, 0.238, 0.030)),
        material=satin_steel,
        name="control_strip",
    )
    hood_body.visual(
        Box((0.826, 0.432, 0.006)),
        origin=Origin(xyz=(0.0, 0.002, 0.023)),
        material=charcoal,
        name="baffle_filter",
    )

    knob_positions = {
        "left": -0.190,
        "center": 0.0,
        "right": 0.190,
    }

    for label, x_pos in knob_positions.items():
        knob = model.part(f"{label}_knob")
        knob.visual(
            Cylinder(radius=0.0185, length=0.006),
            origin=Origin(xyz=(0.0, 0.003, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=charcoal,
            name="bezel",
        )
        knob.visual(
            Cylinder(radius=0.0160, length=0.020),
            origin=Origin(xyz=(0.0, 0.015, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=soft_black,
            name="grip",
        )
        knob.visual(
            Box((0.003, 0.004, 0.014)),
            origin=Origin(xyz=(0.0, 0.026, 0.010)),
            material=satin_steel,
            name="indicator",
        )

        model.articulation(
            f"hood_to_{label}_knob",
            ArticulationType.CONTINUOUS,
            parent=hood_body,
            child=knob,
            origin=Origin(xyz=(x_pos, 0.250, 0.032)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.4, velocity=8.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    hood_body = object_model.get_part("hood_body")
    control_strip = hood_body.get_visual("control_strip")
    left_knob = object_model.get_part("left_knob")
    center_knob = object_model.get_part("center_knob")
    right_knob = object_model.get_part("right_knob")
    left_joint = object_model.get_articulation("hood_to_left_knob")
    center_joint = object_model.get_articulation("hood_to_center_knob")
    right_joint = object_model.get_articulation("hood_to_right_knob")

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
    ctx.fail_if_parts_overlap_in_current_pose()

    for knob in (left_knob, center_knob, right_knob):
        ctx.expect_contact(
            knob,
            hood_body,
            elem_b=control_strip,
            contact_tol=1e-6,
            name=f"{knob.name}_mounted_to_control_strip",
        )
        ctx.expect_gap(
            knob,
            hood_body,
            axis="y",
            negative_elem=control_strip,
            max_gap=0.0005,
            max_penetration=0.0,
            name=f"{knob.name}_flush_to_control_strip",
        )
        ctx.expect_overlap(
            knob,
            hood_body,
            axes="xz",
            elem_b=control_strip,
            min_overlap=0.020,
            name=f"{knob.name}_overlaps_control_strip_footprint",
        )

    ctx.expect_origin_distance(
        left_knob,
        center_knob,
        axes="x",
        min_dist=0.17,
        max_dist=0.21,
        name="left_center_knob_spacing",
    )
    ctx.expect_origin_distance(
        center_knob,
        right_knob,
        axes="x",
        min_dist=0.17,
        max_dist=0.21,
        name="center_right_knob_spacing",
    )

    for joint in (left_joint, center_joint, right_joint):
        limits = joint.motion_limits
        axis = joint.axis
        is_continuous = joint.articulation_type == ArticulationType.CONTINUOUS
        axis_ok = (
            abs(axis[0]) < 1e-9
            and abs(abs(axis[1]) - 1.0) < 1e-9
            and abs(axis[2]) < 1e-9
        )
        limits_ok = (
            limits is not None
            and limits.lower is None
            and limits.upper is None
            and limits.effort > 0.0
            and limits.velocity > 0.0
        )
        ctx.check(
            f"{joint.name}_is_continuous_front_to_back",
            is_continuous and axis_ok and limits_ok,
            details=(
                f"type={joint.articulation_type}, axis={axis}, "
                f"limits={limits}"
            ),
        )

    with ctx.pose({left_joint: 1.3, center_joint: 2.7, right_joint: -0.9}):
        for knob in (left_knob, center_knob, right_knob):
            ctx.expect_gap(
                knob,
                hood_body,
                axis="y",
                negative_elem=control_strip,
                max_gap=0.0005,
                max_penetration=0.0,
                name=f"{knob.name}_flush_when_rotated",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
