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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="casement_window")

    painted_wood = model.material("painted_wood", rgba=(0.94, 0.95, 0.93, 1.0))
    clear_glass = model.material("clear_glass", rgba=(0.72, 0.86, 0.94, 0.35))
    dark_metal = model.material("dark_metal", rgba=(0.23, 0.24, 0.26, 1.0))
    gasket_black = model.material("gasket_black", rgba=(0.08, 0.08, 0.09, 1.0))

    frame_width = 0.90
    frame_height = 1.20
    frame_depth = 0.12
    frame_member = 0.07
    opening_width = frame_width - 2.0 * frame_member
    opening_height = frame_height - 2.0 * frame_member

    sash_width = 0.744
    sash_height = 1.044
    sash_depth = 0.042
    sash_inset_from_hinge = 0.006
    stile_width = 0.06
    rail_height = 0.06

    hinge_leaf_thickness = 0.006
    hinge_leaf_depth = 0.032
    hinge_leaf_height = 0.15
    hinge_z = 0.35

    frame = model.part("frame")
    frame.visual(
        Box((frame_member, frame_depth, frame_height)),
        origin=Origin(xyz=(-(opening_width / 2.0 + frame_member / 2.0), 0.0, 0.0)),
        material=painted_wood,
        name="left_jamb",
    )
    frame.visual(
        Box((frame_member, frame_depth, frame_height)),
        origin=Origin(xyz=((opening_width / 2.0 + frame_member / 2.0), 0.0, 0.0)),
        material=painted_wood,
        name="right_jamb",
    )
    frame.visual(
        Box((opening_width, frame_depth, frame_member)),
        origin=Origin(xyz=(0.0, 0.0, opening_height / 2.0 + frame_member / 2.0)),
        material=painted_wood,
        name="head",
    )
    frame.visual(
        Box((opening_width, frame_depth, frame_member)),
        origin=Origin(xyz=(0.0, 0.0, -(opening_height / 2.0 + frame_member / 2.0))),
        material=painted_wood,
        name="sill",
    )
    frame.visual(
        Box((0.010, 0.018, 0.96)),
        origin=Origin(xyz=(opening_width / 2.0 - 0.005, 0.0, 0.0)),
        material=gasket_black,
        name="latch_stop",
    )
    for index, z in enumerate((hinge_z, -hinge_z), start=1):
        frame.visual(
            Box((hinge_leaf_thickness, hinge_leaf_depth, hinge_leaf_height)),
            origin=Origin(
                xyz=(
                    -opening_width / 2.0 - hinge_leaf_thickness / 2.0,
                    -0.012,
                    z,
                )
            ),
            material=dark_metal,
            name=f"frame_hinge_leaf_{index}",
        )

    sash = model.part("sash")
    sash.visual(
        Box((stile_width, sash_depth, sash_height)),
        origin=Origin(xyz=(sash_inset_from_hinge + stile_width / 2.0, 0.0, 0.0)),
        material=painted_wood,
        name="hinge_stile",
    )
    sash.visual(
        Box((stile_width, sash_depth, sash_height)),
        origin=Origin(
            xyz=(sash_inset_from_hinge + sash_width - stile_width / 2.0, 0.0, 0.0)
        ),
        material=painted_wood,
        name="right_stile",
    )
    sash.visual(
        Box((sash_width - 2.0 * stile_width, sash_depth, rail_height)),
        origin=Origin(
            xyz=(
                sash_inset_from_hinge + sash_width / 2.0,
                0.0,
                sash_height / 2.0 - rail_height / 2.0,
            )
        ),
        material=painted_wood,
        name="top_rail",
    )
    sash.visual(
        Box((sash_width - 2.0 * stile_width, sash_depth, rail_height)),
        origin=Origin(
            xyz=(
                sash_inset_from_hinge + sash_width / 2.0,
                0.0,
                -(sash_height / 2.0 - rail_height / 2.0),
            )
        ),
        material=painted_wood,
        name="bottom_rail",
    )
    sash.visual(
        Box((sash_width - 2.0 * stile_width, 0.012, sash_height - 2.0 * rail_height)),
        origin=Origin(xyz=(sash_inset_from_hinge + sash_width / 2.0, 0.0, 0.0)),
        material=clear_glass,
        name="glazing",
    )
    sash.visual(
        Box((0.032, 0.006, 0.16)),
        origin=Origin(
            xyz=(sash_inset_from_hinge + sash_width - stile_width / 2.0, 0.024, -0.02)
        ),
        material=dark_metal,
        name="handle_escutcheon",
    )
    for index, z in enumerate((hinge_z, -hinge_z), start=1):
        sash.visual(
            Box((hinge_leaf_thickness, hinge_leaf_depth, hinge_leaf_height)),
            origin=Origin(xyz=(hinge_leaf_thickness / 2.0, -0.012, z)),
            material=dark_metal,
            name=f"sash_hinge_leaf_{index}",
        )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.010, length=0.014),
        origin=Origin(xyz=(0.0, 0.007, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="hub",
    )
    handle.visual(
        Box((0.016, 0.012, 0.102)),
        origin=Origin(xyz=(0.0, 0.020, -0.052)),
        material=dark_metal,
        name="lever_bar",
    )
    handle.visual(
        Box((0.040, 0.014, 0.018)),
        origin=Origin(xyz=(0.0, 0.022, -0.103)),
        material=dark_metal,
        name="grip",
    )

    model.articulation(
        "frame_to_sash",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=sash,
        origin=Origin(xyz=(-opening_width / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.4,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "sash_to_handle",
        ArticulationType.REVOLUTE,
        parent=sash,
        child=handle,
        origin=Origin(
            xyz=(sash_inset_from_hinge + sash_width - stile_width / 2.0, 0.027, -0.02)
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=4.0,
            lower=-1.20,
            upper=0.30,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    sash = object_model.get_part("sash")
    handle = object_model.get_part("handle")
    sash_hinge = object_model.get_articulation("frame_to_sash")
    handle_pivot = object_model.get_articulation("sash_to_handle")

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

    ctx.check(
        "window_parts_present",
        len(object_model.parts) == 3,
        f"expected 3 parts, found {len(object_model.parts)}",
    )
    ctx.check(
        "sash_hinge_axis_vertical",
        tuple(round(v, 6) for v in sash_hinge.axis) == (0.0, 0.0, 1.0),
        f"unexpected sash hinge axis: {sash_hinge.axis}",
    )
    ctx.check(
        "handle_axis_normal_to_sash",
        tuple(round(v, 6) for v in handle_pivot.axis) == (0.0, 1.0, 0.0),
        f"unexpected handle axis: {handle_pivot.axis}",
    )

    with ctx.pose({sash_hinge: 0.0, handle_pivot: 0.0}):
        ctx.expect_contact(
            sash,
            frame,
            elem_a="right_stile",
            elem_b="latch_stop",
            name="closed_sash_seats_against_frame_stop",
        )
        ctx.expect_contact(
            handle,
            sash,
            elem_a="hub",
            elem_b="handle_escutcheon",
            name="handle_hub_is_mounted_to_escutcheon",
        )

    with ctx.pose({sash_hinge: 1.0}):
        ctx.expect_gap(
            sash,
            frame,
            axis="y",
            min_gap=0.20,
            positive_elem="right_stile",
            negative_elem="latch_stop",
            name="opened_sash_swings_clear_of_latch_stop",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
