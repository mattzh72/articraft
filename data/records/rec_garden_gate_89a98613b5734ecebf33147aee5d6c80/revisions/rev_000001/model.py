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


TRACK_TOP_Z = 0.10
GATE_WIDTH = 3.45
GATE_CENTER_X_CLOSED = -0.24
GATE_CENTER_Z = 0.905
FRONT_STILE_X = (GATE_WIDTH / 2.0) - 0.03
HANDLE_PIVOT_Z = 0.95
LATCH_POST_X = 1.55
REAR_POST_X = -1.55
POST_Y = -0.12


def add_box(
    part,
    name: str,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    *,
    material=None,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
):
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def add_cylinder_y(
    part,
    name: str,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    *,
    material=None,
):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sliding_garden_gate")

    powder_black = model.material("powder_black", rgba=(0.15, 0.16, 0.17, 1.0))
    galvanized = model.material("galvanized", rgba=(0.69, 0.71, 0.73, 1.0))
    concrete = model.material("concrete", rgba=(0.72, 0.72, 0.70, 1.0))
    rubber = model.material("roller_rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    handle_finish = model.material("handle_finish", rgba=(0.22, 0.23, 0.24, 1.0))

    site_frame = model.part("site_frame")
    add_box(
        site_frame,
        "concrete_strip",
        (5.70, 0.46, 0.08),
        (0.0, -0.02, 0.04),
        material=concrete,
    )
    add_box(
        site_frame,
        "ground_track",
        (5.40, 0.06, 0.02),
        (0.0, 0.0, 0.09),
        material=galvanized,
    )
    add_box(
        site_frame,
        "track_stop_rear",
        (0.06, 0.09, 0.07),
        (-2.67, 0.0, 0.115),
        material=galvanized,
    )
    add_box(
        site_frame,
        "track_stop_front",
        (0.06, 0.09, 0.07),
        (2.67, 0.0, 0.115),
        material=galvanized,
    )

    latch_post = model.part("latch_post")
    add_box(
        latch_post,
        "baseplate",
        (0.18, 0.18, 0.015),
        (LATCH_POST_X, POST_Y, 0.0875),
        material=galvanized,
    )
    add_box(
        latch_post,
        "post_shaft",
        (0.09, 0.09, 1.72),
        (LATCH_POST_X, POST_Y, 0.955),
        material=powder_black,
    )
    add_box(
        latch_post,
        "receiver_plate",
        (0.02, 0.10, 0.12),
        (1.515, -0.03, 0.94),
        material=galvanized,
    )

    rear_guide_post = model.part("rear_guide_post")
    add_box(
        rear_guide_post,
        "baseplate",
        (0.18, 0.18, 0.015),
        (REAR_POST_X, POST_Y, 0.0875),
        material=galvanized,
    )
    add_box(
        rear_guide_post,
        "post_shaft",
        (0.09, 0.09, 1.72),
        (REAR_POST_X, POST_Y, 0.955),
        material=powder_black,
    )
    add_box(
        rear_guide_post,
        "guide_bracket",
        (0.14, 0.08, 0.05),
        (-1.49, -0.10, 1.56),
        material=galvanized,
    )
    add_box(
        rear_guide_post,
        "guide_strap",
        (0.05, 0.08, 0.24),
        (-1.505, -0.10, 1.43),
        material=galvanized,
    )
    add_box(
        rear_guide_post,
        "front_roller_arm",
        (0.04, 0.15, 0.04),
        (-1.485, -0.025, 1.43),
        material=galvanized,
    )
    add_box(
        rear_guide_post,
        "front_roller_upright",
        (0.04, 0.04, 0.12),
        (-1.45, 0.05, 1.49),
        material=galvanized,
    )
    rear_guide_post.visual(
        Cylinder(radius=0.03, length=0.05),
        origin=Origin(xyz=(-1.43, -0.05, 1.56)),
        material=rubber,
        name="guide_roller_back",
    )
    rear_guide_post.visual(
        Cylinder(radius=0.03, length=0.05),
        origin=Origin(xyz=(-1.43, 0.05, 1.56)),
        material=rubber,
        name="guide_roller_front",
    )

    gate_leaf = model.part("gate_leaf")
    add_box(
        gate_leaf,
        "front_stile",
        (0.06, 0.04, 1.45),
        (FRONT_STILE_X, 0.0, GATE_CENTER_Z),
        material=powder_black,
    )
    add_box(
        gate_leaf,
        "rear_stile",
        (0.06, 0.04, 1.45),
        (-FRONT_STILE_X, 0.0, GATE_CENTER_Z),
        material=powder_black,
    )
    add_box(
        gate_leaf,
        "bottom_beam",
        (GATE_WIDTH, 0.06, 0.10),
        (0.0, 0.0, 0.23),
        material=powder_black,
    )
    add_box(
        gate_leaf,
        "top_rail",
        (GATE_WIDTH, 0.04, 0.06),
        (0.0, 0.0, 1.60),
        material=powder_black,
    )
    add_box(
        gate_leaf,
        "mid_rail",
        (GATE_WIDTH - 0.12, 0.03, 0.05),
        (0.0, 0.0, 0.93),
        material=powder_black,
    )

    slat_x_positions = (-1.32, -0.93, -0.54, -0.15, 0.24, 0.63, 1.02, 1.41)
    for index, slat_x in enumerate(slat_x_positions, start=1):
        add_box(
            gate_leaf,
            f"slat_{index}",
            (0.03, 0.02, 1.29),
            (slat_x, 0.0, 0.925),
            material=galvanized,
        )

    carriage_x_positions = (-0.85, 0.55)
    for index, carriage_x in enumerate(carriage_x_positions, start=1):
        add_box(
            gate_leaf,
            f"carriage_block_{index}",
            (0.16, 0.05, 0.05),
            (carriage_x, 0.0, 0.165),
            material=galvanized,
        )
        add_cylinder_y(
            gate_leaf,
            f"roller_wheel_{index}",
            radius=0.045,
            length=0.03,
            xyz=(carriage_x, 0.0, TRACK_TOP_Z + 0.045),
            material=rubber,
        )

    add_box(
        gate_leaf,
        "handle_mount_plate",
        (0.09, 0.008, 0.18),
        (FRONT_STILE_X - 0.005, 0.024, HANDLE_PIVOT_Z),
        material=galvanized,
    )

    latch_handle = model.part("latch_handle")
    latch_handle.visual(
        Cylinder(radius=0.015, length=0.03),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=handle_finish,
        name="handle_hub",
    )
    add_box(
        latch_handle,
        "handle_stem",
        (0.11, 0.018, 0.045),
        (0.055, 0.022, -0.025),
        material=handle_finish,
    )
    add_box(
        latch_handle,
        "handle_grip",
        (0.03, 0.020, 0.07),
        (0.105, 0.024, -0.06),
        material=handle_finish,
    )

    model.articulation(
        "site_to_latch_post",
        ArticulationType.FIXED,
        parent=site_frame,
        child=latch_post,
    )
    model.articulation(
        "site_to_rear_guide_post",
        ArticulationType.FIXED,
        parent=site_frame,
        child=rear_guide_post,
    )
    model.articulation(
        "gate_slide",
        ArticulationType.PRISMATIC,
        parent=site_frame,
        child=gate_leaf,
        origin=Origin(xyz=(GATE_CENTER_X_CLOSED, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=500.0,
            velocity=0.6,
            lower=-1.55,
            upper=0.0,
        ),
    )
    model.articulation(
        "handle_pivot",
        ArticulationType.REVOLUTE,
        parent=gate_leaf,
        child=latch_handle,
        origin=Origin(xyz=(FRONT_STILE_X - 0.005, 0.043, HANDLE_PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=4.0,
            lower=-0.65,
            upper=0.65,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    site_frame = object_model.get_part("site_frame")
    latch_post = object_model.get_part("latch_post")
    rear_guide_post = object_model.get_part("rear_guide_post")
    gate_leaf = object_model.get_part("gate_leaf")
    latch_handle = object_model.get_part("latch_handle")

    slide = object_model.get_articulation("gate_slide")
    handle_pivot = object_model.get_articulation("handle_pivot")

    front_stile = gate_leaf.get_visual("front_stile")
    handle_mount_plate = gate_leaf.get_visual("handle_mount_plate")
    handle_hub = latch_handle.get_visual("handle_hub")
    receiver_plate = latch_post.get_visual("receiver_plate")
    post_shaft = latch_post.get_visual("post_shaft")

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

    for part_name in (
        "site_frame",
        "latch_post",
        "rear_guide_post",
        "gate_leaf",
        "latch_handle",
    ):
        ctx.check(
            f"part_present_{part_name}",
            object_model.get_part(part_name) is not None,
            f"Expected part '{part_name}' to exist.",
        )

    ctx.check(
        "gate_slide_axis_x",
        tuple(slide.axis) == (1.0, 0.0, 0.0),
        f"Expected gate_slide axis to align with +x, got {slide.axis}.",
    )
    ctx.check(
        "handle_pivot_axis_y",
        tuple(handle_pivot.axis) == (0.0, 1.0, 0.0),
        f"Expected handle_pivot axis to align with +y, got {handle_pivot.axis}.",
    )

    ctx.expect_contact(latch_post, site_frame, contact_tol=1e-6, name="latch_post_mounted")
    ctx.expect_contact(
        rear_guide_post,
        site_frame,
        contact_tol=1e-6,
        name="rear_guide_post_mounted",
    )

    with ctx.pose({slide: 0.0, handle_pivot: 0.0}):
        ctx.expect_contact(gate_leaf, site_frame, contact_tol=1e-6, name="gate_on_track")
        ctx.expect_contact(
            gate_leaf,
            rear_guide_post,
            contact_tol=1e-6,
            name="gate_in_top_guides",
        )
        ctx.expect_gap(
            latch_post,
            gate_leaf,
            axis="x",
            min_gap=0.01,
            max_gap=0.03,
            positive_elem=receiver_plate,
            negative_elem=front_stile,
            name="closed_gate_front_gap_to_latch_post",
        )
        ctx.expect_overlap(
            latch_post,
            gate_leaf,
            axes="z",
            min_overlap=1.0,
            elem_a=post_shaft,
            elem_b=front_stile,
            name="latch_post_and_front_stile_vertical_alignment",
        )

    with ctx.pose({slide: -1.20}):
        ctx.expect_gap(
            latch_post,
            gate_leaf,
            axis="x",
            min_gap=1.18,
            positive_elem=receiver_plate,
            negative_elem=front_stile,
            name="opened_gate_clears_latch_post",
        )
        ctx.expect_contact(gate_leaf, site_frame, contact_tol=1e-6, name="gate_stays_on_track_open")

    with ctx.pose({handle_pivot: 0.45}):
        ctx.expect_contact(
            latch_handle,
            gate_leaf,
            contact_tol=0.001,
            elem_a=handle_hub,
            elem_b=handle_mount_plate,
            name="handle_remains_mounted_when_rotated",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
