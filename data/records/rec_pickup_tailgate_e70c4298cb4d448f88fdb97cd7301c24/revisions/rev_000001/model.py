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
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_body_side_swing_tailgate")

    body_white = model.material("service_body_white", rgba=(0.93, 0.94, 0.95, 1.0))
    gate_gray = model.material("tailgate_gray", rgba=(0.72, 0.74, 0.76, 1.0))
    hinge_gray = model.material("hinge_gray", rgba=(0.57, 0.59, 0.62, 1.0))
    trim_black = model.material("trim_black", rgba=(0.10, 0.11, 0.12, 1.0))

    outer_width = 1.76
    body_depth = 0.56
    body_height = 0.82
    stub_width = 0.18
    sill_height = 0.15
    header_height = 0.15
    opening_width = outer_width - 2.0 * stub_width
    opening_height = body_height - sill_height - header_height

    panel_thickness = 0.04
    panel_width = opening_width - 0.006
    panel_height = opening_height - 0.006
    panel_hinge_offset = 0.025
    side_gap = 0.003

    opening_left = -opening_width / 2.0
    opening_right = opening_width / 2.0
    opening_bottom = sill_height
    opening_top = sill_height + opening_height
    tailgate_origin_x = opening_left + side_gap - panel_hinge_offset

    service_body = model.part("service_body")
    service_body.visual(
        Box((stub_width, body_depth, body_height)),
        origin=Origin(
            xyz=(
                -outer_width / 2.0 + stub_width / 2.0,
                -body_depth / 2.0,
                body_height / 2.0,
            )
        ),
        material=body_white,
        name="left_stub",
    )
    service_body.visual(
        Box((stub_width, body_depth, body_height)),
        origin=Origin(
            xyz=(
                outer_width / 2.0 - stub_width / 2.0,
                -body_depth / 2.0,
                body_height / 2.0,
            )
        ),
        material=body_white,
        name="right_stub",
    )
    service_body.visual(
        Box((outer_width, body_depth, sill_height)),
        origin=Origin(xyz=(0.0, -body_depth / 2.0, sill_height / 2.0)),
        material=body_white,
        name="bed_floor",
    )
    service_body.visual(
        Box((outer_width, body_depth, header_height)),
        origin=Origin(
            xyz=(0.0, -body_depth / 2.0, opening_top + header_height / 2.0)
        ),
        material=body_white,
        name="top_header",
    )

    hinge_zs = (
        opening_bottom + opening_height * 0.26,
        opening_bottom + opening_height * 0.74,
    )
    for index, hinge_z in enumerate(hinge_zs, start=1):
        service_body.visual(
            Box((0.024, 0.058, 0.11)),
            origin=Origin(xyz=(tailgate_origin_x, 0.029, hinge_z)),
            material=hinge_gray,
            name=f"hinge_strap_{index}",
        )
        service_body.visual(
            Cylinder(radius=0.018, length=0.12),
            origin=Origin(xyz=(tailgate_origin_x, 0.058, hinge_z)),
            material=hinge_gray,
            name=f"hinge_barrel_{index}",
        )

    service_body.inertial = Inertial.from_geometry(
        Box((outer_width, body_depth, body_height)),
        mass=185.0,
        origin=Origin(xyz=(0.0, -body_depth / 2.0, body_height / 2.0)),
    )

    tailgate = model.part("tailgate")
    stile_width = 0.055
    rail_height = 0.055
    inner_width = panel_width - 2.0 * stile_width
    inner_height = panel_height - 2.0 * rail_height

    tailgate.visual(
        Box((stile_width, panel_thickness, panel_height)),
        origin=Origin(xyz=(panel_hinge_offset + stile_width / 2.0, 0.0, 0.0)),
        material=gate_gray,
        name="hinge_stile",
    )
    tailgate.visual(
        Box((stile_width, panel_thickness, panel_height)),
        origin=Origin(
            xyz=(panel_hinge_offset + panel_width - stile_width / 2.0, 0.0, 0.0)
        ),
        material=gate_gray,
        name="latch_stile",
    )
    tailgate.visual(
        Box((inner_width, panel_thickness, rail_height)),
        origin=Origin(
            xyz=(
                panel_hinge_offset + stile_width + inner_width / 2.0,
                0.0,
                panel_height / 2.0 - rail_height / 2.0,
            )
        ),
        material=gate_gray,
        name="top_rail",
    )
    tailgate.visual(
        Box((inner_width, panel_thickness, rail_height)),
        origin=Origin(
            xyz=(
                panel_hinge_offset + stile_width + inner_width / 2.0,
                0.0,
                -panel_height / 2.0 + rail_height / 2.0,
            )
        ),
        material=gate_gray,
        name="bottom_rail",
    )
    tailgate.visual(
        Box((inner_width, 0.014, inner_height)),
        origin=Origin(
            xyz=(
                panel_hinge_offset + stile_width + inner_width / 2.0,
                panel_thickness / 2.0 - 0.007,
                0.0,
            )
        ),
        material=gate_gray,
        name="outer_skin",
    )
    tailgate.visual(
        Box((0.045, 0.026, 0.14)),
        origin=Origin(
            xyz=(panel_hinge_offset + panel_width - 0.0225, 0.0, 0.0)
        ),
        material=hinge_gray,
        name="latch_case",
    )
    tailgate.visual(
        Box((0.013, 0.024, 0.12)),
        origin=Origin(
            xyz=(
                panel_hinge_offset - 0.0065,
                0.0,
                opening_height * 0.24 - panel_height / 2.0,
            )
        ),
        material=hinge_gray,
        name="lower_leaf",
    )
    tailgate.visual(
        Box((0.013, 0.024, 0.12)),
        origin=Origin(
            xyz=(
                panel_hinge_offset - 0.0065,
                0.0,
                opening_height * 0.76 - panel_height / 2.0,
            )
        ),
        material=hinge_gray,
        name="upper_leaf",
    )
    tailgate.inertial = Inertial.from_geometry(
        Box((panel_width, panel_thickness, panel_height)),
        mass=24.0,
        origin=Origin(xyz=(panel_hinge_offset + panel_width / 2.0, 0.0, 0.0)),
    )

    latch_handle = model.part("latch_handle")
    latch_handle.visual(
        Cylinder(radius=0.012, length=0.028),
        origin=Origin(xyz=(0.0, 0.014, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hinge_gray,
        name="hub",
    )
    latch_handle.visual(
        Box((0.05, 0.014, 0.024)),
        origin=Origin(xyz=(-0.027, 0.023, 0.0)),
        material=trim_black,
        name="arm",
    )
    latch_handle.visual(
        Box((0.022, 0.018, 0.14)),
        origin=Origin(xyz=(-0.05, 0.028, 0.0)),
        material=trim_black,
        name="grip",
    )
    latch_handle.inertial = Inertial.from_geometry(
        Box((0.08, 0.04, 0.14)),
        mass=0.8,
        origin=Origin(xyz=(-0.02, 0.02, 0.0)),
    )

    model.articulation(
        "tailgate_hinge",
        ArticulationType.REVOLUTE,
        parent=service_body,
        child=tailgate,
        origin=Origin(
            xyz=(
                tailgate_origin_x,
                panel_thickness / 2.0,
                opening_bottom + opening_height / 2.0,
            )
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.5,
            lower=0.0,
            upper=1.95,
        ),
    )
    model.articulation(
        "handle_pivot",
        ArticulationType.REVOLUTE,
        parent=tailgate,
        child=latch_handle,
        origin=Origin(
            xyz=(
                panel_hinge_offset + panel_width - 0.06,
                panel_thickness / 2.0,
                0.0,
            )
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=3.0,
            lower=-0.65,
            upper=0.65,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    service_body = object_model.get_part("service_body")
    tailgate = object_model.get_part("tailgate")
    latch_handle = object_model.get_part("latch_handle")
    tailgate_hinge = object_model.get_articulation("tailgate_hinge")
    handle_pivot = object_model.get_articulation("handle_pivot")

    left_stub = service_body.get_visual("left_stub")
    right_stub = service_body.get_visual("right_stub")
    bed_floor = service_body.get_visual("bed_floor")
    top_header = service_body.get_visual("top_header")
    hinge_stile = tailgate.get_visual("hinge_stile")

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
        "tailgate_hinge_axis_vertical",
        tuple(tailgate_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"expected vertical hinge axis, got {tailgate_hinge.axis}",
    )
    ctx.check(
        "handle_axis_through_panel",
        tuple(handle_pivot.axis) == (0.0, 1.0, 0.0),
        details=f"expected handle pivot around panel-normal axis, got {handle_pivot.axis}",
    )
    tailgate_limits = tailgate_hinge.motion_limits
    handle_limits = handle_pivot.motion_limits
    ctx.check(
        "tailgate_hinge_has_side_swing_range",
        tailgate_limits is not None
        and tailgate_limits.lower == 0.0
        and tailgate_limits.upper is not None
        and tailgate_limits.upper >= 1.57,
        details="tailgate should swing outward on a side hinge with at least a quarter-turn of travel",
    )
    ctx.check(
        "handle_has_short_latch_throw",
        handle_limits is not None
        and handle_limits.lower is not None
        and handle_limits.upper is not None
        and handle_limits.lower < 0.0 < handle_limits.upper
        and handle_limits.upper <= 1.0,
        details="latch handle should have a short local rotation range, not a full door swing",
    )

    with ctx.pose({tailgate_hinge: 0.0, handle_pivot: 0.0}):
        ctx.expect_gap(
            tailgate,
            service_body,
            axis="x",
            positive_elem=hinge_stile,
            negative_elem=left_stub,
            min_gap=0.001,
            max_gap=0.006,
            name="tailgate_left_edge_clearance",
        )
        ctx.expect_gap(
            service_body,
            tailgate,
            axis="x",
            positive_elem=right_stub,
            min_gap=0.001,
            max_gap=0.006,
            name="tailgate_right_edge_clearance",
        )
        ctx.expect_gap(
            tailgate,
            service_body,
            axis="z",
            negative_elem=bed_floor,
            min_gap=0.001,
            max_gap=0.006,
            name="tailgate_bottom_clearance",
        )
        ctx.expect_gap(
            service_body,
            tailgate,
            axis="z",
            positive_elem=top_header,
            min_gap=0.001,
            max_gap=0.006,
            name="tailgate_top_clearance",
        )
        ctx.expect_gap(
            tailgate,
            service_body,
            axis="y",
            negative_elem=left_stub,
            min_gap=0.0,
            max_gap=0.001,
            name="tailgate_closed_against_rear_plane",
        )
        ctx.expect_contact(
            latch_handle,
            tailgate,
            contact_tol=1e-6,
            name="handle_mounts_on_tailgate_face",
        )
        ctx.expect_gap(
            service_body,
            latch_handle,
            axis="x",
            positive_elem=right_stub,
            min_gap=0.03,
            max_gap=0.12,
            name="handle_sits_at_free_edge",
        )

    with ctx.pose({tailgate_hinge: math.pi / 2.0, handle_pivot: 0.0}):
        ctx.expect_gap(
            tailgate,
            service_body,
            axis="y",
            negative_elem=left_stub,
            min_gap=0.03,
            name="open_tailgate_clears_body_depth",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
