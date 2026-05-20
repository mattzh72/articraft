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
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _rounded_rect(x_size: float, y_size: float, radius: float, *, segments: int = 10):
    return rounded_rect_profile(x_size, y_size, radius, corner_segments=segments)


def _plate_mesh(name: str, *, x_size: float, y_size: float, radius: float, height: float):
    return _save_mesh(
        name,
        ExtrudeGeometry.from_z0(
            _rounded_rect(x_size, y_size, radius),
            height,
            cap=True,
            closed=True,
        ),
    )


def _ring_mesh(
    name: str,
    *,
    outer_x: float,
    outer_y: float,
    outer_radius: float,
    inner_x: float,
    inner_y: float,
    inner_radius: float,
    height: float,
):
    return _save_mesh(
        name,
        ExtrudeWithHolesGeometry(
            _rounded_rect(outer_x, outer_y, outer_radius),
            [_rounded_rect(inner_x, inner_y, inner_radius)],
            height,
            cap=True,
            center=False,
            closed=True,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_tackle_box")

    shell_paint = model.material("shell_paint", rgba=(0.23, 0.29, 0.36, 1.0))
    polymer_trim = model.material("polymer_trim", rgba=(0.19, 0.20, 0.22, 1.0))
    elastomer = model.material("elastomer", rgba=(0.08, 0.08, 0.09, 1.0))
    hardware_steel = model.material("hardware_steel", rgba=(0.66, 0.68, 0.71, 1.0))

    body_x = 0.240
    body_y = 0.380
    body_h = 0.128
    wall = 0.009
    floor_t = 0.010
    body_inner_x = body_x - 2.0 * wall
    body_inner_y = body_y - 2.0 * wall

    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((body_x, body_y, body_h)),
        mass=1.7,
        origin=Origin(xyz=(0.0, 0.0, body_h * 0.5)),
    )

    body.visual(
        _ring_mesh(
            "tackle_box_body_shell",
            outer_x=body_x,
            outer_y=body_y,
            outer_radius=0.028,
            inner_x=body_inner_x,
            inner_y=body_inner_y,
            inner_radius=0.019,
            height=body_h,
        ),
        material=shell_paint,
        name="shell_ring",
    )
    body.visual(
        _plate_mesh(
            "tackle_box_floor_pan",
            x_size=body_inner_x,
            y_size=body_inner_y,
            radius=0.019,
            height=floor_t,
        ),
        material=shell_paint,
        name="floor_pan",
    )
    body.visual(
        _ring_mesh(
            "tackle_box_rim_gasket",
            outer_x=body_x - 0.004,
            outer_y=body_y - 0.004,
            outer_radius=0.026,
            inner_x=body_inner_x - 0.002,
            inner_y=body_inner_y - 0.002,
            inner_radius=0.020,
            height=0.0025,
        ),
        origin=Origin(xyz=(0.0, 0.0, body_h)),
        material=elastomer,
        name="rim_gasket",
    )
    body.visual(
        Box((0.130, 0.020, 0.006)),
        origin=Origin(xyz=(0.0, -0.120, 0.003)),
        material=elastomer,
        name="left_foot_rail",
    )
    body.visual(
        Box((0.130, 0.020, 0.006)),
        origin=Origin(xyz=(0.0, 0.120, 0.003)),
        material=elastomer,
        name="right_foot_rail",
    )

    hinge_axis_x = -body_x * 0.5
    hinge_axis_z = body_h + 0.010
    for side, y_center in (("left", -0.137), ("right", 0.137)):
        body.visual(
            Box((0.014, 0.102, 0.018)),
            origin=Origin(xyz=(hinge_axis_x + 0.006, y_center, body_h - 0.001)),
            material=shell_paint,
            name=f"hinge_support_{side}",
        )
        body.visual(
            Cylinder(radius=0.008, length=0.102),
            origin=Origin(
                xyz=(hinge_axis_x, y_center, hinge_axis_z),
                rpy=(pi / 2.0, 0.0, 0.0),
            ),
            material=hardware_steel,
            name=f"hinge_barrel_{side}",
        )

    latch_z = body_h - 0.014
    for side, y_center in (("left", -0.118), ("right", 0.118)):
        body.visual(
            Box((0.016, 0.034, 0.020)),
            origin=Origin(xyz=(body_x * 0.5 - 0.008, y_center, latch_z)),
            material=polymer_trim,
            name=f"latch_receiver_{side}",
        )
        body.visual(
            Cylinder(radius=0.0025, length=0.026),
            origin=Origin(
                xyz=(body_x * 0.5 - 0.0025, y_center, body_h - 0.0045),
                rpy=(pi / 2.0, 0.0, 0.0),
            ),
            material=hardware_steel,
            name=f"latch_pin_{side}",
        )

    lid = model.part("lid")
    lid.inertial = Inertial.from_geometry(
        Box((0.250, 0.390, 0.070)),
        mass=0.95,
        origin=Origin(xyz=(0.120, 0.0, 0.005)),
    )

    lid.visual(
        _plate_mesh(
            "tackle_box_lid_top_panel",
            x_size=0.236,
            y_size=0.390,
            radius=0.032,
            height=0.014,
        ),
        origin=Origin(xyz=(0.126, 0.0, 0.006)),
        material=shell_paint,
        name="lid_top_panel",
    )
    lid.visual(
        Box((0.030, 0.148, 0.020)),
        origin=Origin(xyz=(0.015, 0.0, 0.008)),
        material=shell_paint,
        name="rear_spine",
    )
    lid.visual(
        Box((0.008, 0.334, 0.032)),
        origin=Origin(xyz=(0.246, 0.0, -0.005)),
        material=shell_paint,
        name="front_skirt",
    )
    lid.visual(
        Box((0.220, 0.006, 0.032)),
        origin=Origin(xyz=(0.126, -0.193, -0.005)),
        material=shell_paint,
        name="left_side_skirt",
    )
    lid.visual(
        Box((0.220, 0.006, 0.032)),
        origin=Origin(xyz=(0.126, 0.193, -0.005)),
        material=shell_paint,
        name="right_side_skirt",
    )
    lid.visual(
        Box((0.010, 0.320, 0.016)),
        origin=Origin(xyz=(0.224, 0.0, 0.000)),
        material=polymer_trim,
        name="inner_lip_front",
    )
    lid.visual(
        Box((0.186, 0.010, 0.016)),
        origin=Origin(xyz=(0.128, -0.174, 0.000)),
        material=polymer_trim,
        name="inner_lip_left",
    )
    lid.visual(
        Box((0.186, 0.010, 0.016)),
        origin=Origin(xyz=(0.128, 0.174, 0.000)),
        material=polymer_trim,
        name="inner_lip_right",
    )
    lid.visual(
        _plate_mesh(
            "tackle_box_lid_insert",
            x_size=0.122,
            y_size=0.228,
            radius=0.018,
            height=0.004,
        ),
        origin=Origin(xyz=(0.124, 0.0, 0.020)),
        material=polymer_trim,
        name="top_insert",
    )

    lid.visual(
        Cylinder(radius=0.008, length=0.154),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=hardware_steel,
        name="hinge_knuckle",
    )

    for side, y_center in (("left", -0.118), ("right", 0.118)):
        lid.visual(
            Box((0.008, 0.028, 0.034)),
            origin=Origin(xyz=(0.246, y_center, -0.005)),
            material=polymer_trim,
            name=f"front_latch_{side}",
        )

    for side, y_center in (("left", -0.070), ("right", 0.070)):
        lid.visual(
            Box((0.018, 0.028, 0.020)),
            origin=Origin(xyz=(0.124, y_center, 0.030)),
            material=polymer_trim,
            name=f"handle_pedestal_{side}",
        )
    lid.visual(
        Cylinder(radius=0.010, length=0.112),
        origin=Origin(xyz=(0.124, 0.0, 0.050), rpy=(pi / 2.0, 0.0, 0.0)),
        material=elastomer,
        name="carry_grip",
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(hinge_axis_x, 0.0, hinge_axis_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=0.0,
            upper=1.32,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("body_to_lid")

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

    with ctx.pose({hinge: 0.0}):
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_top_panel",
            elem_b="shell_ring",
            min_overlap=0.20,
            name="lid_covers_body",
        )
        ctx.expect_within(
            lid,
            body,
            axes="xy",
            inner_elem="inner_lip_front",
            outer_elem="floor_pan",
            margin=0.0,
            name="locator_lip_nests_inside_body",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="y",
            elem_a="front_latch_left",
            elem_b="latch_receiver_left",
            min_overlap=0.020,
            name="left_latch_aligned",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="y",
            elem_a="front_latch_right",
            elem_b="latch_receiver_right",
            min_overlap=0.020,
            name="right_latch_aligned",
        )
        ctx.expect_gap(
            lid,
            body,
            axis="x",
            positive_elem="front_latch_left",
            negative_elem="latch_receiver_left",
            min_gap=0.001,
            max_gap=0.012,
            name="left_latch_clearance",
        )
        ctx.expect_gap(
            lid,
            body,
            axis="x",
            positive_elem="front_latch_right",
            negative_elem="latch_receiver_right",
            min_gap=0.001,
            max_gap=0.012,
            name="right_latch_clearance",
        )

    with ctx.pose({hinge: 1.10}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="front_latch_left",
            negative_elem="latch_receiver_left",
            min_gap=0.15,
            name="left_latch_lifts_clear_when_open",
        )
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="front_latch_right",
            negative_elem="latch_receiver_right",
            min_gap=0.15,
            name="right_latch_lifts_clear_when_open",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
