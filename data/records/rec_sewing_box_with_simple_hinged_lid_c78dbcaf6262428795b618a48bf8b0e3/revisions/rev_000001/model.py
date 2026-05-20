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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_sewing_box")

    body_width = 0.24
    body_depth = 0.17
    body_height = 0.066
    wall = 0.004
    floor = 0.004

    hinge_radius = 0.005
    hinge_axis_y = -(body_depth / 2.0 + 0.0035)
    hinge_axis_z = body_height - 0.001
    outer_knuckle_len = 0.042
    center_knuckle_len = 0.088
    outer_knuckle_x = 0.073

    lid_width = body_width + 0.008
    lid_depth = body_depth + 0.009
    lid_panel_thickness = 0.003
    lid_skirt_thickness = 0.003
    lid_skirt_height = 0.014
    lid_panel_center_y = lid_depth / 2.0
    lid_panel_center_z = 0.004
    lid_skirt_center_z = (
        lid_panel_center_z - lid_panel_thickness / 2.0 - lid_skirt_height / 2.0
    )

    shell = model.material("shell", rgba=(0.78, 0.79, 0.74, 1.0))
    lid_shell = model.material("lid_shell", rgba=(0.91, 0.88, 0.82, 1.0))
    accent = model.material("accent", rgba=(0.46, 0.18, 0.16, 1.0))
    cushion = model.material("cushion", rgba=(0.68, 0.28, 0.31, 1.0))

    body = model.part("body")
    body.visual(
        Box((body_width, body_depth, floor)),
        origin=Origin(xyz=(0.0, 0.0, floor / 2.0)),
        material=shell,
        name="floor",
    )
    body.visual(
        Box((wall, body_depth - 2.0 * wall, body_height - floor)),
        origin=Origin(
            xyz=(
                -(body_width - wall) / 2.0,
                0.0,
                floor + (body_height - floor) / 2.0,
            )
        ),
        material=shell,
        name="left_wall",
    )
    body.visual(
        Box((wall, body_depth - 2.0 * wall, body_height - floor)),
        origin=Origin(
            xyz=(
                (body_width - wall) / 2.0,
                0.0,
                floor + (body_height - floor) / 2.0,
            )
        ),
        material=shell,
        name="right_wall",
    )
    body.visual(
        Box((body_width, wall, body_height - floor)),
        origin=Origin(
            xyz=(
                0.0,
                (body_depth - wall) / 2.0,
                floor + (body_height - floor) / 2.0,
            )
        ),
        material=shell,
        name="front_wall",
    )
    body.visual(
        Box((body_width, wall, body_height - floor)),
        origin=Origin(
            xyz=(
                0.0,
                -(body_depth - wall) / 2.0,
                floor + (body_height - floor) / 2.0,
            )
        ),
        material=shell,
        name="rear_wall",
    )
    body.visual(
        Box((outer_knuckle_len, 0.004, 0.011)),
        origin=Origin(xyz=(-outer_knuckle_x, hinge_axis_y + 0.002, body_height - 0.0055)),
        material=shell,
        name="left_hinge_leaf",
    )
    body.visual(
        Box((outer_knuckle_len, 0.004, 0.011)),
        origin=Origin(xyz=(outer_knuckle_x, hinge_axis_y + 0.002, body_height - 0.0055)),
        material=shell,
        name="right_hinge_leaf",
    )
    body.visual(
        Cylinder(radius=hinge_radius, length=outer_knuckle_len),
        origin=Origin(
            xyz=(-outer_knuckle_x, hinge_axis_y, hinge_axis_z),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=shell,
        name="left_knuckle",
    )
    body.visual(
        Cylinder(radius=hinge_radius, length=outer_knuckle_len),
        origin=Origin(
            xyz=(outer_knuckle_x, hinge_axis_y, hinge_axis_z),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=shell,
        name="right_knuckle",
    )
    body.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, body_height)),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, body_height / 2.0)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((lid_width, lid_depth, lid_panel_thickness)),
        origin=Origin(xyz=(0.0, lid_panel_center_y, lid_panel_center_z)),
        material=lid_shell,
        name="top_panel",
    )
    lid.visual(
        Box((lid_skirt_thickness, lid_depth - lid_skirt_thickness, lid_skirt_height)),
        origin=Origin(
            xyz=(
                -(lid_width - lid_skirt_thickness) / 2.0,
                lid_panel_center_y,
                lid_skirt_center_z,
            )
        ),
        material=lid_shell,
        name="left_skirt",
    )
    lid.visual(
        Box((lid_skirt_thickness, lid_depth - lid_skirt_thickness, lid_skirt_height)),
        origin=Origin(
            xyz=(
                (lid_width - lid_skirt_thickness) / 2.0,
                lid_panel_center_y,
                lid_skirt_center_z,
            )
        ),
        material=lid_shell,
        name="right_skirt",
    )
    lid.visual(
        Box((lid_width, lid_skirt_thickness, lid_skirt_height)),
        origin=Origin(
            xyz=(0.0, lid_depth - lid_skirt_thickness / 2.0, lid_skirt_center_z)
        ),
        material=lid_shell,
        name="front_skirt",
    )
    lid.visual(
        Box((center_knuckle_len + 0.004, 0.004, 0.011)),
        origin=Origin(xyz=(0.0, -0.002, -0.0015)),
        material=lid_shell,
        name="rear_bridge",
    )
    lid.visual(
        Cylinder(radius=hinge_radius, length=center_knuckle_len),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=lid_shell,
        name="center_knuckle",
    )
    lid.visual(
        Box((0.05, 0.012, 0.006)),
        origin=Origin(xyz=(0.0, lid_depth + 0.003, 0.004)),
        material=accent,
        name="front_pull",
    )
    lid.visual(
        Box((0.10, 0.06, 0.008)),
        origin=Origin(
            xyz=(0.0, lid_panel_center_y + 0.006, lid_panel_center_z + 0.0055)
        ),
        material=cushion,
        name="pincushion_pad",
    )
    lid.inertial = Inertial.from_geometry(
        Box((lid_width, lid_depth + 0.02, 0.024)),
        mass=0.18,
        origin=Origin(xyz=(0.0, lid_panel_center_y, 0.004)),
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_axis_y, hinge_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=0.0,
            upper=1.92,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("body_to_lid")
    limits = hinge.motion_limits

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

    ctx.check("body_present", body is not None, "Expected a rooted storage body.")
    ctx.check("lid_present", lid is not None, "Expected a single hinged lid.")
    ctx.check(
        "hinge_axis_runs_left_to_right",
        tuple(round(v, 6) for v in hinge.axis) == (1.0, 0.0, 0.0),
        f"Expected lid hinge axis along +X, got {hinge.axis!r}.",
    )
    ctx.check(
        "hinge_limits_are_practical",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and abs(limits.lower - 0.0) < 1e-6
        and 1.6 <= limits.upper <= 2.1,
        f"Expected a real rear-hinge travel window, got {limits!r}.",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            min_overlap=0.15,
            name="closed_lid_covers_storage_body",
        )
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="top_panel",
            negative_elem="front_wall",
            min_gap=0.001,
            max_gap=0.004,
            name="closed_panel_sits_just_above_rim",
        )
        ctx.expect_gap(
            lid,
            body,
            axis="y",
            positive_elem="front_skirt",
            negative_elem="front_wall",
            min_gap=0.0005,
            max_gap=0.0045,
            name="front_clearance_keeps_lid_compact_but_safe",
        )

    with ctx.pose({hinge: 1.75}):
        ctx.fail_if_parts_overlap_in_current_pose(name="opened_pose_has_no_part_overlap")
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="front_skirt",
            min_gap=0.045,
            name="open_lid_lifts_front_edge_clear_of_storage_cavity",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
