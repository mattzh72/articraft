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
    model = ArticulatedObject(name="sewing_box")

    body_pp = model.material("body_pp", rgba=(0.86, 0.83, 0.77, 1.0))
    lid_pp = model.material("lid_pp", rgba=(0.62, 0.70, 0.72, 1.0))

    body_length = 0.280
    body_width = 0.190
    body_height = 0.110
    wall = 0.003
    bottom = 0.0035

    lid_length = body_length + 0.006
    lid_width = body_width + 0.006
    lid_thickness = 0.003
    skirt_thickness = 0.0025
    skirt_depth = 0.018
    rear_offset = 0.006

    barrel_radius = 0.005
    outer_barrel_length = 0.026
    center_barrel_length = 0.082
    hinge_y = 0.054
    hinge_axis_x = -(body_length * 0.5) - 0.005
    hinge_axis_z = body_height + 0.003

    body = model.part("body")
    body.visual(
        Box((body_length, body_width, bottom)),
        origin=Origin(xyz=(0.0, 0.0, bottom * 0.5)),
        material=body_pp,
        name="floor",
    )
    body.visual(
        Box((body_length - 2.0 * wall, wall, body_height)),
        origin=Origin(xyz=(0.0, body_width * 0.5 - wall * 0.5, body_height * 0.5)),
        material=body_pp,
        name="left_wall",
    )
    body.visual(
        Box((body_length - 2.0 * wall, wall, body_height)),
        origin=Origin(xyz=(0.0, -(body_width * 0.5 - wall * 0.5), body_height * 0.5)),
        material=body_pp,
        name="right_wall",
    )
    body.visual(
        Box((wall, body_width, body_height)),
        origin=Origin(xyz=(body_length * 0.5 - wall * 0.5, 0.0, body_height * 0.5)),
        material=body_pp,
        name="front_wall",
    )
    body.visual(
        Box((wall, body_width, body_height)),
        origin=Origin(xyz=(-(body_length * 0.5) + wall * 0.5, 0.0, body_height * 0.5)),
        material=body_pp,
        name="back_wall",
    )
    body.visual(
        Box((0.055, body_width - 2.0 * wall, 0.003)),
        origin=Origin(xyz=(0.018, 0.0, body_height - 0.006)),
        material=body_pp,
        name="front_tray_ledge",
    )
    body.visual(
        Box((wall, body_width - 0.036, 0.036)),
        origin=Origin(xyz=(0.020, 0.0, 0.018)),
        material=body_pp,
        name="divider_wall",
    )
    body.visual(
        Cylinder(radius=0.0065, length=0.045),
        origin=Origin(xyz=(-0.050, 0.042, 0.024), rpy=(0.0, 0.0, 0.0)),
        material=body_pp,
        name="left_spool_post",
    )
    body.visual(
        Cylinder(radius=0.0065, length=0.045),
        origin=Origin(xyz=(-0.050, -0.042, 0.024), rpy=(0.0, 0.0, 0.0)),
        material=body_pp,
        name="right_spool_post",
    )
    body.visual(
        Box((0.008, 0.070, 0.006)),
        origin=Origin(xyz=(body_length * 0.5 - 0.005, 0.0, body_height - 0.008)),
        material=body_pp,
        name="latch_rib",
    )
    body.visual(
        Box((0.010, body_width - 0.030, 0.006)),
        origin=Origin(xyz=(hinge_axis_x + 0.004, 0.0, body_height - 0.011)),
        material=body_pp,
        name="hinge_backbone",
    )
    body.visual(
        Box((0.010, 0.020, 0.016)),
        origin=Origin(xyz=(hinge_axis_x + 0.004, hinge_y, body_height - 0.004)),
        material=body_pp,
        name="left_hinge_support",
    )
    body.visual(
        Box((0.010, 0.020, 0.016)),
        origin=Origin(xyz=(hinge_axis_x + 0.004, -hinge_y, body_height - 0.004)),
        material=body_pp,
        name="right_hinge_support",
    )
    body.visual(
        Cylinder(radius=barrel_radius, length=outer_barrel_length),
        origin=Origin(xyz=(hinge_axis_x, hinge_y, hinge_axis_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=body_pp,
        name="left_hinge_barrel",
    )
    body.visual(
        Cylinder(radius=barrel_radius, length=outer_barrel_length),
        origin=Origin(xyz=(hinge_axis_x, -hinge_y, hinge_axis_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=body_pp,
        name="right_hinge_barrel",
    )
    body.inertial = Inertial.from_geometry(
        Box((body_length, body_width, body_height)),
        mass=0.38,
        origin=Origin(xyz=(0.0, 0.0, body_height * 0.5)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((lid_length, lid_width, lid_thickness)),
        origin=Origin(xyz=(rear_offset + lid_length * 0.5, 0.0, lid_thickness * 0.5)),
        material=lid_pp,
        name="top_panel",
    )
    lid.visual(
        Box((lid_length - 0.008, skirt_thickness, skirt_depth)),
        origin=Origin(
            xyz=(
                rear_offset + lid_length * 0.5,
                lid_width * 0.5 - skirt_thickness * 0.5,
                -skirt_depth * 0.5,
            )
        ),
        material=lid_pp,
        name="left_skirt",
    )
    lid.visual(
        Box((lid_length - 0.008, skirt_thickness, skirt_depth)),
        origin=Origin(
            xyz=(
                rear_offset + lid_length * 0.5,
                -(lid_width * 0.5 - skirt_thickness * 0.5),
                -skirt_depth * 0.5,
            )
        ),
        material=lid_pp,
        name="right_skirt",
    )
    lid.visual(
        Box((skirt_thickness, lid_width, skirt_depth)),
        origin=Origin(
            xyz=(rear_offset + lid_length - skirt_thickness * 0.5, 0.0, -skirt_depth * 0.5)
        ),
        material=lid_pp,
        name="front_skirt",
    )
    lid.visual(
        Cylinder(radius=barrel_radius, length=center_barrel_length),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=lid_pp,
        name="center_hinge_barrel",
    )
    lid.visual(
        Box((0.016, center_barrel_length, 0.010)),
        origin=Origin(xyz=(0.008, 0.0, 0.001)),
        material=lid_pp,
        name="hinge_web",
    )
    lid.visual(
        Box((0.026, 0.084, 0.004)),
        origin=Origin(xyz=(0.020, 0.0, -0.002)),
        material=lid_pp,
        name="rear_stiffener",
    )
    lid.visual(
        Box((0.012, 0.016, 0.010)),
        origin=Origin(xyz=(0.010, 0.026, 0.006)),
        material=lid_pp,
        name="left_hinge_gusset",
    )
    lid.visual(
        Box((0.012, 0.016, 0.010)),
        origin=Origin(xyz=(0.010, -0.026, 0.006)),
        material=lid_pp,
        name="right_hinge_gusset",
    )
    lid.visual(
        Box((0.008, 0.060, 0.016)),
        origin=Origin(xyz=(rear_offset + lid_length - 0.019, 0.0, -0.008)),
        material=lid_pp,
        name="front_snap",
    )
    lid.visual(
        Box((0.012, 0.012, 0.018)),
        origin=Origin(xyz=(0.160, 0.040, 0.012)),
        material=lid_pp,
        name="left_handle_post",
    )
    lid.visual(
        Box((0.012, 0.012, 0.018)),
        origin=Origin(xyz=(0.160, -0.040, 0.012)),
        material=lid_pp,
        name="right_handle_post",
    )
    lid.visual(
        Box((0.014, 0.102, 0.010)),
        origin=Origin(xyz=(0.160, 0.0, 0.026)),
        material=lid_pp,
        name="handle_bar",
    )
    lid.inertial = Inertial.from_geometry(
        Box((lid_length, lid_width, 0.050)),
        mass=0.21,
        origin=Origin(xyz=(rear_offset + lid_length * 0.5, 0.0, 0.012)),
    )

    hinge = model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(hinge_axis_x, 0.0, hinge_axis_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(112.0),
        ),
    )
    hinge.meta["description"] = "rear snap-on trunnion hinge"

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

    ctx.expect_overlap(body, lid, axes="xy", min_overlap=0.175, name="lid covers body in closed pose")
    ctx.expect_contact(
        lid,
        body,
        elem_a="front_snap",
        elem_b="latch_rib",
        contact_tol=0.0005,
        name="front snap seats on latch rib",
    )
    ctx.expect_contact(
        lid,
        body,
        elem_a="center_hinge_barrel",
        elem_b="left_hinge_barrel",
        contact_tol=0.0005,
        name="center barrel bears on left body barrel",
    )
    ctx.expect_contact(
        lid,
        body,
        elem_a="center_hinge_barrel",
        elem_b="right_hinge_barrel",
        contact_tol=0.0005,
        name="center barrel bears on right body barrel",
    )

    with ctx.pose({hinge: math.radians(100.0)}):
        ctx.expect_contact(
            lid,
            body,
            elem_a="center_hinge_barrel",
            elem_b="left_hinge_barrel",
            contact_tol=0.0005,
            name="left barrel contact remains when opened",
        )
        ctx.expect_contact(
            lid,
            body,
            elem_a="center_hinge_barrel",
            elem_b="right_hinge_barrel",
            contact_tol=0.0005,
            name="right barrel contact remains when opened",
        )
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="handle_bar",
            min_gap=0.100,
            name="handle lifts clearly above body when open",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
