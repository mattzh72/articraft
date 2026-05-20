from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan, pi

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
    model = ArticulatedObject(name="compact_wheelie_bin")

    body_green = model.material("body_green", rgba=(0.20, 0.33, 0.20, 1.0))
    lid_green = model.material("lid_green", rgba=(0.23, 0.38, 0.23, 1.0))
    axle_grey = model.material("axle_grey", rgba=(0.40, 0.42, 0.43, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    hub_grey = model.material("hub_grey", rgba=(0.62, 0.64, 0.66, 1.0))

    body_bottom_z = 0.050
    body_height = 0.285
    body_top_z = body_bottom_z + body_height
    top_width = 0.200
    bottom_width = 0.156
    top_depth = 0.205
    bottom_depth = 0.145
    wall_thickness = 0.004
    front_back_angle = atan(((top_depth - bottom_depth) * 0.5) / body_height)
    side_angle = atan(((top_width - bottom_width) * 0.5) / body_height)

    body = model.part("bin_body")
    body.inertial = Inertial.from_geometry(
        Box((0.205, 0.225, 0.350)),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.0, 0.175)),
    )
    body.visual(
        Box((0.184, wall_thickness, body_height)),
        origin=Origin(
            xyz=(0.0, -((top_depth + bottom_depth) * 0.25), body_bottom_z + body_height * 0.5),
            rpy=(-front_back_angle, 0.0, 0.0),
        ),
        material=body_green,
        name="front_wall",
    )
    body.visual(
        Box((0.184, wall_thickness, body_height)),
        origin=Origin(
            xyz=(0.0, ((top_depth + bottom_depth) * 0.25), body_bottom_z + body_height * 0.5),
            rpy=(front_back_angle, 0.0, 0.0),
        ),
        material=body_green,
        name="back_wall",
    )
    body.visual(
        Box((wall_thickness, 0.170, body_height)),
        origin=Origin(
            xyz=(((top_width + bottom_width) * 0.25), 0.0, body_bottom_z + body_height * 0.5),
            rpy=(0.0, side_angle, 0.0),
        ),
        material=body_green,
        name="right_side_wall",
    )
    body.visual(
        Box((wall_thickness, 0.170, body_height)),
        origin=Origin(
            xyz=(-((top_width + bottom_width) * 0.25), 0.0, body_bottom_z + body_height * 0.5),
            rpy=(0.0, -side_angle, 0.0),
        ),
        material=body_green,
        name="left_side_wall",
    )
    for name, x_sign, y_sign in (
        ("front_right_corner_post", 1.0, -1.0),
        ("front_left_corner_post", -1.0, -1.0),
        ("rear_right_corner_post", 1.0, 1.0),
        ("rear_left_corner_post", -1.0, 1.0),
    ):
        body.visual(
            Box((0.016, 0.016, body_height)),
            origin=Origin(
                xyz=(x_sign * 0.092, y_sign * 0.082, body_bottom_z + body_height * 0.5),
            ),
            material=body_green,
            name=name,
        )
    for name, x_sign, y_sign in (
        ("front_right_floor_riser", 1.0, -1.0),
        ("front_left_floor_riser", -1.0, -1.0),
        ("rear_right_floor_riser", 1.0, 1.0),
        ("rear_left_floor_riser", -1.0, 1.0),
    ):
        body.visual(
            Box((0.020, 0.020, 0.050)),
            origin=Origin(xyz=(x_sign * 0.074, y_sign * 0.068, 0.025)),
            material=body_green,
            name=name,
        )
    body.visual(
        Box((0.150, 0.140, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, body_bottom_z + 0.003)),
        material=body_green,
        name="floor",
    )
    body.visual(
        Box((0.112, 0.018, 0.050)),
        origin=Origin(xyz=(0.0, -0.067, 0.025)),
        material=body_green,
        name="front_foot_bar",
    )
    body.visual(
        Box((0.186, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, -0.099, body_top_z - 0.005)),
        material=body_green,
        name="rim_front",
    )
    body.visual(
        Box((0.186, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, 0.099, body_top_z - 0.005)),
        material=body_green,
        name="rim_back",
    )
    body.visual(
        Box((0.012, 0.186, 0.010)),
        origin=Origin(xyz=(0.097, 0.0, body_top_z - 0.005)),
        material=body_green,
        name="rim_right",
    )
    body.visual(
        Box((0.012, 0.186, 0.010)),
        origin=Origin(xyz=(-0.097, 0.0, body_top_z - 0.005)),
        material=body_green,
        name="rim_left",
    )
    body.visual(
        Box((0.092, 0.016, 0.020)),
        origin=Origin(xyz=(0.0, 0.093, 0.292)),
        material=body_green,
        name="rear_handle_bridge",
    )
    body.visual(
        Cylinder(radius=0.008, length=0.112),
        origin=Origin(xyz=(0.0, 0.094, 0.308), rpy=(0.0, pi * 0.5, 0.0)),
        material=body_green,
        name="rear_handle_bar",
    )
    body.visual(
        Box((0.016, 0.016, 0.024)),
        origin=Origin(xyz=(-0.030, 0.093, 0.313)),
        material=body_green,
        name="left_handle_stanchion",
    )
    body.visual(
        Box((0.016, 0.016, 0.024)),
        origin=Origin(xyz=(0.030, 0.093, 0.313)),
        material=body_green,
        name="right_handle_stanchion",
    )
    body.visual(
        Box((0.024, 0.016, 0.022)),
        origin=Origin(xyz=(-0.082, 0.102, 0.326)),
        material=body_green,
        name="left_hinge_pedestal",
    )
    body.visual(
        Box((0.024, 0.016, 0.022)),
        origin=Origin(xyz=(0.082, 0.102, 0.326)),
        material=body_green,
        name="right_hinge_pedestal",
    )
    body.visual(
        Box((0.010, 0.030, 0.012)),
        origin=Origin(xyz=(-0.086, 0.108, 0.059)),
        material=body_green,
        name="left_axle_bracket",
    )
    body.visual(
        Box((0.010, 0.008, 0.046)),
        origin=Origin(xyz=(-0.086, 0.104, 0.047)),
        material=body_green,
        name="left_axle_web",
    )
    body.visual(
        Box((0.010, 0.030, 0.012)),
        origin=Origin(xyz=(0.086, 0.108, 0.059)),
        material=body_green,
        name="right_axle_bracket",
    )
    body.visual(
        Box((0.010, 0.008, 0.046)),
        origin=Origin(xyz=(0.086, 0.104, 0.047)),
        material=body_green,
        name="right_axle_web",
    )

    axle = model.part("rear_axle")
    axle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.013, length=0.202),
        mass=0.25,
        origin=Origin(rpy=(0.0, pi * 0.5, 0.0)),
    )
    axle.visual(
        Cylinder(radius=0.0065, length=0.192),
        origin=Origin(rpy=(0.0, pi * 0.5, 0.0)),
        material=axle_grey,
        name="axle_rod",
    )
    axle.visual(
        Cylinder(radius=0.013, length=0.010),
        origin=Origin(xyz=(-0.096, 0.0, 0.0), rpy=(0.0, pi * 0.5, 0.0)),
        material=axle_grey,
        name="left_collar",
    )
    axle.visual(
        Cylinder(radius=0.013, length=0.010),
        origin=Origin(xyz=(0.096, 0.0, 0.0), rpy=(0.0, pi * 0.5, 0.0)),
        material=axle_grey,
        name="right_collar",
    )

    wheel_radius = 0.040
    wheel_width = 0.022
    hub_length = 0.034
    for wheel_name, spin_name, x_pos in (
        ("left_wheel", "left_wheel_spin", -0.118),
        ("right_wheel", "right_wheel_spin", 0.118),
    ):
        wheel = model.part(wheel_name)
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=wheel_radius, length=wheel_width),
            mass=0.18,
            origin=Origin(rpy=(0.0, pi * 0.5, 0.0)),
        )
        wheel.visual(
            Cylinder(radius=wheel_radius, length=wheel_width),
            origin=Origin(rpy=(0.0, pi * 0.5, 0.0)),
            material=wheel_rubber,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.018, length=wheel_width - 0.004),
            origin=Origin(rpy=(0.0, pi * 0.5, 0.0)),
            material=hub_grey,
            name="hub_disc",
        )
        wheel.visual(
            Cylinder(radius=0.010, length=hub_length),
            origin=Origin(rpy=(0.0, pi * 0.5, 0.0)),
            material=axle_grey,
            name="hub_sleeve",
        )
        model.articulation(
            spin_name,
            ArticulationType.CONTINUOUS,
            parent=axle,
            child=wheel,
            origin=Origin(xyz=(x_pos, 0.0, 0.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=20.0),
        )

    lid = model.part("lid")
    lid.inertial = Inertial.from_geometry(
        Box((0.210, 0.225, 0.032)),
        mass=0.55,
        origin=Origin(xyz=(0.0, -0.103, 0.010)),
    )
    lid.visual(
        Box((0.206, 0.220, 0.006)),
        origin=Origin(xyz=(0.0, -0.110, 0.006)),
        material=lid_green,
        name="lid_panel",
    )
    lid.visual(
        Box((0.140, 0.014, 0.010)),
        origin=Origin(xyz=(0.0, -0.006, 0.001)),
        material=lid_green,
        name="rear_bridge",
    )
    lid.visual(
        Cylinder(radius=0.008, length=0.032),
        origin=Origin(xyz=(-0.068, 0.000, 0.000), rpy=(0.0, pi * 0.5, 0.0)),
        material=lid_green,
        name="left_knuckle",
    )
    lid.visual(
        Cylinder(radius=0.008, length=0.032),
        origin=Origin(xyz=(0.068, 0.000, 0.000), rpy=(0.0, pi * 0.5, 0.0)),
        material=lid_green,
        name="right_knuckle",
    )
    lid.visual(
        Box((0.008, 0.176, 0.014)),
        origin=Origin(xyz=(-0.107, -0.102, -0.001)),
        material=lid_green,
        name="left_skirt",
    )
    lid.visual(
        Box((0.008, 0.176, 0.014)),
        origin=Origin(xyz=(0.107, -0.102, -0.001)),
        material=lid_green,
        name="right_skirt",
    )
    lid.visual(
        Box((0.170, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, -0.214, 0.011)),
        material=lid_green,
        name="front_lip",
    )
    lid.visual(
        Box((0.096, 0.028, 0.012)),
        origin=Origin(xyz=(0.0, -0.086, 0.014)),
        material=lid_green,
        name="handle_ridge",
    )

    model.articulation(
        "body_to_axle",
        ArticulationType.FIXED,
        parent=body,
        child=axle,
        origin=Origin(xyz=(0.0, 0.095, 0.040)),
    )
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, 0.116, 0.334)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.5, lower=0.0, upper=1.95),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("bin_body")
    axle = object_model.get_part("rear_axle")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")
    lid = object_model.get_part("lid")

    lid_hinge = object_model.get_articulation("body_to_lid")
    left_spin = object_model.get_articulation("left_wheel_spin")
    right_spin = object_model.get_articulation("right_wheel_spin")

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
        "lid_hinge_axis_is_widthwise",
        tuple(lid_hinge.axis) == (-1.0, 0.0, 0.0),
        details=f"unexpected lid axis {lid_hinge.axis}",
    )
    ctx.check(
        "wheel_spin_axes_are_coaxial",
        tuple(left_spin.axis) == (1.0, 0.0, 0.0) and tuple(right_spin.axis) == (1.0, 0.0, 0.0),
        details=f"left={left_spin.axis}, right={right_spin.axis}",
    )
    ctx.expect_contact(axle, body, elem_a="left_collar", elem_b="left_axle_bracket")
    ctx.expect_contact(axle, body, elem_a="right_collar", elem_b="right_axle_bracket")
    ctx.expect_contact(left_wheel, axle, elem_a="hub_sleeve", elem_b="left_collar")
    ctx.expect_contact(right_wheel, axle, elem_a="hub_sleeve", elem_b="right_collar")
    ctx.expect_gap(body, left_wheel, axis="x", positive_elem="left_side_wall", negative_elem="tire", min_gap=0.003)
    ctx.expect_gap(right_wheel, body, axis="x", positive_elem="tire", negative_elem="right_side_wall", min_gap=0.003)

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_panel",
            negative_elem="rim_front",
            max_gap=0.004,
            max_penetration=0.0,
        )
        ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.160)

    with ctx.pose({lid_hinge: 1.75}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="front_lip",
            negative_elem="rear_handle_bar",
            min_gap=0.090,
        )
        body_aabb = ctx.part_world_aabb(body)
        open_lip_aabb = ctx.part_element_world_aabb(lid, elem="front_lip")
        if body_aabb is not None and open_lip_aabb is not None:
            ctx.check(
                "lid_stays_compact_when_open",
                open_lip_aabb[1][1] <= body_aabb[1][1] + 0.055,
                details=(
                    f"open front lip max y {open_lip_aabb[1][1]:.4f} exceeds compact rear envelope "
                    f"{body_aabb[1][1] + 0.055:.4f}"
                ),
            )
        else:
            ctx.fail("lid_stays_compact_when_open", "missing AABB for body or open lid front lip")

    closed_lip_aabb = None
    open_lip_aabb = None
    with ctx.pose({lid_hinge: 0.0}):
        closed_lip_aabb = ctx.part_element_world_aabb(lid, elem="front_lip")
    with ctx.pose({lid_hinge: 1.75}):
        open_lip_aabb = ctx.part_element_world_aabb(lid, elem="front_lip")
    if closed_lip_aabb is not None and open_lip_aabb is not None:
        ctx.check(
            "lid_front_edge_rises_when_opened",
            open_lip_aabb[0][2] >= closed_lip_aabb[1][2] + 0.120,
            details=(
                f"closed lip top z {closed_lip_aabb[1][2]:.4f}, "
                f"open lip bottom z {open_lip_aabb[0][2]:.4f}"
            ),
        )
    else:
        ctx.fail("lid_front_edge_rises_when_opened", "missing front lip AABB in closed or open pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
