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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    wire_from_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="luggage_padlock")

    body_brass = model.material("body_brass", rgba=(0.77, 0.63, 0.22, 1.0))
    shackle_steel = model.material("shackle_steel", rgba=(0.86, 0.88, 0.90, 1.0))
    button_black = model.material("button_black", rgba=(0.12, 0.13, 0.14, 1.0))

    body = model.part("body")

    body_core_geom = ExtrudeGeometry(
        rounded_rect_profile(0.024, 0.020, 0.003),
        0.014,
        center=True,
    )
    body.visual(
        mesh_from_geometry(body_core_geom, "padlock_body_core"),
        origin=Origin(xyz=(-0.003, 0.0, 0.010), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=body_brass,
        name="body_core",
    )
    body.visual(
        Box((0.006, 0.0025, 0.014)),
        origin=Origin(xyz=(0.012, -0.00575, 0.010)),
        material=body_brass,
        name="button_bezel_front",
    )
    body.visual(
        Box((0.006, 0.0025, 0.014)),
        origin=Origin(xyz=(0.012, 0.00575, 0.010)),
        material=body_brass,
        name="button_bezel_back",
    )
    body.visual(
        Box((0.006, 0.009, 0.003)),
        origin=Origin(xyz=(0.012, 0.0, 0.0015)),
        material=body_brass,
        name="button_bezel_bottom",
    )
    body.visual(
        Box((0.006, 0.009, 0.003)),
        origin=Origin(xyz=(0.012, 0.0, 0.0185)),
        material=body_brass,
        name="button_bezel_top",
    )
    body.visual(
        Box((0.021, 0.006, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0215)),
        material=body_brass,
        name="top_bridge",
    )
    body.visual(
        Cylinder(radius=0.003, length=0.005),
        origin=Origin(xyz=(-0.0085, 0.0, 0.0225)),
        material=body_brass,
        name="captured_leg_boss",
    )
    body.visual(
        Cylinder(radius=0.003, length=0.005),
        origin=Origin(xyz=(0.0085, 0.0, 0.0225)),
        material=body_brass,
        name="release_leg_receiver",
    )
    body.visual(
        Box((0.018, 0.001, 0.010)),
        origin=Origin(xyz=(-0.0015, -0.0075, 0.010)),
        material=body_brass,
        name="front_face_plate",
    )
    body.visual(
        Box((0.018, 0.001, 0.010)),
        origin=Origin(xyz=(-0.0015, 0.0075, 0.010)),
        material=body_brass,
        name="back_face_plate",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.030, 0.015, 0.024)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
    )

    shackle = model.part("shackle")
    shackle_geom = wire_from_points(
        [
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.014),
            (0.017, 0.0, 0.014),
            (0.017, 0.0, 0.0),
        ],
        radius=0.00135,
        radial_segments=18,
        closed_path=False,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.0035,
        corner_segments=10,
    )
    shackle.visual(
        mesh_from_geometry(shackle_geom, "padlock_shackle"),
        material=shackle_steel,
        name="shackle_wire",
    )
    shackle.inertial = Inertial.from_geometry(
        Box((0.020, 0.004, 0.020)),
        mass=0.02,
        origin=Origin(xyz=(0.0085, 0.0, 0.010)),
    )

    release_button = model.part("release_button")
    release_button.visual(
        Box((0.0045, 0.009, 0.014)),
        origin=Origin(xyz=(0.00375, 0.0, 0.0)),
        material=button_black,
        name="button_cap",
    )
    release_button.inertial = Inertial.from_geometry(
        Box((0.0045, 0.009, 0.014)),
        mass=0.005,
        origin=Origin(xyz=(0.00375, 0.0, 0.0)),
    )

    model.articulation(
        "body_to_shackle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=shackle,
        origin=Origin(xyz=(-0.0085, 0.0, 0.025)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(85.0),
        ),
    )
    model.articulation(
        "body_to_release_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=release_button,
        origin=Origin(xyz=(0.009, 0.0, 0.010)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.05,
            lower=0.0,
            upper=0.0015,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    shackle = object_model.get_part("shackle")
    release_button = object_model.get_part("release_button")
    shackle_joint = object_model.get_articulation("body_to_shackle")
    button_joint = object_model.get_articulation("body_to_release_button")

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
        "shackle_joint_axis_is_side_to_side",
        tuple(shackle_joint.axis) in ((0.0, 1.0, 0.0), (0.0, -1.0, 0.0)),
        f"axis={shackle_joint.axis}",
    )
    ctx.check(
        "button_joint_axis_points_into_body",
        tuple(button_joint.axis) == (-1.0, 0.0, 0.0),
        f"axis={button_joint.axis}",
    )

    ctx.expect_contact(shackle, body, name="closed_shackle_contacts_body")
    ctx.expect_contact(release_button, body, name="button_is_seated_in_body")
    ctx.expect_within(
        release_button,
        body,
        axes="yz",
        margin=0.0,
        name="button_fits_within_body_side_recess",
    )
    ctx.expect_overlap(
        release_button,
        body,
        axes="yz",
        min_overlap=0.008,
        name="button_overlaps_body_recess_opening",
    )

    shackle_rest_aabb = ctx.part_world_aabb(shackle)
    button_rest_pos = ctx.part_world_position(release_button)
    assert shackle_rest_aabb is not None
    assert button_rest_pos is not None

    with ctx.pose({shackle_joint: math.radians(75.0)}):
        shackle_open_aabb = ctx.part_world_aabb(shackle)
        assert shackle_open_aabb is not None
        ctx.check(
            "shackle_rotates_upward",
            shackle_open_aabb[1][2] > shackle_rest_aabb[1][2] + 0.004,
            f"closed_max_z={shackle_rest_aabb[1][2]:.4f}, open_max_z={shackle_open_aabb[1][2]:.4f}",
        )

    with ctx.pose({button_joint: 0.0015}):
        button_pressed_pos = ctx.part_world_position(release_button)
        assert button_pressed_pos is not None
        ctx.check(
            "button_plunges_inward",
            button_pressed_pos[0] < button_rest_pos[0] - 0.0010,
            f"rest_x={button_rest_pos[0]:.4f}, pressed_x={button_pressed_pos[0]:.4f}",
        )
        ctx.expect_within(
            release_button,
            body,
            axes="yz",
            margin=0.0,
            name="pressed_button_stays_guided_in_recess",
        )
        ctx.expect_contact(
            release_button,
            body,
            name="pressed_button_remains_body_guided",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
