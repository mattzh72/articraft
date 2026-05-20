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
    model = ArticulatedObject(name="shutter_padlock")

    body_brass = model.material("body_brass", rgba=(0.72, 0.58, 0.26, 1.0))
    shackle_steel = model.material("shackle_steel", rgba=(0.78, 0.80, 0.83, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.18, 0.20, 1.0))
    key_shadow = model.material("key_shadow", rgba=(0.05, 0.05, 0.06, 1.0))
    cover_black = model.material("cover_black", rgba=(0.11, 0.11, 0.12, 1.0))

    body_width = 0.110
    body_depth = 0.034
    body_height = 0.056
    shoulder_width = 0.031
    shoulder_height = 0.019

    body = model.part("body")
    shell_profile = rounded_rect_profile(body_width, body_height, radius=0.008, corner_segments=8)
    shell_geom = ExtrudeGeometry.centered(shell_profile, body_depth).rotate_x(math.pi / 2.0)
    body.visual(
        mesh_from_geometry(shell_geom, "padlock_body_shell"),
        origin=Origin(xyz=(0.0, 0.0, body_height / 2.0)),
        material=body_brass,
        name="body_shell",
    )
    body.visual(
        Box((shoulder_width, body_depth, shoulder_height)),
        origin=Origin(
            xyz=(
                -body_width / 2.0 + shoulder_width / 2.0,
                0.0,
                body_height + shoulder_height / 2.0 - 0.0007,
            )
        ),
        material=body_brass,
        name="left_shoulder",
    )
    body.visual(
        Box((shoulder_width, body_depth, shoulder_height)),
        origin=Origin(
            xyz=(
                body_width / 2.0 - shoulder_width / 2.0,
                0.0,
                body_height + shoulder_height / 2.0 - 0.0007,
            )
        ),
        material=body_brass,
        name="right_shoulder",
    )
    body.visual(
        Box((0.012, body_depth, 0.008)),
        origin=Origin(
            xyz=(
                -body_width / 2.0 + 0.0005,
                0.0,
                body_height + shoulder_height - 0.003,
            )
        ),
        material=body_brass,
        name="pivot_housing",
    )

    key_plate_thickness = 0.0010
    body.visual(
        Box((0.018, key_plate_thickness, 0.021)),
        origin=Origin(
            xyz=(
                0.0,
                body_depth / 2.0 - key_plate_thickness / 2.0,
                0.025,
            )
        ),
        material=dark_steel,
        name="key_plate",
    )
    slot_thickness = 0.0008
    body.visual(
        Box((0.0042, slot_thickness, 0.0115)),
        origin=Origin(
            xyz=(
                0.0,
                body_depth / 2.0 - slot_thickness / 2.0,
                0.0225,
            )
        ),
        material=key_shadow,
        name="keyway_slot",
    )
    body.visual(
        Cylinder(radius=0.0028, length=slot_thickness),
        origin=Origin(
            xyz=(
                0.0,
                body_depth / 2.0 - slot_thickness / 2.0,
                0.0292,
            ),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=key_shadow,
        name="keyway_round",
    )
    body.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, body_height + shoulder_height)),
        mass=1.35,
        origin=Origin(xyz=(0.0, 0.0, (body_height + shoulder_height) / 2.0)),
    )

    shackle = model.part("shackle")
    shackle_radius = 0.0055
    shackle_path = [
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.018),
        (0.022, 0.0, 0.032),
        (0.062, 0.0, 0.036),
        (0.092, 0.0, 0.030),
        (0.106, 0.0, 0.0055),
    ]
    shackle_geom = wire_from_points(
        shackle_path,
        radius=shackle_radius,
        radial_segments=18,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.014,
        corner_segments=10,
    )
    shackle.visual(
        mesh_from_geometry(shackle_geom, "shutter_shackle"),
        material=shackle_steel,
        name="shackle_bar",
    )
    shackle.visual(
        Cylinder(radius=0.0048, length=body_depth * 0.92),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=shackle_steel,
        name="pivot_spindle",
    )
    shackle.inertial = Inertial.from_geometry(
        Box((0.108, shackle_radius * 2.0, 0.042)),
        mass=0.33,
        origin=Origin(xyz=(0.054, 0.0, 0.021)),
    )

    dust_cover = model.part("dust_cover")
    cover_width = 0.028
    cover_height = 0.024
    cover_thickness = 0.0035
    hinge_radius = 0.0014
    dust_cover.visual(
        Cylinder(radius=hinge_radius, length=cover_width * 0.88),
        origin=Origin(
            xyz=(0.0, hinge_radius, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=cover_black,
        name="cover_hinge",
    )
    dust_cover.visual(
        Box((cover_width, cover_thickness, cover_height)),
        origin=Origin(
            xyz=(
                0.0,
                cover_thickness / 2.0,
                -(cover_height / 2.0 - hinge_radius * 0.7),
            )
        ),
        material=cover_black,
        name="cover_plate",
    )
    dust_cover.inertial = Inertial.from_geometry(
        Box((cover_width, cover_thickness, cover_height)),
        mass=0.03,
        origin=Origin(
            xyz=(
                0.0,
                cover_thickness / 2.0,
                -(cover_height / 2.0 - hinge_radius * 0.7),
            )
        ),
    )

    model.articulation(
        "body_to_shackle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=shackle,
        origin=Origin(
            xyz=(
                -body_width / 2.0 - shackle_radius,
                0.0,
                body_height + shoulder_height,
            )
        ),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )
    model.articulation(
        "body_to_dust_cover",
        ArticulationType.REVOLUTE,
        parent=body,
        child=dust_cover,
        origin=Origin(xyz=(0.0, body_depth / 2.0, 0.039)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=4.0,
            lower=0.0,
            upper=math.radians(115.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    shackle = object_model.get_part("shackle")
    dust_cover = object_model.get_part("dust_cover")
    shackle_joint = object_model.get_articulation("body_to_shackle")
    cover_joint = object_model.get_articulation("body_to_dust_cover")

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
        body,
        shackle,
        elem_a="pivot_housing",
        elem_b="pivot_spindle",
        reason="Hidden captive pivot spindle runs inside the side hinge housing.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "shackle joint axis aligned with captive leg",
        tuple(shackle_joint.axis) == (0.0, -1.0, 0.0),
        f"Unexpected shackle axis: {shackle_joint.axis}",
    )
    ctx.check(
        "dust cover hinge axis aligned across face",
        tuple(cover_joint.axis) == (1.0, 0.0, 0.0),
        f"Unexpected cover axis: {cover_joint.axis}",
    )
    ctx.expect_contact(dust_cover, body, name="dust cover rests on front face")
    ctx.expect_overlap(
        dust_cover,
        body,
        axes="xz",
        min_overlap=0.014,
        elem_a="cover_plate",
        elem_b="key_plate",
        name="dust cover spans keyway plate",
    )

    shackle_rest_aabb = ctx.part_world_aabb(shackle)
    cover_rest_aabb = ctx.part_world_aabb(dust_cover)
    assert shackle_rest_aabb is not None
    assert cover_rest_aabb is not None

    shackle_rest_pos = ctx.part_world_position(shackle)
    assert shackle_rest_pos is not None

    with ctx.pose({shackle_joint: math.radians(65.0)}):
        shackle_open_aabb = ctx.part_world_aabb(shackle)
        shackle_open_pos = ctx.part_world_position(shackle)
        assert shackle_open_aabb is not None
        assert shackle_open_pos is not None
        ctx.check(
            "shackle lifts upward when opened",
            shackle_open_aabb[1][2] > shackle_rest_aabb[1][2] + 0.025,
            f"Rest top z={shackle_rest_aabb[1][2]:.4f}, open top z={shackle_open_aabb[1][2]:.4f}",
        )
        ctx.check(
            "shackle pivot stays captive in place",
            max(abs(a - b) for a, b in zip(shackle_open_pos, shackle_rest_pos)) < 1e-6,
            f"Rest origin={shackle_rest_pos}, open origin={shackle_open_pos}",
        )

    with ctx.pose({cover_joint: math.radians(95.0)}):
        cover_open_aabb = ctx.part_world_aabb(dust_cover)
        assert cover_open_aabb is not None
        ctx.expect_contact(dust_cover, body, name="open dust cover stays hinged")
        ctx.check(
            "dust cover swings outward from keyway",
            cover_open_aabb[1][1] > cover_rest_aabb[1][1] + 0.012,
            f"Rest front y={cover_rest_aabb[1][1]:.4f}, open front y={cover_open_aabb[1][1]:.4f}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
