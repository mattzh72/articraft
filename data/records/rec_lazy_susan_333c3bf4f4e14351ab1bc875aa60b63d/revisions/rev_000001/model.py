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
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _save_mesh(geometry, name: str):
    return mesh_from_geometry(geometry, name)


def _annular_band(
    outer_radius: float,
    inner_radius: float,
    height: float,
    *,
    z_center: float = 0.0,
    segments: int = 88,
):
    band = LatheGeometry.from_shell_profiles(
        [
            (outer_radius, -0.5 * height),
            (outer_radius, 0.5 * height),
        ],
        [
            (inner_radius, -0.5 * height),
            (inner_radius, 0.5 * height),
        ],
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    )
    if z_center:
        band.translate(0.0, 0.0, z_center)
    return band


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="display_lazy_susan")

    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.79, 0.80, 0.78, 1.0))
    graphite = model.material("graphite", rgba=(0.20, 0.21, 0.23, 1.0))
    bearing_black = model.material("bearing_black", rgba=(0.10, 0.11, 0.12, 1.0))

    base_ring = model.part("base_ring")
    base_ring.visual(
        _save_mesh(
            _annular_band(
                outer_radius=0.119,
                inner_radius=0.068,
                height=0.0075,
                z_center=0.00375,
            ),
            "base_ring_shell",
        ),
        material=graphite,
        name="base_ring_shell",
    )
    base_ring.visual(
        _save_mesh(
            _annular_band(
                outer_radius=0.100,
                inner_radius=0.080,
                height=0.0025,
                z_center=0.00675,
                segments=72,
            ),
            "bearing_cartridge",
        ),
        material=bearing_black,
        name="bearing_cartridge",
    )
    base_ring.visual(
        _save_mesh(
            TorusGeometry(
                radius=0.1190,
                tube=0.0016,
                radial_segments=18,
                tubular_segments=88,
            ).translate(0.0, 0.0, 0.0031),
            "base_clip_bead",
        ),
        material=graphite,
        name="base_clip_bead",
    )
    base_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.119, length=0.008),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
    )

    top_disc = model.part("top_disc")
    top_disc.visual(
        _save_mesh(
            CylinderGeometry(radius=0.180, height=0.012, radial_segments=120).translate(
                0.0,
                0.0,
                0.009,
            ),
            "top_plate",
        ),
        material=brushed_aluminum,
        name="top_plate",
    )
    top_disc.visual(
        _save_mesh(
            _annular_band(
                outer_radius=0.100,
                inner_radius=0.080,
                height=0.0034,
                z_center=0.0017,
                segments=72,
            ),
            "top_bearing_track",
        ),
        material=bearing_black,
        name="top_bearing_track",
    )
    top_disc.visual(
        _save_mesh(
            _annular_band(
                outer_radius=0.128,
                inner_radius=0.1235,
                height=0.0104,
                z_center=-0.0018,
                segments=88,
            ),
            "capture_skirt",
        ),
        material=brushed_aluminum,
        name="capture_skirt",
    )
    top_disc.visual(
        _save_mesh(
            TorusGeometry(
                radius=0.1225,
                tube=0.0016,
                radial_segments=18,
                tubular_segments=88,
            ).translate(0.0, 0.0, -0.0053),
            "capture_lip",
        ),
        material=brushed_aluminum,
        name="capture_lip",
    )
    for index in range(96):
        angle = math.tau * index / 96.0
        top_disc.visual(
            Box((0.006, 0.0055, 0.010)),
            origin=Origin(
                xyz=(0.179 * math.cos(angle), 0.179 * math.sin(angle), 0.0085),
                rpy=(0.0, 0.0, angle),
            ),
            material=brushed_aluminum,
            name=f"knurl_{index:03d}",
        )
    top_disc.inertial = Inertial.from_geometry(
        Cylinder(radius=0.180, length=0.023),
        mass=2.1,
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
    )

    model.articulation(
        "base_to_top_rotation",
        ArticulationType.CONTINUOUS,
        parent=base_ring,
        child=top_disc,
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=6.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_ring = object_model.get_part("base_ring")
    top_disc = object_model.get_part("top_disc")
    rotation = object_model.get_articulation("base_to_top_rotation")

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
        "rotation_joint_is_continuous",
        rotation.articulation_type == ArticulationType.CONTINUOUS,
        details=f"Expected continuous joint, got {rotation.articulation_type!r}.",
    )
    ctx.check(
        "rotation_axis_is_vertical",
        tuple(rotation.axis) == (0.0, 0.0, 1.0),
        details=f"Expected vertical axis, got {rotation.axis!r}.",
    )

    ctx.expect_contact(
        top_disc,
        base_ring,
        elem_a="top_bearing_track",
        elem_b="bearing_cartridge",
        contact_tol=0.0005,
        name="bearing_track_contacts_cartridge",
    )
    ctx.expect_gap(
        top_disc,
        base_ring,
        axis="z",
        positive_elem="top_plate",
        negative_elem="base_ring_shell",
        min_gap=0.003,
        max_gap=0.0045,
        name="top_plate_floats_just_above_base_ring",
    )
    ctx.expect_overlap(
        top_disc,
        base_ring,
        axes="xy",
        elem_a="top_plate",
        elem_b="base_ring_shell",
        min_overlap=0.22,
        name="top_disc_centered_over_base_ring",
    )

    base_bead_aabb = ctx.part_element_world_aabb(base_ring, elem="base_clip_bead")
    top_lip_aabb = ctx.part_element_world_aabb(top_disc, elem="capture_lip")
    if base_bead_aabb is None or top_lip_aabb is None:
        ctx.fail("capture_geometry_present", "Capture bead or capture lip AABB was unavailable.")
    else:
        ctx.check(
            "capture_lip_sits_below_base_clip_top",
            top_lip_aabb[0][2] < base_bead_aabb[1][2] - 0.0002,
            details=(
                f"Top lip min z={top_lip_aabb[0][2]:.4f} should sit below "
                f"base clip top z={base_bead_aabb[1][2]:.4f}."
            ),
        )
        ctx.check(
            "capture_lip_overlaps_clip_bead_height_band",
            top_lip_aabb[1][2] > base_bead_aabb[0][2] + 0.0002,
            details=(
                f"Top lip max z={top_lip_aabb[1][2]:.4f} should remain within "
                f"the base clip bead height band starting at z={base_bead_aabb[0][2]:.4f}."
            ),
        )

    knurl_rest_aabb = ctx.part_element_world_aabb(top_disc, elem="knurl_000")
    if knurl_rest_aabb is None:
        ctx.fail("knurl_marker_present", "Reference knurl element was unavailable at rest pose.")
    else:
        rest_center = tuple(
            0.5 * (knurl_rest_aabb[0][axis] + knurl_rest_aabb[1][axis]) for axis in range(3)
        )
        with ctx.pose({rotation: math.pi / 2.0}):
            ctx.expect_contact(
                top_disc,
                base_ring,
                elem_a="top_bearing_track",
                elem_b="bearing_cartridge",
                contact_tol=0.0005,
                name="bearing_contact_persists_after_quarter_turn",
            )
            quarter_turn_aabb = ctx.part_element_world_aabb(top_disc, elem="knurl_000")
            if quarter_turn_aabb is None:
                ctx.fail("knurl_marker_present_after_turn", "Reference knurl element was unavailable.")
            else:
                quarter_center = tuple(
                    0.5 * (quarter_turn_aabb[0][axis] + quarter_turn_aabb[1][axis])
                    for axis in range(3)
                )
                ctx.check(
                    "top_rotates_about_vertical_center_axis",
                    (
                        rest_center[0] > 0.17
                        and abs(rest_center[1]) < 0.01
                        and quarter_center[1] > 0.17
                        and abs(quarter_center[0]) < 0.01
                    ),
                    details=(
                        f"Knurl center should move from +X to +Y on a quarter turn; "
                        f"rest={rest_center!r}, turned={quarter_center!r}."
                    ),
                )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
