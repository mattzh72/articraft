from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)

BASE_RADIUS = 0.155
BASE_THICKNESS = 0.016
SUPPORT_RING_OUTER_RADIUS = 0.084
SUPPORT_RING_INNER_RADIUS = 0.050
SUPPORT_RING_HEIGHT = 0.050
TRAY_RADIUS = 0.194
TRAY_FLOOR_BOTTOM = 0.012
TRAY_FLOOR_TOP = 0.018
TRAY_RIM_TOP = 0.041
BEARING_PAD_RADIUS = 0.072
BEARING_PAD_HEIGHT = TRAY_FLOOR_BOTTOM
TRAY_HUB_RADIUS = 0.040
TRAY_HUB_HEIGHT = TRAY_FLOOR_TOP - BEARING_PAD_HEIGHT


def _save_mesh(name: str, geometry) -> object:
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_tier_pantry_lazy_susan", assets=ASSETS)

    tray_white = model.material("tray_white", rgba=(0.95, 0.95, 0.92, 1.0))
    warm_gray = model.material("warm_gray", rgba=(0.73, 0.73, 0.70, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.63, 0.65, 0.68, 1.0))

    support_ring_mesh = _save_mesh(
        "support_ring.obj",
        LatheGeometry.from_shell_profiles(
            outer_profile=[
                (SUPPORT_RING_OUTER_RADIUS - 0.004, 0.0),
                (SUPPORT_RING_OUTER_RADIUS, 0.004),
                (SUPPORT_RING_OUTER_RADIUS, SUPPORT_RING_HEIGHT - 0.004),
                (SUPPORT_RING_OUTER_RADIUS - 0.005, SUPPORT_RING_HEIGHT),
            ],
            inner_profile=[
                (SUPPORT_RING_INNER_RADIUS, 0.0),
                (SUPPORT_RING_INNER_RADIUS, SUPPORT_RING_HEIGHT),
            ],
            segments=52,
            start_cap="flat",
            end_cap="flat",
        ),
    )
    tray_shell_mesh = _save_mesh(
        "rotating_tray.obj",
        LatheGeometry.from_shell_profiles(
            outer_profile=[
                (0.0, TRAY_FLOOR_BOTTOM + 0.001),
                (0.120, TRAY_FLOOR_BOTTOM),
                (TRAY_RADIUS - 0.018, TRAY_FLOOR_BOTTOM),
                (TRAY_RADIUS - 0.008, TRAY_FLOOR_BOTTOM + 0.003),
                (TRAY_RADIUS, 0.026),
                (TRAY_RADIUS, TRAY_RIM_TOP),
                (TRAY_RADIUS - 0.008, TRAY_RIM_TOP + 0.001),
            ],
            inner_profile=[
                (0.0, TRAY_FLOOR_TOP),
                (TRAY_RADIUS - 0.030, TRAY_FLOOR_TOP),
                (TRAY_RADIUS - 0.016, TRAY_FLOOR_TOP + 0.002),
                (TRAY_RADIUS - 0.010, 0.029),
                (TRAY_RADIUS - 0.010, TRAY_RIM_TOP - 0.002),
            ],
            segments=64,
            start_cap="flat",
            end_cap="flat",
        ),
    )

    base = model.part("base")
    base.visual(
        Cylinder(radius=BASE_RADIUS, length=BASE_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material=warm_gray,
        name="base_plate",
    )
    base.visual(
        support_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS)),
        material=brushed_steel,
        name="support_ring",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=BASE_RADIUS, length=BASE_THICKNESS + SUPPORT_RING_HEIGHT),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.0, (BASE_THICKNESS + SUPPORT_RING_HEIGHT) / 2.0)),
    )

    rotating_tray = model.part("rotating_tray")
    rotating_tray.visual(
        tray_shell_mesh,
        material=tray_white,
        name="tray_shell",
    )
    rotating_tray.visual(
        Cylinder(radius=BEARING_PAD_RADIUS, length=BEARING_PAD_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BEARING_PAD_HEIGHT / 2.0)),
        material=brushed_steel,
        name="bearing_pad",
    )
    rotating_tray.visual(
        Cylinder(radius=TRAY_HUB_RADIUS, length=TRAY_HUB_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BEARING_PAD_HEIGHT + (TRAY_HUB_HEIGHT / 2.0))),
        material=tray_white,
        name="tray_hub",
    )
    rotating_tray.inertial = Inertial.from_geometry(
        Cylinder(radius=TRAY_RADIUS, length=TRAY_RIM_TOP + 0.002),
        mass=0.95,
        origin=Origin(xyz=(0.0, 0.0, (TRAY_RIM_TOP + 0.002) / 2.0)),
    )

    model.articulation(
        "base_to_rotating_tray",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=rotating_tray,
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + SUPPORT_RING_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    rotating_tray = object_model.get_part("rotating_tray")
    tray_spin = object_model.get_articulation("base_to_rotating_tray")

    base_plate = base.get_visual("base_plate")
    support_ring = base.get_visual("support_ring")
    tray_shell = rotating_tray.get_visual("tray_shell")
    bearing_pad = rotating_tray.get_visual("bearing_pad")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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
        "tray_spin_is_continuous",
        tray_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"Expected continuous tray spin, got {tray_spin.articulation_type!r}.",
    )
    ctx.check(
        "tray_spin_axis_is_vertical",
        abs(tray_spin.axis[0]) < 1e-9
        and abs(tray_spin.axis[1]) < 1e-9
        and abs(abs(tray_spin.axis[2]) - 1.0) < 1e-9,
        details=f"Expected a vertical spin axis, got {tray_spin.axis!r}.",
    )

    ctx.expect_contact(
        rotating_tray,
        base,
        elem_a=bearing_pad,
        elem_b=support_ring,
        name="tray_bearing_pad_contacts_support_ring",
    )
    ctx.expect_overlap(
        rotating_tray,
        base,
        axes="xy",
        min_overlap=0.14,
        elem_a=bearing_pad,
        elem_b=support_ring,
        name="bearing_pad_overlaps_support_ring_plan",
    )
    ctx.expect_origin_distance(
        rotating_tray,
        base,
        axes="xy",
        max_dist=0.001,
        name="tray_is_centered_on_base",
    )
    ctx.expect_gap(
        rotating_tray,
        base,
        axis="z",
        min_gap=0.055,
        positive_elem=tray_shell,
        negative_elem=base_plate,
        name="tray_floor_clears_base_plate",
    )
    ctx.expect_gap(
        rotating_tray,
        base,
        axis="z",
        min_gap=0.010,
        positive_elem=tray_shell,
        negative_elem=support_ring,
        name="tray_floor_clears_support_ring",
    )
    ctx.expect_within(
        base,
        rotating_tray,
        axes="xy",
        inner_elem=base_plate,
        outer_elem=tray_shell,
        name="base_plate_sits_under_tray_footprint",
    )

    with ctx.pose({tray_spin: 1.7}):
        ctx.expect_contact(
            rotating_tray,
            base,
            elem_a=bearing_pad,
            elem_b=support_ring,
            name="tray_remains_supported_while_spinning",
        )
        ctx.expect_origin_distance(
            rotating_tray,
            base,
            axes="xy",
            max_dist=0.001,
            name="tray_remains_centered_while_spinning",
        )
        ctx.expect_gap(
            rotating_tray,
            base,
            axis="z",
            min_gap=0.010,
            positive_elem=tray_shell,
            negative_elem=support_ring,
            name="tray_floor_stays_above_support_ring_while_spinning",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
