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
    LatheGeometry,
    LoftGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _octagon_loop(circumradius: float, z: float) -> list[tuple[float, float, float]]:
    return [
        (
            circumradius * math.cos((2.0 * math.pi * index) / 8.0 + math.pi / 8.0),
            circumradius * math.sin((2.0 * math.pi * index) / 8.0 + math.pi / 8.0),
            z,
        )
        for index in range(8)
    ]


def _ring_mesh(
    name: str,
    *,
    inner_radius: float,
    outer_radius: float,
    z0: float,
    z1: float,
    segments: int = 72,
):
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(outer_radius, z0), (outer_radius, z1)],
            [(inner_radius, z0), (inner_radius, z1)],
            segments=segments,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="lazy_susan_with_locking_tab")

    walnut = model.material("walnut", rgba=(0.45, 0.29, 0.16, 1.0))
    base_black = model.material("base_black", rgba=(0.16, 0.16, 0.17, 1.0))
    steel = model.material("steel", rgba=(0.39, 0.41, 0.43, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.22, 0.23, 0.24, 1.0))
    brass = model.material("brass", rgba=(0.67, 0.57, 0.30, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    top_shell_mesh = mesh_from_geometry(
        LoftGeometry(
            [
                _octagon_loop(0.303, 0.0),
                _octagon_loop(0.299, 0.022),
                _octagon_loop(0.294, 0.030),
            ],
            cap=True,
            closed=True,
        ),
        "lazy_susan_top_shell",
    )
    bearing_race_mesh = _ring_mesh(
        "lazy_susan_bearing_race",
        inner_radius=0.118,
        outer_radius=0.170,
        z0=0.0,
        z1=0.016,
    )
    retaining_wall_mesh = _ring_mesh(
        "lazy_susan_retaining_wall",
        inner_radius=0.174,
        outer_radius=0.186,
        z0=-0.016,
        z1=0.0,
    )
    capture_lip_mesh = _ring_mesh(
        "lazy_susan_capture_lip",
        inner_radius=0.132,
        outer_radius=0.186,
        z0=-0.020,
        z1=-0.016,
    )

    base_disc = model.part("base_disc")
    base_disc.visual(
        Cylinder(radius=0.225, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=base_black,
        name="base_disc",
    )
    base_disc.visual(
        Cylinder(radius=0.170, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=rubber,
        name="foot_ring",
    )
    for index, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        base_disc.visual(
            Cylinder(radius=0.006, length=0.006),
            origin=Origin(
                xyz=(
                    0.125 * math.cos(angle),
                    0.125 * math.sin(angle),
                    0.027,
                )
            ),
            material=dark_steel,
            name=f"support_pad_{index}",
        )
    base_disc.visual(
        Box((0.054, 0.026, 0.018)),
        origin=Origin(xyz=(0.241, 0.0, 0.033)),
        material=steel,
        name="lock_receiver",
    )
    base_disc.inertial = Inertial.from_geometry(
        Cylinder(radius=0.225, length=0.030),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
    )

    bearing_ring = model.part("bearing_ring")
    bearing_ring.visual(
        bearing_race_mesh,
        material=steel,
        name="bearing_race",
    )
    bearing_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.170, length=0.016),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
    )

    top_plate = model.part("top_plate")
    top_plate.visual(
        top_shell_mesh,
        material=walnut,
        name="top_shell",
    )
    top_plate.visual(
        Cylinder(radius=0.105, length=0.015),
        origin=Origin(xyz=(0.0, 0.0, -0.0075)),
        material=dark_steel,
        name="center_hub",
    )
    top_plate.visual(
        retaining_wall_mesh,
        material=dark_steel,
        name="retaining_wall",
    )
    top_plate.visual(
        capture_lip_mesh,
        material=dark_steel,
        name="capture_lip",
    )
    top_plate.visual(
        Box((0.012, 0.010, 0.024)),
        origin=Origin(xyz=(0.262, -0.018, 0.042)),
        material=dark_steel,
        name="left_mount_ear",
    )
    top_plate.visual(
        Box((0.012, 0.010, 0.024)),
        origin=Origin(xyz=(0.262, 0.018, 0.042)),
        material=dark_steel,
        name="right_mount_ear",
    )
    top_plate.inertial = Inertial.from_geometry(
        Box((0.58, 0.58, 0.060)),
        mass=8.8,
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
    )

    locking_tab = model.part("locking_tab")
    locking_tab.visual(
        Cylinder(radius=0.0055, length=0.026),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="pivot_barrel",
    )
    locking_tab.visual(
        Box((0.034, 0.016, 0.006)),
        origin=Origin(xyz=(0.024, 0.0, 0.010)),
        material=brass,
        name="thumb_pad",
    )
    locking_tab.visual(
        Box((0.022, 0.014, 0.010)),
        origin=Origin(xyz=(0.010, 0.0, 0.004)),
        material=brass,
        name="pivot_bridge",
    )
    locking_tab.visual(
        Box((0.026, 0.014, 0.036)),
        origin=Origin(xyz=(0.025, 0.0, -0.010)),
        material=brass,
        name="locking_arm",
    )
    locking_tab.visual(
        Box((0.018, 0.014, 0.010)),
        origin=Origin(xyz=(0.038, 0.0, -0.027)),
        material=brass,
        name="hook_toe",
    )
    locking_tab.inertial = Inertial.from_geometry(
        Box((0.050, 0.020, 0.045)),
        mass=0.08,
        origin=Origin(xyz=(0.025, 0.0, -0.008)),
    )

    model.articulation(
        "base_to_bearing_ring",
        ArticulationType.FIXED,
        parent=base_disc,
        child=bearing_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
    )
    model.articulation(
        "table_rotation",
        ArticulationType.CONTINUOUS,
        parent=base_disc,
        child=top_plate,
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=2.5),
    )
    model.articulation(
        "tab_pivot",
        ArticulationType.REVOLUTE,
        parent=top_plate,
        child=locking_tab,
        origin=Origin(xyz=(0.268, 0.0, 0.042)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=3.0,
            lower=0.0,
            upper=1.05,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_disc = object_model.get_part("base_disc")
    bearing_ring = object_model.get_part("bearing_ring")
    top_plate = object_model.get_part("top_plate")
    locking_tab = object_model.get_part("locking_tab")

    table_rotation = object_model.get_articulation("table_rotation")
    tab_pivot = object_model.get_articulation("tab_pivot")

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

    ctx.expect_contact(base_disc, bearing_ring)
    ctx.expect_contact(top_plate, bearing_ring, elem_a="top_shell", elem_b="bearing_race")
    ctx.expect_overlap(top_plate, bearing_ring, axes="xy", min_overlap=0.22)
    ctx.expect_contact(locking_tab, top_plate, elem_a="pivot_barrel", elem_b="left_mount_ear")
    ctx.expect_gap(
        locking_tab,
        base_disc,
        axis="z",
        positive_elem="hook_toe",
        negative_elem="lock_receiver",
        min_gap=0.003,
    )

    toe_rest = ctx.part_element_world_aabb(locking_tab, elem="hook_toe")
    assert toe_rest is not None

    with ctx.pose({tab_pivot: 1.0}):
        ctx.expect_contact(locking_tab, base_disc, elem_a="hook_toe", elem_b="lock_receiver")
        ctx.expect_overlap(locking_tab, base_disc, axes="xy", elem_a="hook_toe", elem_b="lock_receiver", min_overlap=0.01)
        ctx.expect_contact(top_plate, bearing_ring, elem_a="top_shell", elem_b="bearing_race")

        toe_engaged = ctx.part_element_world_aabb(locking_tab, elem="hook_toe")
        assert toe_engaged is not None
        ctx.check(
            "tab_drops_into_lock_receiver",
            toe_engaged[0][2] < toe_rest[0][2] - 0.002,
            details=f"toe zmin rest={toe_rest[0][2]:.4f}, engaged={toe_engaged[0][2]:.4f}",
        )

    top_rest = ctx.part_world_position(top_plate)
    assert top_rest is not None
    with ctx.pose({table_rotation: 1.35}):
        top_rotated = ctx.part_world_position(top_plate)
        assert top_rotated is not None
        ctx.expect_contact(top_plate, bearing_ring, elem_a="top_shell", elem_b="bearing_race")
        ctx.expect_gap(
            top_plate,
            base_disc,
            axis="z",
            positive_elem="capture_lip",
            negative_elem="base_disc",
            min_gap=0.0004,
        )
        ctx.check(
            "top_rotates_about_center_axis",
            abs(top_rotated[0] - top_rest[0]) < 1e-6 and abs(top_rotated[1] - top_rest[1]) < 1e-6,
            details=f"rest={top_rest}, rotated={top_rotated}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
