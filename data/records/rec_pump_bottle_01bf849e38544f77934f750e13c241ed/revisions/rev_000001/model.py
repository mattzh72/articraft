from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _shell_ring_mesh(
    *,
    name: str,
    outer_profile: list[tuple[float, float]],
    inner_profile: list[tuple[float, float]],
):
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=64,
            start_cap="flat",
            end_cap="flat",
        ),
        name,
    )


def _build_body_shell_mesh():
    outer_profile = [
        (0.0225, 0.000),
        (0.0235, 0.008),
        (0.0235, 0.118),
        (0.0228, 0.140),
        (0.0195, 0.154),
        (0.0165, 0.160),
    ]
    inner_profile = [
        (0.0186, 0.004),
        (0.0194, 0.010),
        (0.0194, 0.116),
        (0.0186, 0.137),
        (0.0160, 0.149),
        (0.0143, 0.154),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=72,
            start_cap="flat",
            end_cap="flat",
        ),
        "airless_bottle_body_shell_v2",
    )


def _build_fixed_collar_mesh():
    return _shell_ring_mesh(
        name="fixed_collar_shell_v2",
        outer_profile=[
            (0.0205, -0.0095),
            (0.0218, -0.0030),
            (0.0218, 0.0095),
        ],
        inner_profile=[
            (0.0184, -0.0095),
            (0.0184, 0.0095),
        ],
    )


def _build_neck_sleeve_mesh():
    return _shell_ring_mesh(
        name="neck_sleeve_shell_v2",
        outer_profile=[
            (0.0158, -0.0125),
            (0.0158, 0.0080),
            (0.0184, 0.0125),
        ],
        inner_profile=[
            (0.0100, -0.0125),
            (0.0100, 0.0125),
        ],
    )


def _build_lock_ring_mesh():
    return _shell_ring_mesh(
        name="lock_ring_shell_v3",
        outer_profile=[
            (0.0222, -0.0070),
            (0.0240, -0.0020),
            (0.0240, 0.0040),
            (0.0228, 0.0070),
        ],
        inner_profile=[
            (0.0196, -0.0070),
            (0.0196, 0.0070),
        ],
    )


def _build_actuator_cap_mesh():
    profile = [
        (0.0, 0.014),
        (0.0220, 0.014),
        (0.0175, 0.014),
        (0.0230, 0.018),
        (0.0310, 0.021),
        (0.0310, 0.026),
        (0.0280, 0.030),
        (0.0, 0.030),
    ]
    return mesh_from_geometry(
        LatheGeometry(profile, segments=72),
        "actuator_cap_shell_v3",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="airless_cosmetic_pump_bottle")

    body_white = model.material("body_white", rgba=(0.92, 0.92, 0.90, 1.0))
    actuator_white = model.material("actuator_white", rgba=(0.98, 0.98, 0.97, 1.0))
    brushed_silver = model.material("brushed_silver", rgba=(0.72, 0.74, 0.77, 1.0))
    dark_silver = model.material("dark_silver", rgba=(0.58, 0.61, 0.65, 1.0))

    bottle = model.part("bottle")
    bottle.visual(
        _build_body_shell_mesh(),
        material=body_white,
        name="body_shell",
    )
    bottle.visual(
        Cylinder(radius=0.0240, length=0.0060),
        origin=Origin(xyz=(0.0, 0.0, 0.0030)),
        material=dark_silver,
        name="base_foot",
    )
    bottle.visual(
        _build_neck_sleeve_mesh(),
        origin=Origin(xyz=(0.0, 0.0, 0.1660)),
        material=brushed_silver,
        name="neck_sleeve",
    )
    bottle.visual(
        _build_fixed_collar_mesh(),
        origin=Origin(xyz=(0.0, 0.0, 0.1845)),
        material=brushed_silver,
        name="fixed_collar",
    )
    bottle.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0240, length=0.1940),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.0, 0.0970)),
    )

    actuator = model.part("actuator")
    actuator.visual(
        Cylinder(radius=0.0065, length=0.0340),
        origin=Origin(xyz=(0.0, 0.0, -0.0030)),
        material=dark_silver,
        name="actuator_stem",
    )
    actuator.visual(
        _build_actuator_cap_mesh(),
        material=actuator_white,
        name="actuator_cap",
    )
    actuator.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0310, length=0.0460),
        mass=0.035,
        origin=Origin(xyz=(0.0, 0.0, 0.0100)),
    )

    locking_collar = model.part("locking_collar")
    locking_collar.visual(
        _build_lock_ring_mesh(),
        material=dark_silver,
        name="lock_ring",
    )
    locking_collar.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0240, length=0.0140),
        mass=0.014,
    )

    model.articulation(
        "actuator_slide",
        ArticulationType.PRISMATIC,
        parent=bottle,
        child=actuator,
        origin=Origin(xyz=(0.0, 0.0, 0.1850)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.08,
            lower=0.0,
            upper=0.005,
        ),
    )
    model.articulation(
        "locking_collar_twist",
        ArticulationType.REVOLUTE,
        parent=bottle,
        child=locking_collar,
        origin=Origin(xyz=(0.0, 0.0, 0.1665)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=2.5,
            lower=0.0,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bottle = object_model.get_part("bottle")
    actuator = object_model.get_part("actuator")
    locking_collar = object_model.get_part("locking_collar")

    actuator_slide = object_model.get_articulation("actuator_slide")
    locking_collar_twist = object_model.get_articulation("locking_collar_twist")

    fixed_collar = bottle.get_visual("fixed_collar")
    neck_sleeve = bottle.get_visual("neck_sleeve")
    actuator_cap = actuator.get_visual("actuator_cap")
    actuator_stem = actuator.get_visual("actuator_stem")
    lock_ring = locking_collar.get_visual("lock_ring")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.allow_isolated_part(
        actuator,
        reason="Pump head rides with intentional running clearance while remaining prismatically mounted on the bottle axis.",
    )
    ctx.allow_isolated_part(
        locking_collar,
        reason="Locking collar intentionally rotates with small radial and axial clearance below the fixed collar.",
    )

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
        "actuator_slide_axis_is_bottle_axis",
        actuator_slide.axis == (0.0, 0.0, -1.0),
        details=f"expected (0, 0, -1), got {actuator_slide.axis}",
    )
    ctx.check(
        "locking_collar_rotates_about_neck_axis",
        locking_collar_twist.axis == (0.0, 0.0, 1.0),
        details=f"expected (0, 0, 1), got {locking_collar_twist.axis}",
    )

    ctx.expect_overlap(
        actuator,
        bottle,
        axes="xy",
        elem_a=actuator_cap,
        elem_b=fixed_collar,
        min_overlap=0.040,
        name="actuator_head_overhangs_fixed_collar_xy",
    )
    ctx.expect_gap(
        actuator,
        bottle,
        axis="z",
        positive_elem=actuator_cap,
        negative_elem=fixed_collar,
        min_gap=0.004,
        max_gap=0.006,
        name="actuator_cap_sits_just_above_fixed_collar",
    )
    ctx.expect_overlap(
        locking_collar,
        bottle,
        axes="xy",
        elem_a=lock_ring,
        elem_b=neck_sleeve,
        min_overlap=0.036,
        name="locking_collar_encircles_neck_sleeve",
    )
    ctx.expect_gap(
        bottle,
        locking_collar,
        axis="z",
        positive_elem=fixed_collar,
        negative_elem=lock_ring,
        min_gap=0.001,
        max_gap=0.004,
        name="locking_collar_sits_below_fixed_collar",
    )

    actuator_rest = ctx.part_world_position(actuator)
    locking_rest = ctx.part_world_position(locking_collar)
    with ctx.pose({actuator_slide: 0.005}):
        actuator_pressed = ctx.part_world_position(actuator)
        actuator_moves_straight_down = (
            actuator_rest is not None
            and actuator_pressed is not None
            and abs(actuator_pressed[0] - actuator_rest[0]) < 1e-6
            and abs(actuator_pressed[1] - actuator_rest[1]) < 1e-6
            and actuator_pressed[2] < actuator_rest[2] - 0.0045
        )
        ctx.check(
            "actuator_travels_straight_down",
            actuator_moves_straight_down,
            details=f"rest={actuator_rest}, pressed={actuator_pressed}",
        )
        ctx.expect_gap(
            actuator,
            bottle,
            axis="z",
            positive_elem=actuator_cap,
            negative_elem=fixed_collar,
            min_gap=-0.0005,
            max_gap=0.0005,
            name="pressed_actuator_cap_reaches_fixed_collar",
        )
        ctx.expect_overlap(
            actuator,
            bottle,
            axes="xy",
            elem_a=actuator_stem,
            elem_b=neck_sleeve,
            min_overlap=0.013,
            name="actuator_stem_stays_guided_in_neck",
        )

    with ctx.pose({locking_collar_twist: 1.0}):
        locking_turned = ctx.part_world_position(locking_collar)
        lock_ring_turns_in_place = (
            locking_rest is not None
            and locking_turned is not None
            and abs(locking_turned[0] - locking_rest[0]) < 1e-6
            and abs(locking_turned[1] - locking_rest[1]) < 1e-6
            and abs(locking_turned[2] - locking_rest[2]) < 1e-6
        )
        ctx.check(
            "locking_collar_rotates_without_shifting",
            lock_ring_turns_in_place,
            details=f"rest={locking_rest}, turned={locking_turned}",
        )
        ctx.expect_overlap(
            locking_collar,
            bottle,
            axes="xy",
            elem_a=lock_ring,
            elem_b=neck_sleeve,
            min_overlap=0.036,
            name="twisted_locking_collar_stays_concentric",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
