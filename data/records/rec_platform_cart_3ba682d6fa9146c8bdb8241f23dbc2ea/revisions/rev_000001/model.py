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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


DECK_LENGTH = 0.78
DECK_WIDTH = 0.48
DECK_THICKNESS = 0.03
DECK_CENTER_Z = 0.109
DECK_TOP_Z = DECK_CENTER_Z + DECK_THICKNESS / 2.0
DECK_BOTTOM_Z = DECK_CENTER_Z - DECK_THICKNESS / 2.0

WHEEL_RADIUS = 0.038
WHEEL_WIDTH = 0.026
WHEEL_HUB_WIDTH = 0.032

HANDLE_TUBE_RADIUS = 0.011
HANDLE_OUTER_WIDTH = 0.418
HANDLE_BAR_LENGTH = HANDLE_OUTER_WIDTH - (2.0 * HANDLE_TUBE_RADIUS)
HANDLE_UPRIGHT_Y = (HANDLE_OUTER_WIDTH / 2.0) - HANDLE_TUBE_RADIUS
HANDLE_HEIGHT = 0.78
HANDLE_HINGE_X = -(DECK_LENGTH / 2.0) - HANDLE_TUBE_RADIUS
HANDLE_HINGE_Z = DECK_TOP_Z + HANDLE_TUBE_RADIUS + 0.005

CASTER_POSITIONS = {
    "front_left": (0.305, 0.190),
    "front_right": (0.305, -0.190),
    "rear_left": (-0.305, 0.190),
    "rear_right": (-0.305, -0.190),
}


def _rounded_plate_mesh(length: float, width: float, thickness: float, radius: float, name: str):
    return mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(length, width, radius, corner_segments=8),
            thickness,
            center=True,
            cap=True,
        ),
        name,
    )


def _add_caster(
    model: ArticulatedObject,
    deck,
    *,
    prefix: str,
    x: float,
    y: float,
    caster_steel,
    axle_steel,
    rubber,
) -> None:
    top_plate_x = 0.040
    top_plate_y = 0.042
    top_plate_t = 0.005
    stem_radius = 0.008
    stem_len = 0.006
    side_cheek_y = 0.018
    side_cheek_t = 0.005
    side_cheek_h = 0.070
    side_cheek_x = WHEEL_HUB_WIDTH / 2.0 + side_cheek_t / 2.0
    wheel_axle_drop = 0.060

    fork = model.part(f"{prefix}_caster_fork")
    fork.visual(
        Box((top_plate_x, top_plate_y, top_plate_t)),
        origin=Origin(xyz=(0.0, 0.0, -top_plate_t / 2.0)),
        material=caster_steel,
        name="top_plate",
    )
    fork.visual(
        Cylinder(radius=stem_radius, length=stem_len),
        origin=Origin(xyz=(0.0, 0.0, -(top_plate_t + stem_len / 2.0))),
        material=axle_steel,
        name="swivel_stem",
    )
    fork.visual(
        Box((side_cheek_t, side_cheek_y, side_cheek_h)),
        origin=Origin(
            xyz=(side_cheek_x, 0.0, -(top_plate_t + side_cheek_h / 2.0))
        ),
        material=caster_steel,
        name="front_cheek",
    )
    fork.visual(
        Box((side_cheek_t, side_cheek_y, side_cheek_h)),
        origin=Origin(
            xyz=(-side_cheek_x, 0.0, -(top_plate_t + side_cheek_h / 2.0))
        ),
        material=caster_steel,
        name="rear_cheek",
    )
    fork.inertial = Inertial.from_geometry(
        Box((0.050, 0.050, 0.060)),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
    )

    wheel = model.part(f"{prefix}_wheel")
    wheel.visual(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber,
        name="tire",
    )
    wheel.visual(
        Cylinder(radius=0.014, length=WHEEL_HUB_WIDTH),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=axle_steel,
        name="hub",
    )
    wheel.visual(
        Cylinder(radius=0.008, length=0.018),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=caster_steel,
        name="hub_cap",
    )
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        mass=0.55,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )

    model.articulation(
        f"{prefix}_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=fork,
        origin=Origin(xyz=(x, y, DECK_BOTTOM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=6.0),
    )
    model.articulation(
        f"{prefix}_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=fork,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, -wheel_axle_drop)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=24.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fold_handle_platform_trolley")

    steel_deck = model.material("steel_deck", rgba=(0.55, 0.58, 0.61, 1.0))
    caster_steel = model.material("caster_steel", rgba=(0.46, 0.48, 0.50, 1.0))
    axle_steel = model.material("axle_steel", rgba=(0.30, 0.32, 0.35, 1.0))
    handle_paint = model.material("handle_paint", rgba=(0.13, 0.15, 0.17, 1.0))
    tread_rubber = model.material("tread_rubber", rgba=(0.10, 0.10, 0.10, 1.0))
    grip_rubber = model.material("grip_rubber", rgba=(0.07, 0.07, 0.08, 1.0))

    deck = model.part("deck")
    deck.visual(
        _rounded_plate_mesh(
            DECK_LENGTH,
            DECK_WIDTH,
            DECK_THICKNESS,
            0.038,
            "platform_deck_shell",
        ),
        origin=Origin(xyz=(0.0, 0.0, DECK_CENTER_Z)),
        material=steel_deck,
        name="deck_plate",
    )
    deck.visual(
        _rounded_plate_mesh(
            DECK_LENGTH - 0.05,
            DECK_WIDTH - 0.05,
            0.002,
            0.028,
            "platform_deck_pad",
        ),
        origin=Origin(xyz=(0.0, 0.0, DECK_TOP_Z + 0.001)),
        material=handle_paint,
        name="grip_pad",
    )
    deck.visual(
        Box((0.56, 0.045, 0.018)),
        origin=Origin(xyz=(0.0, 0.125, DECK_BOTTOM_Z - 0.009)),
        material=caster_steel,
        name="left_stiffener",
    )
    deck.visual(
        Box((0.56, 0.045, 0.018)),
        origin=Origin(xyz=(0.0, -0.125, DECK_BOTTOM_Z - 0.009)),
        material=caster_steel,
        name="right_stiffener",
    )
    deck.visual(
        Box((0.018, 0.020, 0.028)),
        origin=Origin(
            xyz=(
                -(DECK_LENGTH / 2.0) - 0.001,
                0.219,
                DECK_TOP_Z + 0.014,
            )
        ),
        material=caster_steel,
        name="left_hinge_bracket",
    )
    deck.visual(
        Box((0.018, 0.020, 0.028)),
        origin=Origin(
            xyz=(
                -(DECK_LENGTH / 2.0) - 0.001,
                -0.219,
                DECK_TOP_Z + 0.014,
            )
        ),
        material=caster_steel,
        name="right_hinge_bracket",
    )
    deck.inertial = Inertial.from_geometry(
        Box((DECK_LENGTH, DECK_WIDTH, 0.08)),
        mass=16.0,
        origin=Origin(xyz=(0.0, 0.0, 0.084)),
    )

    fold_handle = model.part("fold_handle")
    fold_handle.visual(
        Cylinder(radius=HANDLE_TUBE_RADIUS, length=HANDLE_HEIGHT),
        origin=Origin(xyz=(0.0, HANDLE_UPRIGHT_Y, HANDLE_HEIGHT / 2.0)),
        material=handle_paint,
        name="left_upright",
    )
    fold_handle.visual(
        Cylinder(radius=HANDLE_TUBE_RADIUS, length=HANDLE_HEIGHT),
        origin=Origin(xyz=(0.0, -HANDLE_UPRIGHT_Y, HANDLE_HEIGHT / 2.0)),
        material=handle_paint,
        name="right_upright",
    )
    fold_handle.visual(
        Cylinder(radius=HANDLE_TUBE_RADIUS, length=HANDLE_BAR_LENGTH),
        origin=Origin(
            xyz=(0.0, 0.0, HANDLE_HEIGHT),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material=handle_paint,
        name="top_bar",
    )
    fold_handle.visual(
        Cylinder(radius=HANDLE_TUBE_RADIUS, length=HANDLE_OUTER_WIDTH),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=caster_steel,
        name="hinge_crossbar",
    )
    fold_handle.visual(
        Cylinder(radius=0.014, length=0.17),
        origin=Origin(
            xyz=(0.0, 0.0, HANDLE_HEIGHT - 0.018),
            rpy=(-pi / 2.0, 0.0, 0.0),
        ),
        material=grip_rubber,
        name="handle_grip",
    )
    fold_handle.inertial = Inertial.from_geometry(
        Box((0.05, HANDLE_OUTER_WIDTH, HANDLE_HEIGHT)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, HANDLE_HEIGHT / 2.0)),
    )

    model.articulation(
        "deck_to_fold_handle",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=fold_handle,
        origin=Origin(xyz=(HANDLE_HINGE_X, 0.0, HANDLE_HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.0,
            lower=0.0,
            upper=pi / 2.0,
        ),
    )

    for prefix, (x, y) in CASTER_POSITIONS.items():
        _add_caster(
            model,
            deck,
            prefix=prefix,
            x=x,
            y=y,
            caster_steel=caster_steel,
            axle_steel=axle_steel,
            rubber=tread_rubber,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    fold_handle = object_model.get_part("fold_handle")
    handle_hinge = object_model.get_articulation("deck_to_fold_handle")
    deck_plate = deck.get_visual("deck_plate")
    grip_pad = deck.get_visual("grip_pad")

    caster_parts = []
    swivel_joints = []
    wheel_joints = []
    for prefix in CASTER_POSITIONS:
        fork = object_model.get_part(f"{prefix}_caster_fork")
        wheel = object_model.get_part(f"{prefix}_wheel")
        swivel = object_model.get_articulation(f"{prefix}_caster_swivel")
        spin = object_model.get_articulation(f"{prefix}_wheel_spin")
        caster_parts.append((prefix, fork, wheel))
        swivel_joints.append(swivel)
        wheel_joints.append(spin)

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
        "handle_hinge_axis",
        tuple(handle_hinge.axis) == (0.0, 1.0, 0.0),
        details=f"expected handle hinge axis (0, 1, 0), got {handle_hinge.axis}",
    )

    for swivel in swivel_joints:
        ctx.check(
            f"{swivel.name}_axis",
            tuple(swivel.axis) == (0.0, 0.0, 1.0),
            details=f"expected caster swivel axis (0, 0, 1), got {swivel.axis}",
        )
    for spin in wheel_joints:
        ctx.check(
            f"{spin.name}_axis",
            tuple(spin.axis) == (1.0, 0.0, 0.0),
            details=f"expected wheel spin axis (1, 0, 0), got {spin.axis}",
        )

    for prefix, fork, wheel in caster_parts:
        ctx.expect_contact(deck, fork, name=f"{prefix}_fork_mounted_to_deck")
        ctx.expect_contact(fork, wheel, name=f"{prefix}_wheel_on_axle")
        ctx.expect_gap(
            deck,
            wheel,
            axis="z",
            min_gap=0.016,
            max_gap=0.026,
            positive_elem=deck_plate,
            name=f"{prefix}_wheel_below_deck",
        )
        ctx.expect_overlap(
            deck,
            fork,
            axes="xy",
            min_overlap=0.035,
            name=f"{prefix}_fork_plate_under_corner",
        )

    limits = handle_hinge.motion_limits
    assert limits is not None and limits.lower is not None and limits.upper is not None

    with ctx.pose({handle_hinge: limits.lower}):
        ctx.fail_if_parts_overlap_in_current_pose(name="handle_upright_no_overlap")
        ctx.fail_if_isolated_parts(name="handle_upright_no_floating")
        ctx.expect_gap(
            fold_handle,
            deck,
            axis="z",
            min_gap=0.004,
            max_gap=0.008,
            negative_elem=deck_plate,
            name="handle_upright_clears_deck_plate",
        )

    upright_aabb = ctx.part_world_aabb(fold_handle)
    assert upright_aabb is not None

    with ctx.pose({handle_hinge: limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="handle_folded_no_overlap")
        ctx.fail_if_isolated_parts(name="handle_folded_no_floating")
        ctx.expect_gap(
            fold_handle,
            deck,
            axis="z",
            max_gap=0.0015,
            max_penetration=0.0,
            negative_elem=grip_pad,
            name="handle_lies_flat_on_deck",
        )
        ctx.expect_overlap(
            fold_handle,
            deck,
            axes="xy",
            min_overlap=0.18,
            name="handle_folded_over_deck",
        )
        ctx.expect_within(
            fold_handle,
            deck,
            axes="y",
            margin=0.025,
            name="handle_folded_within_deck_width",
        )
        folded_aabb = ctx.part_world_aabb(fold_handle)
        assert folded_aabb is not None
        ctx.check(
            "handle_folds_down_from_upright",
            folded_aabb[1][2] < upright_aabb[1][2] - 0.45,
            details=(
                f"expected folded max z below upright max z by > 0.45 m, "
                f"got upright={upright_aabb[1][2]:.4f}, folded={folded_aabb[1][2]:.4f}"
            ),
        )
        ctx.check(
            "handle_swings_forward_over_platform",
            folded_aabb[1][0] > upright_aabb[1][0] + 0.70,
            details=(
                f"expected folded handle to project forward; "
                f"upright max x={upright_aabb[1][0]:.4f}, folded max x={folded_aabb[1][0]:.4f}"
            ),
        )

    swivel_pose = {
        object_model.get_articulation("front_left_caster_swivel"): pi / 2.0,
        object_model.get_articulation("front_right_caster_swivel"): -pi / 2.0,
        object_model.get_articulation("rear_left_caster_swivel"): pi / 2.0,
        object_model.get_articulation("rear_right_caster_swivel"): -pi / 2.0,
    }
    with ctx.pose(swivel_pose):
        ctx.fail_if_parts_overlap_in_current_pose(name="casters_swiveled_no_overlap")
        for prefix, fork, wheel in caster_parts:
            ctx.expect_contact(
                fork,
                wheel,
                name=f"{prefix}_wheel_contact_in_swivel_pose",
            )

    spin_pose = {
        object_model.get_articulation("front_left_wheel_spin"): 1.1,
        object_model.get_articulation("rear_right_wheel_spin"): -0.8,
    }
    with ctx.pose(spin_pose):
        ctx.fail_if_parts_overlap_in_current_pose(name="wheel_spin_pose_no_overlap")
        for prefix, fork, wheel in caster_parts:
            ctx.expect_contact(
                fork,
                wheel,
                name=f"{prefix}_wheel_contact_in_spin_pose",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
