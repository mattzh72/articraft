from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
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
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)

DECK_WIDTH = 0.48
DECK_LENGTH = 0.74
DECK_THICKNESS = 0.03
HINGE_X = 0.17
HINGE_Y = DECK_LENGTH * 0.5 - 0.035
HINGE_Z = DECK_THICKNESS * 0.5 + 0.009
CASTER_X = 0.175
CASTER_Y = 0.28
CASTER_MOUNT_Z = -DECK_THICKNESS * 0.5
HANDLE_FOLD_ANGLE = math.pi / 2.0


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _handle_half_mesh(side_sign: float):
    return tube_from_spline_points(
        [
            (0.0, 0.0, 0.0),
            (-side_sign * 0.004, -0.002, 0.15),
            (-side_sign * 0.012, -0.004, 0.34),
            (-side_sign * 0.030, -0.004, 0.53),
            (-side_sign * 0.154, -0.004, 0.64),
        ],
        radius=0.010,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fold_handle_platform_trolley", assets=ASSETS)

    deck_steel = model.material("deck_steel", rgba=(0.62, 0.66, 0.70, 1.0))
    painted_steel = model.material("painted_steel", rgba=(0.34, 0.37, 0.40, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.19, 0.21, 0.23, 1.0))
    rubber = model.material("rubber", rgba=(0.06, 0.06, 0.06, 1.0))
    grip_rubber = model.material("grip_rubber", rgba=(0.12, 0.16, 0.20, 1.0))

    left_handle_mesh = _save_mesh("platform_trolley_left_handle.obj", _handle_half_mesh(-1.0))
    right_handle_mesh = _save_mesh("platform_trolley_right_handle.obj", _handle_half_mesh(1.0))
    deck_shell_mesh = _save_mesh(
        "platform_trolley_deck_shell.obj",
        ExtrudeGeometry(
            rounded_rect_profile(DECK_WIDTH, DECK_LENGTH, radius=0.032, corner_segments=8),
            DECK_THICKNESS,
            center=True,
        ),
    )

    deck = model.part("deck")
    deck.visual(
        deck_shell_mesh,
        material=deck_steel,
        name="deck_shell",
    )
    deck.visual(
        Box((DECK_WIDTH - 0.10, DECK_LENGTH - 0.16, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, DECK_THICKNESS * 0.5 + 0.002)),
        material=dark_steel,
        name="deck_mat",
    )
    deck.visual(
        Box((0.022, DECK_LENGTH - 0.035, 0.044)),
        origin=Origin(xyz=(DECK_WIDTH * 0.5 - 0.011, 0.0, -0.037)),
        material=painted_steel,
        name="right_side_skirt",
    )
    deck.visual(
        Box((0.022, DECK_LENGTH - 0.035, 0.044)),
        origin=Origin(xyz=(-DECK_WIDTH * 0.5 + 0.011, 0.0, -0.037)),
        material=painted_steel,
        name="left_side_skirt",
    )
    deck.visual(
        Box((DECK_WIDTH - 0.035, 0.022, 0.044)),
        origin=Origin(xyz=(0.0, DECK_LENGTH * 0.5 - 0.011, -0.037)),
        material=painted_steel,
        name="rear_skirt",
    )
    deck.visual(
        Box((DECK_WIDTH - 0.035, 0.022, 0.044)),
        origin=Origin(xyz=(0.0, -DECK_LENGTH * 0.5 + 0.011, -0.037)),
        material=painted_steel,
        name="front_skirt",
    )
    deck.visual(
        Box((0.10, 0.020, 0.024)),
        origin=Origin(xyz=(-HINGE_X, HINGE_Y + 0.023, HINGE_Z)),
        material=painted_steel,
        name="left_hinge_base",
    )
    deck.visual(
        Box((0.10, 0.020, 0.024)),
        origin=Origin(xyz=(HINGE_X, HINGE_Y + 0.023, HINGE_Z)),
        material=painted_steel,
        name="right_hinge_base",
    )
    deck.visual(
        Box((0.010, 0.030, 0.030)),
        origin=Origin(xyz=(-HINGE_X - 0.017, HINGE_Y, HINGE_Z)),
        material=dark_steel,
        name="left_hinge_outboard",
    )
    deck.visual(
        Box((0.010, 0.030, 0.030)),
        origin=Origin(xyz=(-HINGE_X + 0.017, HINGE_Y, HINGE_Z)),
        material=dark_steel,
        name="left_hinge_inboard",
    )
    deck.visual(
        Box((0.010, 0.030, 0.030)),
        origin=Origin(xyz=(HINGE_X - 0.017, HINGE_Y, HINGE_Z)),
        material=dark_steel,
        name="right_hinge_inboard",
    )
    deck.visual(
        Box((0.010, 0.030, 0.030)),
        origin=Origin(xyz=(HINGE_X + 0.017, HINGE_Y, HINGE_Z)),
        material=dark_steel,
        name="right_hinge_outboard",
    )
    deck.inertial = Inertial.from_geometry(
        Box((DECK_WIDTH, DECK_LENGTH, 0.10)),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.0, -0.01)),
    )

    left_handle = model.part("left_handle")
    left_handle.visual(left_handle_mesh, material=painted_steel, name="tube")
    left_handle.visual(
        Cylinder(radius=0.012, length=0.024),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="hinge_knuckle",
    )
    left_handle.visual(
        Cylinder(radius=0.015, length=0.090),
        origin=Origin(xyz=(0.120, 0.007, 0.640), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grip_rubber,
        name="grip_pad",
    )
    left_handle.inertial = Inertial.from_geometry(
        Box((0.18, 0.05, 0.66)),
        mass=1.1,
        origin=Origin(xyz=(0.08, -0.002, 0.32)),
    )

    right_handle = model.part("right_handle")
    right_handle.visual(right_handle_mesh, material=painted_steel, name="tube")
    right_handle.visual(
        Cylinder(radius=0.012, length=0.024),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="hinge_knuckle",
    )
    right_handle.visual(
        Cylinder(radius=0.015, length=0.090),
        origin=Origin(xyz=(-0.120, 0.007, 0.640), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grip_rubber,
        name="grip_pad",
    )
    right_handle.inertial = Inertial.from_geometry(
        Box((0.18, 0.05, 0.66)),
        mass=1.1,
        origin=Origin(xyz=(-0.08, -0.002, 0.32)),
    )

    caster_positions = {
        "front_left": (-CASTER_X, -CASTER_Y),
        "front_right": (CASTER_X, -CASTER_Y),
        "rear_left": (-CASTER_X, CASTER_Y),
        "rear_right": (CASTER_X, CASTER_Y),
    }

    for caster_name, (x_pos, y_pos) in caster_positions.items():
        caster = model.part(f"{caster_name}_caster")
        caster.visual(
            Box((0.058, 0.048, 0.006)),
            origin=Origin(xyz=(0.0, 0.0, -0.003)),
            material=dark_steel,
            name="top_plate",
        )
        caster.visual(
            Cylinder(radius=0.010, length=0.016),
            origin=Origin(xyz=(0.0, 0.0, -0.014)),
            material=dark_steel,
            name="swivel_stem",
        )
        caster.visual(
            Box((0.020, 0.040, 0.012)),
            origin=Origin(xyz=(0.0, -0.012, -0.027)),
            material=painted_steel,
            name="trail_arm",
        )
        caster.visual(
            Box((0.040, 0.018, 0.010)),
            origin=Origin(xyz=(0.0, -0.020, -0.038)),
            material=painted_steel,
            name="fork_crown",
        )
        caster.visual(
            Box((0.004, 0.022, 0.048)),
            origin=Origin(xyz=(-0.018, -0.020, -0.067)),
            material=painted_steel,
            name="left_plate",
        )
        caster.visual(
            Box((0.004, 0.022, 0.048)),
            origin=Origin(xyz=(0.018, -0.020, -0.067)),
            material=painted_steel,
            name="right_plate",
        )
        caster.inertial = Inertial.from_geometry(
            Box((0.07, 0.07, 0.10)),
            mass=0.55,
            origin=Origin(xyz=(0.0, -0.010, -0.050)),
        )

        wheel = model.part(f"{caster_name}_wheel")
        wheel.visual(
            Cylinder(radius=0.037, length=0.018),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=rubber,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.022, length=0.032),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=painted_steel,
            name="hub",
        )
        wheel.visual(
            Cylinder(radius=0.012, length=0.038),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_steel,
            name="axle_cap",
        )
        wheel.visual(
            Box((0.004, 0.006, 0.006)),
            origin=Origin(xyz=(0.0, 0.034, 0.0)),
            material=deck_steel,
            name="valve_marker",
        )
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=0.037, length=0.018),
            mass=0.38,
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        )

        model.articulation(
            f"deck_to_{caster_name}_caster",
            ArticulationType.CONTINUOUS,
            parent=deck,
            child=caster,
            origin=Origin(xyz=(x_pos, y_pos, CASTER_MOUNT_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=8.0, velocity=8.0),
        )
        model.articulation(
            f"{caster_name}_caster_to_wheel",
            ArticulationType.CONTINUOUS,
            parent=caster,
            child=wheel,
            origin=Origin(xyz=(0.0, -0.020, -0.084)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=25.0),
        )

    model.articulation(
        "deck_to_left_handle",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=left_handle,
        origin=Origin(xyz=(-HINGE_X, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.8,
            lower=0.0,
            upper=HANDLE_FOLD_ANGLE,
        ),
    )
    model.articulation(
        "deck_to_right_handle",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=right_handle,
        origin=Origin(xyz=(HINGE_X, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.8,
            lower=0.0,
            upper=HANDLE_FOLD_ANGLE,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    deck = object_model.get_part("deck")
    left_handle = object_model.get_part("left_handle")
    right_handle = object_model.get_part("right_handle")
    front_left_caster = object_model.get_part("front_left_caster")
    front_right_caster = object_model.get_part("front_right_caster")
    rear_left_caster = object_model.get_part("rear_left_caster")
    rear_right_caster = object_model.get_part("rear_right_caster")
    front_left_wheel = object_model.get_part("front_left_wheel")
    front_right_wheel = object_model.get_part("front_right_wheel")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    rear_right_wheel = object_model.get_part("rear_right_wheel")

    left_handle_hinge = object_model.get_articulation("deck_to_left_handle")
    right_handle_hinge = object_model.get_articulation("deck_to_right_handle")
    front_left_swivel = object_model.get_articulation("deck_to_front_left_caster")
    front_right_swivel = object_model.get_articulation("deck_to_front_right_caster")
    rear_left_swivel = object_model.get_articulation("deck_to_rear_left_caster")
    rear_right_swivel = object_model.get_articulation("deck_to_rear_right_caster")
    front_left_spin = object_model.get_articulation("front_left_caster_to_wheel")
    front_right_spin = object_model.get_articulation("front_right_caster_to_wheel")
    rear_left_spin = object_model.get_articulation("rear_left_caster_to_wheel")
    rear_right_spin = object_model.get_articulation("rear_right_caster_to_wheel")

    deck_shell = deck.get_visual("deck_shell")
    left_hinge_outboard = deck.get_visual("left_hinge_outboard")
    left_hinge_inboard = deck.get_visual("left_hinge_inboard")
    right_hinge_inboard = deck.get_visual("right_hinge_inboard")
    right_hinge_outboard = deck.get_visual("right_hinge_outboard")
    left_knuckle = left_handle.get_visual("hinge_knuckle")
    right_knuckle = right_handle.get_visual("hinge_knuckle")
    left_grip_pad = left_handle.get_visual("grip_pad")
    right_grip_pad = right_handle.get_visual("grip_pad")

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

    ctx.expect_contact(left_handle, deck, elem_a=left_knuckle, elem_b=left_hinge_outboard)
    ctx.expect_contact(left_handle, deck, elem_a=left_knuckle, elem_b=left_hinge_inboard)
    ctx.expect_contact(right_handle, deck, elem_a=right_knuckle, elem_b=right_hinge_inboard)
    ctx.expect_contact(right_handle, deck, elem_a=right_knuckle, elem_b=right_hinge_outboard)
    ctx.expect_gap(right_handle, left_handle, axis="x", min_gap=0.008)

    for caster, wheel in (
        (front_left_caster, front_left_wheel),
        (front_right_caster, front_right_wheel),
        (rear_left_caster, rear_left_wheel),
        (rear_right_caster, rear_right_wheel),
    ):
        ctx.expect_contact(caster, deck, elem_a=caster.get_visual("top_plate"), elem_b=deck_shell)
        ctx.expect_within(caster, deck, axes="xy", inner_elem=caster.get_visual("top_plate"), outer_elem=deck_shell)
        ctx.expect_contact(wheel, caster, elem_a=wheel.get_visual("hub"), elem_b=caster.get_visual("left_plate"))
        ctx.expect_contact(wheel, caster, elem_a=wheel.get_visual("hub"), elem_b=caster.get_visual("right_plate"))
        ctx.expect_overlap(wheel, caster, axes="y", min_overlap=0.012)

    def _is_continuous_vertical(joint_obj) -> bool:
        return (
            getattr(joint_obj, "articulation_type", None) == ArticulationType.CONTINUOUS
            and tuple(getattr(joint_obj, "axis", ())) == (0.0, 0.0, 1.0)
        )

    def _is_continuous_axle(joint_obj) -> bool:
        return (
            getattr(joint_obj, "articulation_type", None) == ArticulationType.CONTINUOUS
            and tuple(getattr(joint_obj, "axis", ())) == (1.0, 0.0, 0.0)
        )

    for joint_obj, joint_name in (
        (front_left_swivel, "front_left_swivel"),
        (front_right_swivel, "front_right_swivel"),
        (rear_left_swivel, "rear_left_swivel"),
        (rear_right_swivel, "rear_right_swivel"),
    ):
        ctx.check(
            f"{joint_name}_axis",
            _is_continuous_vertical(joint_obj),
            "Caster swivel should be a continuous vertical-axis articulation.",
        )

    for joint_obj, joint_name in (
        (front_left_spin, "front_left_spin"),
        (front_right_spin, "front_right_spin"),
        (rear_left_spin, "rear_left_spin"),
        (rear_right_spin, "rear_right_spin"),
    ):
        ctx.check(
            f"{joint_name}_axis",
            _is_continuous_axle(joint_obj),
            "Wheel should spin continuously around its axle axis.",
        )

    for hinge_obj, joint_name in (
        (left_handle_hinge, "left_handle_hinge"),
        (right_handle_hinge, "right_handle_hinge"),
    ):
        limits = getattr(hinge_obj, "motion_limits", None)
        ctx.check(
            f"{joint_name}_limits",
            getattr(hinge_obj, "articulation_type", None) == ArticulationType.REVOLUTE
            and tuple(getattr(hinge_obj, "axis", ())) == (1.0, 0.0, 0.0)
            and limits is not None
            and abs(getattr(limits, "lower", -1.0) - 0.0) < 1e-9
            and abs(getattr(limits, "upper", 0.0) - HANDLE_FOLD_ANGLE) < 1e-9,
            "Handle hinge should revolve about the shared rear x-axis through about ninety degrees.",
        )

    with ctx.pose({left_handle_hinge: HANDLE_FOLD_ANGLE, right_handle_hinge: HANDLE_FOLD_ANGLE}):
        ctx.expect_gap(
            left_handle,
            deck,
            axis="z",
            max_gap=0.018,
            max_penetration=0.0,
            positive_elem=left_grip_pad,
            negative_elem=deck_shell,
        )
        ctx.expect_gap(
            right_handle,
            deck,
            axis="z",
            max_gap=0.018,
            max_penetration=0.0,
            positive_elem=right_grip_pad,
            negative_elem=deck_shell,
        )
        ctx.expect_overlap(right_handle, deck, axes="x", min_overlap=0.08, elem_a=right_grip_pad, elem_b=deck_shell)
        ctx.expect_overlap(left_handle, deck, axes="x", min_overlap=0.08, elem_a=left_grip_pad, elem_b=deck_shell)

    fl_wheel_rest = ctx.part_world_position(front_left_wheel)
    assert fl_wheel_rest is not None
    with ctx.pose({front_left_swivel: math.pi / 2.0}):
        fl_wheel_swiveled = ctx.part_world_position(front_left_wheel)
        assert fl_wheel_swiveled is not None
        ctx.check(
            "front_left_caster_swivel_moves_wheel_center",
            fl_wheel_swiveled[0] > fl_wheel_rest[0] + 0.015
            and abs(fl_wheel_swiveled[1] - fl_wheel_rest[1]) > 0.015,
            "Caster swivel should move the trailing wheel center around the vertical kingpin axis.",
        )

    valve_rest = ctx.part_element_world_aabb(front_left_wheel, elem="valve_marker")
    assert valve_rest is not None
    valve_rest_center = tuple((lo + hi) * 0.5 for lo, hi in zip(valve_rest[0], valve_rest[1]))
    with ctx.pose({front_left_spin: math.pi / 2.0}):
        valve_turned = ctx.part_element_world_aabb(front_left_wheel, elem="valve_marker")
        assert valve_turned is not None
        valve_turned_center = tuple((lo + hi) * 0.5 for lo, hi in zip(valve_turned[0], valve_turned[1]))
        ctx.check(
            "front_left_wheel_spin_moves_valve_marker",
            abs(valve_turned_center[1] - valve_rest_center[1]) > 0.020
            or abs(valve_turned_center[2] - valve_rest_center[2]) > 0.020,
            "Wheel spin should carry an off-axis valve marker around the axle.",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
