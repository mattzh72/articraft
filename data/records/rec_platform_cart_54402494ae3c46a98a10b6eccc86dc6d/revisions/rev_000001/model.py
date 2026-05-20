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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    wire_from_points,
)


DECK_WIDTH = 0.62
DECK_LENGTH = 0.96
DECK_THICKNESS = 0.020
DECK_TOP_Z = 0.182
DECK_CENTER_Z = DECK_TOP_Z - DECK_THICKNESS * 0.5
DECK_BOTTOM_Z = DECK_TOP_Z - DECK_THICKNESS

WHEEL_RADIUS = 0.062
WHEEL_WIDTH = 0.034
CASTER_AXLE_Z = -0.100

CASTER_POSITIONS = {
    "front_left": (-0.235, 0.355),
    "front_right": (0.235, 0.355),
    "rear_left": (-0.235, -0.355),
    "rear_right": (0.235, -0.355),
}


def _deck_mesh():
    return mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(DECK_WIDTH, DECK_LENGTH, 0.045), DECK_THICKNESS),
        "platform_cart_deck_plate",
    )


def _wheel_mesh():
    wheel_profile = [
        (0.016, -WHEEL_WIDTH * 0.5),
        (0.044, -WHEEL_WIDTH * 0.5),
        (0.054, -WHEEL_WIDTH * 0.36),
        (0.060, -WHEEL_WIDTH * 0.18),
        (WHEEL_RADIUS, 0.0),
        (0.060, WHEEL_WIDTH * 0.18),
        (0.054, WHEEL_WIDTH * 0.36),
        (0.044, WHEEL_WIDTH * 0.5),
        (0.016, WHEEL_WIDTH * 0.5),
        (0.012, WHEEL_WIDTH * 0.24),
        (0.012, -WHEEL_WIDTH * 0.24),
        (0.016, -WHEEL_WIDTH * 0.5),
    ]
    return mesh_from_geometry(
        LatheGeometry(wheel_profile, segments=48).rotate_y(math.pi / 2.0),
        "platform_cart_caster_wheel",
    )


def _tow_loop_mesh():
    return mesh_from_geometry(
        wire_from_points(
            [
                (-0.056, 0.0, 0.0),
                (-0.042, 0.0, 0.0),
                (-0.040, 0.030, 0.0),
                (-0.038, 0.078, 0.0),
                (0.038, 0.078, 0.0),
                (0.040, 0.030, 0.0),
                (0.042, 0.0, 0.0),
                (0.056, 0.0, 0.0),
            ],
            radius=0.0065,
            radial_segments=14,
            cap_ends=True,
            corner_mode="fillet",
            corner_radius=0.022,
            corner_segments=8,
        ),
        "platform_cart_tow_loop",
    )


def _add_caster(
    model: ArticulatedObject,
    platform,
    *,
    label: str,
    position: tuple[float, float],
    caster_metal,
    caster_dark,
    wheel_rubber,
) -> None:
    caster = model.part(f"{label}_caster")
    caster.visual(
        Box((0.075, 0.055, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=caster_metal,
        name="mount_plate",
    )
    caster.visual(
        Cylinder(radius=0.011, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, -0.017), rpy=(0.0, 0.0, 0.0)),
        material=caster_dark,
        name="swivel_post",
    )
    caster.visual(
        Box((0.050, 0.040, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.033)),
        material=caster_metal,
        name="fork_crown",
    )
    caster.visual(
        Box((0.008, 0.032, 0.064)),
        origin=Origin(xyz=(-0.021, 0.0, -0.070)),
        material=caster_metal,
        name="left_fork_leg",
    )
    caster.visual(
        Box((0.008, 0.032, 0.064)),
        origin=Origin(xyz=(0.021, 0.0, -0.070)),
        material=caster_metal,
        name="right_fork_leg",
    )
    caster.visual(
        Cylinder(radius=0.005, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, CASTER_AXLE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=caster_dark,
        name="axle_bolt",
    )
    caster.inertial = Inertial.from_geometry(
        Box((0.080, 0.060, 0.110)),
        mass=1.15,
        origin=Origin(xyz=(0.0, 0.0, -0.055)),
    )

    wheel = model.part(f"{label}_wheel")
    wheel.visual(_wheel_mesh(), material=wheel_rubber, name="wheel_shell")
    wheel.visual(
        Box((0.010, 0.010, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, WHEEL_RADIUS - 0.003)),
        material=wheel_rubber,
        name="tread_mark",
    )
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        mass=0.85,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        f"platform_to_{label}_caster",
        ArticulationType.CONTINUOUS,
        parent=platform,
        child=caster,
        origin=Origin(xyz=(position[0], position[1], DECK_BOTTOM_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=8.0),
    )
    model.articulation(
        f"{label}_caster_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=caster,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, CASTER_AXLE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=25.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_platform_cart")

    platform_gray = model.material("platform_gray", rgba=(0.34, 0.36, 0.38, 1.0))
    deck_black = model.material("deck_black", rgba=(0.16, 0.17, 0.18, 1.0))
    caster_metal = model.material("caster_metal", rgba=(0.72, 0.74, 0.77, 1.0))
    caster_dark = model.material("caster_dark", rgba=(0.24, 0.25, 0.27, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    grip_black = model.material("grip_black", rgba=(0.10, 0.10, 0.10, 1.0))

    platform = model.part("platform")
    platform.visual(
        _deck_mesh(),
        origin=Origin(xyz=(0.0, 0.0, DECK_CENTER_Z)),
        material=platform_gray,
        name="deck_plate",
    )
    platform.visual(
        Box((0.560, 0.900, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, DECK_TOP_Z + 0.002)),
        material=deck_black,
        name="deck_mat",
    )
    platform.visual(
        Box((0.060, 0.720, 0.058)),
        origin=Origin(xyz=(-0.155, 0.0, 0.133)),
        material=platform_gray,
        name="left_frame_rail",
    )
    platform.visual(
        Box((0.060, 0.720, 0.058)),
        origin=Origin(xyz=(0.155, 0.0, 0.133)),
        material=platform_gray,
        name="right_frame_rail",
    )
    platform.visual(
        Box((0.440, 0.060, 0.058)),
        origin=Origin(xyz=(0.0, 0.252, 0.133)),
        material=platform_gray,
        name="front_cross_rail",
    )
    platform.visual(
        Box((0.440, 0.060, 0.058)),
        origin=Origin(xyz=(0.0, -0.252, 0.133)),
        material=platform_gray,
        name="rear_cross_rail",
    )
    platform.visual(
        Box((0.110, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, DECK_LENGTH * 0.5 + 0.007, 0.167)),
        material=platform_gray,
        name="tow_hinge_block",
    )
    platform.visual(
        Box((0.016, 0.022, 0.016)),
        origin=Origin(xyz=(-0.068, DECK_LENGTH * 0.5 + 0.010, 0.170)),
        material=platform_gray,
        name="tow_left_bracket",
    )
    platform.visual(
        Box((0.016, 0.022, 0.016)),
        origin=Origin(xyz=(0.068, DECK_LENGTH * 0.5 + 0.010, 0.170)),
        material=platform_gray,
        name="tow_right_bracket",
    )
    platform.inertial = Inertial.from_geometry(
        Box((DECK_WIDTH, DECK_LENGTH, 0.180)),
        mass=27.0,
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
    )

    tow_handle = model.part("tow_handle")
    tow_handle.visual(_tow_loop_mesh(), material=caster_dark, name="tow_loop")
    tow_handle.visual(
        Cylinder(radius=0.010, length=0.052),
        origin=Origin(xyz=(0.0, 0.078, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grip_black,
        name="tow_grip",
    )
    tow_handle.inertial = Inertial.from_geometry(
        Box((0.120, 0.090, 0.025)),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.045, 0.0)),
    )

    model.articulation(
        "platform_to_tow_handle",
        ArticulationType.REVOLUTE,
        parent=platform,
        child=tow_handle,
        origin=Origin(xyz=(0.0, DECK_LENGTH * 0.5 + 0.004, 0.178)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.2,
            lower=0.0,
            upper=1.10,
        ),
    )

    for label, position in CASTER_POSITIONS.items():
        _add_caster(
            model,
            platform,
            label=label,
            position=position,
            caster_metal=caster_metal,
            caster_dark=caster_dark,
            wheel_rubber=wheel_rubber,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    platform = object_model.get_part("platform")
    tow_handle = object_model.get_part("tow_handle")
    tow_hinge = object_model.get_articulation("platform_to_tow_handle")

    caster_labels = tuple(CASTER_POSITIONS.keys())
    caster_parts = [object_model.get_part(f"{label}_caster") for label in caster_labels]
    wheel_parts = [object_model.get_part(f"{label}_wheel") for label in caster_labels]
    swivel_joints = [object_model.get_articulation(f"platform_to_{label}_caster") for label in caster_labels]
    spin_joints = [object_model.get_articulation(f"{label}_caster_to_wheel") for label in caster_labels]

    ctx.check(
        "tow hinge uses a front-mounted upward-folding revolute joint",
        tow_hinge.articulation_type == ArticulationType.REVOLUTE
        and tow_hinge.axis == (1.0, 0.0, 0.0)
        and tow_hinge.motion_limits is not None
        and tow_hinge.motion_limits.lower == 0.0
        and tow_hinge.motion_limits.upper is not None
        and tow_hinge.motion_limits.upper >= 1.0,
        details=f"type={tow_hinge.articulation_type}, axis={tow_hinge.axis}, limits={tow_hinge.motion_limits}",
    )

    for label, caster in zip(caster_labels, caster_parts):
        ctx.expect_contact(
            caster,
            platform,
            elem_a="mount_plate",
            elem_b="deck_plate",
            name=f"{label} caster mount plate meets the platform deck",
        )

    for label, wheel in zip(caster_labels, wheel_parts):
        ctx.expect_gap(
            platform,
            wheel,
            axis="z",
            min_gap=0.028,
            max_gap=0.050,
            positive_elem="deck_plate",
            negative_elem="wheel_shell",
            name=f"{label} wheel stays below the platform deck",
        )

    caster_joint_ok = True
    caster_joint_details: list[str] = []
    for label, swivel_joint, spin_joint in zip(caster_labels, swivel_joints, spin_joints):
        swivel_ok = swivel_joint.articulation_type == ArticulationType.CONTINUOUS and swivel_joint.axis == (
            0.0,
            0.0,
            1.0,
        )
        spin_ok = spin_joint.articulation_type == ArticulationType.CONTINUOUS and spin_joint.axis == (1.0, 0.0, 0.0)
        caster_joint_ok = caster_joint_ok and swivel_ok and spin_ok
        caster_joint_details.append(
            f"{label}: swivel=({swivel_joint.articulation_type}, {swivel_joint.axis}), "
            f"spin=({spin_joint.articulation_type}, {spin_joint.axis})"
        )
    ctx.check(
        "each caster has continuous swivel and wheel-spin joints on the correct axes",
        caster_joint_ok,
        details="; ".join(caster_joint_details),
    )

    rest_grip = ctx.part_element_world_aabb(tow_handle, elem="tow_grip")
    with ctx.pose({tow_hinge: 1.0}):
        open_grip = ctx.part_element_world_aabb(tow_handle, elem="tow_grip")
    tow_opens_up = (
        rest_grip is not None
        and open_grip is not None
        and ((rest_grip[0][2] + rest_grip[1][2]) * 0.5) + 0.055 < ((open_grip[0][2] + open_grip[1][2]) * 0.5)
    )
    ctx.check(
        "tow loop raises when the hinge opens",
        tow_opens_up,
        details=f"rest_grip={rest_grip}, open_grip={open_grip}",
    )

    front_left_wheel = object_model.get_part("front_left_wheel")
    front_left_swivel = object_model.get_articulation("platform_to_front_left_caster")
    front_left_spin = object_model.get_articulation("front_left_caster_to_wheel")

    def _aabb_dims(aabb):
        return (
            aabb[1][0] - aabb[0][0],
            aabb[1][1] - aabb[0][1],
            aabb[1][2] - aabb[0][2],
        ) if aabb is not None else None

    rest_wheel_aabb = ctx.part_world_aabb(front_left_wheel)
    with ctx.pose({front_left_swivel: math.pi / 2.0}):
        turned_wheel_aabb = ctx.part_world_aabb(front_left_wheel)
    rest_dims = _aabb_dims(rest_wheel_aabb)
    turned_dims = _aabb_dims(turned_wheel_aabb)
    ctx.check(
        "front left caster swivel rotates the wheel footprint by ninety degrees",
        rest_dims is not None
        and turned_dims is not None
        and rest_dims[1] > rest_dims[0] + 0.040
        and turned_dims[0] > turned_dims[1] + 0.040,
        details=f"rest_dims={rest_dims}, turned_dims={turned_dims}",
    )

    rest_marker = ctx.part_element_world_aabb(front_left_wheel, elem="tread_mark")
    with ctx.pose({front_left_spin: math.pi / 2.0}):
        spun_marker = ctx.part_element_world_aabb(front_left_wheel, elem="tread_mark")
    tread_rotates = (
        rest_marker is not None
        and spun_marker is not None
        and ((rest_marker[0][2] + rest_marker[1][2]) * 0.5) > ((spun_marker[0][2] + spun_marker[1][2]) * 0.5) + 0.040
    )
    ctx.check(
        "front left caster wheel spin moves the tread marker around the axle",
        tread_rotates,
        details=f"rest_marker={rest_marker}, spun_marker={spun_marker}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
