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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


CABINET_WIDTH = 0.80
CABINET_DEPTH = 0.55
CABINET_HEIGHT = 2.05
SIDE_THICKNESS = 0.018
PANEL_THICKNESS = 0.018
BACK_THICKNESS = 0.006
DIVIDER_THICKNESS = 0.018
PLINTH_HEIGHT = 0.080
DOOR_WIDTH = 0.796
DOOR_BACKER_THICKNESS = 0.014
DOOR_FRAME_THICKNESS = 0.006
HINGE_AXIS_X = -0.408
HINGE_AXIS_Y = CABINET_DEPTH * 0.5 + 0.001
HINGE_PIN_RADIUS = 0.004
HINGE_BORE_RADIUS = 0.0046
HINGE_SLEEVE_RADIUS = 0.007
HINGE_LENGTH = 0.120
HINGE_WASHER_RADIUS = 0.010
HINGE_WASHER_THICKNESS = 0.004
HANDLE_PIN_RADIUS = 0.004
HANDLE_BORE_RADIUS = 0.0046
HANDLE_COLLAR_RADIUS = 0.006
HANDLE_COLLAR_LENGTH = 0.020
HANDLE_WASHER_RADIUS = 0.008
HANDLE_WASHER_THICKNESS = 0.004
HANDLE_AXIS_Y = 0.026
DOOR_LEFT_OFFSET = 0.012
UPPER_DOOR_HEIGHT = 1.048
LOWER_DOOR_HEIGHT = 0.872
LOWER_DOOR_CENTER_Z = 0.5375
UPPER_DOOR_CENTER_Z = 1.5045
LOWER_HINGE_ZS = (-0.285, 0.285)
UPPER_HINGE_ZS = (-0.355, 0.355)
HANDLE_X = 0.734
UPPER_HANDLE_SPACING = 0.180
LOWER_HANDLE_SPACING = 0.160
UPPER_HANDLE_LENGTH = 0.300
LOWER_HANDLE_LENGTH = 0.260


def _shell_mesh(name: str, inner_radius: float, outer_radius: float, length: float):
    outer_profile = [
        (outer_radius, -length * 0.5),
        (outer_radius, length * 0.5),
    ]
    inner_profile = [
        (inner_radius, -length * 0.5),
        (inner_radius, length * 0.5),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=48,
            start_cap="flat",
            end_cap="flat",
        ),
        name,
    )


def _add_carcass_hinge(
    carcass,
    *,
    z: float,
    metal,
) -> None:
    carcass.visual(
        Cylinder(radius=HINGE_PIN_RADIUS, length=HINGE_LENGTH),
        origin=Origin(xyz=(HINGE_AXIS_X, HINGE_AXIS_Y, z)),
        material=metal,
        name=f"hinge_pin_{z:.3f}",
    )
    carcass.visual(
        Box((0.020, 0.014, HINGE_LENGTH)),
        origin=Origin(
            xyz=(
                -0.396,
                HINGE_AXIS_Y - 0.014,
                z,
            )
        ),
        material=metal,
        name=f"hinge_leaf_{z:.3f}",
    )
    carcass.visual(
        Cylinder(radius=HINGE_WASHER_RADIUS, length=HINGE_WASHER_THICKNESS),
        origin=Origin(
            xyz=(
                HINGE_AXIS_X,
                HINGE_AXIS_Y,
                z + HINGE_LENGTH * 0.5 + HINGE_WASHER_THICKNESS * 0.5,
            )
        ),
        material=metal,
        name=f"hinge_washer_top_{z:.3f}",
    )
    carcass.visual(
        Cylinder(radius=HINGE_WASHER_RADIUS, length=HINGE_WASHER_THICKNESS),
        origin=Origin(
            xyz=(
                HINGE_AXIS_X,
                HINGE_AXIS_Y,
                z - HINGE_LENGTH * 0.5 - HINGE_WASHER_THICKNESS * 0.5,
            )
        ),
        material=metal,
        name=f"hinge_washer_bottom_{z:.3f}",
    )


def _add_door(
    model: ArticulatedObject,
    *,
    name: str,
    door_height: float,
    hinge_zs: tuple[float, float],
    handle_spacing: float,
    door_mass: float,
    hinge_sleeve_mesh,
    painted,
    panel_paint,
    metal,
) -> None:
    door = model.part(name)

    backer_center_y = DOOR_BACKER_THICKNESS * 0.5 - 0.001
    frame_center_y = DOOR_BACKER_THICKNESS + DOOR_FRAME_THICKNESS * 0.5 - 0.001
    panel_center_y = 0.013
    frame_width = 0.060
    rail_height = 0.070

    door.visual(
        Box((DOOR_WIDTH, DOOR_BACKER_THICKNESS, door_height)),
        origin=Origin(
            xyz=(DOOR_LEFT_OFFSET + DOOR_WIDTH * 0.5, backer_center_y, 0.0)
        ),
        material=painted,
        name="slab",
    )
    door.visual(
        Box((frame_width, DOOR_FRAME_THICKNESS, door_height)),
        origin=Origin(
            xyz=(DOOR_LEFT_OFFSET + frame_width * 0.5, frame_center_y, 0.0)
        ),
        material=painted,
        name="left_stile",
    )
    door.visual(
        Box((frame_width, DOOR_FRAME_THICKNESS, door_height)),
        origin=Origin(
            xyz=(
                DOOR_LEFT_OFFSET + DOOR_WIDTH - frame_width * 0.5,
                frame_center_y,
                0.0,
            )
        ),
        material=painted,
        name="right_stile",
    )
    door.visual(
        Box((DOOR_WIDTH - 2.0 * frame_width, DOOR_FRAME_THICKNESS, rail_height)),
        origin=Origin(
            xyz=(
                DOOR_LEFT_OFFSET + DOOR_WIDTH * 0.5,
                frame_center_y,
                door_height * 0.5 - rail_height * 0.5,
            )
        ),
        material=painted,
        name="top_rail",
    )
    door.visual(
        Box((DOOR_WIDTH - 2.0 * frame_width, DOOR_FRAME_THICKNESS, rail_height)),
        origin=Origin(
            xyz=(
                DOOR_LEFT_OFFSET + DOOR_WIDTH * 0.5,
                frame_center_y,
                -door_height * 0.5 + rail_height * 0.5,
            )
        ),
        material=painted,
        name="bottom_rail",
    )
    door.visual(
        Box((DOOR_WIDTH - 0.150, 0.004, door_height - 0.150)),
        origin=Origin(
            xyz=(DOOR_LEFT_OFFSET + DOOR_WIDTH * 0.5, panel_center_y, 0.0)
        ),
        material=panel_paint,
        name="center_panel",
    )

    for hinge_z in hinge_zs:
        door.visual(
            hinge_sleeve_mesh,
            origin=Origin(xyz=(0.0, 0.0, hinge_z)),
            material=metal,
            name=f"sleeve_{hinge_z:.3f}",
        )
        door.visual(
            Box((0.016, 0.020, HINGE_LENGTH)),
            origin=Origin(xyz=(0.006, 0.017, hinge_z)),
            material=metal,
            name=f"hinge_leaf_{hinge_z:.3f}",
        )

    for mount_z in (-handle_spacing * 0.5, handle_spacing * 0.5):
        door.visual(
            Box((0.018, 0.012, 0.022)),
            origin=Origin(xyz=(HANDLE_X, 0.014, mount_z)),
            material=metal,
            name=f"handle_mount_{mount_z:.3f}",
        )
        door.visual(
            Cylinder(radius=HANDLE_PIN_RADIUS, length=HANDLE_COLLAR_LENGTH),
            origin=Origin(xyz=(HANDLE_X, HANDLE_AXIS_Y, mount_z)),
            material=metal,
            name=f"handle_pin_{mount_z:.3f}",
        )
        door.visual(
            Cylinder(radius=HANDLE_WASHER_RADIUS, length=HANDLE_WASHER_THICKNESS),
            origin=Origin(
                xyz=(
                    HANDLE_X,
                    HANDLE_AXIS_Y,
                    mount_z + HANDLE_COLLAR_LENGTH * 0.5 + HANDLE_WASHER_THICKNESS * 0.5,
                )
            ),
            material=metal,
            name=f"handle_washer_top_{mount_z:.3f}",
        )
        door.visual(
            Cylinder(radius=HANDLE_WASHER_RADIUS, length=HANDLE_WASHER_THICKNESS),
            origin=Origin(
                xyz=(
                    HANDLE_X,
                    HANDLE_AXIS_Y,
                    mount_z - HANDLE_COLLAR_LENGTH * 0.5 - HANDLE_WASHER_THICKNESS * 0.5,
                )
            ),
            material=metal,
            name=f"handle_washer_bottom_{mount_z:.3f}",
        )

    door.inertial = Inertial.from_geometry(
        Box((DOOR_WIDTH, 0.022, door_height)),
        mass=door_mass,
        origin=Origin(xyz=(DOOR_LEFT_OFFSET + DOOR_WIDTH * 0.5, 0.009, 0.0)),
    )


def _add_handle(
    model: ArticulatedObject,
    *,
    name: str,
    grip_length: float,
    spacing: float,
    collar_mesh,
    metal,
) -> None:
    handle = model.part(name)

    for mount_z in (-spacing * 0.5, spacing * 0.5):
        handle.visual(
            collar_mesh,
            origin=Origin(xyz=(0.0, 0.0, mount_z)),
            material=metal,
            name=f"collar_{mount_z:.3f}",
        )
        handle.visual(
            Box((0.014, 0.022, 0.018)),
            origin=Origin(xyz=(0.0, 0.017, mount_z)),
            material=metal,
            name=f"arm_{mount_z:.3f}",
        )

    handle.visual(
        Cylinder(radius=0.008, length=grip_length),
        origin=Origin(xyz=(0.0, 0.044, 0.0)),
        material=metal,
        name="grip",
    )
    handle.visual(
        Box((0.012, 0.020, grip_length - 0.030)),
        origin=Origin(xyz=(0.0, 0.032, 0.0)),
        material=metal,
        name="spine",
    )

    handle.inertial = Inertial.from_geometry(
        Box((0.030, 0.070, grip_length)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.034, 0.0)),
    )


def _aabb_center(aabb):
    return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pantry_cabinet")

    carcass_paint = model.material("carcass_paint", rgba=(0.93, 0.92, 0.88, 1.0))
    panel_paint = model.material("panel_paint", rgba=(0.88, 0.87, 0.83, 1.0))
    interior_wood = model.material("interior_wood", rgba=(0.77, 0.69, 0.56, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.22, 0.23, 0.25, 1.0))
    handle_metal = model.material("handle_metal", rgba=(0.62, 0.63, 0.66, 1.0))

    hinge_sleeve_mesh = _shell_mesh(
        "door_hinge_sleeve",
        inner_radius=HINGE_BORE_RADIUS,
        outer_radius=HINGE_SLEEVE_RADIUS,
        length=HINGE_LENGTH,
    )
    handle_collar_mesh = _shell_mesh(
        "handle_collar",
        inner_radius=HANDLE_BORE_RADIUS,
        outer_radius=HANDLE_COLLAR_RADIUS,
        length=HANDLE_COLLAR_LENGTH,
    )

    carcass = model.part("carcass")
    side_height = CABINET_HEIGHT - PLINTH_HEIGHT
    carcass.visual(
        Box((SIDE_THICKNESS, CABINET_DEPTH, side_height)),
        origin=Origin(
            xyz=(
                -CABINET_WIDTH * 0.5 + SIDE_THICKNESS * 0.5,
                0.0,
                PLINTH_HEIGHT + side_height * 0.5,
            )
        ),
        material=carcass_paint,
        name="left_side",
    )
    carcass.visual(
        Box((SIDE_THICKNESS, CABINET_DEPTH, side_height)),
        origin=Origin(
            xyz=(
                CABINET_WIDTH * 0.5 - SIDE_THICKNESS * 0.5,
                0.0,
                PLINTH_HEIGHT + side_height * 0.5,
            )
        ),
        material=carcass_paint,
        name="right_side",
    )
    carcass.visual(
        Box((CABINET_WIDTH - 2.0 * SIDE_THICKNESS, CABINET_DEPTH, PANEL_THICKNESS)),
        origin=Origin(
            xyz=(0.0, 0.0, PLINTH_HEIGHT + PANEL_THICKNESS * 0.5)
        ),
        material=interior_wood,
        name="bottom_panel",
    )
    carcass.visual(
        Box((CABINET_WIDTH - 2.0 * SIDE_THICKNESS, CABINET_DEPTH, PANEL_THICKNESS)),
        origin=Origin(
            xyz=(0.0, 0.0, CABINET_HEIGHT - PANEL_THICKNESS * 0.5)
        ),
        material=carcass_paint,
        name="top_panel",
    )
    back_height = CABINET_HEIGHT - PLINTH_HEIGHT - 2.0 * PANEL_THICKNESS
    carcass.visual(
        Box((CABINET_WIDTH - 2.0 * SIDE_THICKNESS, BACK_THICKNESS, back_height)),
        origin=Origin(
            xyz=(
                0.0,
                -CABINET_DEPTH * 0.5 + BACK_THICKNESS * 0.5,
                PLINTH_HEIGHT + PANEL_THICKNESS + back_height * 0.5,
            )
        ),
        material=interior_wood,
        name="back_panel",
    )
    carcass.visual(
        Box(
            (
                CABINET_WIDTH - 2.0 * SIDE_THICKNESS,
                CABINET_DEPTH - BACK_THICKNESS,
                DIVIDER_THICKNESS,
            )
        ),
        origin=Origin(xyz=(0.0, BACK_THICKNESS * 0.5, 0.968)),
        material=interior_wood,
        name="divider_rail",
    )
    carcass.visual(
        Box((CABINET_WIDTH - 0.080, CABINET_DEPTH - 0.100, PLINTH_HEIGHT)),
        origin=Origin(xyz=(0.0, -0.050, PLINTH_HEIGHT * 0.5)),
        material=carcass_paint,
        name="plinth",
    )
    carcass.visual(
        Box((CABINET_WIDTH - 2.0 * SIDE_THICKNESS, 0.360, PANEL_THICKNESS)),
        origin=Origin(xyz=(0.0, -0.040, 0.470)),
        material=interior_wood,
        name="lower_shelf",
    )
    carcass.visual(
        Box((CABINET_WIDTH - 2.0 * SIDE_THICKNESS, 0.360, PANEL_THICKNESS)),
        origin=Origin(xyz=(0.0, -0.040, 1.520)),
        material=interior_wood,
        name="upper_shelf",
    )

    for hinge_world_z in (
        LOWER_DOOR_CENTER_Z + LOWER_HINGE_ZS[0],
        LOWER_DOOR_CENTER_Z + LOWER_HINGE_ZS[1],
        UPPER_DOOR_CENTER_Z + UPPER_HINGE_ZS[0],
        UPPER_DOOR_CENTER_Z + UPPER_HINGE_ZS[1],
    ):
        _add_carcass_hinge(carcass, z=hinge_world_z, metal=dark_metal)

    carcass.inertial = Inertial.from_geometry(
        Box((CABINET_WIDTH, CABINET_DEPTH, CABINET_HEIGHT)),
        mass=68.0,
        origin=Origin(xyz=(0.0, 0.0, CABINET_HEIGHT * 0.5)),
    )

    _add_door(
        model,
        name="upper_door",
        door_height=UPPER_DOOR_HEIGHT,
        hinge_zs=UPPER_HINGE_ZS,
        handle_spacing=UPPER_HANDLE_SPACING,
        door_mass=12.0,
        hinge_sleeve_mesh=hinge_sleeve_mesh,
        painted=carcass_paint,
        panel_paint=panel_paint,
        metal=dark_metal,
    )
    _add_door(
        model,
        name="lower_door",
        door_height=LOWER_DOOR_HEIGHT,
        hinge_zs=LOWER_HINGE_ZS,
        handle_spacing=LOWER_HANDLE_SPACING,
        door_mass=10.0,
        hinge_sleeve_mesh=hinge_sleeve_mesh,
        painted=carcass_paint,
        panel_paint=panel_paint,
        metal=dark_metal,
    )

    _add_handle(
        model,
        name="upper_handle",
        grip_length=UPPER_HANDLE_LENGTH,
        spacing=UPPER_HANDLE_SPACING,
        collar_mesh=handle_collar_mesh,
        metal=handle_metal,
    )
    _add_handle(
        model,
        name="lower_handle",
        grip_length=LOWER_HANDLE_LENGTH,
        spacing=LOWER_HANDLE_SPACING,
        collar_mesh=handle_collar_mesh,
        metal=handle_metal,
    )

    upper_door = model.get_part("upper_door")
    lower_door = model.get_part("lower_door")

    model.articulation(
        "carcass_to_upper_door",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=upper_door,
        origin=Origin(xyz=(HINGE_AXIS_X, HINGE_AXIS_Y, UPPER_DOOR_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(108.0),
        ),
    )
    model.articulation(
        "carcass_to_lower_door",
        ArticulationType.REVOLUTE,
        parent=carcass,
        child=lower_door,
        origin=Origin(xyz=(HINGE_AXIS_X, HINGE_AXIS_Y, LOWER_DOOR_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(108.0),
        ),
    )
    model.articulation(
        "upper_door_to_handle",
        ArticulationType.REVOLUTE,
        parent=upper_door,
        child="upper_handle",
        origin=Origin(xyz=(HANDLE_X, HANDLE_AXIS_Y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.5,
            lower=-0.25,
            upper=0.25,
        ),
    )
    model.articulation(
        "lower_door_to_handle",
        ArticulationType.REVOLUTE,
        parent=lower_door,
        child="lower_handle",
        origin=Origin(xyz=(HANDLE_X, HANDLE_AXIS_Y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.5,
            lower=-0.25,
            upper=0.25,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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

    carcass = object_model.get_part("carcass")
    upper_door = object_model.get_part("upper_door")
    lower_door = object_model.get_part("lower_door")
    upper_handle = object_model.get_part("upper_handle")
    lower_handle = object_model.get_part("lower_handle")

    upper_joint = object_model.get_articulation("carcass_to_upper_door")
    lower_joint = object_model.get_articulation("carcass_to_lower_door")
    upper_handle_joint = object_model.get_articulation("upper_door_to_handle")
    lower_handle_joint = object_model.get_articulation("lower_door_to_handle")

    upper_slab = upper_door.get_visual("slab")
    lower_slab = lower_door.get_visual("slab")

    ctx.check(
        "upper door hinge axis is vertical",
        tuple(upper_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={upper_joint.axis}",
    )
    ctx.check(
        "lower door hinge axis is vertical",
        tuple(lower_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={lower_joint.axis}",
    )
    ctx.check(
        "upper handle axis is vertical",
        tuple(upper_handle_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={upper_handle_joint.axis}",
    )
    ctx.check(
        "lower handle axis is vertical",
        tuple(lower_handle_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={lower_handle_joint.axis}",
    )

    ctx.expect_contact(upper_door, carcass, name="upper door supported at rest")
    ctx.expect_contact(lower_door, carcass, name="lower door supported at rest")
    ctx.expect_contact(upper_handle, upper_door, name="upper handle mounted at rest")
    ctx.expect_contact(lower_handle, lower_door, name="lower handle mounted at rest")
    ctx.expect_gap(
        upper_door,
        lower_door,
        axis="z",
        min_gap=0.002,
        max_gap=0.020,
        positive_elem=upper_slab,
        negative_elem=lower_slab,
        name="upper and lower doors keep a seam gap",
    )

    upper_rest = ctx.part_world_aabb(upper_door)
    lower_rest = ctx.part_world_aabb(lower_door)
    upper_grip_rest = ctx.part_element_world_aabb(upper_handle, elem="grip")
    lower_grip_rest = ctx.part_element_world_aabb(lower_handle, elem="grip")
    assert upper_rest is not None
    assert lower_rest is not None
    assert upper_grip_rest is not None
    assert lower_grip_rest is not None

    with ctx.pose({upper_joint: math.radians(78.0)}):
        ctx.expect_contact(upper_door, carcass, name="upper door remains clipped into hinges")
        upper_open = ctx.part_world_aabb(upper_door)
        assert upper_open is not None
        upper_rest_center = _aabb_center(upper_rest)
        upper_open_center = _aabb_center(upper_open)
        ctx.check(
            "upper door swings forward",
            upper_open_center[1] > upper_rest_center[1] + 0.22
            and upper_open_center[0] < upper_rest_center[0] - 0.10,
            details=f"rest={upper_rest_center}, open={upper_open_center}",
        )

    with ctx.pose({lower_joint: math.radians(82.0)}):
        ctx.expect_contact(lower_door, carcass, name="lower door remains clipped into hinges")
        lower_open = ctx.part_world_aabb(lower_door)
        assert lower_open is not None
        lower_rest_center = _aabb_center(lower_rest)
        lower_open_center = _aabb_center(lower_open)
        ctx.check(
            "lower door swings forward",
            lower_open_center[1] > lower_rest_center[1] + 0.22
            and lower_open_center[0] < lower_rest_center[0] - 0.10,
            details=f"rest={lower_rest_center}, open={lower_open_center}",
        )

    with ctx.pose({upper_handle_joint: 0.18}):
        ctx.expect_contact(upper_handle, upper_door, name="upper handle stays on its mounts")
        upper_grip_open = ctx.part_element_world_aabb(upper_handle, elem="grip")
        assert upper_grip_open is not None
        upper_grip_rest_center = _aabb_center(upper_grip_rest)
        upper_grip_open_center = _aabb_center(upper_grip_open)
        ctx.check(
            "upper handle rotates slightly",
            upper_grip_open_center[0] < upper_grip_rest_center[0] - 0.004,
            details=f"rest={upper_grip_rest_center}, open={upper_grip_open_center}",
        )

    with ctx.pose({lower_handle_joint: -0.18}):
        ctx.expect_contact(lower_handle, lower_door, name="lower handle stays on its mounts")
        lower_grip_open = ctx.part_element_world_aabb(lower_handle, elem="grip")
        assert lower_grip_open is not None
        lower_grip_rest_center = _aabb_center(lower_grip_rest)
        lower_grip_open_center = _aabb_center(lower_grip_open)
        ctx.check(
            "lower handle rotates slightly",
            lower_grip_open_center[0] > lower_grip_rest_center[0] + 0.004,
            details=f"rest={lower_grip_rest_center}, open={lower_grip_open_center}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
