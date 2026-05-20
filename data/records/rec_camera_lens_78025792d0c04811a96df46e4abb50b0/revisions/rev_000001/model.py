from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    Cylinder,
    mesh_from_geometry,
)


def _shell_mesh_x(
    name: str,
    outer_profile: list[tuple[float, float]],
    inner_profile: list[tuple[float, float]],
    *,
    segments: int = 72,
    start_cap: str = "flat",
    end_cap: str = "flat",
):
    shell = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=segments,
        start_cap=start_cap,
        end_cap=end_cap,
    )
    shell.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(shell, name)


def _aabb_center(aabb):
    if aabb is None:
        return None
    return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="super_telephoto_prime_lens")

    alloy_white = model.material("alloy_white", rgba=(0.86, 0.87, 0.84, 1.0))
    satin_black = model.material("satin_black", rgba=(0.10, 0.11, 0.12, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.16, 0.17, 0.18, 1.0))
    mount_metal = model.material("mount_metal", rgba=(0.58, 0.61, 0.66, 1.0))
    glass_blue = model.material("glass_blue", rgba=(0.12, 0.20, 0.28, 0.92))

    barrel = model.part("barrel")
    barrel.visual(
        _shell_mesh_x(
            "rear_barrel_tube",
            [
                (0.050, -0.004),
                (0.050, 0.112),
            ],
            [
                (0.042, -0.004),
                (0.042, 0.112),
            ],
            segments=88,
        ),
        material=alloy_white,
        name="rear_barrel_tube",
    )
    barrel.visual(
        _shell_mesh_x(
            "rear_mount_adapter",
            [
                (0.034, -0.006),
                (0.040, 0.002),
                (0.046, 0.012),
                (0.050, 0.026),
            ],
            [
                (0.022, -0.006),
                (0.028, 0.002),
                (0.035, 0.012),
                (0.042, 0.026),
            ],
            segments=88,
        ),
        material=mount_metal,
        name="rear_mount_adapter",
    )
    barrel.visual(
        _shell_mesh_x(
            "transition_tube",
            [
                (0.062, 0.108),
                (0.062, 0.185),
            ],
            [
                (0.054, 0.108),
                (0.054, 0.185),
            ],
            segments=88,
        ),
        material=alloy_white,
        name="transition_tube",
    )
    barrel.visual(
        _shell_mesh_x(
            "rear_shoulder",
            [
                (0.050, 0.106),
                (0.056, 0.110),
                (0.062, 0.116),
            ],
            [
                (0.042, 0.106),
                (0.048, 0.110),
                (0.054, 0.116),
            ],
            segments=88,
        ),
        material=alloy_white,
        name="rear_shoulder",
    )
    barrel.visual(
        _shell_mesh_x(
            "collar_land",
            [
                (0.062, 0.181),
                (0.062, 0.249),
            ],
            [
                (0.054, 0.181),
                (0.054, 0.249),
            ],
            segments=88,
        ),
        material=satin_black,
        name="collar_land",
    )
    barrel.visual(
        _shell_mesh_x(
            "forward_tube",
            [
                (0.070, 0.245),
                (0.070, 0.305),
            ],
            [
                (0.062, 0.245),
                (0.062, 0.305),
            ],
            segments=88,
        ),
        material=alloy_white,
        name="forward_tube",
    )
    barrel.visual(
        _shell_mesh_x(
            "focus_land",
            [
                (0.070, 0.301),
                (0.070, 0.363),
            ],
            [
                (0.062, 0.301),
                (0.062, 0.363),
            ],
            segments=88,
        ),
        material=satin_black,
        name="focus_land",
    )
    barrel.visual(
        _shell_mesh_x(
            "objective_tube",
            [
                (0.072, 0.359),
                (0.078, 0.376),
                (0.084, 0.444),
                (0.084, 0.458),
            ],
            [
                (0.064, 0.359),
                (0.070, 0.376),
                (0.074, 0.444),
                (0.074, 0.458),
            ],
            segments=88,
        ),
        material=alloy_white,
        name="objective_tube",
    )
    barrel.visual(
        _shell_mesh_x(
            "mount_plate",
            [
                (0.034, -0.006),
                (0.034, 0.001),
            ],
            [
                (0.022, -0.006),
                (0.022, 0.001),
            ],
            segments=64,
        ),
        material=mount_metal,
        name="mount_plate",
    )
    for index, angle in enumerate((math.radians(15.0), math.radians(135.0), math.radians(255.0))):
        barrel.visual(
            Box((0.003, 0.012, 0.004)),
            origin=Origin(
                xyz=(-0.0075, 0.028 * math.cos(angle), 0.028 * math.sin(angle)),
                rpy=(angle + math.pi / 2.0, 0.0, 0.0),
            ),
            material=mount_metal,
            name=f"bayonet_lug_{index}",
        )
    barrel.visual(
        _shell_mesh_x(
            "front_bezel",
            [
                (0.085, 0.442),
                (0.085, 0.460),
            ],
            [
                (0.074, 0.442),
                (0.074, 0.460),
            ],
            segments=88,
        ),
        material=satin_black,
        name="front_bezel",
    )
    barrel.visual(
        Cylinder(radius=0.0745, length=0.006),
        origin=Origin(xyz=(0.446, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass_blue,
        name="front_glass",
    )
    barrel.inertial = Inertial.from_geometry(
        Box((0.466, 0.170, 0.170)),
        mass=3.9,
        origin=Origin(xyz=(0.227, 0.0, 0.0)),
    )

    collar = model.part("tripod_collar")
    collar.visual(
        _shell_mesh_x(
            "collar_body",
            [
                (0.072, -0.032),
                (0.072, -0.022),
                (0.078, -0.018),
                (0.078, 0.018),
                (0.072, 0.022),
                (0.072, 0.032),
            ],
            [
                (0.066, -0.032),
                (0.066, -0.022),
                (0.066, -0.018),
                (0.066, 0.018),
                (0.066, 0.022),
                (0.066, 0.032),
            ],
            segments=80,
        ),
        material=alloy_white,
        name="collar_body",
    )
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        collar.visual(
            Cylinder(radius=0.004, length=0.048),
            origin=Origin(
                xyz=(0.0, 0.066 * math.cos(angle), 0.066 * math.sin(angle)),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=satin_black,
            name=f"collar_pad_{index}",
        )
    collar.visual(
        Box((0.062, 0.044, 0.031)),
        origin=Origin(xyz=(0.0, 0.0, -0.0935)),
        material=alloy_white,
        name="foot_pedestal",
    )
    collar.visual(
        Box((0.130, 0.050, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, -0.116)),
        material=alloy_white,
        name="foot_plate",
    )
    collar.visual(
        Box((0.085, 0.030, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, -0.1245)),
        material=dark_rubber,
        name="foot_pad",
    )
    collar.inertial = Inertial.from_geometry(
        Box((0.130, 0.080, 0.160)),
        mass=0.65,
        origin=Origin(xyz=(0.0, 0.0, -0.062)),
    )

    focus_ring = model.part("focus_ring")
    focus_ring.visual(
        _shell_mesh_x(
            "focus_ring_shell",
            [
                (0.079, -0.029),
                (0.082, -0.024),
                (0.079, -0.019),
                (0.082, -0.014),
                (0.079, -0.009),
                (0.082, -0.004),
                (0.079, 0.001),
                (0.082, 0.006),
                (0.079, 0.011),
                (0.082, 0.016),
                (0.079, 0.021),
                (0.082, 0.026),
                (0.079, 0.029),
            ],
            [
                (0.074, -0.029),
                (0.074, -0.024),
                (0.074, -0.019),
                (0.074, -0.014),
                (0.074, -0.009),
                (0.074, -0.004),
                (0.074, 0.001),
                (0.074, 0.006),
                (0.074, 0.011),
                (0.074, 0.016),
                (0.074, 0.021),
                (0.074, 0.026),
                (0.074, 0.029),
            ],
            segments=88,
        ),
        material=dark_rubber,
        name="focus_ring_shell",
    )
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        focus_ring.visual(
            Cylinder(radius=0.004, length=0.042),
            origin=Origin(
                xyz=(0.0, 0.074 * math.cos(angle), 0.074 * math.sin(angle)),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=satin_black,
            name=f"focus_pad_{index}",
        )
    focus_ring.visual(
        Box((0.010, 0.005, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.081)),
        material=satin_black,
        name="focus_index_rib",
    )
    focus_ring.inertial = Inertial.from_geometry(
        Box((0.058, 0.166, 0.166)),
        mass=0.24,
        origin=Origin(),
    )

    model.articulation(
        "barrel_to_tripod_collar",
        ArticulationType.CONTINUOUS,
        parent=barrel,
        child=collar,
        origin=Origin(xyz=(0.215, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=3.5),
    )
    model.articulation(
        "barrel_to_focus_ring",
        ArticulationType.REVOLUTE,
        parent=barrel,
        child=focus_ring,
        origin=Origin(xyz=(0.332, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.8,
            velocity=3.0,
            lower=-1.8,
            upper=1.8,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    barrel = object_model.get_part("barrel")
    collar = object_model.get_part("tripod_collar")
    focus_ring = object_model.get_part("focus_ring")
    collar_spin = object_model.get_articulation("barrel_to_tripod_collar")
    focus_spin = object_model.get_articulation("barrel_to_focus_ring")

    barrel.get_visual("mount_plate")
    collar.get_visual("foot_plate")
    focus_ring.get_visual("focus_index_rib")

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
        "tripod collar uses continuous rotation around lens axis",
        collar_spin.articulation_type == ArticulationType.CONTINUOUS
        and collar_spin.motion_limits is not None
        and abs(collar_spin.axis[0]) > 0.99
        and abs(collar_spin.axis[1]) < 1e-6
        and abs(collar_spin.axis[2]) < 1e-6,
        details=f"type={collar_spin.articulation_type}, axis={collar_spin.axis}",
    )
    ctx.check(
        "focus ring rotates on the optical axis with realistic travel",
        focus_spin.articulation_type == ArticulationType.REVOLUTE
        and focus_spin.motion_limits is not None
        and abs(focus_spin.axis[0]) > 0.99
        and focus_spin.motion_limits.lower is not None
        and focus_spin.motion_limits.upper is not None
        and focus_spin.motion_limits.lower < 0.0 < focus_spin.motion_limits.upper
        and (focus_spin.motion_limits.upper - focus_spin.motion_limits.lower) > 3.0,
        details=f"type={focus_spin.articulation_type}, axis={focus_spin.axis}, limits={focus_spin.motion_limits}",
    )
    ctx.expect_contact(
        collar,
        barrel,
        elem_a="collar_pad_0",
        elem_b="collar_land",
        name="tripod collar support pads seat on the barrel land",
    )
    ctx.expect_overlap(
        collar,
        barrel,
        axes="x",
        elem_a="collar_body",
        elem_b="collar_land",
        min_overlap=0.055,
        name="tripod collar spans a substantial clamping land",
    )
    ctx.expect_contact(
        focus_ring,
        barrel,
        elem_a="focus_pad_0",
        elem_b="focus_land",
        name="focus ring support pads ride on the front barrel land",
    )
    ctx.expect_overlap(
        focus_ring,
        barrel,
        axes="x",
        elem_a="focus_ring_shell",
        elem_b="focus_land",
        min_overlap=0.050,
        name="focus ring covers a full hand-width grip band",
    )

    barrel_aabb = ctx.part_world_aabb(barrel)
    barrel_length = None if barrel_aabb is None else barrel_aabb[1][0] - barrel_aabb[0][0]
    barrel_diameter = None if barrel_aabb is None else barrel_aabb[1][2] - barrel_aabb[0][2]
    ctx.check(
        "lens proportions read as a super telephoto prime",
        barrel_length is not None
        and barrel_diameter is not None
        and 0.44 <= barrel_length <= 0.48
        and 0.16 <= barrel_diameter <= 0.18,
        details=f"length={barrel_length}, diameter={barrel_diameter}",
    )

    mount_plate_aabb = ctx.part_element_world_aabb(barrel, elem="mount_plate")
    focus_pos = ctx.part_world_position(focus_ring)
    collar_pos = ctx.part_world_position(collar)
    ctx.check(
        "rear bayonet mount plate projects behind the main barrel",
        mount_plate_aabb is not None and mount_plate_aabb[0][0] < -0.005,
        details=f"mount_plate_aabb={mount_plate_aabb}",
    )
    ctx.check(
        "focus ring sits near the front element group",
        focus_pos is not None and 0.30 <= focus_pos[0] <= 0.36,
        details=f"focus_position={focus_pos}",
    )
    ctx.check(
        "tripod collar sits near the barrel midpoint",
        collar_pos is not None and 0.18 <= collar_pos[0] <= 0.25,
        details=f"collar_position={collar_pos}",
    )

    rest_foot_center = _aabb_center(ctx.part_element_world_aabb(collar, elem="foot_plate"))
    with ctx.pose({collar_spin: math.pi / 2.0}):
        quarter_turn_foot_center = _aabb_center(ctx.part_element_world_aabb(collar, elem="foot_plate"))
    ctx.check(
        "tripod foot swings around the barrel when the collar rotates",
        rest_foot_center is not None
        and quarter_turn_foot_center is not None
        and rest_foot_center[2] < -0.10
        and abs(quarter_turn_foot_center[2]) < 0.02
        and quarter_turn_foot_center[1] > 0.10,
        details=f"rest={rest_foot_center}, quarter_turn={quarter_turn_foot_center}",
    )

    rest_index_center = _aabb_center(ctx.part_element_world_aabb(focus_ring, elem="focus_index_rib"))
    with ctx.pose({focus_spin: 1.0}):
        turned_index_center = _aabb_center(ctx.part_element_world_aabb(focus_ring, elem="focus_index_rib"))
    ctx.check(
        "focus ring visibly rotates its grip index around the barrel",
        rest_index_center is not None
        and turned_index_center is not None
        and rest_index_center[2] > 0.07
        and abs(turned_index_center[1] - rest_index_center[1]) > 0.05
        and turned_index_center[2] < rest_index_center[2] - 0.02,
        details=f"rest={rest_index_center}, turned={turned_index_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
