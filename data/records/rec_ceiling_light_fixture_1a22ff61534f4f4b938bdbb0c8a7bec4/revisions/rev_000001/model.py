from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _merge_meshes(*geometries: MeshGeometry) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _ring_shell(
    *,
    outer_radius: float,
    inner_radius: float,
    z_min: float,
    z_max: float,
    segments: int = 56,
) -> MeshGeometry:
    return LatheGeometry.from_shell_profiles(
        [(outer_radius, z_min), (outer_radius, z_max)],
        [(inner_radius, z_min), (inner_radius, z_max)],
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    )


def _build_threaded_ring_mesh() -> MeshGeometry:
    ring = _ring_shell(
        outer_radius=0.034,
        inner_radius=0.0245,
        z_min=-0.004,
        z_max=0.016,
        segments=56,
    )
    lower_bead = _ring_shell(
        outer_radius=0.036,
        inner_radius=0.026,
        z_min=-0.0055,
        z_max=-0.001,
        segments=56,
    )

    tab = BoxGeometry((0.010, 0.0045, 0.010)).translate(0.0365, 0.0, 0.002)
    grip_tabs = MeshGeometry()
    for angle in (0.0, math.tau / 3.0, 2.0 * math.tau / 3.0):
        grip_tabs.merge(tab.copy().rotate_z(angle))

    return _merge_meshes(ring, lower_bead, grip_tabs)


def _build_globe_glass_mesh() -> MeshGeometry:
    outer_profile = [
        (0.0250, 0.000),
        (0.0280, -0.010),
        (0.0500, -0.026),
        (0.0700, -0.058),
        (0.0740, -0.088),
        (0.0620, -0.118),
        (0.0310, -0.142),
        (0.0100, -0.149),
        (0.0000, -0.152),
    ]
    inner_profile = [
        (0.0210, -0.004),
        (0.0240, -0.012),
        (0.0455, -0.028),
        (0.0655, -0.058),
        (0.0690, -0.088),
        (0.0575, -0.116),
        (0.0280, -0.139),
        (0.0090, -0.145),
        (0.0000, -0.148),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=72,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )


def _shade_mount_xy(index: int, radius: float = 0.092) -> tuple[float, float]:
    angle = index * (math.tau / 3.0)
    return (radius * math.cos(angle), radius * math.sin(angle))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ceiling_fan_light_kit")

    hub_metal = model.material("hub_metal", rgba=(0.64, 0.66, 0.69, 1.0))
    trim_metal = model.material("trim_metal", rgba=(0.55, 0.57, 0.60, 1.0))
    frosted_glass = model.material("frosted_glass", rgba=(0.94, 0.93, 0.89, 0.62))

    fan_hub = model.part("fan_hub")
    fan_hub.visual(
        Cylinder(radius=0.032, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0725)),
        material=hub_metal,
        name="downrod_collar",
    )
    fan_hub.visual(
        Cylinder(radius=0.084, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=hub_metal,
        name="switch_housing_cap",
    )
    fan_hub.visual(
        Cylinder(radius=0.112, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=trim_metal,
        name="switch_housing_skirt",
    )
    fan_hub.visual(
        Cylinder(radius=0.138, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=hub_metal,
        name="holder_plate",
    )
    fan_hub.visual(
        Cylinder(radius=0.038, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=trim_metal,
        name="center_boss",
    )

    for index in range(3):
        mount_x, mount_y = _shade_mount_xy(index)
        fan_hub.visual(
            Cylinder(radius=0.022, length=0.016),
            origin=Origin(xyz=(mount_x, mount_y, -0.015)),
            material=trim_metal,
            name=f"socket_{index + 1}",
        )

    fan_hub.inertial = Inertial.from_geometry(
        Box((0.30, 0.30, 0.12)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
    )

    threaded_ring_mesh = mesh_from_geometry(_build_threaded_ring_mesh(), "threaded_shade_ring")
    globe_glass_mesh = mesh_from_geometry(_build_globe_glass_mesh(), "globe_shade_glass")

    for index in range(3):
        shade = model.part(f"shade_{index + 1}")
        shade.visual(
            threaded_ring_mesh,
            material=trim_metal,
            name="threaded_ring",
        )
        shade.visual(
            globe_glass_mesh,
            material=frosted_glass,
            name="shade_glass",
        )
        shade.inertial = Inertial.from_geometry(
            Cylinder(radius=0.076, length=0.154),
            mass=0.42,
            origin=Origin(xyz=(0.0, 0.0, -0.068)),
        )

        mount_x, mount_y = _shade_mount_xy(index)
        model.articulation(
            f"hub_to_shade_{index + 1}",
            ArticulationType.REVOLUTE,
            parent=fan_hub,
            child=shade,
            origin=Origin(xyz=(mount_x, mount_y, -0.024)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=1.5,
                velocity=4.0,
                lower=-math.tau,
                upper=math.tau,
            ),
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
    fan_hub = object_model.get_part("fan_hub")
    shades = [object_model.get_part(f"shade_{index}") for index in (1, 2, 3)]
    joints = [object_model.get_articulation(f"hub_to_shade_{index}") for index in (1, 2, 3)]

    ctx.check(
        "three globe shades are authored",
        len(shades) == 3 and all(shade is not None for shade in shades),
        details=f"shade_names={[shade.name for shade in shades]}",
    )

    expected_positions = [_shade_mount_xy(index - 1) for index in (1, 2, 3)]
    for index, (shade, joint, (expected_x, expected_y)) in enumerate(
        zip(shades, joints, expected_positions),
        start=1,
    ):
        limits = joint.motion_limits
        axis_ok = tuple(round(value, 6) for value in joint.axis) == (0.0, 0.0, 1.0)
        travel_ok = (
            limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and limits.lower <= -math.pi
            and limits.upper >= math.pi
        )
        ctx.check(
            f"shade {index} uses a vertical threaded-ring joint",
            axis_ok and travel_ok,
            details=f"axis={joint.axis}, limits={limits}",
        )

        rest_origin = ctx.part_world_position(shade)
        rest_glass_aabb = ctx.part_element_world_aabb(shade, elem="shade_glass")
        rested_below_plate = (
            rest_origin is not None
            and rest_glass_aabb is not None
            and abs(rest_origin[0] - expected_x) <= 0.003
            and abs(rest_origin[1] - expected_y) <= 0.003
            and rest_origin[2] < -0.020
            and rest_glass_aabb[1][2] <= rest_origin[2] + 0.002
            and rest_glass_aabb[0][2] <= rest_origin[2] - 0.115
        )
        ctx.check(
            f"shade {index} hangs downward from its socket",
            rested_below_plate,
            details=(
                f"origin={rest_origin}, glass_aabb={rest_glass_aabb}, "
                f"expected_xy=({expected_x:.4f}, {expected_y:.4f})"
            ),
        )

        with ctx.pose({joint: 1.35}):
            turned_origin = ctx.part_world_position(shade)
        ctx.check(
            f"shade {index} turns in place on its threaded ring",
            rest_origin is not None
            and turned_origin is not None
            and abs(rest_origin[0] - turned_origin[0]) <= 1e-6
            and abs(rest_origin[1] - turned_origin[1]) <= 1e-6
            and abs(rest_origin[2] - turned_origin[2]) <= 1e-6,
            details=f"rest_origin={rest_origin}, turned_origin={turned_origin}",
        )

    for first, second in ((0, 1), (1, 2), (2, 0)):
        pos_a = ctx.part_world_position(shades[first])
        pos_b = ctx.part_world_position(shades[second])
        spaced_ok = False
        if pos_a is not None and pos_b is not None:
            dx = pos_a[0] - pos_b[0]
            dy = pos_a[1] - pos_b[1]
            spaced_ok = math.hypot(dx, dy) >= 0.115
        ctx.check(
            f"shades {first + 1} and {second + 1} are evenly spaced around the holder plate",
            spaced_ok,
            details=f"pos_a={pos_a}, pos_b={pos_b}",
        )

    ctx.expect_overlap(
        shades[0],
        fan_hub,
        axes="xy",
        min_overlap=0.020,
        name="one shade stays centered beneath the holder plate footprint",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
