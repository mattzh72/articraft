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


def _shell_along_x(
    outer_profile: list[tuple[float, float]],
    inner_profile: list[tuple[float, float]],
    name: str,
    *,
    segments: int = 72,
):
    geometry = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=segments,
    )
    geometry.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telephoto_prime_lens")

    lens_white = model.material("lens_white", rgba=(0.92, 0.93, 0.91, 1.0))
    focus_black = model.material("focus_black", rgba=(0.10, 0.10, 0.11, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.18, 0.18, 0.20, 1.0))
    foot_gray = model.material("foot_gray", rgba=(0.42, 0.44, 0.46, 1.0))

    barrel_shell_mesh = _shell_along_x(
        [
            (0.042, 0.024),
            (0.048, 0.038),
            (0.050, 0.112),
            (0.0495, 0.198),
            (0.0495, 0.286),
            (0.0495, 0.362),
            (0.0485, 0.386),
            (0.0485, 0.450),
            (0.049, 0.456),
            (0.052, 0.486),
            (0.056, 0.505),
        ],
        [
            (0.033, 0.024),
            (0.035, 0.112),
            (0.035, 0.362),
            (0.034, 0.505),
        ],
        "barrel_shell",
        segments=84,
    )
    front_bezel_mesh = _shell_along_x(
        [
            (0.052, -0.012),
            (0.054, 0.000),
            (0.055, 0.012),
        ],
        [
            (0.034, -0.012),
            (0.034, 0.012),
        ],
        "front_bezel",
        segments=72,
    )
    focus_ring_mesh = _shell_along_x(
        [
            (0.0575, -0.032),
            (0.0562, -0.027),
            (0.0578, -0.021),
            (0.0563, -0.015),
            (0.0578, -0.009),
            (0.0563, -0.003),
            (0.0578, 0.003),
            (0.0563, 0.009),
            (0.0578, 0.015),
            (0.0563, 0.021),
            (0.0578, 0.027),
            (0.0568, 0.032),
        ],
        [
            (0.0515, -0.032),
            (0.0515, 0.032),
        ],
        "focus_ring_shell",
        segments=88,
    )
    collar_ring_mesh = _shell_along_x(
        [
            (0.067, -0.028),
            (0.069, -0.020),
            (0.069, 0.020),
            (0.067, 0.028),
        ],
        [
            (0.055, -0.028),
            (0.055, 0.028),
        ],
        "tripod_collar_shell",
        segments=88,
    )

    lens_barrel = model.part("lens_barrel")
    lens_barrel.visual(
        Cylinder(radius=0.039, length=0.020),
        origin=Origin(xyz=(0.010, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="rear_mount",
    )
    lens_barrel.visual(
        Cylinder(radius=0.046, length=0.010),
        origin=Origin(xyz=(0.021, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="mount_flange",
    )
    lens_barrel.visual(
        barrel_shell_mesh,
        material=lens_white,
        name="barrel_shell",
    )
    lens_barrel.visual(
        Cylinder(radius=0.0575, length=0.006),
        origin=Origin(xyz=(0.204, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_white,
        name="collar_rear_stop",
    )
    lens_barrel.visual(
        Cylinder(radius=0.0575, length=0.006),
        origin=Origin(xyz=(0.266, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_white,
        name="collar_front_stop",
    )
    lens_barrel.visual(
        Cylinder(radius=0.053, length=0.006),
        origin=Origin(xyz=(0.383, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_white,
        name="focus_rear_stop",
    )
    lens_barrel.visual(
        Cylinder(radius=0.053, length=0.006),
        origin=Origin(xyz=(0.453, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_white,
        name="focus_front_stop",
    )
    lens_barrel.visual(
        front_bezel_mesh,
        origin=Origin(xyz=(0.492, 0.0, 0.0)),
        material=dark_metal,
        name="front_bezel",
    )
    lens_barrel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.058, length=0.505),
        mass=3.1,
        origin=Origin(xyz=(0.2525, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    tripod_collar = model.part("tripod_collar")
    tripod_collar.visual(
        collar_ring_mesh,
        material=lens_white,
        name="collar_ring",
    )
    tripod_collar.visual(
        Box((0.034, 0.028, 0.066)),
        origin=Origin(xyz=(0.0, 0.0, -0.091)),
        material=foot_gray,
        name="collar_stem",
    )
    tripod_collar.visual(
        Box((0.056, 0.028, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.114)),
        material=foot_gray,
        name="collar_saddle",
    )
    tripod_collar.visual(
        Box((0.094, 0.032, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, -0.126)),
        material=foot_gray,
        name="collar_foot",
    )
    tripod_collar.visual(
        Box((0.014, 0.010, 0.020)),
        origin=Origin(xyz=(0.0, 0.060, 0.020)),
        material=foot_gray,
        name="clamp_block",
    )
    tripod_collar.visual(
        Cylinder(radius=0.007, length=0.022),
        origin=Origin(
            xyz=(0.0, 0.071, 0.020),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_metal,
        name="clamp_knob",
    )
    tripod_collar.inertial = Inertial.from_geometry(
        Box((0.094, 0.140, 0.154)),
        mass=0.48,
        origin=Origin(xyz=(0.0, 0.0, -0.048)),
    )

    focus_ring = model.part("focus_ring")
    focus_ring.visual(
        focus_ring_mesh,
        material=focus_black,
        name="focus_ring_shell",
    )
    focus_ring.visual(
        Box((0.012, 0.006, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0585)),
        material=lens_white,
        name="focus_index_tab",
    )
    focus_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.058, length=0.064),
        mass=0.08,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "barrel_to_tripod_collar",
        ArticulationType.REVOLUTE,
        parent=lens_barrel,
        child=tripod_collar,
        origin=Origin(xyz=(0.235, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "barrel_to_focus_ring",
        ArticulationType.REVOLUTE,
        parent=lens_barrel,
        child=focus_ring,
        origin=Origin(xyz=(0.418, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=4.0,
            lower=-1.3,
            upper=1.3,
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

    lens_barrel = object_model.get_part("lens_barrel")
    tripod_collar = object_model.get_part("tripod_collar")
    focus_ring = object_model.get_part("focus_ring")
    collar_joint = object_model.get_articulation("barrel_to_tripod_collar")
    focus_joint = object_model.get_articulation("barrel_to_focus_ring")

    def elem_center(part_obj, elem_name: str):
        aabb = ctx.part_element_world_aabb(part_obj, elem=elem_name)
        if aabb is None:
            return None
        low, high = aabb
        return tuple((low[index] + high[index]) * 0.5 for index in range(3))

    ctx.check(
        "tripod collar rotates about the lens axis",
        collar_joint.axis == (1.0, 0.0, 0.0)
        and collar_joint.motion_limits is not None
        and collar_joint.motion_limits.lower == -math.pi
        and collar_joint.motion_limits.upper == math.pi,
        details=f"axis={collar_joint.axis}, limits={collar_joint.motion_limits}",
    )
    ctx.check(
        "focus ring rotates about the lens axis",
        focus_joint.axis == (1.0, 0.0, 0.0)
        and focus_joint.motion_limits is not None
        and focus_joint.motion_limits.lower is not None
        and focus_joint.motion_limits.upper is not None
        and focus_joint.motion_limits.lower < 0.0 < focus_joint.motion_limits.upper,
        details=f"axis={focus_joint.axis}, limits={focus_joint.motion_limits}",
    )

    ctx.expect_origin_distance(
        tripod_collar,
        lens_barrel,
        axes="yz",
        max_dist=0.001,
        name="tripod collar stays coaxial with the barrel",
    )
    ctx.expect_overlap(
        tripod_collar,
        lens_barrel,
        axes="x",
        min_overlap=0.045,
        name="tripod collar surrounds a barrel section",
    )
    ctx.expect_contact(
        tripod_collar,
        lens_barrel,
        elem_b="collar_rear_stop",
        contact_tol=0.0005,
        name="tripod collar seats against the rear barrel stop",
    )
    ctx.expect_contact(
        tripod_collar,
        lens_barrel,
        elem_b="collar_front_stop",
        contact_tol=0.0005,
        name="tripod collar seats against the front barrel stop",
    )
    ctx.expect_origin_distance(
        focus_ring,
        lens_barrel,
        axes="yz",
        max_dist=0.001,
        name="focus ring stays coaxial with the front barrel",
    )
    ctx.expect_overlap(
        focus_ring,
        lens_barrel,
        axes="x",
        min_overlap=0.060,
        name="focus ring spans the front barrel section",
    )
    ctx.expect_contact(
        focus_ring,
        lens_barrel,
        elem_b="focus_rear_stop",
        contact_tol=0.0005,
        name="focus ring meets the rear focusing stop",
    )
    ctx.expect_contact(
        focus_ring,
        lens_barrel,
        elem_b="focus_front_stop",
        contact_tol=0.0005,
        name="focus ring meets the front focusing stop",
    )

    foot_rest = elem_center(tripod_collar, "collar_foot")
    with ctx.pose({collar_joint: math.pi / 2.0}):
        foot_portrait = elem_center(tripod_collar, "collar_foot")

    ctx.check(
        "tripod foot hangs below the barrel at rest",
        foot_rest is not None and foot_rest[2] < -0.10,
        details=f"foot_rest={foot_rest}",
    )
    ctx.check(
        "tripod foot rotates to the portrait side",
        foot_portrait is not None and foot_portrait[1] > 0.10 and abs(foot_portrait[2]) < 0.03,
        details=f"foot_portrait={foot_portrait}",
    )

    marker_rest = elem_center(focus_ring, "focus_index_tab")
    with ctx.pose({focus_joint: 1.2}):
        marker_turned = elem_center(focus_ring, "focus_index_tab")

    ctx.check(
        "focus index starts above the lens axis",
        marker_rest is not None and marker_rest[2] > 0.05,
        details=f"marker_rest={marker_rest}",
    )
    ctx.check(
        "focus ring marker swings around the barrel",
        marker_turned is not None and marker_turned[1] < -0.045 and marker_turned[2] < 0.03,
        details=f"marker_turned={marker_turned}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
