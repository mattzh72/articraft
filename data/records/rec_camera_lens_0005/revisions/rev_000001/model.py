from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(filename: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(filename))


def _build_barrel_shell_mesh():
    outer_profile = [
        (0.0315, 0.000),
        (0.0415, 0.004),
        (0.0460, 0.008),
        (0.0460, 0.018),
        (0.0438, 0.028),
        (0.0428, 0.060),
        (0.0428, 0.188),
        (0.0400, 0.206),
        (0.0362, 0.217),
        (0.0362, 0.239),
        (0.0448, 0.241),
        (0.0455, 0.252),
        (0.0442, 0.262),
    ]
    inner_profile = [
        (0.0225, 0.000),
        (0.0225, 0.018),
        (0.0240, 0.040),
        (0.0285, 0.140),
        (0.0315, 0.220),
        (0.0330, 0.248),
        (0.0330, 0.258),
    ]
    shell = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=96,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )
    return shell


def _build_focus_ring_mesh():
    ring = LatheGeometry.from_shell_profiles(
        [
            (0.0468, -0.0090),
            (0.0479, -0.0072),
            (0.0482, 0.0000),
            (0.0479, 0.0072),
            (0.0468, 0.0090),
        ],
        [
            (0.0413, -0.0090),
            (0.0413, 0.0090),
        ],
        segments=96,
        start_cap="flat",
        end_cap="flat",
        lip_samples=6,
    )
    for z_pos in (-0.0060, -0.0020, 0.0020, 0.0060):
        ring.merge(
            TorusGeometry(
                radius=0.0474,
                tube=0.00055,
                radial_segments=16,
                tubular_segments=72,
            ).translate(0.0, 0.0, z_pos)
        )
    return ring


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="prime_telephoto_lens", assets=ASSETS)

    alloy = model.material("dark_anodized_alloy", rgba=(0.20, 0.21, 0.23, 1.0))
    satin_alloy = model.material("satin_alloy", rgba=(0.34, 0.35, 0.38, 1.0))
    ring_rubber = model.material("focus_rubber", rgba=(0.07, 0.07, 0.08, 1.0))
    coated_glass = model.material("coated_glass", rgba=(0.17, 0.25, 0.32, 0.52))

    barrel = model.part("barrel")
    barrel.visual(
        _save_mesh("telephoto_barrel_shell.obj", _build_barrel_shell_mesh()),
        material=alloy,
        name="barrel_shell",
    )
    barrel.visual(
        Cylinder(radius=0.0334, length=0.0060),
        origin=Origin(xyz=(0.0, 0.0, 0.249)),
        material=coated_glass,
        name="front_glass",
    )
    barrel.visual(
        Cylinder(radius=0.0228, length=0.0040),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=coated_glass,
        name="rear_glass",
    )
    barrel.visual(
        Cylinder(radius=0.0462, length=0.0020),
        origin=Origin(xyz=(0.0, 0.0, 0.218)),
        material=satin_alloy,
        name="rear_thrust_collar",
    )
    barrel.visual(
        Cylinder(radius=0.0462, length=0.0020),
        origin=Origin(xyz=(0.0, 0.0, 0.238)),
        material=satin_alloy,
        name="front_thrust_collar",
    )
    barrel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.046, length=0.262),
        mass=1.28,
        origin=Origin(xyz=(0.0, 0.0, 0.131)),
    )

    focus_ring = model.part("focus_ring")
    focus_ring.visual(
        _save_mesh("focus_ring_shell.obj", _build_focus_ring_mesh()),
        material=ring_rubber,
        name="focus_ring_shell",
    )
    focus_ring.visual(
        Box((0.0022, 0.0050, 0.0035)),
        origin=Origin(xyz=(0.0487, 0.0, 0.0)),
        material=satin_alloy,
        name="focus_index",
    )
    focus_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0485, length=0.018),
        mass=0.09,
    )

    model.articulation(
        "focus_ring_rotate",
        ArticulationType.REVOLUTE,
        parent=barrel,
        child=focus_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.228)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=3.5,
            lower=-1.2,
            upper=1.2,
        ),
    )
    return model


def _aabb_center(aabb):
    return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    barrel = object_model.get_part("barrel")
    focus_ring = object_model.get_part("focus_ring")
    focus_joint = object_model.get_articulation("focus_ring_rotate")

    barrel_shell = barrel.get_visual("barrel_shell")
    front_glass = barrel.get_visual("front_glass")
    rear_glass = barrel.get_visual("rear_glass")
    rear_thrust_collar = barrel.get_visual("rear_thrust_collar")
    front_thrust_collar = barrel.get_visual("front_thrust_collar")
    focus_ring_shell = focus_ring.get_visual("focus_ring_shell")
    focus_index = focus_ring.get_visual("focus_index")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    ctx.expect_origin_distance(
        focus_ring,
        barrel,
        axes="xy",
        max_dist=0.001,
        name="focus_ring_is_coaxial_with_barrel",
    )
    ctx.expect_contact(
        focus_ring,
        barrel,
        elem_a=focus_ring_shell,
        elem_b=rear_thrust_collar,
        contact_tol=0.0002,
        name="focus_ring_is_retained_by_rear_thrust_collar",
    )
    ctx.expect_overlap(
        focus_ring,
        barrel,
        axes="xy",
        elem_a=focus_ring_shell,
        elem_b=barrel_shell,
        min_overlap=0.070,
        name="focus_ring_wraps_around_the_barrel_axis",
    )
    ctx.expect_contact(
        focus_ring,
        barrel,
        elem_a=focus_ring_shell,
        elem_b=front_thrust_collar,
        contact_tol=0.0002,
        name="focus_ring_is_retained_by_front_thrust_collar",
    )
    ctx.expect_origin_gap(
        focus_ring,
        barrel,
        axis="z",
        min_gap=0.215,
        max_gap=0.240,
        name="focus_ring_is_positioned_near_the_front_of_the_telephoto_barrel",
    )
    ctx.expect_gap(
        barrel,
        focus_ring,
        axis="z",
        positive_elem=front_glass,
        negative_elem=focus_ring_shell,
        min_gap=0.008,
        max_gap=0.020,
        name="front_glass_sits_just_ahead_of_the_focus_ring",
    )
    ctx.expect_gap(
        focus_ring,
        barrel,
        axis="z",
        positive_elem=focus_ring_shell,
        negative_elem=rear_glass,
        min_gap=0.200,
        name="long_barrel_separates_focus_ring_from_rear_optics",
    )

    barrel_aabb = ctx.part_world_aabb(barrel)
    focus_ring_aabb = ctx.part_world_aabb(focus_ring)
    ctx.check(
        "telephoto_barrel_has_realistic_length",
        barrel_aabb is not None and (barrel_aabb[1][2] - barrel_aabb[0][2]) >= 0.25,
        details=f"barrel_aabb={barrel_aabb}",
    )
    ctx.check(
        "focus_ring_reads_as_narrow_front_band",
        focus_ring_aabb is not None and 0.014 <= (focus_ring_aabb[1][2] - focus_ring_aabb[0][2]) <= 0.024,
        details=f"focus_ring_aabb={focus_ring_aabb}",
    )
    ctx.check(
        "focus_joint_uses_lens_axis",
        tuple(round(value, 6) for value in focus_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={focus_joint.axis}",
    )
    limits = focus_joint.motion_limits
    ctx.check(
        "focus_joint_has_realistic_manual_focus_throw",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and 0.9 <= abs(limits.lower) <= 1.4
        and 0.9 <= abs(limits.upper) <= 1.4,
        details=f"limits={limits}",
    )

    marker_rest = ctx.part_element_world_aabb(focus_ring, elem="focus_index")
    ctx.check(
        "focus_index_marker_is_present",
        marker_rest is not None,
        details=f"focus_index_aabb={marker_rest}",
    )

    if limits is not None and limits.lower is not None and limits.upper is not None:
        with ctx.pose({focus_joint: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="focus_ring_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="focus_ring_lower_no_floating")
            ctx.expect_contact(
                focus_ring,
                barrel,
                elem_a=focus_ring_shell,
                elem_b=rear_thrust_collar,
                contact_tol=0.0002,
                name="focus_ring_lower_pose_stays_retained",
            )
        with ctx.pose({focus_joint: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="focus_ring_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="focus_ring_upper_no_floating")
            ctx.expect_contact(
                focus_ring,
                barrel,
                elem_a=focus_ring_shell,
                elem_b=rear_thrust_collar,
                contact_tol=0.0002,
                name="focus_ring_upper_pose_stays_retained",
            )
            marker_turned = ctx.part_element_world_aabb(focus_ring, elem="focus_index")
            moved = False
            if marker_rest is not None and marker_turned is not None:
                rest_center = _aabb_center(marker_rest)
                turned_center = _aabb_center(marker_turned)
                moved = turned_center[0] < rest_center[0] - 0.010 and turned_center[1] > rest_center[1] + 0.020
            ctx.check(
                "focus_index_rotates_around_the_barrel",
                moved,
                details=f"rest={marker_rest}, turned={marker_turned}",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
