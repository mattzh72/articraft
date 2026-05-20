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


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _build_dome_shade_mesh():
    shell = LatheGeometry.from_shell_profiles(
        [
            (0.024, -0.018),
            (0.052, -0.036),
            (0.090, -0.072),
            (0.126, -0.112),
            (0.152, -0.148),
            (0.160, -0.170),
        ],
        [
            (0.018, -0.022),
            (0.046, -0.040),
            (0.084, -0.075),
            (0.119, -0.113),
            (0.145, -0.147),
            (0.152, -0.166),
        ],
        segments=80,
        start_cap="flat",
        end_cap="flat",
    )
    shell.merge(
        TorusGeometry(
            radius=0.153,
            tube=0.004,
            radial_segments=18,
            tubular_segments=72,
        ).translate(0.0, 0.0, -0.167)
    )
    return shell


def _build_canopy_shell_mesh():
    shell = LatheGeometry.from_shell_profiles(
        [
            (0.010, 0.012),
            (0.026, 0.014),
            (0.044, 0.019),
            (0.058, 0.027),
            (0.061, 0.034),
        ],
        [
            (0.006, 0.015),
            (0.022, 0.017),
            (0.040, 0.021),
            (0.026, 0.028),
            (0.006, 0.032),
        ],
        segments=72,
        start_cap="flat",
        end_cap="flat",
    )
    shell.merge(
        TorusGeometry(
            radius=0.059,
            tube=0.0025,
            radial_segments=14,
            tubular_segments=64,
        ).translate(0.0, 0.0, 0.031)
    )
    return shell


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pendant_ceiling_light", assets=ASSETS)

    brass = model.material("brass", rgba=(0.66, 0.58, 0.40, 1.0))
    fabric_black = model.material("fabric_black", rgba=(0.10, 0.10, 0.11, 1.0))
    enamel_cream = model.material("enamel_cream", rgba=(0.90, 0.88, 0.83, 1.0))
    socket_black = model.material("socket_black", rgba=(0.14, 0.14, 0.15, 1.0))

    canopy = model.part("canopy")
    canopy.visual(
        _save_mesh("canopy_shell.obj", _build_canopy_shell_mesh()),
        material=brass,
        name="canopy_shell",
    )
    canopy.visual(
        Box((0.009, 0.024, 0.030)),
        origin=Origin(xyz=(-0.019, 0.0, 0.0)),
        material=brass,
        name="clevis_left",
    )
    canopy.visual(
        Box((0.009, 0.024, 0.030)),
        origin=Origin(xyz=(0.019, 0.0, 0.0)),
        material=brass,
        name="clevis_right",
    )
    canopy.visual(
        Cylinder(radius=0.012, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.033)),
        material=brass,
        name="ceiling_nipple",
    )
    canopy.inertial = Inertial.from_geometry(
        Cylinder(radius=0.062, length=0.064),
        mass=0.7,
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
    )

    hanger = model.part("hanger")
    hanger.visual(
        Cylinder(radius=0.0065, length=0.029),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="trunnion_barrel",
    )
    hanger.visual(
        Cylinder(radius=0.010, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, -0.014)),
        material=brass,
        name="swivel_stem",
    )
    hanger.visual(
        Cylinder(radius=0.0065, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.034)),
        material=brass,
        name="cord_grip",
    )
    hanger.visual(
        Cylinder(radius=0.0045, length=0.652),
        origin=Origin(xyz=(0.0, 0.0, -0.369)),
        material=fabric_black,
        name="fabric_cord",
    )
    hanger.inertial = Inertial.from_geometry(
        Cylinder(radius=0.014, length=0.700),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, -0.350)),
    )

    shade = model.part("shade")
    shade.visual(
        Cylinder(radius=0.022, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, -0.014)),
        material=brass,
        name="shade_collar",
    )
    shade.visual(
        Cylinder(radius=0.018, length=0.048),
        origin=Origin(xyz=(0.0, 0.0, -0.038)),
        material=socket_black,
        name="socket_body",
    )
    shade.visual(
        _save_mesh("dome_shade_shell.obj", _build_dome_shade_mesh()),
        material=enamel_cream,
        name="dome_shade",
    )
    shade.inertial = Inertial.from_geometry(
        Cylinder(radius=0.160, length=0.180),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.0, -0.090)),
    )

    model.articulation(
        "canopy_swivel",
        ArticulationType.REVOLUTE,
        parent=canopy,
        child=hanger,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.2,
            lower=-0.75,
            upper=0.75,
        ),
    )
    model.articulation(
        "hanger_to_shade",
        ArticulationType.FIXED,
        parent=hanger,
        child=shade,
        origin=Origin(xyz=(0.0, 0.0, -0.695)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    canopy = object_model.get_part("canopy")
    hanger = object_model.get_part("hanger")
    shade = object_model.get_part("shade")
    canopy_swivel = object_model.get_articulation("canopy_swivel")
    clevis_left = canopy.get_visual("clevis_left")
    clevis_right = canopy.get_visual("clevis_right")
    trunnion_barrel = hanger.get_visual("trunnion_barrel")
    fabric_cord = hanger.get_visual("fabric_cord")
    shade_collar = shade.get_visual("shade_collar")
    dome_shade = shade.get_visual("dome_shade")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=32)

    ctx.expect_contact(canopy, hanger, elem_a=clevis_left, elem_b=trunnion_barrel)
    ctx.expect_contact(canopy, hanger, elem_a=clevis_right, elem_b=trunnion_barrel)
    ctx.expect_contact(hanger, shade, elem_a=fabric_cord, elem_b=shade_collar)
    ctx.expect_origin_distance(hanger, canopy, axes="xy", max_dist=0.002)
    ctx.expect_origin_distance(shade, canopy, axes="xy", max_dist=0.002)
    ctx.expect_within(hanger, shade, axes="xy", inner_elem=fabric_cord, outer_elem=dome_shade)
    ctx.expect_overlap(shade, canopy, axes="xy", min_overlap=0.120)
    ctx.expect_gap(canopy, shade, axis="z", min_gap=0.500)

    canopy_aabb = ctx.part_world_aabb(canopy)
    shade_aabb = ctx.part_world_aabb(shade)
    rest_pos = ctx.part_world_position(shade)
    if canopy_aabb is not None:
        canopy_diam_x = canopy_aabb[1][0] - canopy_aabb[0][0]
        canopy_diam_y = canopy_aabb[1][1] - canopy_aabb[0][1]
        ctx.check(
            "canopy_realistic_diameter",
            0.11 <= canopy_diam_x <= 0.14 and 0.11 <= canopy_diam_y <= 0.14,
            f"canopy spans {canopy_diam_x:.3f} x {canopy_diam_y:.3f} m",
        )
    else:
        ctx.fail("canopy_has_aabb", "Canopy world bounds were unavailable.")

    if shade_aabb is not None:
        shade_span_x = shade_aabb[1][0] - shade_aabb[0][0]
        shade_span_z = shade_aabb[1][2] - shade_aabb[0][2]
        ctx.check(
            "shade_reads_as_large_dome",
            0.29 <= shade_span_x <= 0.34 and 0.16 <= shade_span_z <= 0.20,
            f"shade spans {shade_span_x:.3f} x {shade_span_z:.3f} m",
        )
    else:
        ctx.fail("shade_has_aabb", "Shade world bounds were unavailable.")

    limits = canopy_swivel.motion_limits
    assert limits is not None

    with ctx.pose({canopy_swivel: limits.lower}):
        ctx.fail_if_parts_overlap_in_current_pose(name="canopy_swivel_lower_no_overlap")
        ctx.fail_if_isolated_parts(name="canopy_swivel_lower_no_floating")
        ctx.expect_contact(canopy, hanger, elem_a=clevis_left, elem_b=trunnion_barrel)
        ctx.expect_contact(canopy, hanger, elem_a=clevis_right, elem_b=trunnion_barrel)
        ctx.expect_contact(hanger, shade, elem_a=fabric_cord, elem_b=shade_collar)
        ctx.expect_gap(canopy, shade, axis="z", min_gap=0.330)

    with ctx.pose({canopy_swivel: limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="canopy_swivel_upper_no_overlap")
        ctx.fail_if_isolated_parts(name="canopy_swivel_upper_no_floating")
        ctx.expect_contact(canopy, hanger, elem_a=clevis_left, elem_b=trunnion_barrel)
        ctx.expect_contact(canopy, hanger, elem_a=clevis_right, elem_b=trunnion_barrel)
        ctx.expect_contact(hanger, shade, elem_a=fabric_cord, elem_b=shade_collar)
        ctx.expect_gap(canopy, shade, axis="z", min_gap=0.330)

    if rest_pos is not None:
        with ctx.pose({canopy_swivel: 0.55}):
            plus_pos = ctx.part_world_position(shade)
        with ctx.pose({canopy_swivel: -0.55}):
            minus_pos = ctx.part_world_position(shade)
        if plus_pos is not None and minus_pos is not None:
            ctx.check(
                "shade_swings_side_to_side",
                plus_pos[1] > 0.30 and minus_pos[1] < -0.30,
                f"shade y positions were {plus_pos[1]:.3f} and {minus_pos[1]:.3f} m",
            )
            ctx.check(
                "shade_lifts_when_swiveled",
                plus_pos[2] > rest_pos[2] + 0.08 and minus_pos[2] > rest_pos[2] + 0.08,
                (
                    f"rest z={rest_pos[2]:.3f}, "
                    f"plus z={plus_pos[2]:.3f}, minus z={minus_pos[2]:.3f}"
                ),
            )
        else:
            ctx.fail("shade_pose_positions_available", "Could not read shade positions in swivel poses.")
    else:
        ctx.fail("shade_rest_position_available", "Could not read shade position at rest.")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
