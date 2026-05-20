from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import os

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    wire_from_points,
    mesh_from_geometry,
)

_ORIGINAL_GETCWD = os.getcwd
ASSETS = AssetContext.from_script(__file__)


def _safe_getcwd() -> str:
    try:
        return _ORIGINAL_GETCWD()
    except FileNotFoundError:
        try:
            os.chdir("/")
        except OSError:
            pass
        return "/"


os.getcwd = _safe_getcwd
_safe_getcwd()


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="semi_flush_gimbal_ceiling_light", assets=ASSETS)

    satin_brass = model.material("satin_brass", rgba=(0.79, 0.69, 0.48, 1.0))
    frosted_opal = model.material("frosted_opal", rgba=(0.96, 0.96, 0.94, 0.66))

    canopy_radius = 0.085
    canopy_height = 0.024
    stem_radius = 0.010
    stem_length = 0.020
    knuckle_radius = 0.014
    knuckle_length = 0.014
    axis_drop = 0.020
    pivot_z = -(canopy_height + stem_length + knuckle_length + axis_drop)

    bowl_shell_geom = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.152, -0.120),
            (0.142, -0.102),
            (0.122, -0.074),
            (0.094, -0.040),
            (0.068, -0.032),
            (0.054, -0.022),
            (0.050, -0.020),
        ],
        inner_profile=[
            (0.145, -0.114),
            (0.135, -0.098),
            (0.115, -0.072),
            (0.087, -0.040),
            (0.062, -0.032),
            (0.048, -0.024),
            (0.043, -0.020),
        ],
        segments=72,
    )
    bowl_shell_mesh = _save_mesh("bowl_shade.obj", bowl_shell_geom)

    shade_hardware_geom = CylinderGeometry(radius=0.050, height=0.004).translate(0.0, 0.0, -0.018)
    shade_hardware_geom.merge(
        CylinderGeometry(radius=0.032, height=0.010).translate(0.0, 0.0, -0.012)
    )
    shade_hardware_geom.merge(
        BoxGeometry((0.016, 0.010, 0.008)).translate(-0.044, 0.0, -0.006)
    )
    shade_hardware_geom.merge(
        BoxGeometry((0.016, 0.010, 0.008)).translate(0.044, 0.0, -0.006)
    )
    shade_hardware_geom.merge(
        CylinderGeometry(radius=0.006, height=0.020)
        .rotate_y(math.pi / 2.0)
        .translate(-0.062, 0.0, 0.0)
    )
    shade_hardware_geom.merge(
        CylinderGeometry(radius=0.006, height=0.020)
        .rotate_y(math.pi / 2.0)
        .translate(0.062, 0.0, 0.0)
    )
    shade_hardware_mesh = _save_mesh("shade_hardware.obj", shade_hardware_geom)

    ring_geom = CylinderGeometry(radius=0.010, height=0.010).translate(0.0, 0.0, -0.005)
    ring_geom.merge(
        CylinderGeometry(radius=0.0045, height=0.014).translate(0.0, 0.0, -0.007)
    )
    ring_geom.merge(
        CylinderGeometry(radius=0.006, height=0.024).translate(0.0, 0.0, 0.012)
    )
    ring_geom.merge(
        BoxGeometry((0.010, 0.014, 0.014)).translate(-0.072, 0.0, 0.000)
    )
    ring_geom.merge(
        BoxGeometry((0.010, 0.014, 0.014)).translate(0.072, 0.0, 0.000)
    )
    ring_geom.merge(
        wire_from_points(
            [
                (0.0, 0.038, -0.002),
                (-0.020, 0.038, 0.004),
                (-0.046, 0.034, 0.006),
                (-0.062, 0.024, 0.004),
                (-0.072, 0.008, 0.002),
            ],
            radius=0.004,
            radial_segments=16,
            cap_ends=True,
            corner_mode="fillet",
            corner_radius=0.010,
            corner_segments=10,
        )
    )
    ring_geom.merge(
        wire_from_points(
            [
                (0.0, 0.038, -0.002),
                (0.020, 0.038, 0.004),
                (0.046, 0.034, 0.006),
                (0.062, 0.024, 0.004),
                (0.072, 0.008, 0.002),
            ],
            radius=0.004,
            radial_segments=16,
            cap_ends=True,
            corner_mode="fillet",
            corner_radius=0.010,
            corner_segments=10,
        )
    )
    ring_geom.merge(
        wire_from_points(
            [
                (0.0, -0.038, -0.002),
                (-0.020, -0.038, 0.004),
                (-0.046, -0.034, 0.006),
                (-0.062, -0.024, 0.004),
                (-0.072, -0.008, 0.002),
            ],
            radius=0.004,
            radial_segments=16,
            cap_ends=True,
            corner_mode="fillet",
            corner_radius=0.010,
            corner_segments=10,
        )
    )
    ring_geom.merge(
        wire_from_points(
            [
                (0.0, -0.038, -0.002),
                (0.020, -0.038, 0.004),
                (0.046, -0.034, 0.006),
                (0.062, -0.024, 0.004),
                (0.072, -0.008, 0.002),
            ],
            radius=0.004,
            radial_segments=16,
            cap_ends=True,
            corner_mode="fillet",
            corner_radius=0.010,
            corner_segments=10,
        )
    )
    ring_mesh = _save_mesh("gimbal_ring.obj", ring_geom)

    mount = model.part("mount")
    mount.visual(
        Cylinder(radius=canopy_radius, length=canopy_height),
        origin=Origin(xyz=(0.0, 0.0, -canopy_height / 2.0)),
        material=satin_brass,
        name="canopy",
    )
    mount.visual(
        Cylinder(radius=stem_radius, length=stem_length),
        origin=Origin(xyz=(0.0, 0.0, -(canopy_height + stem_length / 2.0))),
        material=satin_brass,
        name="stem_tube",
    )
    mount.visual(
        Cylinder(radius=knuckle_radius, length=knuckle_length),
        origin=Origin(
            xyz=(0.0, 0.0, -(canopy_height + stem_length + knuckle_length / 2.0))
        ),
        material=satin_brass,
        name="stem_knuckle",
    )
    mount.inertial = Inertial.from_geometry(
        Box((0.19, 0.19, 0.08)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.0, -0.04)),
    )

    ring = model.part("gimbal_ring")
    ring.visual(
        ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0048)),
        material=satin_brass,
        name="ring_frame",
    )
    ring.inertial = Inertial.from_geometry(
        Box((0.16, 0.04, 0.05)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
    )

    shade = model.part("shade")
    shade.visual(
        bowl_shell_mesh,
        material=frosted_opal,
        name="bowl_shell",
    )
    shade.visual(
        shade_hardware_mesh,
        material=satin_brass,
        name="shade_hardware",
    )
    shade.inertial = Inertial.from_geometry(
        Cylinder(radius=0.16, length=0.14),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, -0.045)),
    )

    model.articulation(
        "mount_to_ring",
        ArticulationType.FIXED,
        parent=mount,
        child=ring,
        origin=Origin(xyz=(0.0, 0.0, pivot_z)),
    )
    model.articulation(
        "gimbal_tilt",
        ArticulationType.REVOLUTE,
        parent=ring,
        child=shade,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.2,
            lower=-math.radians(18.0),
            upper=math.radians(18.0),
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    mount = object_model.get_part("mount")
    ring = object_model.get_part("gimbal_ring")
    shade = object_model.get_part("shade")
    mount_to_ring = object_model.get_articulation("mount_to_ring")
    gimbal_tilt = object_model.get_articulation("gimbal_tilt")

    canopy = mount.get_visual("canopy")
    stem_tube = mount.get_visual("stem_tube")
    stem_knuckle = mount.get_visual("stem_knuckle")
    ring_frame = ring.get_visual("ring_frame")
    bowl_shell = shade.get_visual("bowl_shell")
    shade_hardware = shade.get_visual("shade_hardware")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.allow_overlap(
        mount,
        ring,
        elem_a=stem_knuckle,
        elem_b=ring_frame,
        reason="The fixed yoke uses a captured swivel knuckle seated inside the ring's top bearing seat.",
    )
    ctx.allow_overlap(
        ring,
        shade,
        elem_a=ring_frame,
        elem_b=shade_hardware,
        reason="The gimbal ring intentionally encloses the compact tilt-collar hardware around the shade pivots.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=64)

    ctx.expect_overlap(
        mount,
        mount,
        axes="xy",
        min_overlap=0.001,
        elem_a=canopy,
        elem_b=stem_tube,
        name="stem stays centered under canopy",
    )
    ctx.expect_gap(
        mount,
        mount,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=canopy,
        negative_elem=stem_tube,
        name="stem tube seats flush under canopy",
    )
    ctx.expect_gap(
        mount,
        mount,
        axis="z",
        max_gap=0.001,
        max_penetration=1e-6,
        positive_elem=stem_tube,
        negative_elem=stem_knuckle,
        name="stem tube meets brass knuckle",
    )
    ctx.expect_contact(
        mount,
        ring,
        elem_a=stem_knuckle,
        elem_b=ring_frame,
        name="gimbal ring is mounted to the short stem",
    )
    ctx.expect_contact(
        ring,
        shade,
        elem_a=ring_frame,
        elem_b=shade_hardware,
        name="shade pivots on the gimbal ring hardware",
    )
    ctx.expect_within(
        mount,
        shade,
        axes="xy",
        inner_elem=canopy,
        outer_elem=bowl_shell,
        name="wide bowl shade footprint exceeds the canopy footprint",
    )
    ctx.expect_origin_distance(
        mount,
        shade,
        axes="xy",
        max_dist=0.002,
        name="shade hangs centered below the canopy at rest",
    )
    ctx.expect_gap(
        mount,
        shade,
        axis="z",
        min_gap=0.045,
        max_gap=0.080,
        positive_elem=canopy,
        negative_elem=bowl_shell,
        name="semi-flush drop keeps the shade close to the ceiling",
    )

    limits = gimbal_tilt.motion_limits
    assert limits is not None
    assert limits.lower is not None
    assert limits.upper is not None

    for pose_name, pose_value in (
        ("lower", limits.lower),
        ("upper", limits.upper),
    ):
        with ctx.pose({gimbal_tilt: pose_value}):
            ctx.fail_if_parts_overlap_in_current_pose(
                name=f"gimbal_tilt_{pose_name}_no_overlap"
            )
            ctx.fail_if_isolated_parts(name=f"gimbal_tilt_{pose_name}_no_floating")
            ctx.expect_contact(
                ring,
                shade,
                elem_a=ring_frame,
                elem_b=shade_hardware,
                name=f"pivot hardware stays seated at {pose_name} tilt",
            )
            ctx.expect_gap(
                mount,
                shade,
                axis="z",
                min_gap=0.020,
                positive_elem=canopy,
                negative_elem=bowl_shell,
                name=f"shade clears canopy at {pose_name} tilt",
            )

    ctx.expect_contact(
        mount,
        ring,
        elem_a=stem_knuckle,
        elem_b=ring_frame,
        name=f"{mount_to_ring.name} fixed mount remains seated",
    )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
