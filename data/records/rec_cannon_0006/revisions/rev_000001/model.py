from __future__ import annotations

import os

_ORIG_GETCWD = os.getcwd


def _safe_getcwd() -> str:
    try:
        return _ORIG_GETCWD()
    except FileNotFoundError:
        return "/"


os.getcwd = _safe_getcwd

try:
    os.chdir("/")
except OSError:
    pass

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
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def _build_wheel(model: ArticulatedObject, name: str, *, wood, dark_wood, tire_iron) -> None:
    wheel = model.part(name)
    wheel.visual(
        Cylinder(radius=0.078, length=0.120),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_wood,
        name="hub",
    )
    for index, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        wheel.visual(
            Box((0.160, 0.018, 0.045)),
            origin=Origin(
                xyz=(0.158 * math.cos(angle), 0.0, 0.158 * math.sin(angle)),
                rpy=(0.0, angle, 0.0),
            ),
            material=wood,
            name=f"spoke_{index}",
        )
        wheel.visual(
            Box((0.165, 0.045, 0.055)),
            origin=Origin(
                xyz=(0.234 * math.cos(angle), 0.0, 0.234 * math.sin(angle)),
                rpy=(0.0, angle + (math.pi / 2.0), 0.0),
            ),
            material=wood,
            name=f"rim_segment_{index}",
        )
        wheel.visual(
            Box((0.170, 0.016, 0.024)),
            origin=Origin(
                xyz=(0.257 * math.cos(angle), 0.0, 0.257 * math.sin(angle)),
                rpy=(0.0, angle + (math.pi / 2.0), 0.0),
            ),
            material=tire_iron,
            name=f"tire_segment_{index}",
        )
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.268, length=0.045),
        mass=26.0,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="medieval_iron_cannon")

    iron = model.material("iron", rgba=(0.20, 0.21, 0.23, 1.0))
    soot = model.material("soot", rgba=(0.05, 0.05, 0.06, 1.0))
    oak = model.material("oak", rgba=(0.50, 0.33, 0.18, 1.0))
    dark_oak = model.material("dark_oak", rgba=(0.35, 0.23, 0.12, 1.0))
    tire_iron = model.material("tire_iron", rgba=(0.14, 0.15, 0.17, 1.0))

    axle_x = -0.355
    axle_z = 0.266
    wheel_center_y = 0.332
    cheek_y = 0.185
    pivot_x = 0.020
    pivot_z = 0.415
    trunnion_y = 0.122
    trunnion_radius = 0.030

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.860, 0.050, 0.120)),
        origin=Origin(xyz=(-0.140, cheek_y, 0.340)),
        material=oak,
        name="left_cheek",
    )
    carriage.visual(
        Box((0.860, 0.050, 0.120)),
        origin=Origin(xyz=(-0.140, -cheek_y, 0.340)),
        material=oak,
        name="right_cheek",
    )
    carriage.visual(
        Box((0.140, 0.420, 0.080)),
        origin=Origin(xyz=(0.080, 0.0, 0.250)),
        material=oak,
        name="front_transom",
    )
    carriage.visual(
        Box((0.160, 0.420, 0.080)),
        origin=Origin(xyz=(-0.470, 0.0, 0.270)),
        material=oak,
        name="rear_transom",
    )
    carriage.visual(
        Box((0.080, 0.420, 0.050)),
        origin=Origin(xyz=(axle_x, 0.0, 0.241)),
        material=dark_oak,
        name="axle_bed",
    )
    carriage.visual(
        Cylinder(radius=0.034, length=0.664),
        origin=Origin(xyz=(axle_x, 0.0, axle_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_oak,
        name="axle_core",
    )
    carriage.visual(
        Box((0.120, 0.080, 0.060)),
        origin=Origin(xyz=(pivot_x + 0.030, 0.148, pivot_z - 0.060)),
        material=oak,
        name="left_trunnion_block",
    )
    carriage.visual(
        Box((0.120, 0.080, 0.060)),
        origin=Origin(xyz=(pivot_x + 0.030, -0.148, pivot_z - 0.060)),
        material=oak,
        name="right_trunnion_block",
    )
    carriage.visual(
        Box((0.620, 0.140, 0.160)),
        origin=Origin(xyz=(-0.710, 0.0, 0.160)),
        material=dark_oak,
        name="trail_beam",
    )
    carriage.visual(
        Box((0.200, 0.180, 0.100)),
        origin=Origin(xyz=(-1.000, 0.0, 0.050)),
        material=dark_oak,
        name="trail_shoe",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((1.500, 0.460, 0.450)),
        mass=96.0,
        origin=Origin(xyz=(-0.470, 0.0, 0.220)),
    )

    barrel = model.part("barrel")
    barrel.visual(
        Cylinder(radius=0.112, length=0.240),
        origin=Origin(xyz=(-0.200, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="breech_body",
    )
    barrel.visual(
        Cylinder(radius=0.118, length=0.040),
        origin=Origin(xyz=(-0.080, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="breech_ring",
    )
    barrel.visual(
        Cylinder(radius=0.094, length=0.540),
        origin=Origin(xyz=(0.190, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="chase",
    )
    barrel.visual(
        Cylinder(radius=0.086, length=0.140),
        origin=Origin(xyz=(0.530, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="front_taper",
    )
    barrel.visual(
        Cylinder(radius=0.098, length=0.050),
        origin=Origin(xyz=(0.625, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="muzzle_ring",
    )
    barrel.visual(
        Cylinder(radius=trunnion_radius, length=0.065),
        origin=Origin(xyz=(0.000, trunnion_y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="left_trunnion",
    )
    barrel.visual(
        Cylinder(radius=trunnion_radius, length=0.065),
        origin=Origin(xyz=(0.000, -trunnion_y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="right_trunnion",
    )
    barrel.visual(
        Cylinder(radius=0.038, length=0.060),
        origin=Origin(xyz=(-0.350, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="cascabel_neck",
    )
    barrel.visual(
        Sphere(radius=0.038),
        origin=Origin(xyz=(-0.395, 0.0, 0.0)),
        material=iron,
        name="cascabel_knob",
    )
    barrel.visual(
        Cylinder(radius=0.040, length=0.070),
        origin=Origin(xyz=(0.610, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=soot,
        name="bore_shadow",
    )
    barrel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.112, length=1.080),
        mass=155.0,
        origin=Origin(xyz=(0.120, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    _build_wheel(model, "left_wheel", wood=oak, dark_wood=dark_oak, tire_iron=tire_iron)
    _build_wheel(model, "right_wheel", wood=oak, dark_wood=dark_oak, tire_iron=tire_iron)

    model.articulation(
        "barrel_tilt",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=barrel,
        origin=Origin(xyz=(pivot_x, 0.0, pivot_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.9,
            lower=0.0,
            upper=math.radians(10.0),
        ),
    )
    model.articulation(
        "left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=carriage,
        child="left_wheel",
        origin=Origin(xyz=(axle_x, wheel_center_y, axle_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=12.0),
    )
    model.articulation(
        "right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=carriage,
        child="right_wheel",
        origin=Origin(xyz=(axle_x, -wheel_center_y, axle_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=12.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    carriage = object_model.get_part("carriage")
    barrel = object_model.get_part("barrel")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")

    barrel_tilt = object_model.get_articulation("barrel_tilt")
    left_wheel_spin = object_model.get_articulation("left_wheel_spin")
    right_wheel_spin = object_model.get_articulation("right_wheel_spin")

    axle_bed = carriage.get_visual("axle_bed")
    axle_core = carriage.get_visual("axle_core")
    front_transom = carriage.get_visual("front_transom")
    left_trunnion_block = carriage.get_visual("left_trunnion_block")
    right_trunnion_block = carriage.get_visual("right_trunnion_block")
    left_hub = left_wheel.get_visual("hub")
    right_hub = right_wheel.get_visual("hub")
    chase = barrel.get_visual("chase")
    muzzle_ring = barrel.get_visual("muzzle_ring")
    left_trunnion = barrel.get_visual("left_trunnion")
    right_trunnion = barrel.get_visual("right_trunnion")
    bore_shadow = barrel.get_visual("bore_shadow")

    ctx.allow_overlap(
        carriage,
        left_wheel,
        elem_a=axle_core,
        elem_b=left_hub,
        reason="wheel hub sleeves over the wooden axle spindle",
    )
    ctx.allow_overlap(
        carriage,
        right_wheel,
        elem_a=axle_core,
        elem_b=right_hub,
        reason="wheel hub sleeves over the wooden axle spindle",
    )

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(carriage, carriage, elem_a=axle_core, elem_b=axle_bed, name="axle_seats_in_axle_bed")
    ctx.expect_overlap(
        carriage,
        left_wheel,
        axes="yz",
        elem_a=axle_core,
        elem_b=left_hub,
        min_overlap=0.06,
        name="left_hub_wraps_axle",
    )
    ctx.expect_overlap(
        carriage,
        right_wheel,
        axes="yz",
        elem_a=axle_core,
        elem_b=right_hub,
        min_overlap=0.06,
        name="right_hub_wraps_axle",
    )
    ctx.expect_gap(
        barrel,
        carriage,
        axis="z",
        max_gap=0.001,
        max_penetration=0.001,
        positive_elem=left_trunnion,
        negative_elem=left_trunnion_block,
        name="left_trunnion_supported",
    )
    ctx.expect_gap(
        barrel,
        carriage,
        axis="z",
        max_gap=0.001,
        max_penetration=0.001,
        positive_elem=right_trunnion,
        negative_elem=right_trunnion_block,
        name="right_trunnion_supported",
    )
    ctx.expect_origin_distance(left_wheel, right_wheel, axes="xz", max_dist=0.001, name="wheels_share_axle_height")
    ctx.expect_origin_gap(left_wheel, carriage, axis="y", min_gap=0.325, max_gap=0.339, name="left_wheel_offset")
    ctx.expect_origin_gap(carriage, right_wheel, axis="y", min_gap=0.325, max_gap=0.339, name="right_wheel_offset")
    ctx.expect_origin_gap(barrel, carriage, axis="z", min_gap=0.390, max_gap=0.430, name="barrel_above_carriage_origin")
    ctx.expect_gap(
        barrel,
        carriage,
        axis="z",
        min_gap=0.020,
        positive_elem=muzzle_ring,
        negative_elem=front_transom,
        name="barrel_clears_front_transom_at_rest",
    )
    ctx.expect_within(
        barrel,
        barrel,
        axes="yz",
        inner_elem=bore_shadow,
        outer_elem=muzzle_ring,
        margin=0.005,
        name="bore_inside_muzzle_ring",
    )

    limits = barrel_tilt.motion_limits
    if limits is not None and limits.lower is not None and limits.upper is not None:
        with ctx.pose({barrel_tilt: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="barrel_lower_pose_no_overlap")
            ctx.fail_if_isolated_parts(name="barrel_lower_pose_no_floating")
            ctx.expect_gap(
                barrel,
                carriage,
                axis="z",
                min_gap=0.020,
                positive_elem=muzzle_ring,
                negative_elem=front_transom,
                name="barrel_clears_front_transom_depressed",
            )
            ctx.expect_gap(
                barrel,
                carriage,
                axis="z",
                max_gap=0.001,
                max_penetration=0.001,
                positive_elem=left_trunnion,
                negative_elem=left_trunnion_block,
                name="left_trunnion_supported_depressed",
            )
            ctx.expect_gap(
                barrel,
                carriage,
                axis="z",
                max_gap=0.001,
                max_penetration=0.001,
                positive_elem=right_trunnion,
                negative_elem=right_trunnion_block,
                name="right_trunnion_supported_depressed",
            )
        with ctx.pose({barrel_tilt: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="barrel_upper_pose_no_overlap")
            ctx.fail_if_isolated_parts(name="barrel_upper_pose_no_floating")
            ctx.expect_gap(
                barrel,
                carriage,
                axis="z",
                min_gap=0.120,
                positive_elem=muzzle_ring,
                negative_elem=front_transom,
                name="barrel_rises_when_elevated",
            )
            ctx.expect_gap(
                barrel,
                carriage,
                axis="z",
                max_gap=0.001,
                max_penetration=0.001,
                positive_elem=left_trunnion,
                negative_elem=left_trunnion_block,
                name="left_trunnion_supported_elevated",
            )
            ctx.expect_gap(
                barrel,
                carriage,
                axis="z",
                max_gap=0.001,
                max_penetration=0.001,
                positive_elem=right_trunnion,
                negative_elem=right_trunnion_block,
                name="right_trunnion_supported_elevated",
            )

    with ctx.pose({left_wheel_spin: math.pi / 3.0, right_wheel_spin: math.pi / 4.0}):
        ctx.fail_if_parts_overlap_in_current_pose(name="wheel_spin_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="wheel_spin_pose_no_floating")
        ctx.expect_overlap(
            carriage,
            left_wheel,
            axes="yz",
            elem_a=axle_core,
            elem_b=left_hub,
            min_overlap=0.06,
            name="left_hub_keeps_axle_engagement",
        )
        ctx.expect_overlap(
            carriage,
            right_wheel,
            axes="yz",
            elem_a=axle_core,
            elem_b=right_hub,
            min_overlap=0.06,
            name="right_hub_keeps_axle_engagement",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
