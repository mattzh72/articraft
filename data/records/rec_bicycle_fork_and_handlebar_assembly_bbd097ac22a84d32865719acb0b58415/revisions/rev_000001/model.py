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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cyclocross_disc_fork_with_stem_and_handlebar")

    carbon = model.material("carbon", rgba=(0.10, 0.11, 0.12, 1.0))
    stem_black = model.material("stem_black", rgba=(0.13, 0.13, 0.14, 1.0))
    clamp_black = model.material("clamp_black", rgba=(0.08, 0.08, 0.09, 1.0))
    grip_black = model.material("grip_black", rgba=(0.06, 0.06, 0.06, 1.0))
    bolt_steel = model.material("bolt_steel", rgba=(0.55, 0.57, 0.60, 1.0))

    fork = model.part("fork")
    fork.inertial = Inertial.from_geometry(
        Box((0.17, 0.10, 0.76)),
        mass=1.05,
        origin=Origin(xyz=(0.0, 0.01, -0.07)),
    )

    fork.visual(
        Cylinder(radius=0.0143, length=0.32),
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        material=carbon,
        name="steerer_tube",
    )

    crown = ExtrudeGeometry(
        rounded_rect_profile(0.135, 0.050, 0.015, corner_segments=8),
        0.030,
    )
    fork.visual(
        _mesh("fork_crown", crown),
        origin=Origin(xyz=(0.0, 0.0, -0.013)),
        material=carbon,
        name="crown",
    )

    left_blade_geom = tube_from_spline_points(
        [
            (-0.056, 0.000, -0.006),
            (-0.056, 0.008, -0.105),
            (-0.055, 0.018, -0.235),
            (-0.053, 0.028, -0.355),
            (-0.052, 0.034, -0.412),
        ],
        radius=0.0135,
        samples_per_segment=18,
        radial_segments=20,
        cap_ends=True,
    )
    right_blade_geom = tube_from_spline_points(
        [
            (0.056, 0.000, -0.006),
            (0.056, 0.008, -0.105),
            (0.055, 0.018, -0.235),
            (0.053, 0.028, -0.355),
            (0.052, 0.034, -0.412),
        ],
        radius=0.0135,
        samples_per_segment=18,
        radial_segments=20,
        cap_ends=True,
    )
    fork.visual(_mesh("left_blade", left_blade_geom), material=carbon, name="left_blade")
    fork.visual(_mesh("right_blade", right_blade_geom), material=carbon, name="right_blade")

    fork.visual(
        Cylinder(radius=0.012, length=0.022),
        origin=Origin(xyz=(-0.058, 0.035, -0.425), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=carbon,
        name="left_axle_boss",
    )
    fork.visual(
        Cylinder(radius=0.012, length=0.022),
        origin=Origin(xyz=(0.058, 0.035, -0.425), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=carbon,
        name="right_axle_boss",
    )
    fork.visual(
        Box((0.016, 0.020, 0.030)),
        origin=Origin(xyz=(-0.055, 0.034, -0.416)),
        material=carbon,
        name="left_dropout_body",
    )
    fork.visual(
        Box((0.016, 0.020, 0.030)),
        origin=Origin(xyz=(0.055, 0.034, -0.416)),
        material=carbon,
        name="right_dropout_body",
    )

    fork.visual(
        Box((0.014, 0.040, 0.072)),
        origin=Origin(xyz=(-0.045, 0.014, -0.389)),
        material=carbon,
        name="disc_mount_plate",
    )
    fork.visual(
        Cylinder(radius=0.0042, length=0.012),
        origin=Origin(xyz=(-0.037, 0.014, -0.406), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bolt_steel,
        name="disc_mount_boss_lower",
    )
    fork.visual(
        Cylinder(radius=0.0042, length=0.012),
        origin=Origin(xyz=(-0.037, 0.014, -0.372), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bolt_steel,
        name="disc_mount_boss_upper",
    )

    stem = model.part("stem")
    stem.inertial = Inertial.from_geometry(
        Box((0.055, 0.135, 0.060)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.055, -0.004)),
    )

    stem.visual(
        Box((0.012, 0.034, 0.044)),
        origin=Origin(xyz=(-0.0203, 0.0, 0.0)),
        material=clamp_black,
        name="stem_steerer_left_cheek",
    )
    stem.visual(
        Box((0.012, 0.034, 0.044)),
        origin=Origin(xyz=(0.0203, 0.0, 0.0)),
        material=clamp_black,
        name="stem_steerer_right_cheek",
    )
    stem.visual(
        Box((0.040, 0.008, 0.044)),
        origin=Origin(xyz=(0.0, -0.0183, 0.0)),
        material=clamp_black,
        name="stem_steerer_rear_bridge",
    )
    stem.visual(
        Box((0.040, 0.008, 0.044)),
        origin=Origin(xyz=(0.0, 0.0183, 0.0)),
        material=clamp_black,
        name="stem_steerer_front_bridge",
    )
    stem.visual(
        Box((0.028, 0.064, 0.010)),
        origin=Origin(xyz=(0.0, 0.052, 0.018)),
        material=stem_black,
        name="stem_upper_bridge",
    )
    stem.visual(
        Box((0.030, 0.076, 0.012)),
        origin=Origin(xyz=(0.0, 0.054, -0.025)),
        material=stem_black,
        name="stem_lower_bridge",
    )
    stem.visual(
        Box((0.054, 0.036, 0.008)),
        origin=Origin(xyz=(0.0, 0.098, 0.0139)),
        material=clamp_black,
        name="stem_bar_top_jaw",
    )
    stem.visual(
        Box((0.054, 0.038, 0.008)),
        origin=Origin(xyz=(0.0, 0.098, -0.0259)),
        material=clamp_black,
        name="stem_bar_bottom_jaw",
    )
    stem.visual(
        Box((0.054, 0.010, 0.028)),
        origin=Origin(xyz=(0.0, 0.0771, -0.006)),
        material=clamp_black,
        name="stem_bar_rear_jaw",
    )
    stem.visual(
        Box((0.054, 0.008, 0.044)),
        origin=Origin(xyz=(0.0, 0.1179, -0.006)),
        material=clamp_black,
        name="stem_bar_clamp",
    )
    stem.visual(
        Box((0.012, 0.016, 0.036)),
        origin=Origin(xyz=(-0.013, 0.126, -0.006)),
        material=stem_black,
        name="faceplate_left",
    )
    stem.visual(
        Box((0.012, 0.016, 0.036)),
        origin=Origin(xyz=(0.013, 0.126, -0.006)),
        material=stem_black,
        name="faceplate_right",
    )
    stem.visual(
        Cylinder(radius=0.0035, length=0.016),
        origin=Origin(xyz=(-0.013, 0.133, 0.005), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bolt_steel,
        name="faceplate_bolt_upper_left",
    )
    stem.visual(
        Cylinder(radius=0.0035, length=0.016),
        origin=Origin(xyz=(-0.013, 0.133, -0.017), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bolt_steel,
        name="faceplate_bolt_lower_left",
    )
    stem.visual(
        Cylinder(radius=0.0035, length=0.016),
        origin=Origin(xyz=(0.013, 0.133, 0.005), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bolt_steel,
        name="faceplate_bolt_upper_right",
    )
    stem.visual(
        Cylinder(radius=0.0035, length=0.016),
        origin=Origin(xyz=(0.013, 0.133, -0.017), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bolt_steel,
        name="faceplate_bolt_lower_right",
    )

    handlebar = model.part("handlebar")
    handlebar.inertial = Inertial.from_geometry(
        Box((0.760, 0.130, 0.080)),
        mass=0.26,
        origin=Origin(xyz=(0.0, -0.032, 0.012)),
    )

    bar_geom = tube_from_spline_points(
        [
            (-0.340, -0.045, 0.015),
            (-0.230, -0.031, 0.010),
            (-0.110, -0.012, 0.004),
            (0.000, 0.000, 0.000),
            (0.110, -0.012, 0.004),
            (0.230, -0.031, 0.010),
            (0.340, -0.045, 0.015),
        ],
        radius=0.011,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    handlebar.visual(_mesh("flat_handlebar", bar_geom), material=carbon, name="bar_tube")
    handlebar.visual(
        Cylinder(radius=0.0159, length=0.100),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=carbon,
        name="handlebar_center_sleeve",
    )
    handlebar.visual(
        Cylinder(radius=0.0165, length=0.110),
        origin=Origin(xyz=(-0.315, -0.041, 0.014), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grip_black,
        name="left_grip",
    )
    handlebar.visual(
        Cylinder(radius=0.0165, length=0.110),
        origin=Origin(xyz=(0.315, -0.041, 0.014), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grip_black,
        name="right_grip",
    )

    model.articulation(
        "fork_to_stem_angle",
        ArticulationType.REVOLUTE,
        parent=fork,
        child=stem,
        origin=Origin(xyz=(0.0, 0.0, 0.235)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=-0.75,
            upper=0.75,
        ),
    )
    model.articulation(
        "stem_to_handlebar_roll",
        ArticulationType.REVOLUTE,
        parent=stem,
        child=handlebar,
        origin=Origin(xyz=(0.0, 0.098, -0.006)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=-0.35,
            upper=0.35,
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

    fork = object_model.get_part("fork")
    stem = object_model.get_part("stem")
    handlebar = object_model.get_part("handlebar")
    stem_joint = object_model.get_articulation("fork_to_stem_angle")
    bar_joint = object_model.get_articulation("stem_to_handlebar_roll")

    def elem_center(part, elem_name: str):
        aabb = ctx.part_element_world_aabb(part, elem=elem_name)
        if aabb is None:
            return None
        return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))

    ctx.expect_overlap(
        fork,
        stem,
        axes="xy",
        min_overlap=0.028,
        name="stem stays centered on the steerer axis",
    )
    ctx.expect_overlap(
        handlebar,
        stem,
        axes="yz",
        min_overlap=0.030,
        name="handlebar center stays within the stem clamp envelope",
    )

    rest_right = elem_center(handlebar, "right_grip")
    with ctx.pose({stem_joint: 0.45}):
        turned_right = elem_center(handlebar, "right_grip")
    ctx.check(
        "stem rotates around steerer axis",
        rest_right is not None
        and turned_right is not None
        and turned_right[1] > rest_right[1] + 0.10,
        details=f"rest_right={rest_right}, turned_right={turned_right}",
    )

    with ctx.pose({bar_joint: 0.25}):
        rolled_right = elem_center(handlebar, "right_grip")
    ctx.check(
        "handlebar rolls inside stem clamp",
        rest_right is not None
        and rolled_right is not None
        and rolled_right[2] < rest_right[2] - 0.008,
        details=f"rest_right={rest_right}, rolled_right={rolled_right}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
