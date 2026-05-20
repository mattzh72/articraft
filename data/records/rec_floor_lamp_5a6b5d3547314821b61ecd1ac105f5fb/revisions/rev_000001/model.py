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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="floor_spotlight")

    cast_iron = model.material("cast_iron", rgba=(0.15, 0.15, 0.16, 1.0))
    satin_black = model.material("satin_black", rgba=(0.10, 0.10, 0.11, 1.0))
    powder_coat = model.material("powder_coat", rgba=(0.18, 0.18, 0.20, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.24, 0.24, 0.26, 1.0))
    glass = model.material("glass", rgba=(0.80, 0.89, 0.95, 0.35))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.165, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=satin_black,
        name="rubber_foot",
    )
    base.visual(
        Cylinder(radius=0.180, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0325)),
        material=cast_iron,
        name="base_disc",
    )
    base.visual(
        Cylinder(radius=0.122, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.0630)),
        material=powder_coat,
        name="top_plinth",
    )
    base.visual(
        Cylinder(radius=0.022, length=0.710),
        origin=Origin(xyz=(0.0, 0.0, 0.4260)),
        material=powder_coat,
        name="post_tube",
    )
    base.visual(
        Cylinder(radius=0.045, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.8060)),
        material=dark_gray,
        name="azimuth_bearing_base",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.36, 0.36, 0.83)),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.0, 0.415)),
    )

    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.052, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=dark_gray,
        name="turntable_disc",
    )
    pan_head.visual(
        Cylinder(radius=0.031, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=powder_coat,
        name="bearing_stem",
    )
    pan_head.visual(
        Box((0.070, 0.055, 0.130)),
        origin=Origin(xyz=(0.050, 0.0, 0.085)),
        material=powder_coat,
        name="yoke_spine",
    )
    pan_head.visual(
        Box((0.045, 0.188, 0.022)),
        origin=Origin(xyz=(0.048, 0.0, 0.032)),
        material=powder_coat,
        name="lower_bridge",
    )
    pan_head.visual(
        Box((0.055, 0.188, 0.022)),
        origin=Origin(xyz=(0.055, 0.0, 0.153)),
        material=powder_coat,
        name="upper_bridge",
    )
    pan_head.visual(
        Box((0.075, 0.018, 0.140)),
        origin=Origin(xyz=(0.095, 0.103, 0.090)),
        material=powder_coat,
        name="left_yoke_arm",
    )
    pan_head.visual(
        Box((0.075, 0.018, 0.140)),
        origin=Origin(xyz=(0.095, -0.103, 0.090)),
        material=powder_coat,
        name="right_yoke_arm",
    )
    pan_head.inertial = Inertial.from_geometry(
        Box((0.16, 0.23, 0.18)),
        mass=2.2,
        origin=Origin(xyz=(0.070, 0.0, 0.090)),
    )

    housing = model.part("housing")
    housing.visual(
        Cylinder(radius=0.014, length=0.018),
        origin=Origin(xyz=(0.0, 0.085, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_gray,
        name="left_trunnion",
    )
    housing.visual(
        Cylinder(radius=0.014, length=0.018),
        origin=Origin(xyz=(0.0, -0.085, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_gray,
        name="right_trunnion",
    )
    housing.visual(
        Cylinder(radius=0.060, length=0.030),
        origin=Origin(xyz=(-0.010, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_gray,
        name="rear_vent_ring",
    )
    housing.visual(
        Cylinder(radius=0.068, length=0.040),
        origin=Origin(xyz=(-0.004, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=powder_coat,
        name="rear_cap",
    )
    housing.visual(
        Cylinder(radius=0.078, length=0.180),
        origin=Origin(xyz=(0.060, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=powder_coat,
        name="main_can",
    )
    housing.visual(
        Cylinder(radius=0.092, length=0.030),
        origin=Origin(xyz=(0.157, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_gray,
        name="front_bezel",
    )
    housing.visual(
        Cylinder(radius=0.073, length=0.005),
        origin=Origin(xyz=(0.160, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="front_lens",
    )
    housing.inertial = Inertial.from_geometry(
        Box((0.22, 0.19, 0.19)),
        mass=3.2,
        origin=Origin(xyz=(0.080, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_pan",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, 0.831)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=4.0),
    )
    model.articulation(
        "pan_to_housing",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=housing,
        origin=Origin(xyz=(0.125, 0.0, 0.090)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.8,
            lower=math.radians(-55.0),
            upper=math.radians(70.0),
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

    base = object_model.get_part("base")
    pan_head = object_model.get_part("pan_head")
    housing = object_model.get_part("housing")
    pan_joint = object_model.get_articulation("base_to_pan")
    tilt_joint = object_model.get_articulation("pan_to_housing")

    ctx.expect_contact(
        pan_head,
        base,
        elem_a="turntable_disc",
        elem_b="azimuth_bearing_base",
        name="turntable sits on the bearing base",
    )
    ctx.expect_contact(
        housing,
        pan_head,
        name="housing is physically supported by the yoke",
    )

    rest_pos = ctx.part_world_position(housing)
    with ctx.pose({pan_joint: math.pi / 2.0}):
        turned_pos = ctx.part_world_position(housing)
    ctx.check(
        "pan bearing rotates housing around the post",
        rest_pos is not None
        and turned_pos is not None
        and rest_pos[0] > 0.10
        and abs(rest_pos[1]) < 0.01
        and turned_pos[1] > 0.10
        and abs(turned_pos[0]) < 0.02,
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    rest_bezel = ctx.part_element_world_aabb(housing, elem="front_bezel")
    with ctx.pose({tilt_joint: tilt_joint.motion_limits.upper}):
        raised_bezel = ctx.part_element_world_aabb(housing, elem="front_bezel")
        ctx.expect_contact(
            housing,
            pan_head,
            name="tilt hinge stays seated in the yoke at max tilt",
        )
    rest_bezel_z = None if rest_bezel is None else 0.5 * (rest_bezel[0][2] + rest_bezel[1][2])
    raised_bezel_z = (
        None if raised_bezel is None else 0.5 * (raised_bezel[0][2] + raised_bezel[1][2])
    )
    ctx.check(
        "positive tilt raises the spotlight beam",
        rest_bezel_z is not None
        and raised_bezel_z is not None
        and raised_bezel_z > rest_bezel_z + 0.08,
        details=f"rest_bezel_z={rest_bezel_z}, raised_bezel_z={raised_bezel_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
