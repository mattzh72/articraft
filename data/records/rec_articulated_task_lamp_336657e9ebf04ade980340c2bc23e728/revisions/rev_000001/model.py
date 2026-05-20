from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


def _cyl_x(radius: float, length: float) -> tuple[Cylinder, Origin]:
    return Cylinder(radius=radius, length=length), Origin(rpy=(0.0, pi / 2.0, 0.0))


def _cyl_y(radius: float, length: float) -> tuple[Cylinder, Origin]:
    return Cylinder(radius=radius, length=length), Origin(rpy=(pi / 2.0, 0.0, 0.0))


def _add_cylinder_x(part, *, name: str, radius: float, length: float, xyz, material):
    geom, orient = _cyl_x(radius, length)
    return part.visual(
        geom,
        origin=Origin(xyz=xyz, rpy=orient.rpy),
        material=material,
        name=name,
    )


def _add_cylinder_y(part, *, name: str, radius: float, length: float, xyz, material):
    geom, orient = _cyl_y(radius, length)
    return part.visual(
        geom,
        origin=Origin(xyz=xyz, rpy=orient.rpy),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="explosion_proof_wall_lamp")

    steel = model.material("steel", rgba=(0.22, 0.24, 0.26, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.14, 0.16, 0.18, 1.0))
    housing_paint = model.material("housing_paint", rgba=(0.52, 0.55, 0.50, 1.0))
    bezel_paint = model.material("bezel_paint", rgba=(0.40, 0.42, 0.38, 1.0))
    glass = model.material("lamp_glass", rgba=(0.82, 0.90, 0.97, 0.35))

    wall_bracket = model.part("wall_bracket")
    wall_bracket.visual(
        Box((0.012, 0.18, 0.26)),
        origin=Origin(xyz=(-0.056, 0.0, 0.0)),
        material=dark_steel,
        name="bracket_plate",
    )
    wall_bracket.visual(
        Box((0.032, 0.080, 0.115)),
        origin=Origin(xyz=(-0.034, 0.0, 0.0)),
        material=steel,
        name="standoff_body",
    )
    wall_bracket.visual(
        Box((0.024, 0.050, 0.014)),
        origin=Origin(xyz=(-0.012, 0.0, 0.018)),
        material=steel,
        name="shoulder_web_upper",
    )
    wall_bracket.visual(
        Box((0.024, 0.050, 0.014)),
        origin=Origin(xyz=(-0.012, 0.0, -0.018)),
        material=steel,
        name="shoulder_web_lower",
    )
    wall_bracket.visual(
        Cylinder(radius=0.020, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=steel,
        name="shoulder_barrel_upper",
    )
    wall_bracket.visual(
        Cylinder(radius=0.020, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
        material=steel,
        name="shoulder_barrel_lower",
    )
    for index, bolt_y in enumerate((-0.058, 0.058), start=1):
        for row, bolt_z in enumerate((-0.085, 0.085), start=1):
            _add_cylinder_x(
                wall_bracket,
                name=f"mount_bolt_{index}_{row}",
                radius=0.008,
                length=0.004,
                xyz=(-0.050, bolt_y, bolt_z),
                material=bezel_paint,
            )
    wall_bracket.inertial = Inertial.from_geometry(
        Box((0.068, 0.18, 0.26)),
        mass=6.5,
        origin=Origin(xyz=(-0.034, 0.0, 0.0)),
    )

    lower_arm = model.part("lower_arm")
    lower_arm.visual(
        Cylinder(radius=0.016, length=0.024),
        origin=Origin(),
        material=dark_steel,
        name="rear_hinge_barrel",
    )
    lower_arm.visual(
        Box((0.050, 0.046, 0.020)),
        origin=Origin(xyz=(0.030, 0.0, 0.0)),
        material=dark_steel,
        name="rear_bridge",
    )
    lower_arm.visual(
        Box((0.230, 0.038, 0.022)),
        origin=Origin(xyz=(0.155, 0.0, 0.0)),
        material=steel,
        name="lower_arm_beam",
    )
    lower_arm.visual(
        Box((0.044, 0.030, 0.014)),
        origin=Origin(xyz=(0.278, 0.0, 0.018)),
        material=dark_steel,
        name="front_web_upper",
    )
    lower_arm.visual(
        Box((0.044, 0.030, 0.014)),
        origin=Origin(xyz=(0.278, 0.0, -0.018)),
        material=dark_steel,
        name="front_web_lower",
    )
    lower_arm.visual(
        Cylinder(radius=0.016, length=0.012),
        origin=Origin(xyz=(0.300, 0.0, 0.018)),
        material=dark_steel,
        name="front_knuckle_upper",
    )
    lower_arm.visual(
        Cylinder(radius=0.016, length=0.012),
        origin=Origin(xyz=(0.300, 0.0, -0.018)),
        material=dark_steel,
        name="front_knuckle_lower",
    )
    lower_arm.inertial = Inertial.from_geometry(
        Box((0.320, 0.050, 0.050)),
        mass=3.2,
        origin=Origin(xyz=(0.160, 0.0, 0.0)),
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.013, length=0.024),
        origin=Origin(),
        material=dark_steel,
        name="rear_hinge_barrel",
    )
    upper_arm.visual(
        Box((0.048, 0.040, 0.020)),
        origin=Origin(xyz=(0.028, 0.0, 0.0)),
        material=dark_steel,
        name="rear_bridge",
    )
    upper_arm.visual(
        Box((0.196, 0.032, 0.020)),
        origin=Origin(xyz=(0.134, 0.0, 0.0)),
        material=steel,
        name="upper_arm_beam",
    )
    upper_arm.visual(
        Box((0.048, 0.024, 0.044)),
        origin=Origin(xyz=(0.228, 0.0, 0.0)),
        material=dark_steel,
        name="front_mount_block",
    )
    upper_arm.visual(
        Box((0.018, 0.010, 0.022)),
        origin=Origin(xyz=(0.254, 0.017, 0.0)),
        material=dark_steel,
        name="yoke_rib_left",
    )
    upper_arm.visual(
        Box((0.018, 0.010, 0.022)),
        origin=Origin(xyz=(0.254, -0.017, 0.0)),
        material=dark_steel,
        name="yoke_rib_right",
    )
    upper_arm.visual(
        Cylinder(radius=0.013, length=0.012),
        origin=Origin(xyz=(0.270, 0.016, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="lamp_yoke_left",
    )
    _add_cylinder_y(
        upper_arm,
        name="lamp_yoke_right",
        radius=0.013,
        length=0.012,
        xyz=(0.270, -0.016, 0.0),
        material=dark_steel,
    )
    upper_arm.inertial = Inertial.from_geometry(
        Box((0.290, 0.050, 0.060)),
        mass=2.6,
        origin=Origin(xyz=(0.145, 0.0, 0.0)),
    )

    lamp_head = model.part("lamp_head")
    lamp_head.visual(
        Cylinder(radius=0.013, length=0.020),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=bezel_paint,
        name="trunnion_barrel",
    )
    lamp_head.visual(
        Box((0.034, 0.012, 0.030)),
        origin=Origin(xyz=(0.017, 0.0, 0.0)),
        material=bezel_paint,
        name="trunnion_collar",
    )
    _add_cylinder_x(
        lamp_head,
        name="rear_cap",
        radius=0.040,
        length=0.044,
        xyz=(0.038, 0.0, 0.0),
        material=housing_paint,
    )
    _add_cylinder_x(
        lamp_head,
        name="housing_body",
        radius=0.050,
        length=0.120,
        xyz=(0.112, 0.0, 0.0),
        material=housing_paint,
    )
    for index, fin_x in enumerate((0.074, 0.098, 0.122, 0.146), start=1):
        _add_cylinder_x(
            lamp_head,
            name=f"cooling_fin_{index}",
            radius=0.054,
            length=0.007,
            xyz=(fin_x, 0.0, 0.0),
            material=bezel_paint,
        )
    _add_cylinder_x(
        lamp_head,
        name="front_bezel",
        radius=0.062,
        length=0.036,
        xyz=(0.190, 0.0, 0.0),
        material=bezel_paint,
    )
    lamp_head.visual(
        Cylinder(radius=0.047, length=0.005),
        origin=Origin(xyz=(0.203, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=glass,
        name="front_lens",
    )
    lamp_head.inertial = Inertial.from_geometry(
        Box((0.215, 0.124, 0.124)),
        mass=5.8,
        origin=Origin(xyz=(0.108, 0.0, 0.0)),
    )

    model.articulation(
        "shoulder_joint",
        ArticulationType.REVOLUTE,
        parent=wall_bracket,
        child=lower_arm,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=1.5,
            lower=-2.3,
            upper=2.3,
        ),
    )
    model.articulation(
        "elbow_joint",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=upper_arm,
        origin=Origin(xyz=(0.300, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=65.0,
            velocity=1.8,
            lower=-2.4,
            upper=2.4,
        ),
    )
    model.articulation(
        "lamp_tilt_joint",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=lamp_head,
        origin=Origin(xyz=(0.270, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=2.0,
            lower=-0.95,
            upper=1.05,
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

    wall_bracket = object_model.get_part("wall_bracket")
    lower_arm = object_model.get_part("lower_arm")
    upper_arm = object_model.get_part("upper_arm")
    lamp_head = object_model.get_part("lamp_head")

    shoulder_joint = object_model.get_articulation("shoulder_joint")
    elbow_joint = object_model.get_articulation("elbow_joint")
    lamp_tilt_joint = object_model.get_articulation("lamp_tilt_joint")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))

    ctx.expect_overlap(
        lower_arm,
        wall_bracket,
        axes="xy",
        elem_a="rear_hinge_barrel",
        elem_b="shoulder_barrel_lower",
        min_overlap=0.020,
        name="lower arm rear barrel stays coaxial with the wall bracket shoulder",
    )
    ctx.expect_overlap(
        upper_arm,
        lower_arm,
        axes="xy",
        elem_a="rear_hinge_barrel",
        elem_b="front_knuckle_lower",
        min_overlap=0.018,
        name="upper arm rear barrel stays coaxial with the elbow knuckle",
    )
    ctx.expect_overlap(
        lamp_head,
        upper_arm,
        axes="xz",
        elem_a="trunnion_barrel",
        elem_b="lamp_yoke_left",
        min_overlap=0.020,
        name="lamp trunnion remains captured in the end yoke",
    )

    upper_rest = ctx.part_world_position(upper_arm)
    lamp_rest = ctx.part_world_position(lamp_head)
    lens_rest = _aabb_center(ctx.part_element_world_aabb(lamp_head, elem="front_lens"))

    with ctx.pose({shoulder_joint: 0.80}):
        upper_swung = ctx.part_world_position(upper_arm)
    ctx.check(
        "shoulder joint swings the linked arm laterally",
        upper_rest is not None
        and upper_swung is not None
        and upper_swung[1] > upper_rest[1] + 0.18
        and upper_swung[0] < upper_rest[0] - 0.05,
        details=f"rest={upper_rest}, swung={upper_swung}",
    )

    with ctx.pose({elbow_joint: 1.00}):
        lamp_bent = ctx.part_world_position(lamp_head)
    ctx.check(
        "elbow joint folds the second link toward the side",
        lamp_rest is not None
        and lamp_bent is not None
        and lamp_bent[1] > lamp_rest[1] + 0.18,
        details=f"rest={lamp_rest}, bent={lamp_bent}",
    )

    with ctx.pose({lamp_tilt_joint: 0.55}):
        lens_tilted = _aabb_center(ctx.part_element_world_aabb(lamp_head, elem="front_lens"))
    ctx.check(
        "lamp head tilts upward at the end knuckle",
        lens_rest is not None
        and lens_tilted is not None
        and lens_tilted[2] > lens_rest[2] + 0.05,
        details=f"rest={lens_rest}, tilted={lens_tilted}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
