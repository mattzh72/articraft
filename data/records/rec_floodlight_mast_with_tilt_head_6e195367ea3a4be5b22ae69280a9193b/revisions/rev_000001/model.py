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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="twin_head_floodlight_mast")

    painted_steel = model.material("painted_steel", rgba=(0.95, 0.82, 0.12, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.22, 0.23, 0.25, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.10, 0.10, 0.10, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.85, 0.92, 0.98, 0.45))
    reflector = model.material("reflector", rgba=(0.82, 0.84, 0.87, 1.0))

    def add_arm(part, sign: float) -> None:
        part.visual(
            Cylinder(radius=0.028, length=0.11),
            origin=Origin(
                xyz=(sign * 0.028, 0.0, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=dark_steel,
            name="shoulder_collar",
        )
        part.visual(
            Box((0.37, 0.06, 0.038)),
            origin=Origin(xyz=(sign * 0.185, 0.0, 0.0)),
            material=painted_steel,
            name="main_beam",
        )
        part.visual(
            Box((0.22, 0.038, 0.020)),
            origin=Origin(xyz=(sign * 0.145, 0.0, -0.029)),
            material=painted_steel,
            name="lower_stiffener",
        )
        part.visual(
            Box((0.04, 0.08, 0.11)),
            origin=Origin(xyz=(sign * 0.38, 0.0, 0.0)),
            material=dark_steel,
            name="tip_block",
        )
        part.visual(
            Box((0.018, 0.09, 0.055)),
            origin=Origin(xyz=(sign * 0.35, 0.0, 0.0)),
            material=dark_steel,
            name="tip_reinforcement",
        )

    def add_head(part, sign: float, lens_name: str) -> None:
        for offset_y, name in ((0.033, "yoke_plate_outer"), (-0.033, "yoke_plate_inner")):
            part.visual(
                Box((0.06, 0.005, 0.10)),
                origin=Origin(xyz=(sign * 0.03, offset_y, 0.0)),
                material=dark_steel,
                name=name,
            )
        part.visual(
            Box((0.012, 0.072, 0.10)),
            origin=Origin(xyz=(sign * 0.006, 0.0, 0.0)),
            material=dark_steel,
            name="yoke_bridge",
        )
        part.visual(
            Box((0.11, 0.09, 0.14)),
            origin=Origin(xyz=(sign * 0.085, 0.0, 0.0)),
            material=painted_steel,
            name="housing_body",
        )
        part.visual(
            Box((0.022, 0.11, 0.16)),
            origin=Origin(xyz=(sign * 0.151, 0.0, 0.0)),
            material=painted_steel,
            name="front_bezel",
        )
        part.visual(
            Box((0.005, 0.092, 0.132)),
            origin=Origin(xyz=(sign * 0.145, 0.0, 0.0)),
            material=lens_glass,
            name=lens_name,
        )
        part.visual(
            Box((0.008, 0.084, 0.124)),
            origin=Origin(xyz=(sign * 0.138, 0.0, 0.0)),
            material=reflector,
            name="reflector_panel",
        )
        part.visual(
            Box((0.03, 0.114, 0.012)),
            origin=Origin(xyz=(sign * 0.132, 0.0, 0.078)),
            material=painted_steel,
            name="visor_lip",
        )
        for fin_z, fin_name in ((-0.045, "rear_fin_low"), (0.0, "rear_fin_mid"), (0.045, "rear_fin_high")):
            part.visual(
                Box((0.026, 0.078, 0.007)),
                origin=Origin(xyz=(sign * 0.022, 0.0, fin_z)),
                material=dark_steel,
                name=fin_name,
            )
        part.visual(
            Cylinder(radius=0.012, length=0.078),
            origin=Origin(
                xyz=(sign * 0.014, 0.0, -0.062),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=black_rubber,
            name="strain_relief",
        )

    base_mast = model.part("base_mast")
    base_plate_profile = rounded_rect_profile(0.58, 0.40, 0.045)
    base_plate_mesh = mesh_from_geometry(
        ExtrudeGeometry(base_plate_profile, 0.03),
        "weighted_base_plate",
    )
    base_mast.visual(
        base_plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=dark_steel,
        name="base_plate",
    )
    base_mast.visual(
        Box((0.22, 0.16, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=black_rubber,
        name="ballast_housing",
    )
    base_mast.visual(
        Box((0.14, 0.14, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material=dark_steel,
        name="mast_socket",
    )
    base_mast.visual(
        Box((0.09, 0.09, 1.44)),
        origin=Origin(xyz=(0.0, 0.0, 0.75)),
        material=painted_steel,
        name="mast_tube",
    )
    base_mast.visual(
        Box((0.11, 0.11, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 1.495)),
        material=black_rubber,
        name="mast_cap",
    )
    base_mast.visual(
        Box((0.014, 0.10, 0.11)),
        origin=Origin(xyz=(0.038, 0.0, 0.88)),
        material=dark_steel,
        name="lower_mount_bracket",
    )
    base_mast.visual(
        Box((0.014, 0.10, 0.11)),
        origin=Origin(xyz=(-0.038, 0.0, 1.18)),
        material=dark_steel,
        name="upper_mount_bracket",
    )

    lower_arm = model.part("lower_arm")
    add_arm(lower_arm, sign=1.0)

    upper_arm = model.part("upper_arm")
    add_arm(upper_arm, sign=-1.0)

    lower_head = model.part("lower_head")
    add_head(lower_head, sign=1.0, lens_name="lower_lens")

    upper_head = model.part("upper_head")
    add_head(upper_head, sign=-1.0, lens_name="upper_lens")

    model.articulation(
        "mast_to_lower_arm",
        ArticulationType.REVOLUTE,
        parent=base_mast,
        child=lower_arm,
        origin=Origin(xyz=(0.045, 0.0, 0.88)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=math.radians(-35.0),
            upper=math.radians(60.0),
        ),
    )
    model.articulation(
        "mast_to_upper_arm",
        ArticulationType.REVOLUTE,
        parent=base_mast,
        child=upper_arm,
        origin=Origin(xyz=(-0.045, 0.0, 1.18)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=math.radians(-35.0),
            upper=math.radians(60.0),
        ),
    )
    model.articulation(
        "lower_arm_to_head",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=lower_head,
        origin=Origin(xyz=(0.40, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.5,
            lower=math.radians(-70.0),
            upper=math.radians(40.0),
        ),
    )
    model.articulation(
        "upper_arm_to_head",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=upper_head,
        origin=Origin(xyz=(-0.40, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.5,
            lower=math.radians(-70.0),
            upper=math.radians(40.0),
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

    lower_arm_joint = object_model.get_articulation("mast_to_lower_arm")
    upper_arm_joint = object_model.get_articulation("mast_to_upper_arm")
    lower_head_joint = object_model.get_articulation("lower_arm_to_head")
    upper_head_joint = object_model.get_articulation("upper_arm_to_head")
    lower_head = object_model.get_part("lower_head")
    upper_head = object_model.get_part("upper_head")

    def aabb_center(aabb):
        if aabb is None:
            return None
        (min_corner, max_corner) = aabb
        return tuple((lo + hi) * 0.5 for lo, hi in zip(min_corner, max_corner))

    lower_rest = ctx.part_world_position(lower_head)
    upper_rest = ctx.part_world_position(upper_head)
    ctx.check(
        "heads mount on opposite mast sides",
        lower_rest is not None
        and upper_rest is not None
        and lower_rest[0] > 0.25
        and upper_rest[0] < -0.25,
        details=f"lower_head={lower_rest}, upper_head={upper_rest}",
    )
    ctx.check(
        "upper head sits higher than lower head",
        lower_rest is not None
        and upper_rest is not None
        and upper_rest[2] > lower_rest[2] + 0.20,
        details=f"lower_head={lower_rest}, upper_head={upper_rest}",
    )

    lower_lens_rest = aabb_center(ctx.part_element_world_aabb(lower_head, elem="lower_lens"))
    upper_lens_rest = aabb_center(ctx.part_element_world_aabb(upper_head, elem="upper_lens"))

    with ctx.pose({lower_arm_joint: math.radians(35.0)}):
        lower_raised = ctx.part_world_position(lower_head)
        upper_static = ctx.part_world_position(upper_head)
    ctx.check(
        "lower arm raises only the lower head",
        lower_rest is not None
        and lower_raised is not None
        and upper_rest is not None
        and upper_static is not None
        and lower_raised[2] > lower_rest[2] + 0.12
        and abs(upper_static[2] - upper_rest[2]) < 0.01
        and abs(upper_static[0] - upper_rest[0]) < 0.01,
        details=(
            f"lower_rest={lower_rest}, lower_raised={lower_raised}, "
            f"upper_rest={upper_rest}, upper_static={upper_static}"
        ),
    )

    with ctx.pose({upper_arm_joint: math.radians(35.0)}):
        upper_raised = ctx.part_world_position(upper_head)
        lower_static = ctx.part_world_position(lower_head)
    ctx.check(
        "upper arm raises only the upper head",
        upper_rest is not None
        and upper_raised is not None
        and lower_rest is not None
        and lower_static is not None
        and upper_raised[2] > upper_rest[2] + 0.12
        and abs(lower_static[2] - lower_rest[2]) < 0.01
        and abs(lower_static[0] - lower_rest[0]) < 0.01,
        details=(
            f"upper_rest={upper_rest}, upper_raised={upper_raised}, "
            f"lower_rest={lower_rest}, lower_static={lower_static}"
        ),
    )

    with ctx.pose({lower_head_joint: math.radians(-35.0)}):
        lower_lens_tilted = aabb_center(ctx.part_element_world_aabb(lower_head, elem="lower_lens"))
    ctx.check(
        "lower lamp head tilts downward",
        lower_lens_rest is not None
        and lower_lens_tilted is not None
        and lower_lens_tilted[2] < lower_lens_rest[2] - 0.04,
        details=f"rest={lower_lens_rest}, tilted={lower_lens_tilted}",
    )

    with ctx.pose({upper_head_joint: math.radians(-35.0)}):
        upper_lens_tilted = aabb_center(ctx.part_element_world_aabb(upper_head, elem="upper_lens"))
    ctx.check(
        "upper lamp head tilts downward",
        upper_lens_rest is not None
        and upper_lens_tilted is not None
        and upper_lens_tilted[2] < upper_lens_rest[2] - 0.04,
        details=f"rest={upper_lens_rest}, tilted={upper_lens_tilted}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
