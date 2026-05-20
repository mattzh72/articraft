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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ceiling_uplighter")

    canopy_white = model.material("canopy_white", rgba=(0.94, 0.94, 0.92, 1.0))
    satin_nickel = model.material("satin_nickel", rgba=(0.74, 0.75, 0.77, 1.0))
    warm_reflector = model.material("warm_reflector", rgba=(0.96, 0.95, 0.90, 1.0))

    cup_shell_geom = LatheGeometry.from_shell_profiles(
        [
            (0.010, -0.028),
            (0.018, -0.024),
            (0.032, -0.010),
            (0.040, 0.010),
            (0.045, 0.028),
        ],
        [
            (0.000, -0.024),
            (0.012, -0.021),
            (0.027, -0.008),
            (0.034, 0.010),
            (0.038, 0.024),
        ],
        segments=56,
    )
    cup_shell = mesh_from_geometry(cup_shell_geom, "uplight_cup_shell")

    hub = model.part("hub")
    hub.visual(
        Cylinder(radius=0.115, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.259)),
        material=canopy_white,
        name="ceiling_plate",
    )
    hub.visual(
        Cylinder(radius=0.018, length=0.064),
        origin=Origin(xyz=(0.0, 0.0, 0.227)),
        material=satin_nickel,
        name="drop_stem",
    )
    hub.visual(
        Cylinder(radius=0.056, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
        material=canopy_white,
        name="hub_body",
    )
    hub.visual(
        Cylinder(radius=0.045, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.137)),
        material=satin_nickel,
        name="hub_collar",
    )

    joint_radius = 0.064
    joint_z = 0.145
    ear_y = 0.011
    arm_names = ("east", "north", "west", "south")

    for index, arm_name in enumerate(arm_names):
        yaw = index * (math.pi / 2.0)
        mount_x = 0.050 * math.cos(yaw)
        mount_y = 0.050 * math.sin(yaw)
        joint_x = joint_radius * math.cos(yaw)
        joint_y = joint_radius * math.sin(yaw)
        tangent_dx = -ear_y * math.sin(yaw)
        tangent_dy = ear_y * math.cos(yaw)
        hub.visual(
            Box((0.024, 0.026, 0.020)),
            origin=Origin(xyz=(mount_x, mount_y, joint_z), rpy=(0.0, 0.0, yaw)),
            material=satin_nickel,
            name=f"{arm_name}_mount_block",
        )
        hub.visual(
            Box((0.012, 0.010, 0.016)),
            origin=Origin(
                xyz=(joint_x + tangent_dx, joint_y + tangent_dy, joint_z),
                rpy=(0.0, 0.0, yaw),
            ),
            material=satin_nickel,
            name=f"{arm_name}_mount_ear_a",
        )
        hub.visual(
            Box((0.012, 0.010, 0.016)),
            origin=Origin(
                xyz=(joint_x - tangent_dx, joint_y - tangent_dy, joint_z),
                rpy=(0.0, 0.0, yaw),
            ),
            material=satin_nickel,
            name=f"{arm_name}_mount_ear_b",
        )

    hub.inertial = Inertial.from_geometry(
        Box((0.24, 0.24, 0.14)),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.0, 0.190)),
    )

    for index, arm_name in enumerate(arm_names):
        yaw = index * (math.pi / 2.0)
        arm = model.part(f"arm_{arm_name}")
        arm.visual(
            Cylinder(radius=0.006, length=0.008),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=satin_nickel,
            name="hinge_barrel",
        )
        arm.visual(
            Box((0.030, 0.016, 0.018)),
            origin=Origin(xyz=(0.015, 0.0, -0.004)),
            material=satin_nickel,
            name="arm_shoulder",
        )
        arm.visual(
            Cylinder(radius=0.0075, length=0.170),
            origin=Origin(xyz=(0.105, 0.0, -0.006), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=satin_nickel,
            name="arm_tube",
        )
        arm.visual(
            Cylinder(radius=0.010, length=0.016),
            origin=Origin(xyz=(0.198, 0.0, -0.014)),
            material=satin_nickel,
            name="tip_collar",
        )
        arm.visual(
            Cylinder(radius=0.0065, length=0.070),
            origin=Origin(xyz=(0.198, 0.0, -0.053)),
            material=satin_nickel,
            name="lamp_stem",
        )
        arm.visual(
            cup_shell,
            origin=Origin(xyz=(0.198, 0.0, -0.058)),
            material=warm_reflector,
            name="cup_shell",
        )
        arm.inertial = Inertial.from_geometry(
            Box((0.28, 0.10, 0.12)),
            mass=0.42,
            origin=Origin(xyz=(0.130, 0.0, -0.040)),
        )

        model.articulation(
            f"hub_to_arm_{arm_name}",
            ArticulationType.REVOLUTE,
            parent=hub,
            child=arm,
            origin=Origin(
                xyz=(joint_radius * math.cos(yaw), joint_radius * math.sin(yaw), joint_z),
                rpy=(0.0, 0.0, yaw),
            ),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=1.2,
                lower=-0.60,
                upper=0.30,
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

    hub = object_model.get_part("hub")
    ceiling_plate = hub.get_visual("ceiling_plate")
    arm_names = ("east", "north", "west", "south")

    def cup_center_z(part_name: str) -> float | None:
        aabb = ctx.part_element_world_aabb(part_name, elem="cup_shell")
        if aabb is None:
            return None
        return 0.5 * (aabb[0][2] + aabb[1][2])

    for arm_name in arm_names:
        arm = object_model.get_part(f"arm_{arm_name}")
        joint = object_model.get_articulation(f"hub_to_arm_{arm_name}")
        limits = joint.motion_limits

        ctx.check(
            f"{arm_name} arm joint uses upward pitching axis",
            tuple(round(v, 6) for v in joint.axis) == (0.0, -1.0, 0.0),
            details=f"axis={joint.axis}",
        )
        ctx.check(
            f"{arm_name} arm has aiming travel on both sides of neutral",
            limits is not None and limits.lower is not None and limits.upper is not None and limits.lower < 0.0 < limits.upper,
            details=f"limits={limits}",
        )

        rest_z = cup_center_z(arm.name)
        with ctx.pose({joint: limits.upper if limits is not None and limits.upper is not None else 0.0}):
            raised_z = cup_center_z(arm.name)
            ctx.check(
                f"{arm_name} cup raises when the arm pitches upward",
                rest_z is not None and raised_z is not None and raised_z > rest_z + 0.04,
                details=f"rest_z={rest_z}, raised_z={raised_z}",
            )
            ctx.expect_gap(
                hub,
                arm,
                axis="z",
                positive_elem=ceiling_plate,
                negative_elem="cup_shell",
                min_gap=0.006,
                name=f"{arm_name} cup stays below the ceiling plate at full raise",
            )

        with ctx.pose({joint: limits.lower if limits is not None and limits.lower is not None else 0.0}):
            lowered_z = cup_center_z(arm.name)
            ctx.check(
                f"{arm_name} cup drops when the arm swings outward",
                rest_z is not None and lowered_z is not None and lowered_z < rest_z - 0.04,
                details=f"rest_z={rest_z}, lowered_z={lowered_z}",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
