from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
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
    model = ArticulatedObject(name="pantry_lazy_susan")

    base_plastic = model.material("base_plastic", rgba=(0.20, 0.21, 0.22, 1.0))
    bearing_metal = model.material("bearing_metal", rgba=(0.70, 0.72, 0.75, 1.0))
    platform_plastic = model.material("platform_plastic", rgba=(0.80, 0.82, 0.84, 1.0))
    bin_clear = model.material("bin_clear", rgba=(0.84, 0.90, 0.95, 0.50))

    base_stand = model.part("base_stand")
    base_stand.visual(
        Cylinder(radius=0.145, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=base_plastic,
        name="foot_disc",
    )
    base_stand.visual(
        Cylinder(radius=0.092, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.021)),
        material=base_plastic,
        name="bearing_housing",
    )
    base_stand.visual(
        Cylinder(radius=0.052, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=bearing_metal,
        name="bearing_pad",
    )
    base_stand.inertial = Inertial.from_geometry(
        Cylinder(radius=0.145, length=0.034),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
    )

    turning_bin = model.part("turning_bin")
    turning_bin.visual(
        Cylinder(radius=0.175, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=platform_plastic,
        name="platform_disc",
    )
    turning_bin.visual(
        Cylinder(radius=0.070, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=platform_plastic,
        name="center_boss",
    )

    shell_outer_profile = [
        (0.160, 0.000),
        (0.160, 0.300),
        (0.161, 0.360),
        (0.163, 0.386),
    ]
    shell_inner_profile = [
        (0.154, 0.008),
        (0.154, 0.300),
        (0.155, 0.360),
        (0.157, 0.380),
    ]
    shell_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            shell_outer_profile,
            shell_inner_profile,
            segments=64,
            lip_samples=8,
        ),
        "lazy_susan_bin_shell",
    )
    turning_bin.visual(
        shell_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=bin_clear,
        name="bin_shell",
    )
    turning_bin.inertial = Inertial.from_geometry(
        Cylinder(radius=0.175, length=0.394),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.0, 0.197)),
    )

    model.articulation(
        "base_to_bin_spin",
        ArticulationType.CONTINUOUS,
        parent=base_stand,
        child=turning_bin,
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=4.0,
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

    base_stand = object_model.get_part("base_stand")
    turning_bin = object_model.get_part("turning_bin")
    spin_joint = object_model.get_articulation("base_to_bin_spin")
    platform_disc = turning_bin.get_visual("platform_disc")
    bearing_pad = base_stand.get_visual("bearing_pad")

    ctx.check(
        "bin spins continuously on a vertical axis",
        spin_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(spin_joint.axis) == (0.0, 0.0, 1.0)
        and spin_joint.motion_limits is not None
        and spin_joint.motion_limits.lower is None
        and spin_joint.motion_limits.upper is None,
        details=(
            f"type={spin_joint.articulation_type}, axis={spin_joint.axis}, "
            f"limits={spin_joint.motion_limits}"
        ),
    )

    rest_pos = ctx.part_world_position(turning_bin)
    with ctx.pose({spin_joint: 0.0}):
        ctx.expect_contact(
            turning_bin,
            base_stand,
            elem_a=platform_disc,
            elem_b=bearing_pad,
            contact_tol=5e-4,
            name="platform sits on center bearing pad",
        )
        ctx.expect_overlap(
            turning_bin,
            base_stand,
            axes="xy",
            elem_a=platform_disc,
            elem_b=bearing_pad,
            min_overlap=0.10,
            name="bearing pad stays centered beneath platform",
        )

    with ctx.pose({spin_joint: 1.8}):
        spun_pos = ctx.part_world_position(turning_bin)
        ctx.expect_contact(
            turning_bin,
            base_stand,
            elem_a=platform_disc,
            elem_b=bearing_pad,
            contact_tol=5e-4,
            name="platform remains supported while spun",
        )
        ctx.check(
            "spin keeps bin concentric over the base",
            rest_pos is not None
            and spun_pos is not None
            and abs(rest_pos[0] - spun_pos[0]) <= 1e-6
            and abs(rest_pos[1] - spun_pos[1]) <= 1e-6
            and abs(rest_pos[2] - spun_pos[2]) <= 1e-6,
            details=f"rest={rest_pos}, spun={spun_pos}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
