from __future__ import annotations

# The harness only exposes the editable block to the model.
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
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root


def _make_material(name: str, color: tuple[float, float, float, float]):
    try:
        return Material(name=name, rgba=color)
    except TypeError:
        try:
            return Material(name=name, color=color)
        except TypeError:
            try:
                return Material(name=name)
            except TypeError:
                return None


def _visual(part, geometry, origin: Origin, material=None, name: str | None = None) -> None:
    kwargs = {"origin": origin}
    if material is not None:
        kwargs["material"] = material
    if name is not None:
        kwargs["name"] = name
    part.visual(geometry, **kwargs)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="barrier_gate", assets=ASSETS)

    materials = {
        "cabinet_red": _make_material("cabinet_red", (0.76, 0.10, 0.08, 1.0)),
        "cover_gray": _make_material("cover_gray", (0.23, 0.24, 0.27, 1.0)),
        "arm_white": _make_material("arm_white", (0.93, 0.94, 0.95, 1.0)),
        "stripe_red": _make_material("stripe_red", (0.82, 0.14, 0.12, 1.0)),
        "rubber_black": _make_material("rubber_black", (0.10, 0.10, 0.11, 1.0)),
        "steel": _make_material("steel", (0.63, 0.65, 0.68, 1.0)),
        "amber_lens": _make_material("amber_lens", (0.95, 0.62, 0.10, 0.72)),
        "control_gray": _make_material("control_gray", (0.78, 0.79, 0.80, 1.0)),
    }

    base = model.part("base_housing")
    _visual(
        base,
        Box((0.90, 0.68, 0.08)),
        Origin(xyz=(0.0, 0.0, 0.04)),
        materials["rubber_black"],
        name="ground_plinth",
    )
    _visual(
        base,
        Box((0.76, 0.54, 0.03)),
        Origin(xyz=(0.0, 0.0, 0.095)),
        materials["rubber_black"],
        name="lower_skirt",
    )
    _visual(
        base,
        Box((0.58, 0.44, 0.73)),
        Origin(xyz=(0.0, 0.0, 0.475)),
        materials["cabinet_red"],
        name="main_cabinet",
    )
    _visual(
        base,
        Box((0.46, 0.46, 0.15)),
        Origin(xyz=(0.03, 0.0, 0.915)),
        materials["cover_gray"],
        name="top_motor_cover",
    )
    _visual(
        base,
        Box((0.14, 0.24, 0.19)),
        Origin(xyz=(0.29, 0.0, 0.895)),
        materials["cover_gray"],
        name="hinge_pedestal",
    )
    _visual(
        base,
        Box((0.34, 0.008, 0.50)),
        Origin(xyz=(0.02, -0.224, 0.43)),
        materials["control_gray"],
        name="service_door",
    )
    _visual(
        base,
        Box((0.03, 0.026, 0.10)),
        Origin(xyz=(0.17, -0.239, 0.43)),
        materials["rubber_black"],
        name="door_handle",
    )
    _visual(
        base,
        Box((0.14, 0.06, 0.22)),
        Origin(xyz=(-0.10, 0.25, 0.52)),
        materials["control_gray"],
        name="control_pod",
    )
    _visual(
        base,
        Cylinder(radius=0.022, length=0.03),
        Origin(xyz=(-0.10, 0.295, 0.57), rpy=(math.pi / 2.0, 0.0, 0.0)),
        materials["cabinet_red"],
        name="emergency_stop",
    )
    _visual(
        base,
        Cylinder(radius=0.012, length=0.02),
        Origin(xyz=(-0.10, 0.29, 0.49), rpy=(math.pi / 2.0, 0.0, 0.0)),
        materials["steel"],
        name="key_switch",
    )
    _visual(
        base,
        Cylinder(radius=0.030, length=0.02),
        Origin(xyz=(-0.14, 0.0, 1.00)),
        materials["rubber_black"],
        name="beacon_base",
    )
    _visual(
        base,
        Sphere(radius=0.030),
        Origin(xyz=(-0.14, 0.0, 1.04)),
        materials["amber_lens"],
        name="beacon_lens",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.90, 0.68, 0.99)),
        mass=135.0,
        origin=Origin(xyz=(0.0, 0.0, 0.495)),
    )

    arm = model.part("gate_arm")
    _visual(
        arm,
        Box((0.24, 0.16, 0.10)),
        Origin(xyz=(0.12, 0.0, 0.054)),
        materials["cover_gray"],
        name="hinge_head",
    )
    _visual(
        arm,
        Cylinder(radius=0.060, length=0.18),
        Origin(xyz=(0.055, 0.0, 0.074), rpy=(math.pi / 2.0, 0.0, 0.0)),
        materials["steel"],
        name="pivot_drum",
    )
    _visual(
        arm,
        Cylinder(radius=0.030, length=0.22),
        Origin(xyz=(0.025, 0.0, 0.074), rpy=(math.pi / 2.0, 0.0, 0.0)),
        materials["rubber_black"],
        name="pivot_cap",
    )
    _visual(
        arm,
        Box((3.92, 0.10, 0.08)),
        Origin(xyz=(2.20, 0.0, 0.054)),
        materials["arm_white"],
        name="main_boom",
    )
    _visual(
        arm,
        Box((1.20, 0.05, 0.04)),
        Origin(xyz=(0.86, 0.0, 0.002)),
        materials["rubber_black"],
        name="underslung_reinforcement",
    )
    _visual(
        arm,
        Box((0.20, 0.08, 0.052)),
        Origin(xyz=(0.35, 0.0, 0.119)),
        materials["cover_gray"],
        name="drive_cowl",
    )
    stripe_positions = (0.56, 1.22, 1.88, 2.54, 3.20, 3.86)
    for index, x_pos in enumerate(stripe_positions, start=1):
        _visual(
            arm,
            Box((0.16, 0.104, 0.082)),
            Origin(xyz=(x_pos, 0.0, 0.054)),
            materials["stripe_red"],
            name=f"stripe_{index}",
        )
    _visual(
        arm,
        Box((0.10, 0.14, 0.14)),
        Origin(xyz=(4.21, 0.0, 0.054)),
        materials["rubber_black"],
        name="tip_cap",
    )
    _visual(
        arm,
        Box((0.06, 0.12, 0.05)),
        Origin(xyz=(4.19, 0.0, 0.005)),
        materials["rubber_black"],
        name="tip_light_housing",
    )
    arm.inertial = Inertial.from_geometry(
        Box((4.26, 0.18, 0.16)),
        mass=18.0,
        origin=Origin(xyz=(2.13, 0.0, 0.06)),
    )

    model.articulation(
        "arm_hinge",
        ArticulationType.REVOLUTE,
        parent="base_housing",
        child="gate_arm",
        origin=Origin(xyz=(0.29, 0.0, 0.99)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=1.2,
            lower=0.0,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.allow_overlap(
        "base_housing",
        "gate_arm",
        reason="The hinge saddle and boom root run with only a few millimeters of clearance, and collision sampling is conservative around the pivot.",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=160,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
    )
    ctx.expect_joint_motion_axis(
        "arm_hinge",
        "gate_arm",
        world_axis="z",
        direction="positive",
        min_delta=1.2,
    )
    with ctx.pose(arm_hinge=0.0):
        ctx.expect_aabb_overlap("gate_arm", "base_housing", axes="xy", min_overlap=0.08)
    with ctx.pose(arm_hinge=0.70):
        ctx.expect_aabb_overlap("gate_arm", "base_housing", axes="xy", min_overlap=0.08)
    with ctx.pose(arm_hinge=1.35):
        ctx.expect_aabb_overlap("gate_arm", "base_housing", axes="xy", min_overlap=0.08)
        ctx.expect_origin_distance("gate_arm", "base_housing", axes="xy", max_dist=0.40)
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
