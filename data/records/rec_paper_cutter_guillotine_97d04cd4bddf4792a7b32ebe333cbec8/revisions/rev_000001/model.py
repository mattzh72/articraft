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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="print_shop_guillotine")

    painted_steel = model.material("painted_steel", rgba=(0.20, 0.23, 0.26, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.28, 0.30, 0.33, 1.0))
    blade_steel = model.material("blade_steel", rgba=(0.72, 0.74, 0.78, 1.0))
    handle_black = model.material("handle_black", rgba=(0.10, 0.10, 0.11, 1.0))
    guide_gray = model.material("guide_gray", rgba=(0.58, 0.60, 0.62, 1.0))

    base = model.part("base")
    table_mesh = mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(0.72, 0.48, 0.025), 0.028),
        "guillotine_table_top",
    )
    base.visual(
        table_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=painted_steel,
        name="table_top",
    )
    base.visual(
        Box((0.62, 0.36, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=gunmetal,
        name="under_plinth",
    )
    base.visual(
        Box((0.56, 0.020, 0.010)),
        origin=Origin(xyz=(0.055, -0.068, 0.033)),
        material=blade_steel,
        name="fixed_blade_bar",
    )
    base.visual(
        Box((0.60, 0.022, 0.044)),
        origin=Origin(xyz=(0.015, 0.196, 0.050)),
        material=guide_gray,
        name="paper_fence",
    )
    base.visual(
        Box((0.018, 0.320, 0.010)),
        origin=Origin(xyz=(-0.205, 0.018, 0.033)),
        material=guide_gray,
        name="side_guide",
    )
    base.visual(
        Box((0.024, 0.190, 0.086)),
        origin=Origin(xyz=(-0.349, 0.0, 0.071)),
        material=painted_steel,
        name="hinge_column",
    )
    base.visual(
        Cylinder(radius=0.022, length=0.046),
        origin=Origin(
            xyz=(-0.309, -0.062, 0.077),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=painted_steel,
        name="hinge_knuckle_rear",
    )
    base.visual(
        Box((0.050, 0.050, 0.048)),
        origin=Origin(xyz=(-0.336, -0.062, 0.052)),
        material=painted_steel,
        name="hinge_cheek_rear",
    )
    base.visual(
        Cylinder(radius=0.022, length=0.046),
        origin=Origin(
            xyz=(-0.309, 0.062, 0.077),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=painted_steel,
        name="hinge_knuckle_front",
    )
    base.visual(
        Box((0.050, 0.050, 0.048)),
        origin=Origin(xyz=(-0.336, 0.062, 0.052)),
        material=painted_steel,
        name="hinge_cheek_front",
    )
    base.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(
            xyz=(-0.356, 0.102, 0.074),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=painted_steel,
        name="lock_mount",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.72, 0.48, 0.12)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
    )

    blade_arm = model.part("blade_arm")
    arm_profile = [
        (0.000, -0.030),
        (0.040, -0.044),
        (0.090, -0.078),
        (0.310, -0.071),
        (0.560, -0.056),
        (0.680, -0.038),
        (0.715, -0.018),
        (0.720, 0.000),
        (0.715, 0.018),
        (0.680, 0.038),
        (0.560, 0.056),
        (0.310, 0.071),
        (0.090, 0.078),
        (0.040, 0.044),
        (0.000, 0.030),
    ]
    blade_arm.visual(
        Cylinder(radius=0.024, length=0.078),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=gunmetal,
        name="arm_barrel",
    )
    blade_arm.visual(
        Box((0.110, 0.068, 0.038)),
        origin=Origin(xyz=(0.070, 0.0, -0.018)),
        material=painted_steel,
        name="heel_block",
    )
    blade_arm.visual(
        mesh_from_geometry(ExtrudeGeometry(arm_profile, 0.016), "guillotine_arm_panel"),
        origin=Origin(xyz=(0.0, 0.0, -0.017)),
        material=painted_steel,
        name="arm_panel",
    )
    blade_arm.visual(
        Box((0.575, 0.012, 0.012)),
        origin=Origin(xyz=(0.360, -0.052, -0.031)),
        material=blade_steel,
        name="blade_strip",
    )
    blade_arm.visual(
        Cylinder(radius=0.018, length=0.110),
        origin=Origin(
            xyz=(0.645, 0.0, -0.030),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=handle_black,
        name="grip_sleeve",
    )
    blade_arm.inertial = Inertial.from_geometry(
        Box((0.74, 0.18, 0.09)),
        mass=4.2,
        origin=Origin(xyz=(0.360, 0.0, -0.020)),
    )

    lock_handle = model.part("lock_handle")
    lock_handle.visual(
        Cylinder(radius=0.008, length=0.026),
        origin=Origin(
            xyz=(-0.013, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=gunmetal,
        name="shaft",
    )
    lock_handle.visual(
        Cylinder(radius=0.016, length=0.018),
        origin=Origin(
            xyz=(-0.018, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=painted_steel,
        name="hub",
    )
    lock_handle.visual(
        Cylinder(radius=0.030, length=0.012),
        origin=Origin(
            xyz=(-0.028, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=painted_steel,
        name="lock_wheel",
    )
    lock_handle.visual(
        Cylinder(radius=0.006, length=0.020),
        origin=Origin(
            xyz=(-0.040, 0.0, 0.020),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=handle_black,
        name="lock_grip",
    )
    lock_handle.inertial = Inertial.from_geometry(
        Box((0.080, 0.070, 0.070)),
        mass=0.35,
        origin=Origin(xyz=(-0.030, 0.0, 0.0)),
    )

    model.articulation(
        "base_to_blade_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=blade_arm,
        origin=Origin(xyz=(-0.309, 0.0, 0.077)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.8,
            lower=0.0,
            upper=math.radians(72.0),
        ),
    )
    model.articulation(
        "base_to_lock_handle",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lock_handle,
        origin=Origin(xyz=(-0.361, 0.102, 0.074)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.5,
            lower=-math.radians(70.0),
            upper=math.radians(70.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    blade_arm = object_model.get_part("blade_arm")
    lock_handle = object_model.get_part("lock_handle")
    arm_joint = object_model.get_articulation("base_to_blade_arm")
    lock_joint = object_model.get_articulation("base_to_lock_handle")

    def aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))

    with ctx.pose({arm_joint: 0.0, lock_joint: 0.0}):
        ctx.expect_gap(
            blade_arm,
            base,
            axis="z",
            positive_elem="blade_strip",
            negative_elem="fixed_blade_bar",
            max_gap=0.003,
            max_penetration=0.0,
            name="blade strip rests just above the fixed cutting bar",
        )
        ctx.expect_overlap(
            blade_arm,
            base,
            axes="x",
            elem_a="blade_strip",
            elem_b="fixed_blade_bar",
            min_overlap=0.50,
            name="blade strip spans the cutting bed",
        )
        ctx.expect_contact(
            lock_handle,
            base,
            elem_a="shaft",
            elem_b="lock_mount",
            contact_tol=0.0005,
            name="lock handle shaft mounts against the hinge block boss",
        )

    closed_grip = None
    open_grip = None
    with ctx.pose({arm_joint: 0.0}):
        closed_grip = aabb_center(ctx.part_element_world_aabb(blade_arm, elem="grip_sleeve"))
    with ctx.pose({arm_joint: math.radians(65.0)}):
        open_grip = aabb_center(ctx.part_element_world_aabb(blade_arm, elem="grip_sleeve"))
    ctx.check(
        "blade arm opens upward on the side hinge",
        closed_grip is not None
        and open_grip is not None
        and open_grip[2] > closed_grip[2] + 0.20,
        details=f"closed_grip={closed_grip}, open_grip={open_grip}",
    )

    lock_grip_a = None
    lock_grip_b = None
    with ctx.pose({lock_joint: -math.radians(45.0)}):
        lock_grip_a = aabb_center(ctx.part_element_world_aabb(lock_handle, elem="lock_grip"))
    with ctx.pose({lock_joint: math.radians(45.0)}):
        lock_grip_b = aabb_center(ctx.part_element_world_aabb(lock_handle, elem="lock_grip"))
    ctx.check(
        "rotary lock handle turns around its own short shaft",
        lock_grip_a is not None
        and lock_grip_b is not None
        and math.hypot(lock_grip_a[1] - lock_grip_b[1], lock_grip_a[2] - lock_grip_b[2]) > 0.025,
        details=f"lock_grip_a={lock_grip_a}, lock_grip_b={lock_grip_b}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
