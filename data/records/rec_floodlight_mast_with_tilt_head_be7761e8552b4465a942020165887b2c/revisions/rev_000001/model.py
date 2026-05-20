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
    model = ArticulatedObject(name="stadium_floodlight_mast")

    concrete = model.material("concrete", rgba=(0.67, 0.68, 0.69, 1.0))
    galvanized = model.material("galvanized", rgba=(0.73, 0.75, 0.77, 1.0))
    machinery_gray = model.material("machinery_gray", rgba=(0.36, 0.39, 0.42, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.18, 0.19, 0.21, 1.0))
    matte_black = model.material("matte_black", rgba=(0.08, 0.09, 0.10, 1.0))
    lamp_glass = model.material("lamp_glass", rgba=(0.77, 0.84, 0.90, 0.40))
    safety_red = model.material("safety_red", rgba=(0.72, 0.14, 0.10, 1.0))

    mast = model.part("mast")
    mast.visual(
        Box((3.20, 3.20, 1.20)),
        origin=Origin(xyz=(0.0, 0.0, 0.60)),
        material=concrete,
        name="foundation_block",
    )
    mast.visual(
        Cylinder(radius=0.56, length=1.10),
        origin=Origin(xyz=(0.0, 0.0, 1.75)),
        material=machinery_gray,
        name="base_collar",
    )
    mast.visual(
        Cylinder(radius=0.28, length=18.20),
        origin=Origin(xyz=(0.0, 0.0, 11.40)),
        material=galvanized,
        name="main_column",
    )
    mast.visual(
        Cylinder(radius=0.48, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, 20.60)),
        material=machinery_gray,
        name="top_flange",
    )
    mast.visual(
        Box((0.70, 0.26, 0.95)),
        origin=Origin(xyz=(0.0, 0.28, 2.05)),
        material=dark_gray,
        name="service_door",
    )
    mast.inertial = Inertial.from_geometry(
        Box((3.20, 3.20, 20.70)),
        mass=4200.0,
        origin=Origin(xyz=(0.0, 0.0, 10.35)),
    )

    pan_bracket = model.part("pan_bracket")
    pan_bracket.visual(
        Cylinder(radius=0.52, length=0.30),
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
        material=dark_gray,
        name="slew_ring",
    )
    pan_bracket.visual(
        Box((1.10, 1.20, 0.28)),
        origin=Origin(xyz=(0.0, 0.0, 0.44)),
        material=machinery_gray,
        name="slew_housing",
    )
    pan_bracket.visual(
        Box((0.95, 0.95, 0.90)),
        origin=Origin(xyz=(-0.12, 0.0, 0.99)),
        material=machinery_gray,
        name="pedestal_body",
    )
    pan_bracket.visual(
        Box((1.00, 5.50, 0.20)),
        origin=Origin(xyz=(0.20, 0.0, 1.08)),
        material=machinery_gray,
        name="lower_yoke_beam",
    )
    pan_bracket.visual(
        Box((0.80, 5.10, 0.20)),
        origin=Origin(xyz=(0.16, 0.0, 2.68)),
        material=machinery_gray,
        name="upper_yoke_beam",
    )
    for side, y_sign in (("left", 1.0), ("right", -1.0)):
        pan_bracket.visual(
            Box((0.26, 0.26, 1.68)),
            origin=Origin(xyz=(0.48, y_sign * 2.68, 1.82)),
            material=machinery_gray,
            name=f"{side}_side_arm",
        )
        pan_bracket.visual(
            Box((0.24, 0.10, 0.34)),
            origin=Origin(xyz=(0.56, y_sign * 2.50, 1.86)),
            material=dark_gray,
            name=f"{side}_bearing_block",
        )
    pan_bracket.visual(
        Box((0.12, 0.08, 0.18)),
        origin=Origin(xyz=(0.48, 2.85, 2.02)),
        material=dark_gray,
        name="lockout_mount",
    )
    pan_bracket.inertial = Inertial.from_geometry(
        Box((1.20, 5.60, 2.70)),
        mass=650.0,
        origin=Origin(xyz=(0.05, 0.0, 1.35)),
    )

    lamp_array = model.part("lamp_array")
    lamp_array.visual(
        Box((0.08, 4.60, 1.30)),
        origin=Origin(xyz=(-0.04, 0.0, 0.0)),
        material=dark_gray,
        name="rear_frame",
    )
    lamp_array.visual(
        Box((0.18, 4.60, 0.16)),
        origin=Origin(xyz=(0.06, 0.0, 0.58)),
        material=machinery_gray,
        name="top_frame_rail",
    )
    lamp_array.visual(
        Box((0.18, 4.60, 0.16)),
        origin=Origin(xyz=(0.06, 0.0, -0.58)),
        material=machinery_gray,
        name="bottom_frame_rail",
    )
    lamp_array.visual(
        Box((0.18, 0.16, 1.30)),
        origin=Origin(xyz=(0.06, 0.0, 0.0)),
        material=machinery_gray,
        name="center_frame_post",
    )
    for side, y_sign in (("left", 1.0), ("right", -1.0)):
        lamp_array.visual(
            Box((0.18, 0.12, 1.30)),
            origin=Origin(xyz=(0.06, y_sign * 2.28, 0.0)),
            material=machinery_gray,
            name=f"{side}_frame_post",
        )
        lamp_array.visual(
            Cylinder(radius=0.08, length=0.18),
            origin=Origin(
                xyz=(0.0, y_sign * 2.36, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=dark_gray,
            name=f"{side}_trunnion",
        )

    module_y_positions = (-1.74, -0.58, 0.58, 1.74)
    module_z_positions = (0.30, -0.30)
    for row_index, z_center in enumerate(module_z_positions):
        for col_index, y_center in enumerate(module_y_positions):
            lamp_array.visual(
                Box((0.34, 0.96, 0.54)),
                origin=Origin(xyz=(0.17, y_center, z_center)),
                material=matte_black,
                name=f"housing_r{row_index}_c{col_index}",
            )
            lamp_array.visual(
                Box((0.008, 0.90, 0.48)),
                origin=Origin(xyz=(0.344, y_center, z_center)),
                material=lamp_glass,
                name=f"glass_r{row_index}_c{col_index}",
            )

    lamp_array.inertial = Inertial.from_geometry(
        Box((0.45, 4.80, 1.35)),
        mass=520.0,
        origin=Origin(xyz=(0.12, 0.0, 0.0)),
    )

    safety_lockout_arm = model.part("safety_lockout_arm")
    safety_lockout_arm.visual(
        Cylinder(radius=0.035, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_gray,
        name="pivot_barrel",
    )
    safety_lockout_arm.visual(
        Box((0.05, 0.08, 0.86)),
        origin=Origin(xyz=(0.0, 0.075, -0.40)),
        material=safety_red,
        name="lock_bar",
    )
    safety_lockout_arm.visual(
        Box((0.06, 0.14, 0.12)),
        origin=Origin(xyz=(0.0, 0.12, -0.83)),
        material=dark_gray,
        name="lock_hook",
    )
    safety_lockout_arm.visual(
        Box((0.04, 0.10, 0.22)),
        origin=Origin(xyz=(0.0, 0.10, -0.16)),
        material=dark_gray,
        name="lock_stiffener",
    )
    safety_lockout_arm.inertial = Inertial.from_geometry(
        Box((0.10, 0.18, 0.95)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.09, -0.40)),
    )

    model.articulation(
        "pan_rotation",
        ArticulationType.CONTINUOUS,
        parent=mast,
        child=pan_bracket,
        origin=Origin(xyz=(0.0, 0.0, 20.70)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=15000.0, velocity=0.60),
    )
    model.articulation(
        "lamp_tilt",
        ArticulationType.REVOLUTE,
        parent=pan_bracket,
        child=lamp_array,
        origin=Origin(xyz=(0.56, 0.0, 1.86)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6000.0,
            velocity=0.55,
            lower=math.radians(-18.0),
            upper=math.radians(70.0),
        ),
    )
    model.articulation(
        "lockout_arm_hinge",
        ArticulationType.REVOLUTE,
        parent=pan_bracket,
        child=safety_lockout_arm,
        origin=Origin(xyz=(0.48, 2.925, 2.02)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.2,
            lower=math.radians(-8.0),
            upper=math.radians(62.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    pan_bracket = object_model.get_part("pan_bracket")
    lamp_array = object_model.get_part("lamp_array")
    lockout_arm = object_model.get_part("safety_lockout_arm")

    pan_rotation = object_model.get_articulation("pan_rotation")
    lamp_tilt = object_model.get_articulation("lamp_tilt")
    lockout_arm_hinge = object_model.get_articulation("lockout_arm_hinge")

    ctx.expect_origin_gap(
        lamp_array,
        pan_bracket,
        axis="x",
        min_gap=0.25,
        name="lamp array sits forward of the pan pedestal",
    )
    ctx.expect_overlap(
        lamp_array,
        pan_bracket,
        axes="z",
        min_overlap=0.20,
        name="lamp array shares the yoke elevation band",
    )
    ctx.expect_overlap(
        lockout_arm,
        pan_bracket,
        axes="xz",
        min_overlap=0.04,
        name="lockout arm is mounted on the pan bracket side",
    )

    rest_array_pos = ctx.part_world_position(lamp_array)
    with ctx.pose({pan_rotation: math.pi / 2.0}):
        slewed_array_pos = ctx.part_world_position(lamp_array)
    ctx.check(
        "pan bracket slews the lamp head around the mast",
        rest_array_pos is not None
        and slewed_array_pos is not None
        and abs(rest_array_pos[0]) > 0.20
        and abs(slewed_array_pos[1]) > 0.20
        and abs(slewed_array_pos[0]) < abs(rest_array_pos[0]),
        details=f"rest={rest_array_pos}, slewed={slewed_array_pos}",
    )

    rest_lower_glass = ctx.part_element_world_aabb(lamp_array, elem="glass_r1_c1")
    with ctx.pose({lamp_tilt: math.radians(55.0)}):
        tilted_lower_glass = ctx.part_element_world_aabb(lamp_array, elem="glass_r1_c1")
    ctx.check(
        "lamp array tilts upward at positive tilt",
        rest_lower_glass is not None
        and tilted_lower_glass is not None
        and tilted_lower_glass[1][2] > rest_lower_glass[1][2] + 0.20,
        details=f"rest={rest_lower_glass}, tilted={tilted_lower_glass}",
    )

    rest_lock_hook = ctx.part_element_world_aabb(lockout_arm, elem="lock_hook")
    with ctx.pose({lockout_arm_hinge: math.radians(50.0)}):
        raised_lock_hook = ctx.part_element_world_aabb(lockout_arm, elem="lock_hook")
    ctx.check(
        "lockout arm can be raised on its hinge",
        rest_lock_hook is not None
        and raised_lock_hook is not None
        and raised_lock_hook[1][2] > rest_lock_hook[1][2] + 0.12,
        details=f"rest={rest_lock_hook}, raised={raised_lock_hook}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
