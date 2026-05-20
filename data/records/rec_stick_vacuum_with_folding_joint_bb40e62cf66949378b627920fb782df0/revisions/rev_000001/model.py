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
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_stick_vacuum")

    charcoal = model.material("charcoal", rgba=(0.18, 0.18, 0.20, 1.0))
    wand_metal = model.material("wand_metal", rgba=(0.77, 0.79, 0.82, 1.0))
    dust_bin = model.material("dust_bin", rgba=(0.88, 0.72, 0.25, 0.45))
    accent_red = model.material("accent_red", rgba=(0.72, 0.12, 0.12, 1.0))
    head_black = model.material("head_black", rgba=(0.12, 0.12, 0.13, 1.0))

    def yz_section(
        width: float,
        height: float,
        radius: float,
        x: float,
        z_center: float = 0.0,
    ) -> list[tuple[float, float, float]]:
        return [(x, y, z + z_center) for z, y in rounded_rect_profile(height, width, radius)]

    motor_body = model.part("motor_body")
    motor_body.visual(
        Box((0.040, 0.060, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=charcoal,
        name="fold_hinge_block",
    )
    motor_body.visual(
        Box((0.060, 0.070, 0.110)),
        origin=Origin(xyz=(-0.030, 0.0, 0.085)),
        material=charcoal,
        name="body_spine",
    )
    motor_body.visual(
        Cylinder(radius=0.055, length=0.160),
        origin=Origin(xyz=(0.060, 0.0, 0.160), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dust_bin,
        name="dust_bin_cup",
    )
    motor_body.visual(
        Cylinder(radius=0.050, length=0.120),
        origin=Origin(xyz=(-0.080, 0.0, 0.160), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="motor_pod",
    )
    motor_body.visual(
        Cylinder(radius=0.058, length=0.055),
        origin=Origin(xyz=(-0.1675, 0.0, 0.160), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=accent_red,
        name="rear_filter",
    )
    motor_body.visual(
        Box((0.060, 0.042, 0.155)),
        origin=Origin(xyz=(-0.082, 0.0, 0.080)),
        material=charcoal,
        name="grip_body",
    )
    motor_body.visual(
        Box((0.100, 0.055, 0.045)),
        origin=Origin(xyz=(-0.090, 0.0, 0.020)),
        material=accent_red,
        name="battery_pack",
    )
    motor_body.inertial = Inertial.from_geometry(
        Box((0.340, 0.120, 0.240)),
        mass=2.8,
        origin=Origin(xyz=(-0.030, 0.0, 0.110)),
    )

    wand = model.part("wand")
    wand.visual(
        Box((0.040, 0.050, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, -0.0125)),
        material=charcoal,
        name="upper_knuckle",
    )
    wand.visual(
        Cylinder(radius=0.017, length=0.820),
        origin=Origin(xyz=(0.0, 0.0, -0.435), rpy=(0.0, 0.0, 0.0)),
        material=wand_metal,
        name="wand_tube",
    )
    wand.visual(
        Box((0.040, 0.050, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, -0.8625)),
        material=charcoal,
        name="lower_knuckle",
    )
    wand.inertial = Inertial.from_geometry(
        Cylinder(radius=0.020, length=0.880),
        mass=0.7,
        origin=Origin(xyz=(0.0, 0.0, -0.440)),
    )

    floor_head = model.part("floor_head")
    floor_head.visual(
        Box((0.050, 0.055, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, -0.0175)),
        material=charcoal,
        name="neck_block",
    )

    head_shell = section_loft(
        [
            yz_section(0.160, 0.040, 0.012, -0.080, z_center=-0.037),
            yz_section(0.255, 0.038, 0.015, 0.020, z_center=-0.045),
            yz_section(0.275, 0.032, 0.014, 0.110, z_center=-0.040),
        ]
    )
    floor_head.visual(
        mesh_from_geometry(head_shell, "floor_head_shell"),
        material=head_black,
        name="head_shell",
    )
    floor_head.visual(
        Box((0.180, 0.240, 0.008)),
        origin=Origin(xyz=(0.040, 0.0, -0.058)),
        material=accent_red,
        name="brush_window",
    )
    floor_head.inertial = Inertial.from_geometry(
        Box((0.280, 0.260, 0.075)),
        mass=0.9,
        origin=Origin(xyz=(0.040, 0.0, -0.040)),
    )

    model.articulation(
        "body_to_wand_fold",
        ArticulationType.REVOLUTE,
        parent=motor_body,
        child=wand,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.6,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )

    model.articulation(
        "wand_to_floor_head_pitch",
        ArticulationType.REVOLUTE,
        parent=wand,
        child=floor_head,
        origin=Origin(xyz=(0.0, 0.0, -0.880)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.2,
            lower=math.radians(-18.0),
            upper=math.radians(32.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    motor_body = object_model.get_part("motor_body")
    wand = object_model.get_part("wand")
    floor_head = object_model.get_part("floor_head")
    fold = object_model.get_articulation("body_to_wand_fold")
    head_pitch = object_model.get_articulation("wand_to_floor_head_pitch")

    ctx.expect_origin_distance(
        motor_body,
        wand,
        axes="y",
        max_dist=1e-6,
        name="body and wand stay on the center plane",
    )
    ctx.expect_origin_distance(
        wand,
        floor_head,
        axes="y",
        max_dist=1e-6,
        name="wand and floor head stay on the center plane",
    )

    motor_pos = ctx.part_world_position(motor_body)
    head_pos = ctx.part_world_position(floor_head)
    motor_aabb = ctx.part_world_aabb(motor_body)
    wand_aabb = ctx.part_world_aabb(wand)
    head_aabb = ctx.part_world_aabb(floor_head)
    ctx.check(
        "upright chain stacks vertically at rest",
        motor_aabb is not None
        and wand_aabb is not None
        and head_aabb is not None
        and (motor_aabb[0][2] + motor_aabb[1][2]) / 2.0 > (wand_aabb[0][2] + wand_aabb[1][2]) / 2.0
        and (wand_aabb[0][2] + wand_aabb[1][2]) / 2.0 > (head_aabb[0][2] + head_aabb[1][2]) / 2.0,
        details=f"motor={motor_aabb}, wand={wand_aabb}, head={head_aabb}",
    )

    with ctx.pose({fold: fold.motion_limits.upper}):
        folded_head_pos = ctx.part_world_position(floor_head)
    ctx.check(
        "fold joint swings the wand forward",
        head_pos is not None
        and folded_head_pos is not None
        and folded_head_pos[0] > head_pos[0] + 0.35
        and folded_head_pos[2] > head_pos[2] + 0.10,
        details=f"rest={head_pos}, folded={folded_head_pos}",
    )

    rest_shell = ctx.part_element_world_aabb(floor_head, elem="head_shell")
    with ctx.pose({head_pitch: head_pitch.motion_limits.upper}):
        pitched_shell = ctx.part_element_world_aabb(floor_head, elem="head_shell")
    ctx.check(
        "floor head hinge lifts the nose at positive pitch",
        rest_shell is not None
        and pitched_shell is not None
        and pitched_shell[1][2] > rest_shell[1][2] + 0.015
        and pitched_shell[1][0] > rest_shell[1][0] - 0.01,
        details=f"rest={rest_shell}, pitched={pitched_shell}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
