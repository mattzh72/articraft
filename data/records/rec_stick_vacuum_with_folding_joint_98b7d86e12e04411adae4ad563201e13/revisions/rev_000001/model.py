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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_stick_vacuum")

    dark_body = model.material("dark_body", rgba=(0.16, 0.17, 0.19, 1.0))
    red_accent = model.material("red_accent", rgba=(0.74, 0.10, 0.08, 1.0))
    aluminum = model.material("aluminum", rgba=(0.72, 0.75, 0.79, 1.0))
    smoky_clear = model.material("smoky_clear", rgba=(0.72, 0.78, 0.86, 0.45))
    head_black = model.material("head_black", rgba=(0.12, 0.13, 0.14, 1.0))

    motor_body = model.part("motor_body")
    motor_body.visual(
        Box((0.050, 0.055, 0.050)),
        origin=Origin(xyz=(-0.025, 0.0, 0.025)),
        material=dark_body,
        name="pivot_block",
    )
    motor_body.visual(
        Box((0.075, 0.070, 0.060)),
        origin=Origin(xyz=(-0.0875, 0.0, 0.060)),
        material=red_accent,
        name="neck_housing",
    )
    motor_body.visual(
        Cylinder(radius=0.055, length=0.160),
        origin=Origin(xyz=(-0.185, 0.0, 0.095), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=smoky_clear,
        name="dust_cup",
    )
    motor_body.visual(
        Cylinder(radius=0.050, length=0.100),
        origin=Origin(xyz=(-0.305, 0.0, 0.095), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_body,
        name="motor_can",
    )
    handle_geom = tube_from_spline_points(
        [
            (-0.310, 0.0, 0.120),
            (-0.300, 0.0, 0.200),
            (-0.245, 0.0, 0.290),
            (-0.185, 0.0, 0.315),
            (-0.130, 0.0, 0.245),
            (-0.110, 0.0, 0.130),
        ],
        radius=0.016,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    motor_body.visual(
        mesh_from_geometry(handle_geom, "vacuum_handle"),
        material=dark_body,
        name="handle_loop",
    )
    motor_body.visual(
        Box((0.130, 0.055, 0.090)),
        origin=Origin(xyz=(-0.245, 0.0, 0.005)),
        material=dark_body,
        name="battery_pack",
    )
    motor_body.inertial = Inertial.from_geometry(
        Box((0.380, 0.120, 0.350)),
        mass=2.4,
        origin=Origin(xyz=(-0.190, 0.0, 0.135)),
    )

    wand = model.part("wand")
    wand.visual(
        Cylinder(radius=0.018, length=0.044),
        origin=Origin(xyz=(0.018, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_body,
        name="fold_collar",
    )
    wand.visual(
        Box((0.045, 0.032, 0.022)),
        origin=Origin(xyz=(0.0225, 0.0, 0.0)),
        material=dark_body,
        name="hinge_block",
    )
    wand.visual(
        Cylinder(radius=0.016, length=0.740),
        origin=Origin(xyz=(0.415, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aluminum,
        name="wand_tube",
    )
    wand.inertial = Inertial.from_geometry(
        Box((0.800, 0.050, 0.050)),
        mass=0.7,
        origin=Origin(xyz=(0.400, 0.0, 0.0)),
    )

    floor_head = model.part("floor_head")
    floor_head.visual(
        Cylinder(radius=0.012, length=0.036),
        origin=Origin(xyz=(0.012, 0.0, -0.002), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_body,
        name="pitch_barrel",
    )
    floor_head.visual(
        Box((0.008, 0.028, 0.020)),
        origin=Origin(xyz=(0.004, 0.0, -0.002)),
        material=dark_body,
        name="pitch_mount",
    )
    floor_head.visual(
        Box((0.028, 0.032, 0.040)),
        origin=Origin(xyz=(0.018, 0.0, -0.024)),
        material=dark_body,
        name="neck_yoke",
    )
    floor_head.visual(
        Box((0.070, 0.048, 0.032)),
        origin=Origin(xyz=(0.060, 0.0, -0.058)),
        material=dark_body,
        name="nozzle_throat",
    )
    head_shell_geom = ExtrudeGeometry(
        rounded_rect_profile(0.280, 0.085, 0.018, corner_segments=8),
        0.024,
        center=True,
    )
    floor_head.visual(
        mesh_from_geometry(head_shell_geom, "floor_head_shell"),
        origin=Origin(xyz=(0.150, 0.0, -0.062)),
        material=head_black,
        name="head_shell",
    )
    floor_head.visual(
        Box((0.022, 0.080, 0.010)),
        origin=Origin(xyz=(0.285, 0.0, -0.069)),
        material=red_accent,
        name="front_lip",
    )
    floor_head.inertial = Inertial.from_geometry(
        Box((0.300, 0.090, 0.080)),
        mass=0.9,
        origin=Origin(xyz=(0.150, 0.0, -0.050)),
    )

    model.articulation(
        "body_to_wand",
        ArticulationType.REVOLUTE,
        parent=motor_body,
        child=wand,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=-1.25,
            upper=0.35,
        ),
    )
    model.articulation(
        "wand_to_floor_head",
        ArticulationType.REVOLUTE,
        parent=wand,
        child=floor_head,
        origin=Origin(xyz=(0.785, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.5,
            lower=-0.65,
            upper=0.65,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    motor_body = object_model.get_part("motor_body")
    wand = object_model.get_part("wand")
    floor_head = object_model.get_part("floor_head")
    fold_joint = object_model.get_articulation("body_to_wand")
    pitch_joint = object_model.get_articulation("wand_to_floor_head")

    ctx.expect_gap(
        wand,
        motor_body,
        axis="x",
        positive_elem="hinge_block",
        negative_elem="pivot_block",
        max_gap=0.002,
        max_penetration=0.0,
        name="wand hinge block seats against motor body pivot",
    )
    ctx.expect_gap(
        wand,
        floor_head,
        axis="z",
        positive_elem="wand_tube",
        negative_elem="head_shell",
        min_gap=0.025,
        max_gap=0.080,
        name="wand stays clearly above the floor head shell",
    )

    rest_head_pos = ctx.part_world_position(floor_head)
    with ctx.pose({fold_joint: -1.10}):
        folded_head_pos = ctx.part_world_position(floor_head)
    ctx.check(
        "fold joint brings the wand and head down toward low furniture reach",
        rest_head_pos is not None
        and folded_head_pos is not None
        and folded_head_pos[0] < rest_head_pos[0] - 0.40
        and folded_head_pos[2] < rest_head_pos[2] - 0.45,
        details=f"rest={rest_head_pos}, folded={folded_head_pos}",
    )

    def visual_center(part_name: str, elem_name: str):
        aabb = ctx.part_element_world_aabb(part_name, elem=elem_name)
        if aabb is None:
            return None
        (xmin, ymin, zmin), (xmax, ymax, zmax) = aabb
        return ((xmin + xmax) * 0.5, (ymin + ymax) * 0.5, (zmin + zmax) * 0.5)

    rest_lip_center = visual_center("floor_head", "front_lip")
    with ctx.pose({pitch_joint: 0.50}):
        pitched_up_lip_center = visual_center("floor_head", "front_lip")
    with ctx.pose({pitch_joint: -0.50}):
        pitched_down_lip_center = visual_center("floor_head", "front_lip")
    ctx.check(
        "compact floor head wrist still gives useful pitch travel",
        rest_lip_center is not None
        and pitched_up_lip_center is not None
        and pitched_down_lip_center is not None
        and pitched_up_lip_center[2] > rest_lip_center[2] + 0.040
        and pitched_down_lip_center[2] < rest_lip_center[2] - 0.040,
        details=(
            f"rest={rest_lip_center}, up={pitched_up_lip_center}, "
            f"down={pitched_down_lip_center}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
