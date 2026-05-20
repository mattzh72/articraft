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
    rounded_rect_profile,
    section_loft,
)


def _section_at_z(width: float, depth: float, radius: float, z: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in rounded_rect_profile(width, depth, radius)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="countertop_juicer")

    body_silver = model.material("body_silver", rgba=(0.82, 0.84, 0.86, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.14, 0.15, 0.16, 1.0))
    smoke_clear = model.material("smoke_clear", rgba=(0.58, 0.63, 0.66, 0.50))
    basket_steel = model.material("basket_steel", rgba=(0.86, 0.87, 0.88, 1.0))
    spout_dark = model.material("spout_dark", rgba=(0.10, 0.10, 0.11, 1.0))

    chamber_center_x = 0.028
    hinge_x = -0.055
    hinge_z = 0.225
    chamber_seat_z = 0.138

    base = model.part("base")

    body_shell = mesh_from_geometry(
        section_loft(
            [
                _section_at_z(0.270, 0.188, 0.052, 0.000),
                _section_at_z(0.258, 0.182, 0.050, 0.046),
                _section_at_z(0.236, 0.168, 0.044, 0.092),
                _section_at_z(0.206, 0.154, 0.036, 0.124),
            ]
        ),
        "base_shell",
    )
    base.visual(body_shell, material=body_silver, name="body_shell")

    base.visual(
        Box((0.158, 0.158, 0.008)),
        origin=Origin(xyz=(chamber_center_x, 0.0, 0.141)),
        material=trim_dark,
        name="top_ring",
    )
    base.visual(
        Box((0.010, 0.156, 0.100)),
        origin=Origin(xyz=(chamber_center_x + 0.073, 0.0, 0.174)),
        material=smoke_clear,
        name="chamber_front",
    )
    base.visual(
        Box((0.010, 0.156, 0.100)),
        origin=Origin(xyz=(chamber_center_x - 0.073, 0.0, 0.174)),
        material=smoke_clear,
        name="chamber_rear",
    )
    base.visual(
        Box((0.146, 0.010, 0.100)),
        origin=Origin(xyz=(chamber_center_x, 0.073, 0.174)),
        material=smoke_clear,
        name="chamber_side_0",
    )
    base.visual(
        Box((0.146, 0.010, 0.100)),
        origin=Origin(xyz=(chamber_center_x, -0.073, 0.174)),
        material=smoke_clear,
        name="chamber_side_1",
    )
    base.visual(
        Box((0.036, 0.114, 0.036)),
        origin=Origin(xyz=(-0.082, 0.0, 0.194)),
        material=body_silver,
        name="hinge_block",
    )
    base.visual(
        Box((0.016, 0.114, 0.028)),
        origin=Origin(xyz=(-0.057, 0.0, 0.194)),
        material=body_silver,
        name="hinge_bridge",
    )
    base.visual(
        Box((0.050, 0.024, 0.060)),
        origin=Origin(xyz=(0.078, 0.096, 0.074)),
        material=body_silver,
        name="control_pod",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.270, 0.188, 0.244)),
        mass=5.8,
        origin=Origin(xyz=(0.0, 0.0, 0.122)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((0.050, 0.180, 0.014)),
        origin=Origin(xyz=(0.024, 0.0, 0.016)),
        material=smoke_clear,
        name="lid_rear_crown",
    )
    lid.visual(
        Box((0.062, 0.180, 0.014)),
        origin=Origin(xyz=(0.136, 0.0, 0.016)),
        material=smoke_clear,
        name="lid_front_crown",
    )
    lid.visual(
        Box((0.070, 0.036, 0.014)),
        origin=Origin(xyz=(chamber_center_x - hinge_x, 0.053, 0.016)),
        material=smoke_clear,
        name="lid_side_crown_0",
    )
    lid.visual(
        Box((0.070, 0.036, 0.014)),
        origin=Origin(xyz=(chamber_center_x - hinge_x, -0.053, 0.016)),
        material=smoke_clear,
        name="lid_side_crown_1",
    )
    lid.visual(
        Box((0.018, 0.188, 0.030)),
        origin=Origin(xyz=(0.172, 0.0, 0.001)),
        material=smoke_clear,
        name="lid_front_skirt",
    )
    lid.visual(
        Box((0.016, 0.188, 0.022)),
        origin=Origin(xyz=(0.002, 0.0, 0.005)),
        material=smoke_clear,
        name="lid_rear_skirt",
    )
    lid.visual(
        Box((0.118, 0.014, 0.024)),
        origin=Origin(xyz=(0.094, 0.093, 0.006)),
        material=smoke_clear,
        name="lid_side_skirt_0",
    )
    lid.visual(
        Box((0.118, 0.014, 0.024)),
        origin=Origin(xyz=(0.094, -0.093, 0.006)),
        material=smoke_clear,
        name="lid_side_skirt_1",
    )
    lid.visual(
        Box((0.040, 0.114, 0.018)),
        origin=Origin(xyz=(0.020, 0.0, 0.006)),
        material=smoke_clear,
        name="rear_bridge",
    )
    lid.visual(
        Box((0.009, 0.070, 0.110)),
        origin=Origin(xyz=(chamber_center_x - hinge_x + 0.0305, 0.0, 0.075)),
        material=smoke_clear,
        name="chute_front",
    )
    lid.visual(
        Box((0.009, 0.070, 0.110)),
        origin=Origin(xyz=(chamber_center_x - hinge_x - 0.0305, 0.0, 0.075)),
        material=smoke_clear,
        name="chute_rear",
    )
    lid.visual(
        Box((0.052, 0.009, 0.110)),
        origin=Origin(xyz=(chamber_center_x - hinge_x, 0.0305, 0.075)),
        material=smoke_clear,
        name="chute_side_0",
    )
    lid.visual(
        Box((0.052, 0.009, 0.110)),
        origin=Origin(xyz=(chamber_center_x - hinge_x, -0.0305, 0.075)),
        material=smoke_clear,
        name="chute_side_1",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.212, 0.200, 0.150)),
        mass=0.9,
        origin=Origin(xyz=(0.090, 0.0, 0.050)),
    )

    basket = model.part("basket")
    basket_shell = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (0.062, 0.000),
                (0.068, 0.006),
                (0.071, 0.018),
                (0.071, 0.060),
            ],
            [
                (0.053, 0.006),
                (0.060, 0.012),
                (0.063, 0.018),
                (0.063, 0.056),
            ],
            segments=48,
        ),
        "basket_shell",
    )
    basket.visual(basket_shell, material=basket_steel, name="basket_shell")
    basket.inertial = Inertial.from_geometry(
        Cylinder(radius=0.071, length=0.060),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
    )

    pusher = model.part("pusher")
    pusher.visual(
        Cylinder(radius=0.026, length=0.148),
        origin=Origin(xyz=(0.0, 0.0, -0.074)),
        material=trim_dark,
        name="pusher_body",
    )
    pusher.visual(
        Cylinder(radius=0.033, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=trim_dark,
        name="pusher_cap",
    )
    pusher.inertial = Inertial.from_geometry(
        Cylinder(radius=0.033, length=0.172),
        mass=0.28,
        origin=Origin(xyz=(0.0, 0.0, -0.050)),
    )

    spout = model.part("spout")
    spout.visual(
        Box((0.042, 0.038, 0.024)),
        origin=Origin(xyz=(0.021, 0.0, 0.0)),
        material=spout_dark,
        name="spout_body",
    )
    spout.visual(
        Box((0.018, 0.026, 0.016)),
        origin=Origin(xyz=(0.051, 0.0, -0.004)),
        material=spout_dark,
        name="spout_tip",
    )
    spout.visual(
        Cylinder(radius=0.011, length=0.042),
        origin=Origin(xyz=(0.011, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_dark,
        name="spout_pivot_barrel",
    )
    spout.inertial = Inertial.from_geometry(
        Box((0.060, 0.042, 0.024)),
        mass=0.12,
        origin=Origin(xyz=(0.030, 0.0, 0.0)),
    )

    speed_lever = model.part("speed_lever")
    speed_lever.visual(
        Box((0.018, 0.010, 0.018)),
        origin=Origin(xyz=(0.0, 0.005, 0.008)),
        material=trim_dark,
        name="lever_mount",
    )
    speed_lever.visual(
        Cylinder(radius=0.007, length=0.026),
        origin=Origin(xyz=(0.0, 0.005, 0.008), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_dark,
        name="lever_barrel",
    )
    speed_lever.visual(
        Box((0.012, 0.014, 0.040)),
        origin=Origin(xyz=(0.0, 0.012, 0.030)),
        material=trim_dark,
        name="lever_stem",
    )
    speed_lever.visual(
        Box((0.022, 0.024, 0.015)),
        origin=Origin(xyz=(0.0, 0.017, 0.056)),
        material=spout_dark,
        name="lever_grip",
    )
    speed_lever.inertial = Inertial.from_geometry(
        Box((0.026, 0.026, 0.072)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.013, 0.032)),
    )

    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.6,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )

    model.articulation(
        "base_to_basket",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=basket,
        origin=Origin(xyz=(chamber_center_x, 0.0, 0.148)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=25.0),
    )

    model.articulation(
        "lid_to_pusher",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=pusher,
        origin=Origin(xyz=(chamber_center_x - hinge_x, 0.0, 0.130)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=0.18,
            lower=0.0,
            upper=0.074,
        ),
    )

    model.articulation(
        "base_to_spout",
        ArticulationType.REVOLUTE,
        parent=base,
        child=spout,
        origin=Origin(xyz=(0.106, 0.0, 0.166)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.2,
            velocity=2.0,
            lower=-0.35,
            upper=0.78,
        ),
    )

    model.articulation(
        "base_to_speed_lever",
        ArticulationType.REVOLUTE,
        parent=base,
        child=speed_lever,
        origin=Origin(xyz=(0.078, 0.108, 0.070)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=2.5,
            lower=-0.55,
            upper=0.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    basket = object_model.get_part("basket")
    lid = object_model.get_part("lid")
    pusher = object_model.get_part("pusher")
    spout = object_model.get_part("spout")
    speed_lever = object_model.get_part("speed_lever")

    lid_hinge = object_model.get_articulation("base_to_lid")
    pusher_slide = object_model.get_articulation("lid_to_pusher")
    spout_hinge = object_model.get_articulation("base_to_spout")
    lever_hinge = object_model.get_articulation("base_to_speed_lever")

    ctx.expect_gap(
        basket,
        base,
        axis="z",
        min_gap=0.001,
        max_gap=0.010,
        positive_elem="basket_shell",
        negative_elem="top_ring",
        name="basket sits just above the top ring",
    )
    ctx.expect_overlap(
        basket,
        base,
        axes="xy",
        min_overlap=0.130,
        elem_a="basket_shell",
        elem_b="top_ring",
        name="basket remains centered over the juicing chamber",
    )

    pusher_rest = ctx.part_world_position(pusher)
    with ctx.pose({pusher_slide: pusher_slide.motion_limits.upper}):
        pusher_extended = ctx.part_world_position(pusher)
    ctx.check(
        "pusher slides upward in the chute",
        pusher_rest is not None
        and pusher_extended is not None
        and pusher_extended[2] > pusher_rest[2] + 0.05,
        details=f"rest={pusher_rest}, extended={pusher_extended}",
    )

    lid_front_rest = ctx.part_element_world_aabb(lid, elem="lid_front_skirt")
    with ctx.pose({lid_hinge: lid_hinge.motion_limits.upper}):
        lid_front_open = ctx.part_element_world_aabb(lid, elem="lid_front_skirt")
    ctx.check(
        "lid opens upward on the rear hinge",
        lid_front_rest is not None
        and lid_front_open is not None
        and lid_front_open[1][2] > lid_front_rest[1][2] + 0.05,
        details=f"closed={lid_front_rest}, open={lid_front_open}",
    )

    spout_tip_rest = ctx.part_element_world_aabb(spout, elem="spout_tip")
    with ctx.pose({spout_hinge: spout_hinge.motion_limits.upper}):
        spout_tip_raised = ctx.part_element_world_aabb(spout, elem="spout_tip")
    ctx.check(
        "spout can pivot upward into a drip-stop pose",
        spout_tip_rest is not None
        and spout_tip_raised is not None
        and spout_tip_raised[1][2] > spout_tip_rest[1][2] + 0.02,
        details=f"rest={spout_tip_rest}, raised={spout_tip_raised}",
    )

    ctx.expect_contact(
        speed_lever,
        base,
        elem_a="lever_mount",
        elem_b="control_pod",
        name="speed lever is mounted on the side control pod",
    )

    lever_grip_rest = ctx.part_element_world_aabb(speed_lever, elem="lever_grip")
    with ctx.pose({lever_hinge: lever_hinge.motion_limits.lower}):
        lever_grip_low = ctx.part_element_world_aabb(speed_lever, elem="lever_grip")
    ctx.check(
        "speed lever sweeps through a side-mounted arc",
        lever_grip_rest is not None
        and lever_grip_low is not None
        and abs(lever_grip_low[0][2] - lever_grip_rest[0][2]) > 0.01,
        details=f"rest={lever_grip_rest}, low={lever_grip_low}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
