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
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _circle_profile(
    radius: float,
    *,
    center: tuple[float, float] = (0.0, 0.0),
    segments: int = 48,
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + radius * math.cos(2.0 * math.pi * i / segments),
            cy + radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="centrifugal_juicer")

    body_white = model.material("body_white", rgba=(0.95, 0.95, 0.93, 1.0))
    smoke = model.material("smoke", rgba=(0.22, 0.24, 0.26, 0.95))
    clear_plastic = model.material("clear_plastic", rgba=(0.78, 0.88, 0.92, 0.28))
    clear_grey = model.material("clear_grey", rgba=(0.72, 0.78, 0.82, 0.42))
    dark_grey = model.material("dark_grey", rgba=(0.16, 0.17, 0.19, 1.0))
    steel = model.material("steel", rgba=(0.76, 0.78, 0.80, 1.0))
    tray_black = model.material("tray_black", rgba=(0.10, 0.11, 0.12, 1.0))
    dial_silver = model.material("dial_silver", rgba=(0.84, 0.85, 0.86, 1.0))

    body = model.part("body")

    body.visual(
        Box((0.15, 0.23, 0.10)),
        origin=Origin(xyz=(-0.035, 0.0, 0.05)),
        material=body_white,
        name="rear_base",
    )
    body.visual(
        Box((0.08, 0.046, 0.10)),
        origin=Origin(xyz=(0.07, 0.074, 0.05)),
        material=body_white,
        name="front_cheek_0",
    )
    body.visual(
        Box((0.08, 0.046, 0.10)),
        origin=Origin(xyz=(0.07, -0.074, 0.05)),
        material=body_white,
        name="front_cheek_1",
    )
    body.visual(
        Box((0.08, 0.12, 0.018)),
        origin=Origin(xyz=(0.08, 0.0, 0.099)),
        material=body_white,
        name="tray_roof",
    )
    body.visual(
        Box((0.020, 0.110, 0.018)),
        origin=Origin(xyz=(0.100, 0.0, 0.041)),
        material=body_white,
        name="front_bridge",
    )
    body.visual(
        Box((0.018, 0.024, 0.118)),
        origin=Origin(xyz=(-0.112, -0.032, 0.159)),
        material=body_white,
        name="rear_post_0",
    )
    body.visual(
        Box((0.018, 0.024, 0.118)),
        origin=Origin(xyz=(-0.112, 0.032, 0.159)),
        material=body_white,
        name="rear_post_1",
    )
    body.visual(
        Box((0.014, 0.030, 0.020)),
        origin=Origin(xyz=(-0.119, -0.032, 0.228)),
        material=body_white,
        name="hinge_mount_0",
    )
    body.visual(
        Box((0.014, 0.030, 0.020)),
        origin=Origin(xyz=(-0.119, 0.032, 0.228)),
        material=body_white,
        name="hinge_mount_1",
    )

    shell_outer = [
        (0.055, 0.0),
        (0.088, 0.015),
        (0.108, 0.050),
        (0.112, 0.105),
        (0.106, 0.129),
    ]
    shell_inner = [
        (0.000, 0.004),
        (0.075, 0.014),
        (0.095, 0.050),
        (0.096, 0.108),
        (0.091, 0.122),
    ]
    shell_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(shell_outer, shell_inner, segments=56),
        "juicer_bowl_shell",
    )
    body.visual(
        shell_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.096)),
        material=body_white,
        name="bowl_shell",
    )

    body.visual(
        Box((0.065, 0.050, 0.022)),
        origin=Origin(xyz=(0.128, 0.0, 0.176)),
        material=body_white,
        name="spout_body",
    )
    body.visual(
        Box((0.028, 0.040, 0.010)),
        origin=Origin(xyz=(0.162, 0.0, 0.169)),
        material=clear_grey,
        name="spout_lip",
    )

    for y in (-0.032, 0.032):
        body.visual(
            Cylinder(radius=0.008, length=0.028),
            origin=Origin(xyz=(-0.112, y, 0.229), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=smoke,
            name=f"hinge_barrel_{0 if y < 0.0 else 1}",
        )

    body.inertial = Inertial.from_geometry(
        Box((0.29, 0.23, 0.39)),
        mass=5.8,
        origin=Origin(xyz=(0.0, 0.0, 0.195)),
    )

    basket = model.part("basket")
    basket.visual(
        Cylinder(radius=0.084, length=0.086),
        origin=Origin(xyz=(0.0, 0.0, 0.043)),
        material=steel,
        name="basket_wall",
    )
    basket.visual(
        Cylinder(radius=0.092, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.086)),
        material=steel,
        name="basket_rim",
    )
    basket.visual(
        Cylinder(radius=0.020, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=dark_grey,
        name="basket_hub",
    )
    basket.inertial = Inertial.from_geometry(
        Cylinder(radius=0.092, length=0.096),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, 0.048)),
    )

    model.articulation(
        "body_to_basket",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=basket,
        origin=Origin(xyz=(0.0, 0.0, 0.107)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=30.0),
    )

    lid = model.part("lid")
    lid_cover_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(0.106, center=(0.118, 0.0), segments=64),
            [_circle_profile(0.031, center=(0.138, 0.046), segments=40)],
            0.008,
            center=True,
        ),
        "juicer_lid_cover",
    )
    lid.visual(
        lid_cover_mesh,
        material=clear_plastic,
        name="cover",
    )
    lid.visual(
        Box((0.028, 0.022, 0.008)),
        origin=Origin(xyz=(0.014, 0.0, 0.0)),
        material=clear_plastic,
        name="hinge_bridge",
    )

    chute_outer = [
        (0.038, 0.000),
        (0.038, 0.018),
        (0.034, 0.040),
        (0.034, 0.148),
    ]
    chute_inner = [
        (0.031, 0.004),
        (0.031, 0.148),
    ]
    chute_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(chute_outer, chute_inner, segments=40),
        "juicer_feed_chute",
    )
    lid.visual(
        chute_mesh,
        origin=Origin(xyz=(0.138, 0.046, 0.002)),
        material=clear_plastic,
        name="chute",
    )
    lid.visual(
        Box((0.026, 0.058, 0.010)),
        origin=Origin(xyz=(0.201, 0.0, 0.007)),
        material=clear_grey,
        name="handle",
    )
    lid.visual(
        Cylinder(radius=0.008, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=smoke,
        name="hinge_barrel",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.23, 0.17, 0.17)),
        mass=0.6,
        origin=Origin(xyz=(0.11, 0.0, 0.05)),
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-0.112, 0.0, 0.229)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.2,
            lower=0.0,
            upper=1.22,
        ),
    )

    pusher = model.part("pusher")
    pusher.visual(
        Cylinder(radius=0.031, length=0.180),
        origin=Origin(xyz=(0.0, 0.0, -0.055)),
        material=smoke,
        name="shaft",
    )
    pusher.visual(
        Cylinder(radius=0.039, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=dark_grey,
        name="stop_flange",
    )
    pusher.visual(
        Cylinder(radius=0.036, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=dark_grey,
        name="cap",
    )
    pusher.inertial = Inertial.from_geometry(
        Cylinder(radius=0.036, length=0.200),
        mass=0.15,
        origin=Origin(xyz=(0.0, 0.0, -0.040)),
    )

    model.articulation(
        "lid_to_pusher",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=pusher,
        origin=Origin(xyz=(0.138, 0.046, 0.150)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=0.12,
            lower=0.0,
            upper=0.06,
        ),
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.007, length=0.012),
        origin=Origin(xyz=(0.006, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_grey,
        name="shaft",
    )
    dial.visual(
        Cylinder(radius=0.024, length=0.014),
        origin=Origin(xyz=(0.019, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dial_silver,
        name="dial_face",
    )
    dial.visual(
        Cylinder(radius=0.027, length=0.006),
        origin=Origin(xyz=(0.026, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_grey,
        name="grip_ring",
    )
    dial.visual(
        Box((0.004, 0.016, 0.007)),
        origin=Origin(xyz=(0.028, 0.012, 0.0)),
        material=dark_grey,
        name="pointer",
    )
    dial.inertial = Inertial.from_geometry(
        Box((0.065, 0.07, 0.04)),
        mass=0.08,
        origin=Origin(xyz=(0.022, 0.0, 0.0)),
    )

    model.articulation(
        "body_to_dial",
        ArticulationType.REVOLUTE,
        parent=body,
        child=dial,
        origin=Origin(xyz=(0.110, 0.0, 0.041)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=1.5,
            lower=-0.6,
            upper=2.2,
        ),
    )

    tray = model.part("tray")
    tray.visual(
        Box((0.110, 0.094, 0.004)),
        origin=Origin(xyz=(0.055, 0.0, 0.002)),
        material=tray_black,
        name="tray_floor",
    )
    tray.visual(
        Box((0.110, 0.006, 0.012)),
        origin=Origin(xyz=(0.055, 0.044, 0.008)),
        material=tray_black,
        name="tray_wall_0",
    )
    tray.visual(
        Box((0.110, 0.006, 0.012)),
        origin=Origin(xyz=(0.055, -0.044, 0.008)),
        material=tray_black,
        name="tray_wall_1",
    )
    tray.visual(
        Box((0.006, 0.094, 0.012)),
        origin=Origin(xyz=(0.003, 0.0, 0.008)),
        material=tray_black,
        name="tray_back",
    )
    tray.visual(
        Box((0.010, 0.094, 0.016)),
        origin=Origin(xyz=(0.105, 0.0, 0.008)),
        material=tray_black,
        name="tray_front",
    )
    tray.inertial = Inertial.from_geometry(
        Box((0.115, 0.10, 0.02)),
        mass=0.12,
        origin=Origin(xyz=(0.055, 0.0, 0.010)),
    )

    model.articulation(
        "body_to_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tray,
        origin=Origin(xyz=(0.040, 0.0, 0.016)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.10,
            lower=0.0,
            upper=0.05,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    basket = object_model.get_part("basket")
    lid = object_model.get_part("lid")
    pusher = object_model.get_part("pusher")
    tray = object_model.get_part("tray")
    dial = object_model.get_part("dial")

    lid_hinge = object_model.get_articulation("body_to_lid")
    basket_spin = object_model.get_articulation("body_to_basket")
    pusher_slide = object_model.get_articulation("lid_to_pusher")
    dial_joint = object_model.get_articulation("body_to_dial")
    tray_slide = object_model.get_articulation("body_to_tray")

    ctx.allow_overlap(
        body,
        basket,
        elem_a="bowl_shell",
        elem_b="basket_wall",
        reason="The juicing chamber is authored as a thin-walled lathed shell, but compiled overlap review treats that closed shell as a solid volume around the spinning basket.",
    )
    ctx.allow_overlap(
        body,
        basket,
        elem_a="bowl_shell",
        elem_b="basket_rim",
        reason="The basket rim intentionally sits inside the chamber shell volume that is represented by the closed lathed bowl shell mesh.",
    )
    ctx.allow_overlap(
        lid,
        pusher,
        elem_a="chute",
        elem_b="shaft",
        reason="The feed chute is authored as a closed thin-walled shell mesh, so the pusher's sliding fit must be scoped as an intentional nested overlap against that proxy volume.",
    )

    ctx.expect_within(
        basket,
        body,
        axes="xy",
        inner_elem="basket_rim",
        outer_elem="bowl_shell",
        margin=0.003,
        name="basket stays inside the juicing bowl",
    )
    ctx.expect_within(
        pusher,
        lid,
        axes="xy",
        inner_elem="shaft",
        outer_elem="chute",
        margin=0.002,
        name="pusher stays centered in the feed chute",
    )
    ctx.expect_gap(
        lid,
        body,
        axis="z",
        positive_elem="cover",
        negative_elem="bowl_shell",
        min_gap=-0.001,
        max_gap=0.012,
        name="lid closes down onto the bowl rim",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="cover",
        elem_b="bowl_shell",
        min_overlap=0.18,
        name="lid covers the juicing bowl footprint",
    )

    closed_handle = ctx.part_element_world_aabb(lid, elem="handle")
    rest_pusher = ctx.part_world_position(pusher)
    rest_tray = ctx.part_world_position(tray)
    dial_pos = ctx.part_world_position(dial)
    body_pos = ctx.part_world_position(body)

    with ctx.pose({lid_hinge: 1.0}):
        open_handle = ctx.part_element_world_aabb(lid, elem="handle")

    ctx.check(
        "lid opens upward from the rear hinge",
        closed_handle is not None
        and open_handle is not None
        and open_handle[1][2] > closed_handle[1][2] + 0.09,
        details=f"closed_handle={closed_handle}, open_handle={open_handle}",
    )

    with ctx.pose({pusher_slide: 0.06}):
        raised_pusher = ctx.part_world_position(pusher)
        ctx.expect_within(
            pusher,
            lid,
            axes="xy",
            inner_elem="shaft",
            outer_elem="chute",
            margin=0.002,
            name="raised pusher stays aligned to the chute",
        )

    ctx.check(
        "pusher retracts upward",
        rest_pusher is not None
        and raised_pusher is not None
        and raised_pusher[2] > rest_pusher[2] + 0.04,
        details=f"rest_pusher={rest_pusher}, raised_pusher={raised_pusher}",
    )

    with ctx.pose({tray_slide: 0.05}):
        extended_tray = ctx.part_world_position(tray)
        ctx.expect_overlap(
            tray,
            body,
            axes="x",
            elem_a="tray_floor",
            elem_b="tray_roof",
            min_overlap=0.02,
            name="extended tray retains insertion under the body",
        )

    ctx.check(
        "tray slides out toward the front",
        rest_tray is not None
        and extended_tray is not None
        and extended_tray[0] > rest_tray[0] + 0.04,
        details=f"rest_tray={rest_tray}, extended_tray={extended_tray}",
    )

    ctx.check(
        "selector dial is mounted on the front face",
        body_pos is not None
        and dial_pos is not None
        and dial_pos[0] > body_pos[0] + 0.09
        and dial_pos[2] < body_pos[2] + 0.08,
        details=f"body_pos={body_pos}, dial_pos={dial_pos}",
    )
    ctx.expect_gap(
        body,
        tray,
        axis="z",
        positive_elem="spout_lip",
        negative_elem="tray_floor",
        min_gap=0.12,
        max_gap=0.18,
        name="juice outlet sits above the drip tray",
    )
    ctx.check(
        "basket spins vertically and dial turns on a front shaft",
        basket_spin.axis == (0.0, 0.0, 1.0) and dial_joint.axis == (1.0, 0.0, 0.0),
        details=f"basket_axis={basket_spin.axis}, dial_axis={dial_joint.axis}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
