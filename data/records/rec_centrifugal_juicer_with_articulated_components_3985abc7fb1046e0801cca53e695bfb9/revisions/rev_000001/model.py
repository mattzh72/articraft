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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stainless_steel_juicer")

    stainless = model.material("stainless", rgba=(0.78, 0.80, 0.82, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.14, 0.14, 0.15, 1.0))
    clear_plastic = model.material("clear_plastic", rgba=(0.82, 0.90, 0.96, 0.34))
    smoke_clear = model.material("smoke_clear", rgba=(0.68, 0.76, 0.82, 0.22))
    tray_black = model.material("tray_black", rgba=(0.12, 0.12, 0.13, 1.0))

    def z_section(width: float, depth: float, radius: float, z: float):
        return [(x, y, z) for x, y in rounded_rect_profile(width, depth, radius)]

    base = model.part("base")

    base_shell = section_loft(
        [
            z_section(0.215, 0.175, 0.035, 0.0),
            z_section(0.285, 0.225, 0.050, 0.050),
            z_section(0.255, 0.205, 0.042, 0.145),
            z_section(0.232, 0.188, 0.036, 0.180),
        ]
    )
    base.visual(
        mesh_from_geometry(base_shell, "base_shell"),
        material=stainless,
        name="base_shell",
    )

    deck_geom = ExtrudeGeometry(rounded_rect_profile(0.180, 0.160, 0.020), 0.012)
    base.visual(
        mesh_from_geometry(deck_geom, "base_deck"),
        origin=Origin(xyz=(0.0, 0.0, 0.176)),
        material=dark_trim,
        name="base_deck",
    )
    base_mount_ring = LatheGeometry.from_shell_profiles(
        [(0.095, 0.0), (0.095, 0.006)],
        [(0.060, 0.0), (0.060, 0.006)],
        segments=56,
    )
    base.visual(
        mesh_from_geometry(base_mount_ring, "base_mount_ring"),
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
        material=dark_trim,
        name="base_mount_ring",
    )
    base.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
        material=dark_trim,
        name="drive_collar",
    )
    base.visual(
        Box((0.060, 0.060, 0.028)),
        origin=Origin(xyz=(-0.132, 0.0, 0.198)),
        material=dark_trim,
        name="hinge_block",
    )
    base.visual(
        Box((0.040, 0.060, 0.016)),
        origin=Origin(xyz=(-0.106, 0.0, 0.178)),
        material=dark_trim,
        name="hinge_support",
    )
    base.visual(
        Box((0.090, 0.112, 0.030)),
        origin=Origin(xyz=(0.176, 0.0, 0.004)),
        material=dark_trim,
        name="tray_support",
    )
    for y in (-0.049, 0.049):
        base.visual(
            Box((0.076, 0.008, 0.010)),
            origin=Origin(xyz=(0.188, y, -0.006)),
            material=dark_trim,
            name=f"tray_rail_{0 if y < 0 else 1}",
        )
    base.inertial = Inertial.from_geometry(
        Box((0.285, 0.225, 0.210)),
        mass=6.8,
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
    )

    chamber = model.part("chamber")
    chamber_shell = LatheGeometry.from_shell_profiles(
        [(0.100, 0.0), (0.100, 0.105)],
        [(0.092, 0.003), (0.092, 0.102)],
        segments=64,
    )
    chamber.visual(
        mesh_from_geometry(chamber_shell, "chamber_shell"),
        material=clear_plastic,
        name="chamber_shell",
    )
    chamber.visual(
        Box((0.048, 0.032, 0.022)),
        origin=Origin(xyz=(0.111, 0.0, 0.030)),
        material=clear_plastic,
        name="spout_body",
    )
    chamber.visual(
        Cylinder(radius=0.011, length=0.034),
        origin=Origin(xyz=(0.147, 0.0, 0.030), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=clear_plastic,
        name="spout_tip",
    )
    chamber.visual(
        Box((0.036, 0.070, 0.010)),
        origin=Origin(xyz=(-0.090, 0.0, 0.101)),
        material=dark_trim,
        name="hinge_bridge",
    )
    for y in (-0.104, 0.104):
        chamber.visual(
            Cylinder(radius=0.011, length=0.020),
            origin=Origin(xyz=(-0.010, y, 0.022), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_trim,
            name=f"clamp_boss_{0 if y < 0 else 1}",
        )
    chamber.inertial = Inertial.from_geometry(
        Cylinder(radius=0.100, length=0.115),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
    )

    model.articulation(
        "base_to_chamber",
        ArticulationType.FIXED,
        parent=base,
        child=chamber,
        origin=Origin(xyz=(0.0, 0.0, 0.186)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((0.067, 0.190, 0.010)),
        origin=Origin(xyz=(0.0335, 0.0, 0.005)),
        material=smoke_clear,
        name="lid_plate",
    )
    lid.visual(
        Box((0.067, 0.190, 0.010)),
        origin=Origin(xyz=(0.1715, 0.0, 0.005)),
        material=smoke_clear,
        name="lid_front_band",
    )
    lid.visual(
        Box((0.071, 0.060, 0.010)),
        origin=Origin(xyz=(0.1025, 0.065, 0.005)),
        material=smoke_clear,
        name="lid_side_band_0",
    )
    lid.visual(
        Box((0.071, 0.060, 0.010)),
        origin=Origin(xyz=(0.1025, -0.065, 0.005)),
        material=smoke_clear,
        name="lid_side_band_1",
    )
    chute_shell = LatheGeometry.from_shell_profiles(
        [(0.040, 0.0), (0.038, 0.130)],
        [(0.032, 0.003), (0.030, 0.127)],
        segments=56,
    )
    lid.visual(
        mesh_from_geometry(chute_shell, "feed_chute"),
        origin=Origin(xyz=(0.1025, 0.0, 0.010)),
        material=smoke_clear,
        name="feed_chute",
    )
    lid.visual(
        Cylinder(radius=0.010, length=0.080),
        origin=Origin(xyz=(0.006, 0.0, 0.009), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="hinge_barrel",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.205, 0.190, 0.150)),
        mass=0.7,
        origin=Origin(xyz=(0.1025, 0.0, 0.070)),
    )

    model.articulation(
        "chamber_to_lid",
        ArticulationType.REVOLUTE,
        parent=chamber,
        child=lid,
        origin=Origin(xyz=(-0.100, 0.0, 0.106)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.5,
            lower=0.0,
            upper=1.25,
        ),
    )

    pusher = model.part("pusher")
    pusher.visual(
        Cylinder(radius=0.029, length=0.175),
        origin=Origin(xyz=(0.0, 0.0, -0.0875)),
        material=dark_trim,
        name="pusher_body",
    )
    pusher.visual(
        Cylinder(radius=0.044, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=dark_trim,
        name="pusher_cap",
    )
    pusher.visual(
        Box((0.050, 0.020, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=dark_trim,
        name="pusher_grip",
    )
    pusher.inertial = Inertial.from_geometry(
        Cylinder(radius=0.036, length=0.210),
        mass=0.25,
        origin=Origin(xyz=(0.0, 0.0, -0.070)),
    )

    model.articulation(
        "lid_to_pusher",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=pusher,
        origin=Origin(xyz=(0.1025, 0.0, 0.140)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=0.15,
            lower=0.0,
            upper=0.075,
        ),
    )

    basket = model.part("basket")
    basket_shell = LatheGeometry.from_shell_profiles(
        [(0.050, 0.0), (0.050, 0.062)],
        [(0.044, 0.003), (0.044, 0.059)],
        segments=56,
    )
    basket.visual(
        mesh_from_geometry(basket_shell, "basket_shell"),
        material=stainless,
        name="basket_shell",
    )
    basket.visual(
        Cylinder(radius=0.047, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=stainless,
        name="basket_disc",
    )
    basket.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=dark_trim,
        name="basket_hub",
    )
    basket.inertial = Inertial.from_geometry(
        Cylinder(radius=0.050, length=0.072),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
    )

    model.articulation(
        "base_to_basket",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=basket,
        origin=Origin(xyz=(0.0, 0.0, 0.186)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=30.0),
    )

    def add_clamp(name: str) -> None:
        clamp = model.part(name)
        clamp.visual(
            Cylinder(radius=0.004, length=0.030),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_trim,
            name="pivot_barrel",
        )
        clamp.visual(
            Box((0.012, 0.008, 0.072)),
            origin=Origin(xyz=(0.0, 0.0, 0.036)),
            material=dark_trim,
            name="clamp_arm",
        )
        clamp.visual(
            Box((0.024, 0.020, 0.014)),
            origin=Origin(xyz=(0.008, 0.0, 0.075)),
            material=dark_trim,
            name="clamp_pad",
        )
        clamp.visual(
            Box((0.010, 0.008, 0.020)),
            origin=Origin(xyz=(0.0, 0.0, 0.012)),
            material=dark_trim,
            name="clamp_knuckle",
        )
        clamp.inertial = Inertial.from_geometry(
            Box((0.030, 0.032, 0.090)),
            mass=0.08,
            origin=Origin(xyz=(0.005, 0.0, 0.040)),
        )

    add_clamp("left_clamp")
    add_clamp("right_clamp")

    model.articulation(
        "chamber_to_left_clamp",
        ArticulationType.REVOLUTE,
        parent=chamber,
        child="left_clamp",
        origin=Origin(xyz=(-0.010, 0.119, 0.022)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=-1.0,
            upper=0.0,
        ),
    )
    model.articulation(
        "chamber_to_right_clamp",
        ArticulationType.REVOLUTE,
        parent=chamber,
        child="right_clamp",
        origin=Origin(xyz=(-0.010, -0.119, 0.022)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=-1.0,
            upper=0.0,
        ),
    )

    tray = model.part("drip_tray")
    tray_plate = ExtrudeGeometry(rounded_rect_profile(0.112, 0.090, 0.015), 0.010)
    tray.visual(
        mesh_from_geometry(tray_plate, "drip_tray_plate"),
        origin=Origin(xyz=(0.056, 0.0, 0.005)),
        material=tray_black,
        name="tray_plate",
    )
    tray.visual(
        Box((0.012, 0.090, 0.014)),
        origin=Origin(xyz=(0.106, 0.0, 0.012)),
        material=tray_black,
        name="tray_stop",
    )
    for y in (-0.041, 0.041):
        tray.visual(
            Box((0.090, 0.008, 0.010)),
            origin=Origin(xyz=(0.045, y, 0.005)),
            material=tray_black,
            name=f"tray_runner_{0 if y < 0 else 1}",
        )
    tray.inertial = Inertial.from_geometry(
        Box((0.118, 0.090, 0.018)),
        mass=0.18,
        origin=Origin(xyz=(0.059, 0.0, 0.009)),
    )

    model.articulation(
        "base_to_drip_tray",
        ArticulationType.PRISMATIC,
        parent=base,
        child=tray,
        origin=Origin(xyz=(0.150, 0.0, -0.021)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=0.12,
            lower=0.0,
            upper=0.060,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    chamber = object_model.get_part("chamber")
    lid = object_model.get_part("lid")
    pusher = object_model.get_part("pusher")
    basket = object_model.get_part("basket")
    drip_tray = object_model.get_part("drip_tray")
    lid_hinge = object_model.get_articulation("chamber_to_lid")
    pusher_slide = object_model.get_articulation("lid_to_pusher")
    left_clamp = object_model.get_articulation("chamber_to_left_clamp")
    right_clamp = object_model.get_articulation("chamber_to_right_clamp")
    tray_slide = object_model.get_articulation("base_to_drip_tray")

    def aabb_center(aabb):
        if aabb is None:
            return None
        return tuple(0.5 * (aabb[0][i] + aabb[1][i]) for i in range(3))

    ctx.expect_gap(
        lid,
        chamber,
        axis="z",
        positive_elem="lid_plate",
        negative_elem="chamber_shell",
        min_gap=0.0005,
        max_gap=0.008,
        name="lid sits just above the chamber rim",
    )
    ctx.expect_overlap(
        lid,
        chamber,
        axes="xy",
        elem_b="chamber_shell",
        min_overlap=0.14,
        name="lid covers the chamber opening footprint",
    )
    ctx.expect_within(
        pusher,
        lid,
        axes="xy",
        inner_elem="pusher_body",
        outer_elem="feed_chute",
        margin=0.004,
        name="pusher stays centered inside the chute",
    )
    ctx.expect_gap(
        pusher,
        basket,
        axis="z",
        positive_elem="pusher_body",
        negative_elem="basket_shell",
        min_gap=0.004,
        max_gap=0.040,
        name="pusher stops above the basket",
    )
    ctx.expect_overlap(
        chamber,
        drip_tray,
        axes="y",
        elem_a="spout_tip",
        elem_b="tray_plate",
        min_overlap=0.020,
        name="drip tray stays centered under the spout",
    )

    lid_rest = aabb_center(ctx.part_element_world_aabb("lid", elem="feed_chute"))
    pusher_rest = ctx.part_world_position(pusher)
    tray_rest = ctx.part_world_position(drip_tray)
    left_pad_rest = ctx.part_element_world_aabb("left_clamp", elem="clamp_pad")
    right_pad_rest = ctx.part_element_world_aabb("right_clamp", elem="clamp_pad")

    with ctx.pose({lid_hinge: lid_hinge.motion_limits.upper}):
        lid_open = aabb_center(ctx.part_element_world_aabb("lid", elem="feed_chute"))
        ctx.check(
            "lid opens upward from the rear hinge",
            lid_rest is not None
            and lid_open is not None
            and lid_open[2] > lid_rest[2] + 0.04
            and lid_open[0] < lid_rest[0] - 0.01,
            details=f"rest={lid_rest}, open={lid_open}",
        )

    with ctx.pose({pusher_slide: pusher_slide.motion_limits.upper}):
        pusher_extended = ctx.part_world_position(pusher)
        ctx.check(
            "pusher slides upward through the chute",
            pusher_rest is not None
            and pusher_extended is not None
            and pusher_extended[2] > pusher_rest[2] + 0.05,
            details=f"rest={pusher_rest}, extended={pusher_extended}",
        )

    with ctx.pose({tray_slide: tray_slide.motion_limits.upper}):
        tray_extended = ctx.part_world_position(drip_tray)
        ctx.check(
            "drip tray slides forward",
            tray_rest is not None
            and tray_extended is not None
            and tray_extended[0] > tray_rest[0] + 0.045,
            details=f"rest={tray_rest}, extended={tray_extended}",
        )
        ctx.expect_overlap(
            chamber,
            drip_tray,
            axes="y",
            elem_a="spout_tip",
            elem_b="tray_plate",
            min_overlap=0.020,
            name="extended drip tray remains aligned with the spout",
        )

    with ctx.pose({left_clamp: left_clamp.motion_limits.lower, right_clamp: right_clamp.motion_limits.lower}):
        left_pad_released = ctx.part_element_world_aabb("left_clamp", elem="clamp_pad")
        right_pad_released = ctx.part_element_world_aabb("right_clamp", elem="clamp_pad")

        def y_center(aabb):
            if aabb is None:
                return None
            return 0.5 * (aabb[0][1] + aabb[1][1])

        ctx.check(
            "side clamps swing outward when released",
            left_pad_rest is not None
            and right_pad_rest is not None
            and left_pad_released is not None
            and right_pad_released is not None
            and y_center(left_pad_released) is not None
            and y_center(right_pad_released) is not None
            and y_center(left_pad_released) > y_center(left_pad_rest) + 0.01
            and y_center(right_pad_released) < y_center(right_pad_rest) - 0.01,
            details=(
                f"left_rest={left_pad_rest}, left_released={left_pad_released}, "
                f"right_rest={right_pad_rest}, right_released={right_pad_released}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
