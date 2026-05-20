from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    Mimic,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_laptop_stand")

    aluminum = model.material("brushed_aluminum", rgba=(0.72, 0.74, 0.76, 1.0))
    dark_link = model.material("dark_anodized_link", rgba=(0.12, 0.13, 0.14, 1.0))
    stainless = model.material("brushed_stainless_pin", rgba=(0.82, 0.83, 0.82, 1.0))
    rubber = model.material("matte_black_rubber", rgba=(0.025, 0.026, 0.027, 1.0))

    base = model.part("base_plate")
    base_shell = mesh_from_geometry(
        ExtrudeGeometry.from_z0(rounded_rect_profile(0.36, 0.25, 0.026, corner_segments=10), 0.014),
        "rounded_base_plate",
    )
    base.visual(base_shell, material=aluminum, name="base_shell")

    for index, (x, y) in enumerate(
        ((-0.135, -0.095), (0.135, -0.095), (-0.135, 0.095), (0.135, 0.095))
    ):
        base.visual(
            Box((0.055, 0.020, 0.004)),
            origin=Origin(xyz=(x, y, -0.002)),
            material=rubber,
            name=f"foot_pad_{index}",
        )

    lower_y = -0.075
    lower_z = 0.042
    side_x = 0.145
    cheek_height = 0.050
    cheek_z = 0.014 + cheek_height / 2.0
    for side_name, sx in (("left", -side_x), ("right", side_x)):
        for cheek_index, cheek_offset in enumerate((-0.008, 0.008)):
            base.visual(
                Box((0.004, 0.038, cheek_height)),
                origin=Origin(xyz=(sx + cheek_offset, lower_y, cheek_z)),
                material=aluminum,
                name=f"{side_name}_lower_cheek_{cheek_index}",
            )
        base.visual(
            Box((0.024, 0.050, 0.004)),
            origin=Origin(xyz=(sx, lower_y, 0.016)),
            material=aluminum,
            name=f"{side_name}_lower_foot",
        )

    base.visual(
        Cylinder(radius=0.005, length=0.318),
        origin=Origin(xyz=(0.0, lower_y, lower_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="lower_pivot_shaft",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.36, 0.25, 0.064)),
        mass=1.4,
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
    )

    dy = 0.135
    dz = 0.128
    link_length = math.hypot(dy, dz)
    link_angle = math.atan2(dz, dy)

    def build_link(name: str) -> object:
        link = model.part(name)
        link.visual(
            Box((0.008, link_length, 0.008)),
            origin=Origin(xyz=(0.0, dy / 2.0, dz / 2.0), rpy=(link_angle, 0.0, 0.0)),
            material=dark_link,
            name="link_bar",
        )
        link.visual(
            Cylinder(radius=0.009, length=0.008),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_link,
            name="lower_boss",
        )
        link.visual(
            Cylinder(radius=0.0042, length=0.011),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=stainless,
            name="lower_boss_pin_face",
        )
        link.visual(
            Cylinder(radius=0.009, length=0.008),
            origin=Origin(xyz=(0.0, dy, dz), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_link,
            name="upper_boss",
        )
        link.visual(
            Cylinder(radius=0.0042, length=0.011),
            origin=Origin(xyz=(0.0, dy, dz), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=stainless,
            name="upper_boss_pin_face",
        )
        link.inertial = Inertial.from_geometry(
            Box((0.012, link_length, 0.020)),
            mass=0.11,
            origin=Origin(xyz=(0.0, dy / 2.0, dz / 2.0)),
        )
        return link

    left_link = build_link("left_lift_link")
    right_link = build_link("right_lift_link")

    tray = model.part("tray")
    tray_panel = mesh_from_geometry(
        SlotPatternPanelGeometry(
            (0.320, 0.230),
            0.012,
            slot_size=(0.058, 0.008),
            pitch=(0.076, 0.035),
            frame=0.030,
            corner_radius=0.018,
            stagger=True,
            center=False,
        ),
        "slotted_top_tray",
    )
    tray.visual(
        tray_panel,
        origin=Origin(xyz=(side_x, 0.045, 0.016)),
        material=aluminum,
        name="slotted_tray_panel",
    )
    tray.visual(
        Box((0.270, 0.012, 0.018)),
        origin=Origin(xyz=(side_x, -0.073, 0.034)),
        material=rubber,
        name="front_laptop_stop",
    )
    for index, x in enumerate((0.045, side_x, 0.245)):
        tray.visual(
            Box((0.050, 0.012, 0.004)),
            origin=Origin(xyz=(x, 0.020, 0.030)),
            material=rubber,
            name=f"top_grip_pad_{index}",
        )
    tray.visual(
        Cylinder(radius=0.005, length=0.318),
        origin=Origin(xyz=(side_x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="upper_pivot_shaft",
    )
    tray.visual(
        Box((0.042, 0.014, 0.018)),
        origin=Origin(xyz=(side_x, 0.0, 0.009)),
        material=aluminum,
        name="center_pivot_hanger",
    )
    for hanger_name, x in (("left_upper_hanger", 0.016), ("right_upper_hanger", 0.274)):
        tray.visual(
            Box((0.020, 0.018, 0.020)),
            origin=Origin(xyz=(x, 0.0, 0.010)),
            material=aluminum,
            name=hanger_name,
        )
    tray.visual(
        Box((0.300, 0.010, 0.012)),
        origin=Origin(xyz=(side_x, 0.161, 0.024)),
        material=aluminum,
        name="rear_stiffening_lip",
    )
    tray.inertial = Inertial.from_geometry(
        Box((0.320, 0.240, 0.040)),
        mass=0.75,
        origin=Origin(xyz=(side_x, 0.045, 0.024)),
    )

    drive_limits = MotionLimits(effort=18.0, velocity=1.5, lower=-0.50, upper=0.45)
    model.articulation(
        "base_to_left_link",
        ArticulationType.REVOLUTE,
        parent=base,
        child=left_link,
        origin=Origin(xyz=(-side_x, lower_y, lower_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=drive_limits,
    )
    model.articulation(
        "base_to_right_link",
        ArticulationType.REVOLUTE,
        parent=base,
        child=right_link,
        origin=Origin(xyz=(side_x, lower_y, lower_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=drive_limits,
        mimic=Mimic("base_to_left_link", multiplier=1.0),
    )
    model.articulation(
        "left_link_to_tray",
        ArticulationType.REVOLUTE,
        parent=left_link,
        child=tray,
        origin=Origin(xyz=(0.0, dy, dz)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=-0.45, upper=0.50),
        mimic=Mimic("base_to_left_link", multiplier=-1.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base_plate")
    left_link = object_model.get_part("left_lift_link")
    right_link = object_model.get_part("right_lift_link")
    tray = object_model.get_part("tray")
    drive = object_model.get_articulation("base_to_left_link")

    for link in (left_link, right_link):
        ctx.allow_overlap(
            base,
            link,
            elem_a="lower_pivot_shaft",
            elem_b="lower_boss",
            reason="The lower lift-link boss surrounds the base-side pivot shaft as a real bushing.",
        )
        ctx.expect_overlap(
            base,
            link,
            axes="xyz",
            elem_a="lower_pivot_shaft",
            elem_b="lower_boss",
            min_overlap=0.006,
            name=f"{link.name} lower boss surrounds base shaft",
        )
        ctx.allow_overlap(
            base,
            link,
            elem_a="lower_pivot_shaft",
            elem_b="lower_boss_pin_face",
            reason="The stainless lower shaft is intentionally captured inside the lift-link pivot bushing face.",
        )
        ctx.expect_overlap(
            base,
            link,
            axes="xyz",
            elem_a="lower_pivot_shaft",
            elem_b="lower_boss_pin_face",
            min_overlap=0.006,
            name=f"{link.name} lower pivot captures base shaft",
        )

        ctx.allow_overlap(
            tray,
            link,
            elem_a="upper_pivot_shaft",
            elem_b="upper_boss",
            reason="The upper lift-link boss surrounds the tray-side pivot shaft as a real bushing.",
        )
        ctx.expect_overlap(
            tray,
            link,
            axes="xyz",
            elem_a="upper_pivot_shaft",
            elem_b="upper_boss",
            min_overlap=0.006,
            name=f"{link.name} upper boss surrounds tray shaft",
        )
        ctx.allow_overlap(
            tray,
            link,
            elem_a="upper_pivot_shaft",
            elem_b="upper_boss_pin_face",
            reason="The tray-side shaft is intentionally captured inside the lift-link upper pivot bushing face.",
        )
        ctx.expect_overlap(
            tray,
            link,
            axes="xyz",
            elem_a="upper_pivot_shaft",
            elem_b="upper_boss_pin_face",
            min_overlap=0.006,
            name=f"{link.name} upper pivot captures tray shaft",
        )

    ctx.expect_origin_gap(
        right_link,
        left_link,
        axis="x",
        min_gap=0.285,
        max_gap=0.295,
        name="left and right lift links are matched side counterparts",
    )
    ctx.expect_gap(
        tray,
        base,
        axis="z",
        min_gap=0.095,
        name="single tray is elevated above the broad base",
    )
    ctx.expect_overlap(
        tray,
        base,
        axes="xy",
        min_overlap=0.12,
        name="tray footprint remains carried over the base",
    )

    rest_pos = ctx.part_world_position(tray)
    with ctx.pose({drive: 0.40}):
        raised_pos = ctx.part_world_position(tray)
        ctx.expect_gap(
            tray,
            base,
            axis="z",
            min_gap=0.14,
            name="matched links raise the tray at upper pose",
        )
    with ctx.pose({drive: -0.40}):
        lowered_pos = ctx.part_world_position(tray)
        ctx.expect_gap(
            tray,
            base,
            axis="z",
            min_gap=0.030,
            name="lowered tray still clears the base plate",
        )

    ctx.check(
        "lift linkage raises and lowers one rigid tray",
        rest_pos is not None
        and raised_pos is not None
        and lowered_pos is not None
        and raised_pos[2] > rest_pos[2] + 0.035
        and lowered_pos[2] < rest_pos[2] - 0.035,
        details=f"rest={rest_pos}, raised={raised_pos}, lowered={lowered_pos}",
    )

    return ctx.report()


object_model = build_object_model()
