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
    model = ArticulatedObject(name="desktop_hole_punch")

    body_metal = model.material("body_metal", rgba=(0.28, 0.30, 0.33, 1.0))
    handle_black = model.material("handle_black", rgba=(0.10, 0.10, 0.11, 1.0))
    steel = model.material("steel", rgba=(0.72, 0.74, 0.76, 1.0))

    def yz_section(
        width: float,
        height: float,
        radius: float,
        x: float,
        *,
        z_center: float = 0.0,
    ) -> list[tuple[float, float, float]]:
        return [(x, y, z_center + z) for y, z in rounded_rect_profile(width, height, radius)]

    body = model.part("body")
    body.visual(
        Box((0.34, 0.14, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=body_metal,
        name="base_plate",
    )
    body.visual(
        Box((0.11, 0.13, 0.040)),
        origin=Origin(xyz=(-0.115, 0.0, 0.032)),
        material=body_metal,
        name="rear_block",
    )
    body.visual(
        Box((0.14, 0.012, 0.060)),
        origin=Origin(xyz=(-0.055, 0.049, 0.042)),
        material=body_metal,
        name="side_cheek_0",
    )
    body.visual(
        Box((0.14, 0.012, 0.060)),
        origin=Origin(xyz=(-0.055, -0.049, 0.042)),
        material=body_metal,
        name="side_cheek_1",
    )
    body.visual(
        Box((0.112, 0.110, 0.020)),
        origin=Origin(xyz=(-0.041, 0.0, 0.082)),
        material=body_metal,
        name="bridge",
    )
    for index, y in enumerate((-0.049, 0.049)):
        body.visual(
            Box((0.024, 0.018, 0.022)),
            origin=Origin(xyz=(-0.126, y, 0.083)),
            material=body_metal,
            name=f"hinge_tower_{index}",
        )
    body.visual(
        Box((0.052, 0.094, 0.030)),
        origin=Origin(xyz=(0.018, 0.0, 0.060)),
        material=body_metal,
        name="punch_head",
    )
    body.visual(
        Box((0.072, 0.100, 0.014)),
        origin=Origin(xyz=(0.020, 0.0, 0.019)),
        material=steel,
        name="die_pad",
    )
    body.visual(
        Box((0.190, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, 0.034, -0.005)),
        material=steel,
        name="tray_rail_0",
    )
    body.visual(
        Box((0.190, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, -0.034, -0.005)),
        material=steel,
        name="tray_rail_1",
    )
    body.visual(
        Box((0.018, 0.078, 0.010)),
        origin=Origin(xyz=(-0.091, 0.0, -0.005)),
        material=steel,
        name="tray_stop",
    )
    body.visual(
        Box((0.022, 0.014, 0.008)),
        origin=Origin(xyz=(-0.103, 0.062, 0.071)),
        material=steel,
        name="latch_bridge",
    )
    body.visual(
        Box((0.018, 0.004, 0.018)),
        origin=Origin(xyz=(-0.103, 0.054, 0.084)),
        material=steel,
        name="latch_ear_0",
    )
    body.visual(
        Box((0.018, 0.004, 0.018)),
        origin=Origin(xyz=(-0.103, 0.068, 0.084)),
        material=steel,
        name="latch_ear_1",
    )
    for index, y in enumerate((-0.022, 0.022)):
        body.visual(
            Cylinder(radius=0.0065, length=0.030),
            origin=Origin(xyz=(0.018, y, 0.042)),
            material=steel,
            name=f"punch_pin_{index}",
        )
    for index, y in enumerate((-0.051, 0.051)):
        body.visual(
            Cylinder(radius=0.010, length=0.018),
            origin=Origin(
                xyz=(-0.128, y, 0.088),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=steel,
            name=f"hinge_knuckle_{index}",
        )
    body.inertial = Inertial.from_geometry(
        Box((0.34, 0.14, 0.10)),
        mass=2.6,
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
    )

    handle = model.part("handle")
    handle_shell = section_loft(
        [
            yz_section(0.114, 0.026, 0.008, 0.014, z_center=0.018),
            yz_section(0.110, 0.025, 0.008, 0.120, z_center=0.0175),
            yz_section(0.102, 0.023, 0.007, 0.235, z_center=0.0165),
            yz_section(0.086, 0.020, 0.006, 0.310, z_center=0.0145),
        ]
    )
    handle.visual(
        mesh_from_geometry(handle_shell, "handle_shell"),
        material=handle_black,
        name="handle_shell",
    )
    handle.visual(
        Box((0.020, 0.078, 0.018)),
        origin=Origin(xyz=(0.012, 0.0, 0.011)),
        material=handle_black,
        name="hinge_web",
    )
    handle.visual(
        Cylinder(radius=0.010, length=0.084),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="hinge_sleeve",
    )
    handle.visual(
        Box((0.050, 0.094, 0.018)),
        origin=Origin(xyz=(0.287, 0.0, 0.014)),
        material=handle_black,
        name="front_grip",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.32, 0.11, 0.03)),
        mass=0.85,
        origin=Origin(xyz=(0.160, 0.0, 0.015)),
    )

    tray = model.part("tray")
    tray.visual(
        Box((0.150, 0.062, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=steel,
        name="tray_pan",
    )
    tray.visual(
        Box((0.142, 0.003, 0.009)),
        origin=Origin(xyz=(0.0, 0.027, 0.0085)),
        material=steel,
        name="tray_side_0",
    )
    tray.visual(
        Box((0.142, 0.003, 0.009)),
        origin=Origin(xyz=(0.0, -0.027, 0.0085)),
        material=steel,
        name="tray_side_1",
    )
    tray.visual(
        Box((0.004, 0.054, 0.009)),
        origin=Origin(xyz=(-0.073, 0.0, 0.0085)),
        material=steel,
        name="tray_back",
    )
    tray.visual(
        Box((0.004, 0.054, 0.009)),
        origin=Origin(xyz=(0.073, 0.0, 0.0085)),
        material=steel,
        name="tray_front",
    )
    tray.visual(
        Box((0.132, 0.008, 0.003)),
        origin=Origin(xyz=(0.0, 0.033, 0.0055)),
        material=steel,
        name="runner_0",
    )
    tray.visual(
        Box((0.132, 0.008, 0.003)),
        origin=Origin(xyz=(0.0, -0.033, 0.0055)),
        material=steel,
        name="runner_1",
    )
    tray.visual(
        Box((0.020, 0.040, 0.012)),
        origin=Origin(xyz=(0.085, 0.0, 0.010)),
        material=handle_black,
        name="pull_tab",
    )
    tray.inertial = Inertial.from_geometry(
        Box((0.170, 0.070, 0.014)),
        mass=0.16,
        origin=Origin(xyz=(0.010, 0.0, 0.007)),
    )

    latch = model.part("latch")
    latch.visual(
        Cylinder(radius=0.0055, length=0.010),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pivot_sleeve",
    )
    latch.visual(
        Box((0.026, 0.004, 0.012)),
        origin=Origin(xyz=(0.013, 0.0, 0.0)),
        material=steel,
        name="latch_arm",
    )
    latch.visual(
        Box((0.010, 0.004, 0.012)),
        origin=Origin(xyz=(0.026, 0.0, -0.008)),
        material=steel,
        name="hook_tip",
    )
    latch.visual(
        Box((0.014, 0.004, 0.016)),
        origin=Origin(xyz=(0.010, 0.0, 0.012)),
        material=handle_black,
        name="thumb_tab",
    )
    latch.inertial = Inertial.from_geometry(
        Box((0.036, 0.010, 0.024)),
        mass=0.05,
        origin=Origin(xyz=(0.012, 0.0, 0.002)),
    )

    model.articulation(
        "body_to_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=handle,
        origin=Origin(xyz=(-0.128, 0.0, 0.088)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(62.0),
        ),
    )
    model.articulation(
        "body_to_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tray,
        origin=Origin(xyz=(0.0, 0.0, -0.017)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.18,
            lower=0.0,
            upper=0.085,
        ),
    )
    model.articulation(
        "body_to_latch",
        ArticulationType.REVOLUTE,
        parent=body,
        child=latch,
        origin=Origin(xyz=(-0.103, 0.061, 0.084)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=0.0,
            upper=1.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    handle = object_model.get_part("handle")
    tray = object_model.get_part("tray")
    latch = object_model.get_part("latch")
    handle_hinge = object_model.get_articulation("body_to_handle")
    tray_slide = object_model.get_articulation("body_to_tray")
    latch_hinge = object_model.get_articulation("body_to_latch")

    ctx.expect_gap(
        handle,
        body,
        axis="z",
        positive_elem="front_grip",
        negative_elem="bridge",
        min_gap=0.001,
        max_gap=0.030,
        name="closed handle parks just above the bridge",
    )
    ctx.expect_gap(
        latch,
        handle,
        axis="y",
        positive_elem="hook_tip",
        negative_elem="handle_shell",
        min_gap=0.001,
        max_gap=0.006,
        name="closed latch sits just outside the parked handle",
    )
    ctx.expect_contact(
        tray,
        body,
        elem_a="runner_0",
        elem_b="tray_rail_0",
        name="tray runner is supported by the underside guide rail",
    )

    closed_tip = ctx.part_element_world_aabb(handle, elem="front_grip")
    closed_tab = ctx.part_element_world_aabb(tray, elem="pull_tab")
    closed_hook = ctx.part_element_world_aabb(latch, elem="hook_tip")
    with ctx.pose({handle_hinge: math.radians(58.0)}):
        open_tip = ctx.part_element_world_aabb(handle, elem="front_grip")
    with ctx.pose({tray_slide: 0.085}):
        ctx.expect_contact(
            tray,
            body,
            elem_a="runner_0",
            elem_b="tray_rail_0",
            name="extended tray remains guided by the rail",
        )
        ctx.expect_overlap(
            tray,
            body,
            axes="x",
            elem_a="runner_0",
            elem_b="tray_rail_0",
            min_overlap=0.070,
            name="extended tray retains insertion in the underside guide",
        )
        open_tab = ctx.part_element_world_aabb(tray, elem="pull_tab")
    with ctx.pose({latch_hinge: 1.0}):
        open_hook = ctx.part_element_world_aabb(latch, elem="hook_tip")

    ctx.check(
        "handle opens upward from the rear hinge",
        closed_tip is not None
        and open_tip is not None
        and open_tip[0][2] > closed_tip[0][2] + 0.10
        and open_tip[0][0] < closed_tip[0][0] - 0.05,
        details=f"closed_tip={closed_tip}, open_tip={open_tip}",
    )
    ctx.check(
        "waste tray slides forward from the underside guide",
        closed_tab is not None
        and open_tab is not None
        and open_tab[0][0] > closed_tab[0][0] + 0.07,
        details=f"closed_tab={closed_tab}, open_tab={open_tab}",
    )
    ctx.check(
        "side latch rotates away from the parked handle",
        closed_hook is not None
        and open_hook is not None
        and open_hook[0][0] < closed_hook[0][0] - 0.015
        and open_hook[1][2] < closed_hook[1][2] - 0.012,
        details=f"closed_hook={closed_hook}, open_hook={open_hook}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
