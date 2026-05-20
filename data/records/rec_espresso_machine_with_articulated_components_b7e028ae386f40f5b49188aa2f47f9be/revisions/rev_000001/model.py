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
    SlotPatternPanelGeometry,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_group_espresso_machine")

    steel = model.material("steel", rgba=(0.80, 0.81, 0.82, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.14, 0.14, 0.14, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.07, 0.07, 0.07, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.64, 0.66, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=steel,
        name="base_floor",
    )
    body.visual(
        Box((0.48, 0.56, 0.010)),
        origin=Origin(xyz=(0.080, 0.0, 0.050)),
        material=steel,
        name="tray_support",
    )
    body.visual(
        Box((0.64, 0.012, 0.29)),
        origin=Origin(xyz=(0.0, 0.334, 0.167)),
        material=steel,
        name="left_side",
    )
    body.visual(
        Box((0.64, 0.012, 0.29)),
        origin=Origin(xyz=(0.0, -0.334, 0.167)),
        material=steel,
        name="right_side",
    )
    body.visual(
        Box((0.012, 0.656, 0.29)),
        origin=Origin(xyz=(-0.314, 0.0, 0.167)),
        material=steel,
        name="rear_wall",
    )
    body.visual(
        Box((0.024, 0.656, 0.182)),
        origin=Origin(xyz=(0.308, 0.0, 0.226)),
        material=steel,
        name="front_panel",
    )
    body.visual(
        Box((0.024, 0.656, 0.05)),
        origin=Origin(xyz=(0.308, 0.0, 0.025)),
        material=steel,
        name="lower_apron",
    )
    body.visual(
        Box((0.38, 0.68, 0.015)),
        origin=Origin(xyz=(0.130, 0.0, 0.3125)),
        material=steel,
        name="top_front",
    )
    body.visual(
        Box((0.24, 0.08, 0.015)),
        origin=Origin(xyz=(-0.180, 0.300, 0.3125)),
        material=steel,
        name="top_left_rail",
    )
    body.visual(
        Box((0.24, 0.08, 0.015)),
        origin=Origin(xyz=(-0.180, -0.300, 0.3125)),
        material=steel,
        name="top_right_rail",
    )
    body.visual(
        Box((0.34, 0.026, 0.012)),
        origin=Origin(xyz=(0.020, 0.315, 0.074)),
        material=dark_plastic,
        name="left_guide",
    )
    body.visual(
        Box((0.34, 0.026, 0.012)),
        origin=Origin(xyz=(0.020, -0.315, 0.074)),
        material=dark_plastic,
        name="right_guide",
    )
    for suffix, y_pos in (("left", 0.140), ("right", -0.140)):
        body.visual(
            Cylinder(radius=0.037, length=0.075),
            origin=Origin(
                xyz=(0.328, y_pos, 0.225),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=steel,
            name=f"{suffix}_head_shell",
        )
        body.visual(
            Cylinder(radius=0.022, length=0.006),
            origin=Origin(
                xyz=(0.372, y_pos, 0.218),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=steel,
            name=f"{suffix}_head_face",
        )
        body.visual(
            Cylinder(radius=0.030, length=0.028),
            origin=Origin(xyz=(0.352, y_pos, 0.206)),
            material=steel,
            name=f"{suffix}_head_collar",
        )
    for index, y_pos in enumerate((0.250, 0.085, -0.085, -0.250)):
        body.visual(
            Cylinder(radius=0.010, length=0.016),
            origin=Origin(xyz=(0.312, y_pos, 0.276)),
            material=black_rubber,
            name=f"knob_{index}",
        )
    for suffix, y_pos in (("left", 0.280), ("right", -0.280)):
        body.visual(
            Box((0.050, 0.120, 0.012)),
            origin=Origin(xyz=(0.140, y_pos, 0.120)),
            material=dark_plastic,
            name=f"{suffix}_cup_rail",
        )
    for index, (x_pos, y_pos) in enumerate(
        ((0.240, 0.260), (0.240, -0.260), (-0.240, 0.260), (-0.240, -0.260))
    ):
        body.visual(
            Cylinder(radius=0.028, length=0.014),
            origin=Origin(xyz=(x_pos, y_pos, 0.007)),
            material=dark_plastic,
            name=f"foot_{index}",
        )
    body.inertial = Inertial.from_geometry(
        Box((0.68, 0.70, 0.33)),
        mass=28.0,
        origin=Origin(xyz=(0.0, 0.0, 0.165)),
    )

    lid = model.part("lid")
    lid.visual(
        Cylinder(radius=0.006, length=0.50),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hinge",
    )
    lid.visual(
        Box((0.24, 0.50, 0.014)),
        origin=Origin(xyz=(0.12, 0.0, 0.0)),
        material=steel,
        name="panel",
    )
    lid.visual(
        Box((0.10, 0.022, 0.020)),
        origin=Origin(xyz=(0.170, 0.0, 0.017)),
        material=dark_plastic,
        name="handle",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.24, 0.50, 0.030)),
        mass=2.2,
        origin=Origin(xyz=(0.120, 0.0, 0.010)),
    )

    tray = model.part("tray")
    tray.visual(
        Box((0.46, 0.60, 0.004)),
        origin=Origin(xyz=(0.23, 0.0, 0.002)),
        material=steel,
        name="bottom",
    )
    tray.visual(
        Box((0.46, 0.012, 0.022)),
        origin=Origin(xyz=(0.23, 0.294, 0.011)),
        material=steel,
        name="left_wall",
    )
    tray.visual(
        Box((0.46, 0.012, 0.022)),
        origin=Origin(xyz=(0.23, -0.294, 0.011)),
        material=steel,
        name="right_wall",
    )
    tray.visual(
        Box((0.012, 0.576, 0.022)),
        origin=Origin(xyz=(0.006, 0.0, 0.011)),
        material=steel,
        name="rear_wall",
    )
    tray.visual(
        Box((0.018, 0.60, 0.05)),
        origin=Origin(xyz=(0.451, 0.0, 0.025)),
        material=dark_plastic,
        name="front_lip",
    )
    tray.visual(
        mesh_from_geometry(
            SlotPatternPanelGeometry(
                (0.42, 0.54),
                0.003,
                slot_size=(0.028, 0.004),
                pitch=(0.040, 0.016),
                frame=0.012,
                corner_radius=0.004,
                stagger=True,
            ),
            "drip_grate",
        ),
        origin=Origin(xyz=(0.230, 0.0, 0.0235)),
        material=steel,
        name="grate",
    )
    for index, (x_pos, y_pos) in enumerate(
        ((0.060, 0.240), (0.060, -0.240), (0.400, 0.240), (0.400, -0.240))
    ):
        tray.visual(
            Box((0.012, 0.012, 0.020)),
            origin=Origin(xyz=(x_pos, y_pos, 0.012)),
            material=steel,
            name=f"post_{index}",
        )
    tray.inertial = Inertial.from_geometry(
        Box((0.46, 0.60, 0.05)),
        mass=3.0,
        origin=Origin(xyz=(0.23, 0.0, 0.025)),
    )

    def add_portafilter(part_name: str) -> object:
        portafilter = model.part(part_name)
        portafilter.visual(
            Cylinder(radius=0.031, length=0.012),
            origin=Origin(xyz=(0.0, 0.0, -0.006)),
            material=steel,
            name="lug",
        )
        portafilter.visual(
            Cylinder(radius=0.040, length=0.028),
            origin=Origin(xyz=(0.0, 0.0, -0.020)),
            material=steel,
            name="basket",
        )
        portafilter.visual(
            Cylinder(radius=0.010, length=0.060),
            origin=Origin(
                xyz=(0.050, 0.0, -0.012),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=steel,
            name="neck",
        )
        portafilter.visual(
            Cylinder(radius=0.013, length=0.130),
            origin=Origin(
                xyz=(0.145, 0.0, -0.016),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=black_rubber,
            name="handle",
        )
        portafilter.visual(
            Sphere(radius=0.015),
            origin=Origin(xyz=(0.210, 0.0, -0.016)),
            material=black_rubber,
            name="tip",
        )
        portafilter.inertial = Inertial.from_geometry(
            Box((0.25, 0.08, 0.05)),
            mass=0.9,
            origin=Origin(xyz=(0.12, 0.0, -0.016)),
        )
        return portafilter

    def add_wand(part_name: str, inward_sign: float) -> object:
        wand = model.part(part_name)
        wand.visual(
            Cylinder(radius=0.009, length=0.030),
            material=dark_plastic,
            name="pivot",
        )
        wand.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    [
                        (0.002, inward_sign * 0.005, -0.012),
                        (0.006, inward_sign * 0.010, -0.060),
                        (0.017, inward_sign * 0.015, -0.136),
                        (0.031, inward_sign * 0.019, -0.184),
                    ],
                    radius=0.0035,
                    samples_per_segment=14,
                    radial_segments=18,
                ),
                f"{part_name}_tube",
            ),
            material=steel,
            name="tube",
        )
        wand.visual(
            Cylinder(radius=0.004, length=0.018),
            origin=Origin(
                xyz=(0.031, inward_sign * 0.012, -0.186),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=steel,
            name="tip",
        )
        wand.inertial = Inertial.from_geometry(
            Box((0.08, 0.06, 0.22)),
            mass=0.25,
            origin=Origin(xyz=(0.014, inward_sign * 0.016, -0.090)),
        )
        return wand

    left_portafilter = add_portafilter("left_portafilter")
    right_portafilter = add_portafilter("right_portafilter")
    left_wand = add_wand("left_wand", inward_sign=-1.0)
    right_wand = add_wand("right_wand", inward_sign=1.0)

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-0.300, 0.0, 0.312)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.0,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "body_to_tray",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tray,
        origin=Origin(xyz=(-0.140, 0.0, 0.055)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.15,
            lower=0.0,
            upper=0.11,
        ),
    )
    model.articulation(
        "body_to_left_portafilter",
        ArticulationType.REVOLUTE,
        parent=body,
        child=left_portafilter,
        origin=Origin(xyz=(0.356, 0.140, 0.188)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.0,
            lower=-0.70,
            upper=0.35,
        ),
    )
    model.articulation(
        "body_to_right_portafilter",
        ArticulationType.REVOLUTE,
        parent=body,
        child=right_portafilter,
        origin=Origin(xyz=(0.356, -0.140, 0.188)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.0,
            lower=-0.35,
            upper=0.70,
        ),
    )
    model.articulation(
        "body_to_left_wand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=left_wand,
        origin=Origin(xyz=(0.319, 0.349, 0.274)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=-1.05,
            upper=0.90,
        ),
    )
    model.articulation(
        "body_to_right_wand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=right_wand,
        origin=Origin(xyz=(0.319, -0.349, 0.274)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=-0.90,
            upper=1.05,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    tray = object_model.get_part("tray")
    left_portafilter = object_model.get_part("left_portafilter")
    right_portafilter = object_model.get_part("right_portafilter")
    left_wand = object_model.get_part("left_wand")
    right_wand = object_model.get_part("right_wand")

    lid_joint = object_model.get_articulation("body_to_lid")
    tray_joint = object_model.get_articulation("body_to_tray")
    left_portafilter_joint = object_model.get_articulation("body_to_left_portafilter")
    right_portafilter_joint = object_model.get_articulation("body_to_right_portafilter")
    left_wand_joint = object_model.get_articulation("body_to_left_wand")
    right_wand_joint = object_model.get_articulation("body_to_right_wand")

    def aabb_center(aabb):
        if aabb is None:
            return None
        min_corner, max_corner = aabb
        return tuple((min_corner[i] + max_corner[i]) * 0.5 for i in range(3))

    ctx.expect_gap(
        body,
        left_portafilter,
        axis="z",
        positive_elem="left_head_shell",
        negative_elem="lug",
        max_gap=0.002,
        max_penetration=0.0,
        name="left portafilter seats under left group head",
    )
    ctx.expect_gap(
        body,
        right_portafilter,
        axis="z",
        positive_elem="right_head_shell",
        negative_elem="lug",
        max_gap=0.002,
        max_penetration=0.0,
        name="right portafilter seats under right group head",
    )
    ctx.expect_overlap(
        left_portafilter,
        body,
        axes="xy",
        elem_a="lug",
        elem_b="left_head_shell",
        min_overlap=0.03,
        name="left brew axis stays aligned with left head",
    )
    ctx.expect_overlap(
        right_portafilter,
        body,
        axes="xy",
        elem_a="lug",
        elem_b="right_head_shell",
        min_overlap=0.03,
        name="right brew axis stays aligned with right head",
    )
    ctx.expect_overlap(
        tray,
        left_portafilter,
        axes="y",
        elem_a="grate",
        elem_b="basket",
        min_overlap=0.06,
        name="tray spans beneath the left portafilter",
    )
    ctx.expect_overlap(
        tray,
        right_portafilter,
        axes="y",
        elem_a="grate",
        elem_b="basket",
        min_overlap=0.06,
        name="tray spans beneath the right portafilter",
    )

    lid_closed = ctx.part_element_world_aabb(lid, elem="panel")
    left_handle_rest = ctx.part_element_world_aabb(left_portafilter, elem="handle")
    right_handle_rest = ctx.part_element_world_aabb(right_portafilter, elem="handle")
    left_tip_rest = ctx.part_element_world_aabb(left_wand, elem="tip")
    right_tip_rest = ctx.part_element_world_aabb(right_wand, elem="tip")
    tray_rest_pos = ctx.part_world_position(tray)

    with ctx.pose({lid_joint: 1.10}):
        lid_open = ctx.part_element_world_aabb(lid, elem="panel")
    with ctx.pose({tray_joint: 0.11}):
        tray_extended_pos = ctx.part_world_position(tray)
        ctx.expect_overlap(
            tray,
            body,
            axes="x",
            min_overlap=0.25,
            name="extended tray keeps retained insertion under the body",
        )
    with ctx.pose({left_portafilter_joint: -0.60}):
        left_handle_open = ctx.part_element_world_aabb(left_portafilter, elem="handle")
    with ctx.pose({right_portafilter_joint: 0.60}):
        right_handle_open = ctx.part_element_world_aabb(right_portafilter, elem="handle")
    with ctx.pose({left_wand_joint: 0.80}):
        left_tip_swung = ctx.part_element_world_aabb(left_wand, elem="tip")
    with ctx.pose({right_wand_joint: -0.80}):
        right_tip_swung = ctx.part_element_world_aabb(right_wand, elem="tip")

    lid_closed_top = lid_closed[1][2] if lid_closed is not None else None
    lid_open_top = lid_open[1][2] if lid_open is not None else None
    ctx.check(
        "service lid opens upward",
        lid_closed_top is not None and lid_open_top is not None and lid_open_top > lid_closed_top + 0.12,
        details=f"closed_top={lid_closed_top}, open_top={lid_open_top}",
    )

    ctx.check(
        "drip tray slides forward",
        tray_rest_pos is not None
        and tray_extended_pos is not None
        and tray_extended_pos[0] > tray_rest_pos[0] + 0.09,
        details=f"rest={tray_rest_pos}, extended={tray_extended_pos}",
    )

    left_handle_rest_center = aabb_center(left_handle_rest)
    left_handle_open_center = aabb_center(left_handle_open)
    ctx.check(
        "left portafilter rotates about the brew axis",
        left_handle_rest_center is not None
        and left_handle_open_center is not None
        and abs(left_handle_open_center[1] - left_handle_rest_center[1]) > 0.05,
        details=f"rest={left_handle_rest_center}, moved={left_handle_open_center}",
    )

    right_handle_rest_center = aabb_center(right_handle_rest)
    right_handle_open_center = aabb_center(right_handle_open)
    ctx.check(
        "right portafilter rotates about the brew axis",
        right_handle_rest_center is not None
        and right_handle_open_center is not None
        and abs(right_handle_open_center[1] - right_handle_rest_center[1]) > 0.05,
        details=f"rest={right_handle_rest_center}, moved={right_handle_open_center}",
    )

    left_tip_rest_center = aabb_center(left_tip_rest)
    left_tip_swung_center = aabb_center(left_tip_swung)
    ctx.check(
        "left steam wand swings around its support pivot",
        left_tip_rest_center is not None
        and left_tip_swung_center is not None
        and math.hypot(
            left_tip_swung_center[0] - left_tip_rest_center[0],
            left_tip_swung_center[1] - left_tip_rest_center[1],
        )
        > 0.020,
        details=f"rest={left_tip_rest_center}, swung={left_tip_swung_center}",
    )

    right_tip_rest_center = aabb_center(right_tip_rest)
    right_tip_swung_center = aabb_center(right_tip_swung)
    ctx.check(
        "right steam wand swings around its support pivot",
        right_tip_rest_center is not None
        and right_tip_swung_center is not None
        and math.hypot(
            right_tip_swung_center[0] - right_tip_rest_center[0],
            right_tip_swung_center[1] - right_tip_rest_center[1],
        )
        > 0.020,
        details=f"rest={right_tip_rest_center}, swung={right_tip_swung_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
