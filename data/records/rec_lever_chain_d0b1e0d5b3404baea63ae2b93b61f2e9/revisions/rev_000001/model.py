from __future__ import annotations

from math import atan2, cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _cylinder_y(
    part,
    *,
    name: str,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    material: Material,
) -> None:
    """Add a cylinder whose axis is the local Y axis."""

    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _add_link_frame(
    part,
    *,
    length: float,
    drop: float,
    plate_width: float,
    rail_span: float,
    rail_thick: float,
    hub_radius: float,
    hub_width: float,
    material: Material,
    pin_material: Material,
    yoke_ear_y: float | None,
    yoke_ear_thick: float,
    yoke_pin_radius: float,
    end_tab: bool = False,
) -> None:
    """Build a flat, open-frame lever link from its proximal pivot."""

    path_len = (length * length + drop * drop) ** 0.5
    pitch = atan2(drop, length)

    def point(s: float, normal_offset: float = 0.0, y: float = 0.0) -> tuple[float, float, float]:
        return (
            cos(pitch) * s + sin(pitch) * normal_offset,
            y,
            -sin(pitch) * s + cos(pitch) * normal_offset,
        )

    # Central proximal tongue/bearing that fits inside the previous yoke.
    _cylinder_y(
        part,
        name="prox_hub",
        radius=hub_radius,
        length=hub_width,
        xyz=(0.0, 0.0, 0.0),
        material=material,
    )

    # A short proximal web ties both straps into the pivot boss without filling
    # the whole link.  It starts forward of the joint pin so the captured shaft
    # allowance remains scoped to the hub itself.
    rail_start = hub_radius + 0.018
    rail_end = path_len + 0.015 if end_tab else path_len - hub_radius * 1.35
    part.visual(
        Box((0.050, plate_width, rail_span + rail_thick)),
        origin=Origin(xyz=point(hub_radius + 0.005, 0.0), rpy=(0.0, pitch, 0.0)),
        material=material,
        name="prox_web",
    )

    # Two long straps leave the center open rather than filling the link as a
    # solid bar.
    rail_len = rail_end - rail_start
    for name, offset in (("upper_rail", rail_span / 2.0), ("lower_rail", -rail_span / 2.0)):
        part.visual(
            Box((rail_len, plate_width, rail_thick)),
            origin=Origin(xyz=point((rail_start + rail_end) / 2.0, offset), rpy=(0.0, pitch, 0.0)),
            material=material,
            name=name,
        )

    if yoke_ear_y is not None:
        # A rear crossbar and two side cheek arms make a real clevis at the
        # distal joint while leaving a central gap for the next link's tongue.
        part.visual(
            Box((0.036, 2.0 * (yoke_ear_y + yoke_ear_thick / 2.0), rail_span + rail_thick)),
            origin=Origin(xyz=point(path_len - 0.082, 0.0), rpy=(0.0, pitch, 0.0)),
            material=material,
            name="distal_crossbar",
        )

        for index, y in enumerate((-yoke_ear_y, yoke_ear_y)):
            part.visual(
                Box((0.100, yoke_ear_thick, rail_span + rail_thick * 0.8)),
                origin=Origin(xyz=point(path_len - 0.042, 0.0, y), rpy=(0.0, pitch, 0.0)),
                material=material,
                name=f"distal_arm_{index}",
            )
            _cylinder_y(
                part,
                name=f"distal_ear_{index}",
                radius=hub_radius * 0.86,
                length=yoke_ear_thick,
                xyz=point(path_len, 0.0, y),
                material=material,
            )

        _cylinder_y(
            part,
            name="distal_pin",
            radius=yoke_pin_radius,
            length=2.0 * (yoke_ear_y + yoke_ear_thick / 2.0),
            xyz=point(path_len, 0.0),
            material=pin_material,
        )

    if end_tab:
        part.visual(
            Box((0.040, plate_width * 0.96, rail_span + rail_thick * 0.5)),
            origin=Origin(xyz=point(path_len + 0.002, 0.0), rpy=(0.0, pitch, 0.0)),
            material=material,
            name="tip_bridge",
        )
        # The small tab is part of the final link and projects beyond the last
        # open frame, like a pickup tab or pull point at the chain tip.
        part.visual(
            Box((0.085, plate_width * 0.95, rail_span * 0.82)),
            origin=Origin(xyz=point(path_len + 0.035, 0.0), rpy=(0.0, pitch, 0.0)),
            material=material,
            name="end_tab",
        )
        _cylinder_y(
            part,
            name="tab_pad",
            radius=hub_radius * 0.40,
            length=plate_width * 1.05,
            xyz=point(path_len + 0.070, 0.0),
            material=pin_material,
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="open_frame_lever_chain")

    steel = model.material("dark_steel", rgba=(0.16, 0.17, 0.18, 1.0))
    blue = model.material("blue_anodized_link", rgba=(0.08, 0.26, 0.55, 1.0))
    blue_tip = model.material("worn_blue_tip", rgba=(0.10, 0.32, 0.62, 1.0))
    pin_black = model.material("blackened_pins", rgba=(0.025, 0.025, 0.028, 1.0))
    brass = model.material("brass_bushings", rgba=(0.82, 0.62, 0.28, 1.0))

    joint_z = 0.420

    pedestal = model.part("pedestal")
    pedestal.visual(
        Box((0.300, 0.220, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=steel,
        name="base",
    )
    pedestal.visual(
        Box((0.095, 0.105, 0.300)),
        origin=Origin(xyz=(0.0, 0.0, 0.190)),
        material=steel,
        name="column",
    )
    pedestal.visual(
        Box((0.080, 0.165, 0.030)),
        origin=Origin(xyz=(-0.046, 0.0, 0.345)),
        material=steel,
        name="saddle",
    )
    for index, y in enumerate((-0.067, 0.067)):
        pedestal.visual(
            Box((0.065, 0.026, 0.145)),
            origin=Origin(xyz=(0.0, y, joint_z)),
            material=steel,
            name=f"fork_{index}",
        )
        _cylinder_y(
            pedestal,
            name=f"bearing_{index}",
            radius=0.037,
            length=0.028,
            xyz=(0.0, y, joint_z),
            material=brass,
        )
    _cylinder_y(
        pedestal,
        name="joint_pin",
        radius=0.012,
        length=0.170,
        xyz=(0.0, 0.0, joint_z),
        material=pin_black,
    )

    link_0 = model.part("link_0")
    _add_link_frame(
        link_0,
        length=0.420,
        drop=0.035,
        plate_width=0.046,
        rail_span=0.112,
        rail_thick=0.016,
        hub_radius=0.045,
        hub_width=0.058,
        material=blue,
        pin_material=pin_black,
        yoke_ear_y=0.047,
        yoke_ear_thick=0.022,
        yoke_pin_radius=0.010,
    )

    link_1 = model.part("link_1")
    _add_link_frame(
        link_1,
        length=0.340,
        drop=0.030,
        plate_width=0.040,
        rail_span=0.098,
        rail_thick=0.014,
        hub_radius=0.039,
        hub_width=0.052,
        material=blue,
        pin_material=pin_black,
        yoke_ear_y=0.041,
        yoke_ear_thick=0.020,
        yoke_pin_radius=0.009,
    )

    link_2 = model.part("link_2")
    _add_link_frame(
        link_2,
        length=0.270,
        drop=0.024,
        plate_width=0.034,
        rail_span=0.082,
        rail_thick=0.012,
        hub_radius=0.034,
        hub_width=0.046,
        material=blue_tip,
        pin_material=pin_black,
        yoke_ear_y=None,
        yoke_ear_thick=0.0,
        yoke_pin_radius=0.0,
        end_tab=True,
    )

    model.articulation(
        "pedestal_to_link_0",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=link_0,
        origin=Origin(xyz=(0.0, 0.0, joint_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-0.85, upper=0.85),
    )
    model.articulation(
        "link_0_to_link_1",
        ArticulationType.REVOLUTE,
        parent=link_0,
        child=link_1,
        origin=Origin(xyz=(0.420, 0.0, -0.035)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.4, lower=-1.05, upper=1.05),
    )
    model.articulation(
        "link_1_to_link_2",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(0.340, 0.0, -0.030)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.8, lower=-1.15, upper=1.15),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    pedestal = object_model.get_part("pedestal")
    link_0 = object_model.get_part("link_0")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")
    joint_0 = object_model.get_articulation("pedestal_to_link_0")
    joint_1 = object_model.get_articulation("link_0_to_link_1")
    joint_2 = object_model.get_articulation("link_1_to_link_2")

    ctx.check(
        "three serial revolute joints",
        len(object_model.articulations) == 3
        and all(
            joint.articulation_type == ArticulationType.REVOLUTE
            for joint in (joint_0, joint_1, joint_2)
        ),
        details=f"joints={[joint.name for joint in object_model.articulations]}",
    )
    ctx.check(
        "joint axes are parallel",
        all(tuple(joint.axis) == (0.0, 1.0, 0.0) for joint in (joint_0, joint_1, joint_2)),
        details=f"axes={[joint.axis for joint in (joint_0, joint_1, joint_2)]}",
    )

    # Each joint uses a real pin through a simplified solid hub.  These are
    # intentional captured-shaft intersections, scoped to the exact pin/hub
    # visuals rather than broad part-pair allowances.
    ctx.allow_overlap(
        pedestal,
        link_0,
        elem_a="joint_pin",
        elem_b="prox_hub",
        reason="The pedestal pin is intentionally represented as captured inside the first link hub.",
    )
    ctx.allow_overlap(
        link_0,
        link_1,
        elem_a="distal_pin",
        elem_b="prox_hub",
        reason="The first link's distal pin is intentionally captured in the second link hub.",
    )
    ctx.allow_overlap(
        link_1,
        link_2,
        elem_a="distal_pin",
        elem_b="prox_hub",
        reason="The second link's distal pin is intentionally captured in the final link hub.",
    )

    ctx.expect_within(
        link_0,
        pedestal,
        axes="y",
        inner_elem="prox_hub",
        outer_elem="joint_pin",
        margin=0.002,
        name="first hub is retained on pedestal pin",
    )
    ctx.expect_within(
        link_1,
        link_0,
        axes="y",
        inner_elem="prox_hub",
        outer_elem="distal_pin",
        margin=0.002,
        name="second hub is retained on first pin",
    )
    ctx.expect_within(
        link_2,
        link_1,
        axes="y",
        inner_elem="prox_hub",
        outer_elem="distal_pin",
        margin=0.002,
        name="final hub is retained on second pin",
    )
    for index, (parent, child, pin) in enumerate(
        (
            (pedestal, link_0, "joint_pin"),
            (link_0, link_1, "distal_pin"),
            (link_1, link_2, "distal_pin"),
        )
    ):
        ctx.expect_overlap(
            parent,
            child,
            axes="y",
            elem_a=pin,
            elem_b="prox_hub",
            min_overlap=0.030,
            name=f"joint_{index} pin crosses hub width",
        )

    rest_0 = ctx.part_world_position(link_0)
    rest_1 = ctx.part_world_position(link_1)
    rest_2 = ctx.part_world_position(link_2)
    ctx.check(
        "chain steps downward at rest",
        rest_0 is not None
        and rest_1 is not None
        and rest_2 is not None
        and rest_0[2] > rest_1[2] > rest_2[2],
        details=f"origins={rest_0}, {rest_1}, {rest_2}",
    )

    tab_aabb = ctx.part_element_world_aabb(link_2, elem="end_tab")
    link_2_pos = ctx.part_world_position(link_2)
    ctx.check(
        "end tab projects beyond final link",
        tab_aabb is not None
        and link_2_pos is not None
        and tab_aabb[1][0] > link_2_pos[0] + 0.300,
        details=f"tab_aabb={tab_aabb}, link_2_pos={link_2_pos}",
    )

    with ctx.pose({joint_0: 0.35, joint_1: -0.30, joint_2: 0.40}):
        posed_2 = ctx.part_world_position(link_2)
    ctx.check(
        "serial joints move the downstream chain",
        rest_2 is not None
        and posed_2 is not None
        and abs(posed_2[0] - rest_2[0]) + abs(posed_2[2] - rest_2[2]) > 0.030,
        details=f"rest={rest_2}, posed={posed_2}",
    )

    return ctx.report()


object_model = build_object_model()
